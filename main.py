import pygame
import logging
import carla

import argparse
import json
import logging
import math
import os

from typing import List, Dict

class HUD(object):
    def __init__(self, width=420, height=420):
        self.dim = (width, height)
        self.set_font()

    def set_font(self):
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self.font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
 
    def tick(self, 
             location: carla.Transform, 
             velocity: carla.Vector3D, 
             control: carla.VehicleControl):
        self.info_text = [
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (location.location.x, location.location.y)),
            'Height:  % 18.0f m' % location.location.z,
            ('Throttle:', control.throttle, 0.0, 1.0),
            ('Steer:', control.steer, -1.0, 1.0),
            ('Brake:', control.brake, 0.0, 1.0),
            ('Reverse:', control.reverse),
            ('Hand brake:', control.hand_brake),
            ('Manual:', control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)
        ]
        
    def render(self, display):
        info_surface = pygame.Surface(self.dim)
        info_surface.set_alpha(0)
        display.blit(info_surface, (0, 0))
        
        v_offset = 4
        for item in self.info_text:
            if isinstance(item, tuple):
                # 处理带范围的参数
                text = f"{item[0]}: {item[1]:.2f}"
                text_surface = self.font_mono.render(text, True, (255, 255, 255))
            else:
                text_surface = self.font_mono.render(item, True, (255, 255, 255))
            
            display.blit(text_surface, (4, v_offset))
            v_offset += 14

class SimEnvironment(object):   
    def __init__(self, 
                 carla_world: carla.World, 
                 configs: Dict):
        self.world = carla_world
        self.configs = configs
        self.vehicle = None
        self.restart()
    
    def restart(self):
        logging.debug("New game begins!")
        self.vehicle = self.setup_vehicle(
            self.world, 
            self.configs
        )
        self.vehicle.set_autopilot(False)

        self.sensors = self.setup_sensors(
            self.world, 
            self.vehicle, 
            self.configs.get("sensors", [])
        )

    def destroy(self):
        logging.debug("Destroy!")
        for sensor in self.sensors:
            sensor.destroy()
        
        if self.vehicle:
            self.vehicle.destroy()
    
    def setup_vehicle(self,
                      carla_world: carla.World,
                      vehicle_config: Dict) -> carla.Vehicle:
        logging.debug(
            "Spawning vehicle: {}".format(vehicle_config.get("type"))
        )
        
        blueprint_lib = carla_world.get_blueprint_library()
        carla_map = carla_world.get_map()

        blueprint = blueprint_lib.filter(vehicle_config.get("type"))[0]
        
        blueprint.set_attribute(
            "role_name", 
            vehicle_config.get("id")
        )
        
        blueprint.set_attribute(
            "ros_name", 
            vehicle_config.get("id")
        ) 

        return carla_world.spawn_actor(
            blueprint,
            carla_map.get_spawn_points()[0],
            attach_to=None
        )        

    def setup_sensors(self, 
                      carla_world: carla.World,
                      carla_vehicle: carla.Vehicle,
                      sensors_config: List[Dict]) -> List[carla.Sensor]:    
        blueprint_lib = carla_world.get_blueprint_library()

        sensors = []
        for sensor in sensors_config:
            logging.debug("Spawning sensor: {}".format(sensor))

            blueprint = blueprint_lib.filter(sensor.get("type"))[0]
            blueprint.set_attribute("ros_name", sensor.get("id")) 
            blueprint.set_attribute("role_name", sensor.get("id")) 
            for key, value in sensor.get("attributes", {}).items():
                blueprint.set_attribute(str(key), str(value))

            location = carla.Location(
                x=sensor["spawn_point"]["x"], 
                y=-sensor["spawn_point"]["y"], 
                z=sensor["spawn_point"]["z"]
            )

            rotation = carla.Rotation(
                roll=sensor["spawn_point"]["roll"], 
                pitch=-sensor["spawn_point"]["pitch"], 
                yaw=-sensor["spawn_point"]["yaw"]
            )

            spawn_point = carla.Transform(location, rotation)

            sensor = carla_world.spawn_actor(
                blueprint,
                spawn_point,
                attach_to=carla_vehicle
            )
            
            sensor.enable_for_ros()
            sensors.append(sensor)

        return sensors

class KeyboardControl(object):
    def __init__(self, 
                 sim_env: SimEnvironment):
        self.reverse_gear = 1
        self.control = carla.VehicleControl()
        self.lights = carla.VehicleLightState.NONE
        sim_env.vehicle.set_light_state(self.lights)
        self.steer_cache = 0.0
        
    def parse_events(self, 
                     sim_env: SimEnvironment, 
                     clock: pygame.time.Clock) -> bool:
        current_lights = self.lights        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            
            if event.type == pygame.KEYUP:
                logging.debug("Detected KEYUP")
                if event.key == pygame.K_r:
                    logging.debug("R pressed!")
                    sim_env.destroy()
                    sim_env.restart()
                
                if event.key == pygame.K_q:
                    logging.debug("Q pressed!")
                    self.control.gear = 1 if self.control.reverse else -1

        self.parse_vehicle_cmd(pygame.key.get_pressed(), clock.get_time())
        self.control.reverse = self.control.gear < 0
        
        # Set automatic control-related vehicle lights
        if self.control.brake:
            current_lights |= carla.VehicleLightState.Brake
        else: # Remove the Brake flag
            current_lights &= ~carla.VehicleLightState.Brake
        if self.control.reverse:
            current_lights |= carla.VehicleLightState.Reverse
        else: # Remove the Reverse flag
            current_lights &= ~carla.VehicleLightState.Reverse
        if current_lights != self.lights: # Change the light state only if necessary
            sim_env.vehicle.set_light_state(carla.VehicleLightState(current_lights))        

        # Apply control
        sim_env.vehicle.apply_control(self.control)
    
    def parse_vehicle_cmd(self, keys, milliseconds):
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            logging.debug('Throttle!')
            self.control.throttle = min(self.control.throttle + 0.05, 1.00)
        else:
            self.control.throttle = 0.0

        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            logging.debug('Brake!')
            self.control.brake = min(self.control.brake + 0.1, 1)
        else:
            self.control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            logging.debug("LEFT!")
            if self.steer_cache > 0:
                self.steer_cache = 0
            else:
                self.steer_cache -= steer_increment
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            logging.debug("RIGHT!")
            if self.steer_cache < 0:
                self.steer_cache = 0
            else:
                self.steer_cache += steer_increment
        else:
            self.steer_cache = 0.0

        self.steer_cache = min(0.7, max(-0.7, self.steer_cache))
        self.control.steer = round(self.steer_cache, 1)
        self.control.hand_brake = keys[pygame.K_SPACE]

def game_loop(args):
    pygame.init()
    pygame.font.init()

    display_width = 300
    display_height = 300
    display = pygame.display.set_mode(
        (display_width, display_height), 
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    
    pygame.display.set_caption('Carla Manual Control')

    carla_world = None
    original_settings = None
    
    try:
        carla_client = carla.Client(args.host, args.port)
        carla_client.set_timeout(2000.0)

        carla_world = carla_client.get_world()
        original_settings = carla_world.get_settings()
        settings = carla_world.get_settings()
        
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        carla_world.apply_settings(settings)
        
        config = None
        with open(args.file) as f:
            config = json.load(f)
        
        sim_env = SimEnvironment(carla_world, config)
        key_controller = KeyboardControl(sim_env)
        hud = HUD()

        _ = carla_world.tick()
        logging.debug("Running...")
        
        clock = pygame.time.Clock()
        while True:
            _ = carla_world.tick()

            clock.tick_busy_loop(60)
            
            # 更新 HUD 信息
            location = sim_env.vehicle.get_transform()
            velocity = sim_env.vehicle.get_velocity()
            hud.tick(location, velocity, key_controller.control)
            
            # 渲染 hud 信息
            display.fill((0, 0, 0))
            hud.render(display)
            
            if key_controller.parse_events(sim_env, clock):
                return

            pygame.display.flip()

    except KeyboardInterrupt:
        logging.debug("Stopped by user!")
        
    finally:
        sim_env.destroy()
        if original_settings is not None:
            carla_world.apply_settings(original_settings)
            original_settings = None

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA ROS2 native')
    argparser.add_argument(
        '--host', 
        metavar='H', 
        default='localhost', 
        help='IP of the host CARLA Simulator (default: localhost)'
    )
    
    argparser.add_argument(
        '--port', 
        metavar='P', 
        default=2000, 
        type=int, 
        help='TCP port of CARLA Simulator (default: 2000)'
    )
    
    argparser.add_argument(
        '-f', 
        '--file', 
        default='stack.json', 
        help='File to be executed'
    )
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    args = argparser.parse_args()
    
    game_loop(args)