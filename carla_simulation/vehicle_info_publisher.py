#!/usr/bin/env python

import logging
from typing import Dict

import rclpy
import rclpy.qos

import carla
import carla_data_to_ros

import std_msgs.msg
import nav_msgs.msg
import sensor_driver_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

def check_config(config: Dict)->bool:
    """Check whether the given configuration file is valid"""

    required_topics = [
        "loop_rate",
        "gnss_sub_topic",
        "imu_sub_topic",
        "lidar_sub_topic",
        "lidar_pub_topic",
        "gps_pub_topic",
        "odom_pub_topic",
        "velocity_pub_topic",
        "imu_pub_topic",
        "position_pub_topic"
    ]   

    flag = True
    for topic in required_topics:
        if not config.get(topic):
            logging.error("Missing configuration of {}".format(topic))
            flag = False
            
    return flag

class VehicleInfoPublisher(rclpy.Node):
    # Data received from carla
    header           = std_msgs.msg.Header()
    carla_lidar_data = sensor_msgs.msg.PointCloud2()
    carla_gnss_data  = sensor_msgs.msg.NavSatFix()
    carla_imu_data   = sensor_msgs.msg.Imu()

    # Data to be published
    lidar_data    = sensor_msgs.msg.PointCloud2()
    imu_data      = sensor_msgs.msg.Imu()
    position_data = geometry_msgs.msg.Vector3Stamped()
    gps_data      = sensor_driver_msgs.msg.GpswithHeading()
    odom_data     = sensor_driver_msgs.msg.OdometrywithGps()
    velocity_data = sensor_driver_msgs.msg.InsVelocity() 

    # Pseudo sensor data
    pseudo_odom     = nav_msgs.msg.Odometry()
    pseudo_position = geometry_msgs.msg.Vector3()
    pseudo_velocity = geometry_msgs.msg.Twist()
    pseudo_heading  = None
    
    def __init__(self, 
                 vehicle: carla.Vehicle, 
                 config: Dict):
        super().__init__('vehicle_info_publisher')
        
        self.vehicle = vehicle
        self.config = config

        #* Subscribe lidar / gnss / imu info
        self.gnss_sub = self.create_subscription(
            sensor_msgs.msg.NavSatFix,
            config.get("gnss_sub_topic"),
            self.update_carla_gnss_data,
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.imu_sub = self.create_subscription(
            sensor_msgs.msg.Imu,
            config.get("imu_sub_topic"),
            self.update_carla_imu_data,
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.lidar_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            config.get("lidar_sub_topic"),
            self.update_carla_lidar_data,
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        #* Publish lidar / gps / odom / velocity / imu / ego_state info
        self.lidar_pub = self.create_publisher(
            sensor_msgs.msg.PointCloud2,
            config.get("lidar_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.gps_pub = self.create_publisher(
            sensor_driver_msgs.msg.GpswithHeading,
            config.get("gps_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.odom_pub = self.create_publisher(
            sensor_driver_msgs.msg.OdometrywithGps,
            config.get("odom_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.velocity_pub = self.create_publisher(
            sensor_driver_msgs.msg.InsVelocity,
            config.get("velocity_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        self.imu_pub = self.create_publisher(
            sensor_msgs.msg.Imu,
            config.get("imu_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self.position_pub = self.create_publisher(
            geometry_msgs.msg.Vector3Stamped,
            config.get("position_pub_topic"),
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        
        #* Timer
        self.timer = self.create_timer(
            config.get("loop_rate"),
            self.publish_vehicle_data
        )
    
    def cleanup(self):
        if hasattr(self, 'timer'):
            self.timer.cancel()
        
        publishers = [
            self.lidar_pub,
            self.gps_pub,
            self.odom_pub,
            self.velocity_pub,
            self.imu_pub,
            self.position_pub
        ]
        
        for pub in publishers:
            pub.destroy()
    
    def publish_vehicle_data(self):
        self.update_vehicle_state_info()
        self.get_lidar_data()
        self.get_gps_data()
        self.get_odom_data()
        self.get_velocity_data()
        self.get_imu_data()
        self.get_position_data()

        self.lidar_pub.publish(self.lidar_data)
        self.gps_pub.publish(self.gps_data)
        self.odom_pub.publish(self.odom_data)
        self.velocity_pub.publish(self.velocity_data)
        self.imu_pub.publish(self.imu_data)
        self.position_pub.publish(self.position_data)
        logging.debug("Data published")
    
    """ Read vehicle state info using carla interface """
    def update_vehicle_state_info(self):
        ego_velocity = self.vehicle.get_velocity()
        ego_angular_velocity = self.vehicle.get_angular_velocity()
        ego_transform = self.vehicle.get_transform()
        ego_location = ego_transform.location
        ego_rotation = ego_transform.rotation

        self.pseudo_heading = ego_rotation        
        self.pseudo_position = carla_data_to_ros.carla_location_to_ros_vector3(ego_location)
        self.pseudo_velocity = carla_data_to_ros.carla_velocity_to_ros_twist(
            ego_velocity,
            ego_angular_velocity
        )

        self.pseudo_odom.header = self.header
        self.pseudo_odom.pose.pose = self.pseudo_position
        self.pseudo_odom.twist.twist = self.pseudo_velocity

    """ Callback functions """
    def update_carla_lidar_data(self, carla_lidar_data: sensor_msgs.msg.PointCloud2):
        self.header = carla_lidar_data.header
        self.header.frame_id = "odom"

        self.carla_lidar_data = carla_lidar_data
        logging.debug("Received lidar data")
            
    def update_carla_gnss_data(self, carla_gnss_data: sensor_msgs.msg.NavSatFix):
        self.carla_gnss_data = carla_gnss_data
        logging.debug("Received gnss data")

    def update_carla_imu_data(self, carla_imu_data: sensor_msgs.msg.Imu):
        self.carla_imu_data = carla_imu_data
        logging.debug("Received imu data") 
   
    """ Prepare data to be published """
    def get_lidar_data(self)->sensor_msgs.msg.PointCloud2:
        self.lidar_data = self.carla_lidar_data
        return self.lidar_data
    
    def get_gps_data(self):
        self.gps_data.header = self.carla_gnss_data.header
        self.gps_data.gps = self.carla_gnss_data

        self.gps_data.roll = self.pseudo_heading.roll
        self.gps_data.pitch = self.pseudo_heading.pitch
        
        #? Do we need to minus 90 degrees?
        self.gps_data.heading = self.pseudo_heading.yaw - 90

    def get_odom_data(self):
        self.odom_data.header = self.header
        self.odom_data.gps = self.carla_gnss_data
        self.odom_data.odometry = self.pseudo_odom

    def get_velocity_data(self):
        self.velocity_data.header = self.header
        self.velocity_data.angular_velocity = self.carla_imu_data.angular_velocity
        self.velocity_data.linear_velocity = self.pseudo_velocity.linear
        self.velocity_data.angular_velocity = self.pseudo_velocity.angular

    def get_imu_data(self):
        self.imu_data = self.carla_imu_data

    def get_position_data(self):
        self.position_data.header = self.header
        self.position_data.vector = self.pseudo_position
    