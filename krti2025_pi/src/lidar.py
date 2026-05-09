#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import tf
from pid import PID
import numpy as np

class Lidar360:
    def __init__(self):
        # Initialize parameters
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)  # Safe distance in meters
        self.current_pose = None   # To store the current fused pose
        self.is_safe = True        # Initialize safety status as an instance variable

        # Initialize PID controllers for linear and angular velocities
        self.linear_pid = PID(kp=0.5, ki=0.0, kd=0.1)
        self.angular_pid = PID(kp=1.0, ki=0.0, kd=0.1)

        # Publisher for filtered LIDAR data
        self.filtered_lidar_pub = rospy.Publisher('/filtered_lidar_maintain', LaserScan, queue_size=10)
        rospy.loginfo("Publisher created")

        # Subscribe to the fused pose published by data_fusion_node
        self.pose_sub = rospy.Subscriber('/fused_pose', PoseStamped, self.pose_callback)

        # Subscribe to the raw LIDAR data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.loginfo("Subscribed to /scan")

        # Initialize a TF listener
        self.tf_listener = tf.TransformListener()
        self.filtered_ranges = None


    def lidar_callback(self, data):
        rospy.loginfo("LIDAR callback triggered")
        try:
            if not data.ranges:  # Check if ranges are empty
                rospy.logwarn("No ranges received from LIDAR.")
                return
            
            # Filter out inf and invalid ranges
            filtered_ranges = np.array(data.ranges)
            
            # Define deadzone threshold
            deadzone_threshold = 0.01  # Example threshold for deadzone
            for i in range(len(filtered_ranges)):
                if np.isinf(filtered_ranges[i]):
                    filtered_ranges[i] = 100  # Set out of range values to 100
                elif filtered_ranges[i] <= deadzone_threshold:
                    filtered_ranges[i] = 0  # Set deadzone values to 0
            self.filtered_ranges = filtered_ranges
            # Calculate distances for front, right, left, and back
            front_distance = filtered_ranges[0]  # Assuming front is at index 0
            right_distance = filtered_ranges[int(len(filtered_ranges) * 0.25)]  # 90 degrees to the right
            back_distance = filtered_ranges[int(len(filtered_ranges) * 0.5)]  # 180 degrees (back)
            left_distance = filtered_ranges[int(len(filtered_ranges) * 0.75)]  # 90 degrees to the left

            # Log the distances
            rospy.loginfo(f"Distances - Front: {front_distance}, Right: {right_distance}, Back: {back_distance}, Left: {left_distance}")

            # Calculate the distance errors
            front_error = self.safe_distance - front_distance
            right_error = self.safe_distance - right_distance
            left_error = self.safe_distance - left_distance
            
            #obstacle avoidance logic, import to main.py after if_safe check (ln 159-178)

            #head = self.drone.get_home_heading()
            #velx = self.vel_msg.linear.x
            #vely = self.vel_msg.linear.y
            #velz = 0
            
            #if front_distance <= self.safe_distance or right_distance <= self.safe_distance or left_distance <= self.safe_distance:
            #    rospy.logwarn("Tight space detected! Hovering to maintain position.")
            #    self.vel_msg.linear.x = 0  # Stop forward movement
            #    self.vel_msg.linear.y = 0  # Stop rotation
            #    self.velocity_publisher.publish(self.vel_msg)
            #    return

            # Determine the movement direction based on the errors
            #if front_distance < self.safe_distance:
            #    rospy.loginfo("Di bawah 1 meter")
                # If too close in front, adjust to the left or right
                #if right_distance > left_distance:
            #    rospy.loginfo("Geser ke kiri")
            #    self.vel_msg.linear.y = -0.5  # Turn left
            #    self.drone.move_vel(velx, vely, velz, head)
            #else:
            #    self.vel_msg.linear.y = 0.5  # Turn right
            #    rospy.loginfo("Geser ke kanan")
            #    self.drone.move_vel(velx, vely, velz, head)
            #rospy.loginfo("Keluar fungsi geser, dan tidak maju")
            #    self.vel_msg.linear.x = 0  # Do not move forward
            #    self.drone.move_vel(velx, vely, velz, head)
            #else:
                # If not too close in front, maintain position
                #self.vel_msg.linear.x = 0  # Maintain position
                #rospy.loginfo("Menjaga posisi")
                #self.drone.move_vel(velx, vely, velz, head)

                # Adjust lateral movement to maintain safe distance
            #if right_distance < self.safe_distance:
            #    self.vel_msg.linear.y = -0.5  # Turn left to create space
            #    rospy.loginfo("Geser ke kiri membuat angkasa")
            #    self.drone.move_vel(velx, vely, velz, head)
            #elif left_distance < self.safe_distance:
            #    self.vel_msg.linear.y = 0.5  # Turn right to create space
            #    rospy.loginfo("Geser ke kanan membuat angkasa")
            #    self.drone.move_vel(velx, vely, velz, head)
            #else:
            #    self.vel_msg.linear.y = 0  # No rotation needed
            #    rospy.loginfo("GA ROTASI")
            #    self.drone.move_vel(velx, vely, velz, head)

            # Publish the velocity command
            self.velocity_publisher.publish(self.vel_msg)

            # Create a new LaserScan message for filtered data
            filtered_scan = LaserScan()
            filtered_scan.header = data.header
            filtered_scan.angle_min = data.angle_min
            filtered_scan.angle_max = data.angle_max
            filtered_scan.angle_increment = data.angle_increment
            filtered_scan.time_increment = data.time_increment
            filtered_scan.scan_time = data.scan_time
            filtered_scan.range_min = data.range_min
            filtered_scan.range_max = data.range_max
            filtered_scan.ranges = filtered_ranges.tolist()
            filtered_scan.intensities = data.intensities

            # Publish the filtered LIDAR data
            self.filtered_lidar_pub.publish(filtered_scan)
            rospy.loginfo("Filtered LIDAR data published.")

            # Calculate the minimum distance and its index
            min_distance = np.nanmin(filtered_ranges) # except 0 values (deadzones)
            min_index = np.nanargmin(filtered_ranges)  # Using the filtered data to get the correct index

            # Log the detection status
            rospy.loginfo(f"Minimum distance detected: {min_distance} meters at index {min_index}")
        
            # Check if the detected point is at or below the safe distance
            if min_distance <= self.safe_distance:
                # Check if the detected obstacle is the GPS tower
                if self.is_gps_tower(min_distance):
                    rospy.loginfo("Ignoring GPS tower obstruction.")
                    return  # Exit the callback to avoid further processing

                return  # Exit the callback to avoid further processing

            # Calculate the error for PID control
            distance_error = self.safe_distance - min_distance
            angle_error = self.avoidance_angle(filtered_ranges, data.angle_increment, min_index)

            # Use PID to calculate the velocity commands
            #self.vel_msg.linear.x = self.linear_pid.update(distance_error)
            #self.vel_msg.linear.y = self.angular_pid.update(distance_error)
        
            # Initialize variables to track safety status and closest obstacle
            distances = [front_distance, right_distance, back_distance, left_distance]
            min_distance = min(distances)  # Find the minimum distance
            closest_index = distances.index(min_distance)  # Get the index of the closest obstacle

            # Check if any distance is below the safe distance
            if min_distance <= self.safe_distance:
                rospy.logwarn(f"Obstacle detected at index {closest_index} with distance {min_distance}!")
                # self.stop_drone(min_distance)  # Stop the drone if an obstacle is detected
                self.is_safe = False  # Set safety status to False
            else:
                rospy.loginfo("No obstacles detected within safe distance.")
                self.is_safe = True  # Set safety status to True

            # Broadcast the safety status and the index of the closest obstacle
            rospy.loginfo(f"Safety Status: {'Safe' if self.is_safe else 'Not Safe'}, Closest Obstacle Index: {closest_index}")

            return self.is_safe, closest_index  # Return the safety status and the index of the closest obstacle
        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")

    def get_distance(self, angle:float = -1.0, index:int=-1) -> float:
        """
            params: 
                angle:  float = angle of measured distance to be returned
                index: int = index of measured distance to be returned
            return:
                distance measured of desired angle/index : float
        """
        
        
        if self.filtered_ranges == None:
            rospy.logwarn("No ranges received from LIDAR.")
            return -1.0
            
        # WIP
        # dist 
        return dist
        
    def avoidance_angle(self, ranges, angle_increment, min_index):
        angle = (min_index - len(ranges) / 2) * angle_increment  # Calculate the avoidance angle
        return angle

    def is_gps_tower(self, min_distance):
        # Check if the detected obstacle is the GPS tower based on distance
        return min_distance < 0.3  # Assuming the GPS tower is detected if within 0.3 meters
if __name__ == '__main__':
    try:
        Lidar360()
    except rospy.ROSInterruptException:
        rospy.loginfo("Obstacle Avoidance Node terminated.")