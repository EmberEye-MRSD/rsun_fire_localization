#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np

import time

class Filter:
    def __init__(self):
        self.poses_sub = rospy.Subscriber("/rsun/hotspot/poses", PoseArray, self.hotspots_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber("/rsun/odometry", Odometry, self.odom_cb, queue_size=1)
        self.hotspot_pub = rospy.Publisher("/hotspots/global_map", MarkerArray, queue_size=10)

        self.all_meas = []
        self.odom = []

        self.measurements = [] # list of lists of measurements for each hotspot
        self.global_poses = [] # list of filtered pose for each hotspot

        nn_threshold_param = rospy.get_param('/temporal_mapping/nn_threshold', default=2.5)
        clipping_distance_param = rospy.get_param('/temporal_mapping/clipping_distance', default=6.0)
        # outlier_frame_threshold_param = rospy.get_param('/outlier_frame_threshold', default=5)
        hotspot_inflation_radius_param = rospy.get_param('/temporal_mapping/hotspot_inflation_radius', default=0.0)

        print("[INFO] nn_threshold :", nn_threshold_param)
        print("[INFO] clipping_distance :", clipping_distance_param)
        # #print("[INFO] outlier_frame_threshold :", outlier_frame_threshold_param)
        print("[INFO] hotspot_inflation_radius :", hotspot_inflation_radius_param)

        self.nn_thresh = nn_threshold_param # nn radius in meters
        self.clipping_distance = clipping_distance_param # clipping distance for thermal stereo depth
        # self.outlier_frame_threshold = outlier_frame_threshold_param # number of frames for a hotspot to be considered legit
        self.hotspot_inflation_radius = hotspot_inflation_radius_param # in the worst case, min dist between drone and hotspot

        self.height_threshold = 0.0

        self.T_map_imu_init_inv, self.T_map_imu = None, None
        self.T_odom, self.R_odom = None, None
        self.T_camera_thermal = np.array([ [y,  0.048],
                                [-0.0174524, 0.9998477,  0.0000000, -0.039],
                                [0.0087252,  0.0001523,  0.9999619, -0.010],
                                [0,          0,          0,          1]])
        self.T_imu_camera_rot = np.array([  [0, 0, 1, 0],
                                        [-1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 0, 1]])
        # self.T_imu_camera = np.array([  [0.99995186,  -0.00374387, 0.00906943, -0.03028595],
        #                                 [0.00370598,  0.99998435 , 0.00419059, 0.00362705],
        #                                 [-0.00908498,  -0.00415678, 0.99995009, 0.01719125],
        #                                 [0, 0, 0, 1.0]])
        self.T_imu_camera = np.eye(4)
        self.T_imu_thermal = self.T_imu_camera_rot @ self.T_imu_camera @ self.T_camera_thermal


        self.odom_ts = None
        self.hotspot_ts = None
    
    def distance(self, p1, p2):
        # dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)
        dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dist

    def isFar(self, drone_odom, pose):
        if self.distance(drone_odom, pose) > self.clipping_distance:
            return True
        return False

    def find_closest_hotspot(self, point):
        """
        input: query point
        processing: uses global list of hotspot locations
        output: nearest neighbour, idx of nn, distance (default None, None, inf)
        """
        closest_point = None
        min_dist = np.inf
        idx = None

        if len(self.global_poses) == 0:
            return None, None, np.inf

        for i, hotspot_pose in enumerate(self.global_poses):
            dist = self.distance(point, hotspot_pose)
            if dist < min_dist:
                min_dist = dist
                closest_point = hotspot_pose
                idx = i
        
        return closest_point, idx, min_dist

    def add_new_hotspot(self, point, curr_pose):
        """
        input: point reading
        """
        self.measurements.append([point])
        self.all_meas.append(point)
        self.global_poses.append(point)
        self.odom.append(curr_pose)

    def update_nn(self, idx, point, curr_pose):
        """
        input: idx of nn, point reading
        """
        self.measurements[idx].append(point)
        self.all_meas.append(point)
        self.odom.append(curr_pose)   

        list_readings = np.array(self.measurements[idx])
        xs = list_readings[:, 0]
        ys = list_readings[:, 1]
        zs = list_readings[:, 2]

        updated_reading = [np.mean(xs), np.mean(ys), np.mean(zs)]

        self.global_poses[idx] = updated_reading
    
    def get_marker(self, point, i):
        marker = Marker()
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # stddev = np.std(np.array(self.measurements[i]))
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "predicted_hotspots"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        return marker
        
    def publish_updated_hotspots(self):
        marker_array = MarkerArray()
        for i, point in enumerate(self.global_poses):

            # create new marker for given hotspot
            marker = self.get_marker(point, i)

            marker_array.markers.append(marker)
        
        self.hotspot_pub.publish(marker_array)
    
    def outlier_rejection(self):
        for i in range(len(self.measurements)):
            if len(self.measurements[i]) < self.outlier_frame_threshold:
                self.global_poses.pop(i)

    def update_global_poses(self, poses_reading):
        # print("--------------- Starting new frame ---------------")
        for pose in poses_reading.poses:
            # get each new hotspot reading
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            point = [x, y, z]
            curr_pose = [self.T_map_imu[0][3], self.T_map_imu[1][3], self.T_map_imu[2][3]]

            #print("Total hotspots: ", len(self.measurements))

            # if len(self.global_poses) == 0:
                # #print("[INFO] Initializing NN tree")
                # self.global_poses.append(point)
                # continue
            # if z > self.height_threshold:
                # continue

            if self.isFar(curr_pose, point):
                continue

            # check if any existing hotspot exists nearby
            nn, idx, nn_dist = self.find_closest_hotspot(point)
            
            # use new reading to update existing hotspot, or add new hotspot
            if (nn_dist < self.nn_thresh) and (idx is not None):
                #print("Updating hotspot : ", point)
                self.update_nn(idx, point, curr_pose)
            else:
                #print("Adding new hotspot : ", point)
                self.add_new_hotspot(point, curr_pose)
            
        # remove outliers (frame jumps)
        # self.outlier_rejection()

        # publish MarkerArray
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.savefig('/home/phoenix/ros_ws/src/rsun_fire_localization/plots/scatter.png')
        self.publish_updated_hotspots()

    def hotspots_cb(self, msg):
        self.hotspot_ts = float(msg.header.stamp.to_sec())
        self.poses_reading = msg.poses
    

    def odom_cb(self, msg):
        self.odom_ts = float(msg.header.stamp.to_sec())
        # transform hotspot to map frame
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.T_odom = np.array([x, y, z]).reshape((3, 1))

        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]
        self.R_odom = R.from_quat(quaternion).as_matrix()
        
    def get_pose_in_map(self, pose):
        T_thermal_hotspot = np.array([pose.x, pose.y, pose.z, 1])
        
        # T_thermal_hotspot = np.array([0, 0, 1, 1])
        # IMU in Map
        # print(self.T_map_imu)
        plt.scatter([self.T_map_imu[0][3]], [self.T_map_imu[1][3]], color="red")
        # Thermal in Map
        plt.scatter([(self.T_map_imu @ self.T_imu_thermal)[0][3]], [(self.T_map_imu @ self.T_imu_thermal)[1][3]], color="blue")
        # Hotspot in Map
        plt.scatter([(self.T_map_imu @ self.T_imu_thermal @ T_thermal_hotspot.reshape((4,1)))[0]], 
                    [(self.T_map_imu @ self.T_imu_thermal @ T_thermal_hotspot.reshape((4,1)))[1]], color="green")

        return self.T_map_imu @ self.T_imu_thermal @ T_thermal_hotspot.reshape((4,1))
    
    
    def run(self, event=None):
        if self.poses_reading is None:
            return
        if self.T_odom is None or self.R_odom is None or len(self.poses_reading) == 0:
            return
        
        if abs(self.odom_ts - self.hotspot_ts) > 0.2:
            print(f"[WARN][RUN()] High Time diff :{self.odom_ts - self.hotspot_ts}")
            return

        RT = np.hstack((self.R_odom, self.T_odom))
        if self.T_map_imu is None:
            self.T_map_imu_init_inv = np.linalg.inv(self.R_odom)
        
        self.T_map_imu = np.vstack((RT, np.array([0, 0, 0, 1])))
        self.T_map_imu[:3, :3] = self.T_map_imu_init_inv @ self.T_map_imu[:3, :3]
        
        poses_reading_map_frame = PoseArray()
        for hotspot in self.poses_reading:
            print("--- Calculating Hotspot Pose ------")
            hotspot_global = self.get_pose_in_map(hotspot.position).flatten()
            p = Pose()
            p.position.x, p.position.y, p.position.z = hotspot_global[0], hotspot_global[1], hotspot_global[2]
            poses_reading_map_frame.poses.append(p)
        self.update_global_poses(poses_reading_map_frame)

        # self.T_odom = self.R_odom = None
        # self.poses_reading = []

def main():
    rospy.init_node("temporal_mapping")
    filter = Filter()

    rospy.Timer(rospy.Duration(1.0/30.0), filter.run)
    rospy.spin()

if __name__ == "__main__":
    main()