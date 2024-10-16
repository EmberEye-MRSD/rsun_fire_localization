#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

import matplotlib.pyplot as plt

import time


class Filter:
    def __init__(self):
        self.poses_sub = rospy.Subscriber("/hotspots/array", PoseArray, self.hotspots_cb, queue_size=100)
        self.hotspot_pub = rospy.Publisher("/hotspots/global_map", MarkerArray, queue_size=10)

        self.all_meas = []
        self.odom = []
        self.nn_idxs = []

        self.measurements = [] # list of lists of measurements for each hotspot
        self.global_poses = [] # list of filtered pose for each hotspot
        # self.measurement_weights = [] # list of lists confidence weights of each measurement for each hotspot
        # self.total_weights = [] # list of total weights for each hotspot

        nn_threshold_param = rospy.get_param('/temporal_mapping/nn_threshold', default=2.5)
        clipping_distance_param = rospy.get_param('/temporal_mapping/clipping_distance', default=6.0)
        # outlier_frame_threshold_param = rospy.get_param('/outlier_frame_threshold', default=5)
        hotspot_inflation_radius_param = rospy.get_param('/temporal_mapping/hotspot_inflation_radius', default=0.0)
        distance_weighing_param = rospy.get_param('/temporal_mapping/distance_weighing', default=False)

        print("[INFO] nn_threshold :", nn_threshold_param)
        print("[INFO] clipping_distance :", clipping_distance_param)
        # print("[INFO] outlier_frame_threshold :", outlier_frame_threshold_param)
        print("[INFO] hotspot_inflation_radius :", hotspot_inflation_radius_param)
        print("[INFO] distance_weighing :", distance_weighing_param)

        self.nn_thresh = nn_threshold_param # nn radius in meters
        self.clipping_distance = clipping_distance_param # clipping distance for thermal stereo depth
        # self.outlier_frame_threshold = outlier_frame_threshold_param # number of frames for a hotspot to be considered legit
        self.hotspot_inflation_radius = hotspot_inflation_radius_param # in the worst case, min dist between drone and hotspot
        self.distance_weighing = distance_weighing_param # False by default

        self.height_threshold = 0.0
    
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
            # print(f"Distance to hotspot", hotspot_pose, )
            if dist < min_dist:
                min_dist = dist
                closest_point = hotspot_pose
                idx = i
        
        return closest_point, idx, min_dist

    def get_weight(self, point, curr_pose):
        weight = 1.0 / (self.distance(point, curr_pose) - self.hotspot_inflation_radius)
        return weight

    def add_new_hotspot(self, point, curr_pose):
        """
        input: point reading
        """
        self.measurements.append([point])
        self.all_meas.append(point)
        self.global_poses.append(point)
        self.odom.append(curr_pose)
        # self.nn_idxs.append(len(self.global_poses))

        if(self.distance_weighing):
            weight = self.get_weight(point, curr_pose)
            self.measurement_weights.append([weight])
            self.total_weights.append(weight)

    def update_nn(self, idx, point, curr_pose):
        """
        input: idx of nn, point reading
        """
        self.measurements[idx].append(point)
        self.all_meas.append(point)
        self.odom.append(curr_pose)
        self.nn_idxs.append(idx)
        

        list_readings = np.array(self.measurements[idx])
        xs = list_readings[:, 0]
        ys = list_readings[:, 1]
        zs = list_readings[:, 2]

        updated_reading = None
        if(self.distance_weighing):
            weight = self.get_weight(point, curr_pose)
            self.measurement_weights[idx].append(weight)
            self.total_weights[idx] += weight
            updated_reading = [ np.sum(np.multiply(xs, self.measurement_weights[idx])) / self.total_weights[idx],
                                np.sum(np.multiply(ys, self.measurement_weights[idx])) / self.total_weights[idx],
                                np.sum(np.multiply(zs, self.measurement_weights[idx])) / self.total_weights[idx]]

        else:            
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
        marker.header.frame_id = "odom"
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
        # print(poses_reading[1:])
        # gt = np.array([[49.833, 0, 0], [49.833, -27.583, 0], [24.833, -57.333, 0], [11.083, -41.833, 0]]) # config 1
        gt = np.array([[30.083, -15.75, 0], [38.5, -37.33, 0], [16.917, -37.33, 0], [2.167, -18.75, 0]]) # config 2

        # print("# global poses : ", len(self.global_poses))
        if len(self.global_poses):
            print("Number of global hotspots : ", len(self.global_poses))
        #     # plt.scatter(-np.array(self.global_poses)[:, 1], np.array(self.global_poses)[:, 0], c='g')
            for i in range(len(self.global_poses)):
                print(f"Measurements for hotspot {i} : ", len(self.measurements[i]))
        #         plt.scatter(-np.array(self.measurements[i])[:, 1], np.array(self.measurements[i])[:, 0], c='r')

        #     for i in range(gt.shape[0]):
        #         plt.scatter(-np.array(gt)[:, 1] * 0.3048, np.array(gt)[:, 0] * 0.3048, c='b')
        #     plt.savefig('scatter_temporal.png')

        # print("Readings : ", poses_reading)
        
        curr_pose = [poses_reading[0].position.x,
                    poses_reading[0].position.y,
                    poses_reading[0].position.z]

        for pose in poses_reading[1:]:
            

            # get each new hotspot reading
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            point = [x, y, z]

            if z > self.height_threshold:
                continue

            if self.isFar(curr_pose, point):
                print("Point too far : ", point, curr_pose)
                # print("Threshold : ", self.clipping_distance)
                continue

            # check if any existing hotspot exists nearby
            nn, idx, nn_dist = self.find_closest_hotspot(point)

            # use new reading to update existing hotspot, or add new hotspot
            if (nn_dist < self.nn_thresh) and (idx is not None):
                print("Updating hotspot : ", point)
                self.update_nn(idx, point, curr_pose)
            else:
                print("Adding new hotspot : ", point)
                self.add_new_hotspot(point, curr_pose)
            
        # remove outliers (frame jumps)
        # self.outlier_rejection()

        # publish MarkerArray
        self.publish_updated_hotspots()

    def hotspots_cb(self, msg):
        poses_reading = msg.poses
        self.update_global_poses(poses_reading)

def main():
    rospy.init_node("temporal_mapping")
    filter = Filter()
    rospy.spin()

    # while rospy.is_shutdown():
        
    #     np.save("meas.npy", filter.all_meas)
    #     np.save("hotspots.npy", filter.global_poses)
    #     np.save("odom.npy", filter.odom)
    #     np.save("nn_idxs.npy", filter.nn_idxs)

    #     exit()

if __name__ == "__main__":
    main()