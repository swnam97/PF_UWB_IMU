#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
# from uwb_imu.msg import UwbMsg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from nav_msgs.msg import Path
import tf2_ros
# import nlink_se3 
# import esekf
class Plotter:
    def __init__(self):
        print("test 0")
        self.x_uwb = []
        self.y_uwb = []
        self.z_uwb = []
        self.x_pf = []
        self.y_pf = []
        self.z_pf = []
        self.ukf_esekf_x_uwb = []
        self.ukf_esekf_y_uwb = []
        self.ukf_esekf_z_uwb = []
        self.q_w = []
        self.q_x = []
        self.q_y = []
        self.q_z = []
        self.gt_x = []
        self.gt_y = []
        self.gt_index = 0
        self.gt_z = []
        self.x_gt = []
        self.y_gt = []
        self.z_gt = []
        self.esekf_uwb_cnt = 0
        self.ukf_uwb_cnt = 0
        self.imu_cnt = 0
        self.cnt = 0
        self.rotation = []
        self.path_1 = Path()
        self.path_1.header.frame_id = "map"
        self.rosbag_time = 100
        self.path_2 = Path()
        self.path_2.header.frame_id = "map"
        
        self.path_3 = Path()
        self.path_3.header.frame_id = "map"

        self.path_gt = Path()
        self.path_gt.header.frame_id = "map"

        # print("test 1")
        rospy.init_node("plotter")
        # print("test 2")
        self.br = tf2_ros.TransformBroadcaster()
        # print("test 3")
        self.pub_gt = rospy.Publisher('/gt_path', Path, queue_size=10)
        # print("test 4")
        # self.pub_1 = rospy.Publisher('/uwb', Path, queue_size=10)
        self.pub_2 = rospy.Publisher('/pf', Path, queue_size=10)
        # print("test 5")
        # self.pub_3 = rospy.Publisher('/esekf', Path, queue_size=10)
        self.read_gt_file()
        rospy.Timer(rospy.Duration(99.719/1000), self.publish_next_gt_point)
        # rospy.Subscriber("/result_uwb", PoseStamped, self.uwb_callback)
        rospy.Subscriber("/estimated_path", Path, self.pf_callback)
        # rospy.Subscriber("/result_esekf", PoseStamped, self.ukf_uwb_esekf_callback)

    def publish_next_gt_point(self, event):
        if self.gt_index < len(self.gt_x):
            self.x_gt.append(self.gt_x[self.gt_index])
            self.y_gt.append(self.gt_y[self.gt_index])
            self.z_gt.append(self.gt_z[self.gt_index])
            self.gt_index += 1

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = self.x_gt
            pose_stamped.pose.position.y = self.y_gt
            pose_stamped.pose.position.z = self.z_gt

            self.path_gt.poses.append(pose_stamped)
            self.path_gt.header.stamp = rospy.Time.now()
            self.pub_gt.publish(self.path_gt)
            # print("ss")

    def rotation_matrix_to_quaternion(self, R):
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2  
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2 
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return np.array([qw, qx, qy, qz])
    
    def read_gt_file(self):
        # theta = -28*np.pi/180
        # gt_file_path = "/home/gihun/pf_ws/src/pf_uwb_imu/data/square_gt.txt"  
        gt_file_path = "/home/gihun/pf_ws/src/pf_uwb_imu/data/spiral_gt.txt"  
        with open(gt_file_path, 'r') as file:
            next(file)
            for line in file:
                data = line.strip().split("\t")
                # print(data)
                _, _, a,b,c,_,_,_,_,d,e,f,g,h,i,j,k,l = map(float, data[0:18])
                # print("a value: ", a)
                self.gt_x.append(a*0.001+4.55)
                self.gt_y.append(b*0.001+4)
                self.gt_z.append(c*0.001)
                rot = np.array([[d,e,f],
                                [g,h,i],
                                [j,k,l]]) 
                q = self.rotation_matrix_to_quaternion(rot)
                self.q_w.append(q[0])
                self.q_x.append(q[1])
                self.q_y.append(q[2])
                self.q_z.append(q[3])

                

    def uwb_callback(self, msg):
        # self.esekf_uwb_cnt += 1
        # if self.esekf_uwb_cnt >= 3:
        self.x_uwb.append(msg.pose.position.x)
        self.y_uwb.append(msg.pose.position.y)
        self.z_uwb.append(msg.pose.position.z+0.2)
            # self.esekf_uwb_cnt = 0

        self.publish_transform_and_path(msg, self.path_1, self.pub_1)

    def pf_callback(self, msg):
        # self.ukf_uwb_cnt += 1
        # if self.ukf_uwb_cnt >= 3:
        for pose_stamped in msg.poses:
            self.x_pf.append(pose_stamped.pose.position.x)
            self.y_pf.append(pose_stamped.pose.position.y)
            self.z_pf.append(pose_stamped.pose.position.z)
            # self.ukf_uwb_cnt = 0

        self.publish_transform_and_path(msg, self.path_2, self.pub_2)

    # def ukf_uwb_esekf_callback(self, msg):
    #     # self.ukf_uwb_cnt += 1
    #     # if self.ukf_uwb_cnt >= 3:
    #     self.ukf_esekf_x_uwb.append(msg.pose.position.x)
    #     self.ukf_esekf_y_uwb.append(msg.pose.position.y)
    #     self.ukf_esekf_z_uwb.append(msg.pose.position.z)
    #         # self.ukf_uwb_cnt = 0

    #     self.publish_transform_and_path(msg, self.path_3, self.pub_3)

    def publish_transform_and_path(self, msg, path, pub):
        for pose_stamped in msg.poses:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = pose_stamped.header.frame_id

            t.transform.translation.x = pose_stamped.pose.position.x
            t.transform.translation.y = pose_stamped.pose.position.y
            t.transform.translation.z = pose_stamped.pose.position.z
            t.transform.rotation = pose_stamped.pose.orientation

            self.br.sendTransform(t)

            new_pose_stamped = PoseStamped()
            new_pose_stamped.header = pose_stamped.header
            new_pose_stamped.pose = pose_stamped.pose

            path.poses.append(new_pose_stamped)
        path.header.stamp = rospy.Time.now()
        pub.publish(path)

    def plot_data(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        while not rospy.is_shutdown():
            ax.clear()
            # self.cnt += 1
            # ax.scatter(self.x_uwb, self.y_uwb, self.z_uwb, c='k', label='UWB Position', s=1)
            ax.scatter(self.x_gt, self.y_gt, self.z_gt, c='k', label='Ground Truth', s=1)
            ax.scatter(self.x_pf, self.y_pf, self.z_pf, c='r', label='Particle Filter', s=1)
            # ax.scatter(self.y_pf, self.ukf_esekf_y_uwb, self.ukf_esekf_z_uwb, c='b', label='ESEKF', s=1)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('Real-Time 3D Plot')
            ax.legend()

            plt.pause(0.1)
            rospy.sleep(0.1)

if __name__ == "__main__":
    plotter = Plotter()
    # esekf.ESEKF()
    # nlink_se3.SE3EKF()

    plotter.plot_data()
    rospy.spin()
