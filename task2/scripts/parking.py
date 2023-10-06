#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import webcolors

class The_Ring:
    def __init__(self):
        rospy.init_node('ring_floor', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        # self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/arm_camera/rgb/image_raw", Image, self.image_callback)
        self.status_sub = rospy.Subscriber("/circle_goal/status", Int8, self.set_completed_status)

        self.curr_location = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.curr_location_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.markers_pub = rospy.Publisher('/ground_circle', Marker, queue_size=10)
        self.approach_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=20)
        # self.twist_approach_pub = rospy.Publisher('/circle_goal', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.goal_status_pub = rospy.Publisher('/circle_goal/status', Int8, queue_size=10)


        self.pending_marker_list = []
        self.pending_position_list = []
        self.no_detection = 0
        self.global_goal = PoseStamped()
        self.completed_status = True
        self.detection_mode = True

        self.x = 0
        self.y = 0
        self.z = 0
        self.z_ang = 0
        self.w_ang = 0
        self.quart = [0, 0, 0, 0]

        self.global_pose_ = PoseStamped()
        self.inital = True
        self.sign = 0

        self.angular_const = 0.1
        self.linear_const = 0.03

    def overwrite_location(self):
        """
        Overwrite the current location with the global pose from get_robot_pose.
         * Placeholder to see if this works better than /amcl_pose
        """
        self.x = self.global_pose_.translation.x
        self.y = self.global_pose_.translation.y
        self.z_ang = self.sign * (self.global_pose_.rotation.z)
        self.w_ang = self.sign * (self.global_pose_.rotation.w)
        print("Inside Overwrite: x: " + str(self.x) + " y: " + str(self.y) + " z_ang: " + str(self.z_ang) + " w_ang: " + str(self.w_ang))


    def get_robot_pose(self):
        """
        Get the pose of the robot in the global frame.
        Set the global pose to represent the pose of the robot in the global frame.
        Format of the global pose is PoseStamped:
        global_pose.translation is our position in the global frame.
        global_pose.rotation is our orientation in the global frame in quarterion format.
        """
        transformed= self.tf_buf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(10.0))
        # print(transformed)
        self.global_pose_ = transformed.transform
        if self.inital:
            self.inital = False
            ## here we have a situation where in amcl rotation w cannot be negative
            ## but in our transformation it can. If W is negative, we know we  have to flip the sign
            ## this is because of Q representing a rotation matrix, and -Q representing the absolute same rotation
            ## quarterion representation of the angle: [x, y, z, w]
            if self.global_pose_.rotation.w >= 0:
                self.sign = 1
            else:
                self.sign = -1
        # print(self.global_pose_)
        self.overwrite_location()


    def face_goal(self, msg):
        print("in goal_callback")
        print("msg: " + str(msg))
        # First deal with orientation
        self.get_robot_pose()
        euc_angle = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print("euc_angle", euc_angle)
        robot_angle = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
        print("robot_angle", robot_angle)

        bool_flag = True
        # Twist turn until robot is facing about the correct position,
        while (bool_flag):
            robot_angle = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
            twist = Twist()
            if(robot_angle[2] > euc_angle[2]):
                twist.angular.z = -self.angular_const
            else:
                twist.angular.z = self.angular_const
            self.twist_pub.publish(twist)
            bool_flag = abs(self.z_ang - msg.pose.orientation.z) > 0.01 or abs(self.w_ang - msg.pose.orientation.w) > 0.01 or self.z_ang * msg.pose.orientation.z < 0
            self.get_robot_pose()
            

        print("done with orientation")
        print(abs(self.z_ang - msg.pose.orientation.z) > 0.04 and abs(self.w_ang - msg.pose.orientation.w) > 0.04 and self.z_ang * msg.pose.orientation.z < 0)
        print("current angle: z:" + str(self.z_ang) + " w: " + str(self.w_ang))
        print("goal angle: z:" + str(msg.pose.orientation.z) + " w: " + str(msg.pose.orientation.w))
        # Then issue movement forward, with x velocity being a modest 0.1.
        self.completed_status = True

        # while (abs(self.x - msg.pose.position.x) > 0.11 or abs(self.y - msg.pose.position.y) > 0.11):
        #     twist = Twist()
        #     twist.linear.x = self.linear_const
        #     self.twist_pub.publish(twist)
        #     print("current position: x:" + str(self.x) + " y: " + str(self.y))
        #     print("goal position: x:" + str(msg.pose.position.x) + " y: " + str(msg.pose.position.y))
        #     self.get_robot_pose()

    
    # def go_ahead(self):


    def marker_maker(self, pose, point_world, rgba_list=[255,0,0,1]) :
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = point_world.header.stamp
        marker.header.frame_id = point_world.header.frame_id
        marker.pose = pose
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.id = self.marker_num
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = ColorRGBA(rgba_list[0],rgba_list[1],rgba_list[2],rgba_list[3])

        return marker      


    def curr_location_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.z_ang = msg.pose.pose.orientation.z
        self.w_ang = msg.pose.pose.orientation.w
        self.quart = [0, 0, self.z_ang, self.w_ang]


    def bgr_2_rgb(self, color_vect):
        return cv2.cvtColor(np.uint8([[color_vect]]), cv2.COLOR_BGR2RGB)[0][0]
    
            
       
    def get_pose(self, e, dist):
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_frame"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        try:
        # Get the point in the "map" coordinate system
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z

        except Exception as e:
            print(e)
            pose = None

        return pose, point_world


    def append_to_pending(self, loc_vector):
        # if pending list empty simply add:
        if len(self.pending_marker_list) == 0:
            self.pending_marker_list.append(loc_vector)
            self.pending_position_list.append([self.x, self.y])
            return True
        else:
            # check if the marker is close enough to current pending list
            distance_vect = np.linalg.norm(np.array(self.pending_marker_list) - loc_vector, axis=1)
            # print(distance_vect)
            if(distance_vect < 0.04).all(): ## 0.35, 0.2, 0.15 0.05 works
                # if the distance between all components is sufficiently small, add our new marker to the list
                self.pending_marker_list.append(loc_vector)
                self.pending_position_list.append([self.x, self.y])
                return True
            else:
                # distance between all components is too large, reset our pending list, and return false
                self.pending_marker_list.clear()
                self.pending_position_list.clear()
                return False
            

    def create_approach_goal(self, marker, z_snap=0, w_snap=0):
        new_goal = PoseStamped()
        new_goal.header.stamp = rospy.Time.now();
        new_goal.header.frame_id = 'map'
        new_goal.pose.position.x = (marker.pose.position.x)
        new_goal.pose.position.y = (marker.pose.position.y)
        # new_goal.pose.position.x = self.x
        # new_goal.pose.position.y = self.y
        mean_pos = np.mean(self.pending_position_list, axis=0)

        k_f = 525 # kinect focal length in pixels
        # yaw = np.arctan2(marker.pose.position.x - self.x, k_f) # , k_f)
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        # I WANT: z: -0.9999345544715936
        # w: 0.011440575758919987

        # q1_inv = tf.transformations.quaternion_inverse(self.quart)
        # new_q = tf.transformations.quaternion_multiply(quaternion, q1_inv)
        # new_goal.pose.orientation.z = new_q[2]
        # new_goal.pose.orientation.w = new_q[3]
        current_pos = np.array([self.x, self.y, 0])
        # current_orientation = np.array([0, 0, self.z_ang, self.w_ang])
        circle_pos = np.array([marker.pose.position.x, marker.pose.position.y, 0])

        direction_vector = circle_pos - current_pos
        direction_vector /= np.linalg.norm(direction_vector)
        # q_desired = tf.transformations.quaternion_from_matrix(tf.transformations.rotation_matrix(direction_vector, [0, 0, 1]))
        # q_error = tf.transformations.quaternion_multiply(q_desired, tf.transformations.quaternion_inverse(current_orientation))
        yaw = np.arctan2(direction_vector[1], direction_vector[0])
        desired_orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        new_goal.pose.orientation.z = desired_orientation[2]
        new_goal.pose.orientation.w = desired_orientation[3]

        """BACKUP 2"""
        # angle_to_desired_position = np.arctan2(marker.pose.position.y - mean_pos[1], marker.pose.position.x - mean_pos[0])
        # desired_orientation = tf.transformations.quaternion_from_euler(0, 0, angle_to_desired_position)
        # quart = Quaternion(desired_orientation[0], desired_orientation[1], desired_orientation[2], desired_orientation[3])
        # new_goal.pose.orientation = quart
        return new_goal

    """BACKUP"""
    # def create_approach_goal(self, marker, z_snap=0,  w_snap=0):
    #     new_goal = PoseStamped()
    #     new_goal.header.stamp = rospy.Time.now();
    #     new_goal.header.frame_id = 'map'
    #     new_goal.pose.position.x = self.x#marker.pose.position.x
    #     new_goal.pose.position.y = self.y#marker.pose.position.y
    #     k_f = 525
    #     yaw = np.arctan2(new_goal.pose.position.x-self.x, new_goal.pose.position.y-self.y) # , k_f)
    #     quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #     # I WANT:   z: -0.9999345544715936
    #     #           w: 0.011440575758919987

    #     q1_inv = tf.transformations.quaternion_inverse(self.quart)
    #     new_q = tf.transformations.quaternion_multiply(quaternion, q1_inv)
    #     # quaternion[2] += self.z_ang
    #     # quaternion[3] += self.w_ang
    #     # quaternion.normalize()
    #     new_goal.pose.orientation.z = quaternion[2]
    #     new_goal.pose.orientation.w = quaternion[3]
    #     return new_goal


    def create_approach_experimental(self, curr_position):
        new_goal = PoseStamped()
        new_goal.header.stamp = rospy.Time.now();
        new_goal.header.frame_id = 'map'
        new_goal.pose.position.x = curr_position[0] # current theoretical position of circle
        new_goal.pose.position.y = curr_position[1]
        new_goal.pose.position.z = curr_position[2]

        robot_pos = np.array([self.x, self.y, 0])

        direction_vector = curr_position - robot_pos
        direction_vector /= np.linalg.norm(direction_vector)
        yaw = np.arctan2(direction_vector[1], direction_vector[0])
        desired_orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        new_goal.pose.orientation.z = desired_orientation[2]
        new_goal.pose.orientation.w = desired_orientation[3]
        return new_goal


    def set_completed_status(self, msg):
        print(f"Completed status received: {msg.data}")
        self.pending_marker_list.clear()
        self.completed_status = True if msg.data == 1 else False


    def image_callback(self,data):
        # print('I got a new image!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        # print(f"cv_image_shape: {cv_image.shape}")
        cv_image = cv_image[0:-40, 0:640]
        self.dims = cv_image.shape
        alpha = 1.8  # Contrast control
        beta = -100 # Brightness control
        adjusted = cv2.convertScaleAbs(cv_image, alpha=alpha, beta=beta)


        # Tranform image to gayscale
        gray = cv2.cvtColor(adjusted, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        # ret, thresh = cv2.threshold(img, 30, 255, cv2.THRESH_BINARY)
        # ret, thresh = cv2.threshold(img, 5, 200, cv2.THRESH_BINARY)
        # ret, thresh = cv2.threshold(img, 170, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)  #default: 15,25
        # thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 35, 25)  #default: 15,25

        cv2.imshow("thresh", thresh)
        cv2.waitKey(1)

        # Extract contours
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_L1)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)

        # print(elps[0])
        
        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            e1 = elps[n]
            # if e1[0][0] < 200: continue
            for m in range(n + 1, len(elps)):
                e2 = elps[m]
                # if e2[0][0] < 200: continue

                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                e1_shape = np.array(e1[1])
                e2_shape = np.array(e2[1])

                e1_aspect_ratio = e1_shape.max() / (e1_shape.min()+1)
                e2_aspect_ratio = e2_shape.max() / (e2_shape.min()+1)

                aspect_ratio_diff = abs(e1_aspect_ratio - e2_aspect_ratio)


                # print("aspect ratio e1: ", e1_aspect_ratio, "aspect ratio e2: ", e2_aspect_ratio, "aspect ratio diff: ", aspect_ratio_diff)
                #             print dist
                if dist < 5:  # and aspect_ratio_diff < 0.3:  # e1_aspect_ratio < 1.4 and e2_aspect_ratio < 1.4
                    candidates.append((e1,e2))

        # print("Processing is done! found", len(candidates), "candidates for rings")

        try:
            depth_img = rospy.wait_for_message('/arm_camera/depth/image_raw', Image)   
        except Exception as e:
            print(e)

        # test1, test2 = candidates[0]
        # print(test1)
        # Extract the depth from the depth image
        ring_flag = False
        for c in candidates:
            # print(c[0])

            # the centers of the ellipses
            e1, e2 = c
            # print(e1, e2)
            el1 = np.array(e1[1])
            el2 = np.array(e2[1])
            e1_aspect_ratio = el1.max() / el1.min()
            e2_aspect_ratio = el2.max() / el2.min()

            aspect_ratio_diff = abs(e1_aspect_ratio - e2_aspect_ratio)

            # print("aspect ratio e1: ", e1_aspect_ratio, "aspect ratio e2: ", e2_aspect_ratio, "aspect ratio diff: ", aspect_ratio_diff)

            # if (cv_image[0][1] == )

            # drawing the ellipses on the image
            # delay until you are certain it's a good detection
            cv2.ellipse(cv_image, e1, (0, 0, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 0, 0), 2)

            size = (e1[1][0] + e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")

            # if center distance is presumed to be 0, or some variation of nan
            # the ring has a hole, therefore it's indeed a 3d ring.
            center_idx = np.rint(center).astype(int)
            # print("center dist: ", depth_image[center_idx[0], center_idx[1]])

            # another guarantee:
            # print("Center pixel bgr value: ", cv_image[center_idx[0], center_idx[1]])
            # color_vect =  cv_image[center_idx[0], center_idx[1]]


            # masked_depth = np.ma.masked_equal(depth_image[x_min:x_max,y_min:y_max], 0)
            """
             * The depth image is a single-channel float32 image, in meters.
             * it's a diagonal value 95% of the time, so subtract the y component:
             ~ 0.63²
            """
            ring_dist= float(np.nanmean(depth_image[x_min:x_max,y_min:y_max]))
            # print(f"Distance to ring : {ring_dist}")
            cv2.imshow("Extrapolated", cv_image[x_min:x_max,y_min:y_max])
            cv2.waitKey(1)
            
            true_ring = True # 255+255+255 = 765
            """
            - Only after the if statement are we certain that it's a proper detection,
            - finally we can start manipulating elements as intended.
            """
            if true_ring and ring_dist < 3.0:  # ring_dist defines how far away the acceptable ring can be detected
                ring_flag = True
                copy = cv_image.copy()
                cv2.ellipse(copy, e1, (0, 255, 0), 2)
                cv2.ellipse(copy, e2, (0, 255, 0), 2)
                # color_vect =  cv_image[center_idx[0], center_idx[1]]


                # cv2.imshow("Circle excerpt", image_excerpt)
                # cv2.waitKey(0)
                pose_temp, point_world = self.get_pose(e1, ring_dist) ##/2.3)

                # firstly, if the pending_marker_list is empty, we can just add the marker to the list
                curr_pose_vector = np.array([pose_temp.position.x, pose_temp.position.y, pose_temp.position.z])
                
                # print(self.append_to_pending(curr_pose_vector)) ## true or false print, but need function call
                self.append_to_pending(curr_pose_vector)

                ## Try improving code by adding mean orientation specifier as well
                # we have enough detections, so we can add the marker to the marker_list
                if(len(self.pending_marker_list) > 10) and self.no_detection < 10 and self.detection_mode:
                    
                    if (self.no_detection == 0):
                        mean_loc = np.mean(np.array(self.pending_marker_list), axis=0)
                        print("mean_loc: ", mean_loc)
                        mean_pos = np.mean(self.pending_position_list, axis=0)
                        print(mean_pos)
                        print(f"self.x: {self.x}, self.y: {self.y}, self.z_ang: {self.z_ang}, self.w_ang: {self.w_ang}")
                        ideal_pose = Pose()
                        ideal_pose.position.x = mean_loc[0]
                        ideal_pose.position.y = mean_loc[1]
                        ideal_pose.position.z = mean_loc[2]
                        ## mean position of target circle. Create marker
                        mark = self.marker_maker(ideal_pose, point_world)
                        self.markers_pub.publish(mark)
                        goal = self.create_approach_goal(mark, 0, 0)
                        print("Current pose vector: ", curr_pose_vector)
                        # goal = self.create_approach_experimental(curr_pose_vector)
                        self.global_goal = goal
                        print("Global goal: ", self.global_goal)
                        # print("Approaching marker:\n", mark.pose.position)


                    self.no_detection += 1

                    print(f"Wanted angle:\n\t z:{self.global_goal.pose.orientation.z},\n\t w:{self.global_goal.pose.orientation.w}")

                    # self.approach_pub.publish(goal)
                    print("Publishing goal: ", self.global_goal.pose.position)
                    self.pending_marker_list.clear()
                    ## experimental
                    # self.twist_approach_pub.publish(self.global_goal)
                    self.face_goal(self.global_goal)
                    self.detection_mode = False
                    self.completed_status = False
                if (self.detection_mode is False and self.completed_status is True):
                    print("Time to approach!")
                    self.go_ahead(cv_image[x_min:x_max,y_min:y_max])
                    # ideal_pose = Pose()
                    # mean_loc = np.mean(np.array(self.pending_marker_list), axis=0)
                    # ideal_pose.position.x = mean_loc[0]
                    # ideal_pose.position.y = mean_loc[1]
                    # ideal_pose.position.z = mean_loc[2]
                    # mark = self.marker_maker(ideal_pose, point_world)
                    # goal = self.create_approach_goal(mark, 0, 0)
                    # self.global_goal = goal
                    # self.twist_approach_pub.publish(self.global_goal)
                    # self.completed_status = False

            # self.markers_pub.publish(self.marker_array)
                
        if len(candidates)>0 and ring_flag:
            # print("Confirmed markers: ", self.confirmed_marker_list)
            print("Pending markers: ", len(self.pending_marker_list))
            cv2.imshow("Image window", copy)
            cv2.waitKey(1)

        # print(f"OUT IF: self.x: {self.x}, self.y: {self.y}, self.z_ang: {self.z_ang}, self.w_ang: {self.w_ang}")


def main():

    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
