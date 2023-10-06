#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import tf
from os.path import dirname, join
import tf2_geometry_msgs
import tf2_ros
from task2.msg import control
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from std_msgs.msg import Int8, String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import ColorRGBA
import webcolors
import math
import dlib
import pytesseract


# color string to rgb.
color_mapping = {"green" : np.array([0, 255, 0]),
                    "blue" :  np.array([0, 255, 255]),
                    "light blue" :  np.array([0, 255, 255]),
                    "red" :  np.array([255, 0, 0]),
                    "yellow" :  np.array([255, 255, 60])} # rg are fine, b should be ~60

class Cylinder_center_detector:
    def __init__(self):
        rospy.init_node('Cylinder_center_detector', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        # self.marker_array = MarkerArray()
        
        # Subscribe to the image and/or depth topic
        self.operation_initialization = rospy.Subscriber("/cylinder_center_detector", String, self.begin_operation_callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.center_finder)
        # self.status_sub = rospy.Subscriber("/circle_goal/status", Int8, self.set_completed_status)
        self.face_embedding_pub = rospy.Publisher('/cylinder_face_embedding', Float32MultiArray, queue_size=10)
        # self.curr_location = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.curr_location_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.start_time = rospy.Time.now()

        # self.begin_operation_sub = rospy.Subscriber("/detect_floor/init", Int8, self.begin_operation_callback)

        self.approach_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=20)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        # Following initiate twist_control functions responsible for either moving the robot forward or rotating it.
        # specifically, second function is only meant for initial spotting of the circle, as that should allow our detect_floor to finally spot the circle

        # self.twist_forward_pub = rospy.Publisher('/circle_goal/forward', Int8, queue_size=10)
        # self.twist_angular_pub = rospy.Publisher('/circle_goal/angular', Int8, queue_size=10)

        # self.twist_controller_pub = rospy.Publisher('/circle_goal/controller', control, queue_size=10)

        self.pending_marker_list = []
        self.no_detection = 0
        self.global_goal = PoseStamped()
        self.completed_status = True
        self.detection_mode = True
        self.forward_init = False
        self.desired_rotation_angle = 45 # in deg
        self.desired_rotation_angle_radians = self.desired_rotation_angle * (np.pi / 180)

        self.sign = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.z_ang = 0
        self.w_ang = 0
        self.quart = [0, 0, 0, 0]

        self.minimum_distance = 999
        self.minimum_distance_x = 0
        self.minimum_distance_y = 0
        self.minimum_distance_z_ang = 0
        self.minimum_distance_w_ang = 0

        self.target_color = ''
        # self.allow_black = True
        self.start_operation_flag = False

        self.angular_const = 0.1
        self.linear_const = 0.03

        # OCR REMNANT
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")
        self.face_rec_model = dlib.face_recognition_model_v1(join(dirname(__file__), 'dlib_face_recognition_resnet_model_v1.dat'))

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        self.shape_predictor = dlib.shape_predictor(join(dirname(__file__), "shape_predictor_68_face_landmarks.dat"))
        self.face_detector = dlib.get_frontal_face_detector()
        pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'
        self.ocr = pytesseract

    def determine_majority_side(self, image):
        # Threshold values
        threshold = 7
        target_color = (255, 255, 60)  # RGB value of the target color

        # Calculate color difference from the target color
        color_diff = np.abs(image - target_color)

        # Calculate pixel-wise difference from the threshold
        pixel_diff = np.sum(color_diff, axis=2)

        # Find the indices of pixels within the threshold
        within_threshold_indices = np.where(pixel_diff <= threshold)

        # Determine the majority side
        left_count = np.sum(within_threshold_indices[1] < image.shape[1] // 2)
        right_count = np.sum(within_threshold_indices[1] >= image.shape[1] // 2)

        if left_count > right_count:
            return "Left"
        elif right_count > left_count:
            return "Right"
        else:
            return "Equal"


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
            ## here we have a situation where in amcl rotation w cannot be negative
            ## but in our transformation it can. If W is negative, we know we  have to flip the sign
            ## this is because of Q representing a rotation matrix, and -Q representing the absolute same rotation
            ## quarterion representation of the angle: [x, y, z, w]
        if self.sign == 0:
            self.sign = 1
        else:
            self.sign = -1
        # print(self.global_pose_)
        self.overwrite_location()


    def begin_operation_callback(self, data):
        self.start_operation_flag = True
        self.target_color = data.data
        print("Begin operation --> Target color: " + self.target_color)
        self.center_finder()
        rospy.sleep(2)
        self.face_recognizer()


    # function to update image information.
    def grab_latest_images(self, data=None):
        if data == None:
            try:
                data = rospy.wait_for_message('/camera/rgb/image_raw', Image)   
            except Exception as e:
                print(e)

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Grab depth image
        try:
            self.depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)   
        except Exception as e:
            print(e)

        # Transform depth to OpenCV format
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(self.depth_img, "passthrough")
        except CvBridgeError as e:
            print(e)

    def grab_arm_image(self):
        try:
            self.arm_img = rospy.wait_for_message('/arm_camera/rgb/image_raw', Image, timeout=5)   
        except Exception as e:
            print("Error grabbing arm image")
            print(e)

        try:
            self.arm_img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.arm_img, "bgr8"), cv2.COLOR_BGR2RGB)
        except Exception as e:
            print("Error converting image")
            print(e)


    def pose_stamped_creator(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose
    
    def calculate_nearest_distance(self, euler1, euler2) -> float:
        # Wrap the Euler angles to the range -pi to pi
        euler1_wrapped = self.wrap_to_pi(euler1)
        euler2_wrapped = self.wrap_to_pi(euler2)

        # Calculate the difference between the wrapped angles
        distance = euler1_wrapped - euler2_wrapped

        # Wrap the distance to the range -pi to pi
        distance_wrapped = self.wrap_to_pi(distance)

        return distance_wrapped

    def wrap_to_pi(self, angle):
        wrapped_angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return wrapped_angle


    def center_finder(self):
        # Grab image, transform to OpenCV BGR format
        target_color = color_mapping[self.target_color]
        threshold = 8 ## diff should be smaller than 8 or else we panik.
        initial_direction = 0;

        for direction in range(0, 9):

            self.grab_latest_images()
            self.get_robot_pose()

            # Get dimensions of image
            self.dims = self.cv_image.shape

            # Grab only desired pixels, convert pic to RGB:
            rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

            # Calculate the Euclidean distance for each pixel
            diff = np.abs(rgb_image - target_color)
            
            pixel_distance = np.sqrt(np.sum(diff ** 2, axis=2)) ## euclidean dist function

            mask = pixel_distance < threshold
            masked_image = np.zeros_like(rgb_image)
            masked_image[mask] = rgb_image[mask]

            print((mask == 1).any())
            if np.any(mask > 0): # desired pixels m'lord. commit analysis.
                # Quaternion()
                print("!!   Found something!!   ")
                returned_str = self.determine_majority_side(rgb_image)
                # print(returned_str)
                if returned_str == "Left":
                    initial_direction = 1
                elif returned_str == "Right":
                    initial_direction = -1

                break
                depth_mask = np.zeros_like(self.depth_img)
                depth_mask[mask] = self.depth_img[mask] ## expect positive float values.
                non_nan_values = depth_mask[np.logical_not(np.isnan(depth_mask))]
                if len(non_nan_values) > 0:
                    min_depth = np.min(non_nan_values[np.nonzero(non_nan_values)])
                # min_depth = np.min(depth_mask[np.nonzero(depth_mask)])
                print("minimum depth: " + str(min_depth))
                min_depth_coords = np.argwhere(depth_mask == min_depth)[0]

                if (min_depth < self.minimum_distance):
                    height, width = self.depth_img.shape
                    center_x, center_y = width // 2, height // 2
                    center_orientation = np.arctan2(min_depth_coords[0] - center_x, min_depth_coords[1] - center_y)
                # distances = np.sqrt(np.sum(diff ** 2, axis=2))
                    # _, _, current_yaw = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
                    # new_orientation_rad = current_yaw + center_orientation

                    # if new_orientation_rad > 3.14159:
                    #     new_orientation_rad -= 2 * 3.14159
                    # elif new_orientation_rad < -3.14159:
                    #     new_orientation_rad += 2 * 3.14159


                    # new_orient_test = tf.transformations.quaternion_from_euler(0, 0, new_orientation_rad)
                    # print("new orientation: " + str(new_orient_test))

                    desired_orientation = self.sign * tf.transformations.quaternion_from_euler(0, 0, center_orientation) ## this is the one.
                    print("desired_orientation: " + str(desired_orientation))
                    ## assignments for later
                    self.minimum_distance = min_depth
                    self.minimum_distance_x = self.x
                    self.minimum_distance_y = self.y
                    self.minimum_distance_z_ang = desired_orientation[2]
                    self.minimum_distance_w_ang = desired_orientation[3]

                    print("new minimum distance: " + str(self.minimum_distance))
                    print("new minimum distance x: " + str(self.minimum_distance_x))
                    print("new minimum distance y: " + str(self.minimum_distance_y))
                    print("new minimum distance z_ang: " + str(self.minimum_distance_z_ang))
                    print("new minimum distance w_ang: " + str(self.minimum_distance_w_ang))

                    print("current robot pose: x: " + str(self.x) + " y: " + str(self.y) + " z_ang: " + str(self.z_ang) + " w_ang: " + str(self.w_ang))

            # -----------
            else: print("No desired pixels found. :(. Rotating!!!!")
            # current_orientation = Quaternion(x=0, y=0, z=self.z_ang, w=self.w_ang)
            _, _, current_yaw = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
            print("Current yaw: " + str(current_yaw))
            desired_yaw = current_yaw + self.desired_rotation_angle_radians
            print("Desired yaw: " + str(desired_yaw))
            desired_orientation = tf.transformations.quaternion_from_euler(0, 0, desired_yaw)
            
            rotation_goal = self.pose_stamped_creator(self.x, self.y, desired_orientation[2], desired_orientation[3])
            self.approach_pub.publish(rotation_goal)
            completed_flag = False
            limit_index = 0
            while not completed_flag and limit_index < 1000:
                current_status = rospy.wait_for_message("/move_base/status", GoalStatusArray, rospy.Duration(2));
                if current_status.status_list[-1].status == 3:
                    completed_flag = True
                limit_index += 1

            if completed_flag:
                print("Rotation completed. :DDDD")
                self.get_robot_pose()
            else: 
                print("Rotation failed. :((((")
                self.get_robot_pose()
    
        # print("Turning checks completed. Continue to face the goal, use twist messages to be more accurate.")
        # print("Minimum distance: " + str(self.minimum_distance))
        # print("Minimum distance x: " + str(self.minimum_distance_x))
        # print("Minimum distance y: " + str(self.minimum_distance_y))
        # print("Minimum distance z_ang: " + str(self.minimum_distance_z_ang))
        # print("Minimum distance w_ang: " + str(self.minimum_distance_w_ang))
        # # face = self.pose_stamped_creator(self.minimum_distance_x, self.minimum_distance_y, self.minimum_distance_z_ang, self.minimum_distance_w_ang)
        # # self.approach_pub.publish(face)
        
        # turning_complete_flag = True
        # limit_index = 0
        # desired_yaw = tf.transformations.euler_from_quaternion([0, 0, self.minimum_distance_z_ang, self.minimum_distance_w_ang])[2]
        # # while not turning_complete_flag and limit_index < 1000:
        # #     self.get_robot_pose()
        #     # turning_complete_flag = abs(self.z_ang - minimum_distance_z_ang) > 0.01 or abs(self.w_ang - minimum_distance_w_ang) > 0.01 or self.z_ang * msg.pose.orientation.z < 0
        # #     current_yaw = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])[2]
        # #     # calculate necessary sign of angular velocity.
        # #     closest = self.calculate_nearest_distance(desired_yaw, current_yaw)
        # #     rotation_sign = 1 if closest >= 0 else -1

        # #     twist = Twist()
        # #     twist.angular.z =rotation_sign * 1.2 * self.angular_const
        # #     self.twist_pub.publish(twist)
        # #     # self.get_robot_pose()
        # #     print("rotating")
        
        # #initial face.
        # while (turning_complete_flag and limit_index < 100000):
        #     robot_angle = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
        #     twist = Twist()
        #     if(robot_angle[2] > desired_yaw):
        #         twist.angular.z = -self.angular_const
        #     else:
        #         twist.angular.z = self.angular_const
        #     self.twist_pub.publish(twist)
        #     turning_complete_flag = abs(self.z_ang - self.minimum_distance_z_ang) > 0.01 or abs(self.w_ang - self.minimum_distance_w_ang) > 0.01 or self.z_ang * self.minimum_distance_z_ang < 0
        #     self.get_robot_pose()
        #     limit_index += 1

        # #find closest
        # print("Initial face completed. :DDDD")
        # print("Initial face completed. :DDDD")
        # print("Initial face completed. :DDDD")

        # --------

        self.minimum_distance = 999
        first_direction = True
        yellow_encountered = False
        turning_complete_flag = True

        #fine direction finder
        while(turning_complete_flag):
            print("first_direction: " + str(first_direction))
            print("yellow_encountered: " + str(yellow_encountered))
            print("turning_complete_flag: " + str(turning_complete_flag))
            self.grab_latest_images()
            self.get_robot_pose()
            twist = Twist()
            twist.angular.z = self.angular_const*0.7*initial_direction if first_direction else -self.angular_const*0.7 * initial_direction
            rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            image_center_column = rgb_image[:, self.dims[1] // 2]
            depth_image_center_column = self.depth_img[:, self.dims[1] // 2]
            # depth_mask = np.zeros_like(self.depth_img)
            # depth_image_center_column = depth_mask[:, self.dims[1] // 2]
            # print(image_center_column.shape)
            diff = np.abs(image_center_column - target_color)
                
            pixel_distance = np.sqrt(np.sum(diff ** 2, axis=1)) ## euclidean dist function

            mask = pixel_distance < threshold
            masked_image = np.zeros_like(image_center_column)
            masked_image[mask] = image_center_column[mask]

            # depth_image_mask = np.zeros_like(image_center_column)
            # depth_image_mask[mask] = self.depth_img[mask] ## expect positive float values.
            # plt.imshow(image_center_column)
            # plt.waitforbuttonpress(1)
            if np.any(mask > 0):
                yellow_encountered = True
                depth_image_mask = np.zeros_like(depth_image_center_column)
                depth_image_mask[mask] = depth_image_center_column[mask] ## expect positive float values.

                min_depth = np.min(depth_image_mask[np.nonzero(depth_image_mask)])
                # min_depth_coords = np.argwhere(depth_mask == min_depth)[0]
                if min_depth < self.minimum_distance:
                    self.minimum_distance = min_depth
                    self.minimum_distance_x = self.x
                    self.minimum_distance_y = self.y
                    self.minimum_distance_z_ang = self.z_ang
                    self.minimum_distance_w_ang = self.w_ang
                    print("new minimum distance: " + str(self.minimum_distance))
                    print("new minimum distance x: " + str(self.minimum_distance_x))
                    print("new minimum distance y: " + str(self.minimum_distance_y))
                    print("new minimum distance z_ang: " + str(self.minimum_distance_z_ang))
                    print("new minimum distance w_ang: " + str(self.minimum_distance_w_ang))
            elif(yellow_encountered and not first_direction):
                turning_complete_flag = False
            elif(yellow_encountered):
                yellow_encountered = False
                first_direction = False
            self.twist_pub.publish(twist)

        turning_complete_flag = True
        limit_index = 0
        desired_yaw = tf.transformations.euler_from_quaternion([0, 0, self.minimum_distance_z_ang, self.minimum_distance_w_ang])[2]

        while (turning_complete_flag and limit_index < 100000):
            robot_angle = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
            twist = Twist()
            if(robot_angle[2] > desired_yaw):
                twist.angular.z = -self.angular_const
            else:
                twist.angular.z = self.angular_const
            self.twist_pub.publish(twist)
            turning_complete_flag = abs(self.z_ang - self.minimum_distance_z_ang) > 0.01 or abs(self.w_ang - self.minimum_distance_w_ang) > 0.01 or self.z_ang * self.minimum_distance_z_ang < 0
            self.get_robot_pose()
            limit_index += 1

        # move forward until we reach desired distance.
        while (self.minimum_distance > 0.403):
            self.grab_latest_images()
            self.get_robot_pose()
            rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            image_center_column = rgb_image[:, self.dims[1] // 2]
            depth_image_center_column = self.depth_img[:, self.dims[1] // 2]

            twist = Twist()
            twist.linear.x = self.linear_const

            diff = np.abs(image_center_column - target_color)
                
            pixel_distance = np.sqrt(np.sum(diff ** 2, axis=1)) ## euclidean dist function

            mask = pixel_distance < threshold
            masked_image = np.zeros_like(image_center_column)
            masked_image[mask] = image_center_column[mask]

            if np.any(mask > 0):
                yellow_encountered = True
                depth_image_mask = np.zeros_like(depth_image_center_column)
                depth_image_mask[mask] = depth_image_center_column[mask] ## expect positive float values.

                min_depth = np.min(depth_image_mask[np.nonzero(depth_image_mask)])

                if min_depth < self.minimum_distance:
                    self.minimum_distance = min_depth
                print("minimum depth: " + str(min_depth))
            print("minimum distance: " + str(self.minimum_distance))
            self.twist_pub.publish(twist)

        print("EINE FAJNE BOBR")
        approach_flag = True
        start_time = rospy.Time.now()
        while (approach_flag):
            twist = Twist()
            twist.linear.x = self.linear_const/2
            self.twist_pub.publish(twist)
            curr_time = rospy.Time.now()
            # rospy.Duration.from_sec()
            if (curr_time - start_time > rospy.Duration.from_sec(4.6)):
                print("time's up!!!")
                approach_flag = False

        



        # while (turning_complete_flag):
        #     robot_angle = tf.transformations.euler_from_quaternion([0, 0, self.z_ang, self.w_ang])
        #     twist = Twist()
        #     if(robot_angle[2] > desired_yaw):
        #         twist.angular.z = -self.angular_const
        #     else:
        #         twist.angular.z = self.angular_const
        #     sign_check = self.z_ang * self.minimum_distance_z_ang
        #     turning_complete_flag = not(sign_check >= 0 and abs(robot_angle[2] - desired_yaw) < 0.05)
        #     self.get_robot_pose()


    def face_recognizer(self):
        self.grab_arm_image()
        print("time to recognize faces")
        # Set the dimensions of the image
        self.dims = self.arm_img.shape
        h = self.dims[0] 
        w = self.dims[1]

        # Tranform image to gayscale
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equalization
        #img = cv2.equalizeHist(gray)
        # arm_rgb = cv2.cvtColor(self.arm_img, cv2.COLOR_BGR2RGB)
        # Detect the faces in the image
        #face_rectangles = self.face_detector(rgb_image, 0)
        blob = cv2.dnn.blobFromImage(cv2.resize(self.arm_img, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()
        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            # print("current face_confidence = ", confidence)

            if confidence > 0.70: # 0.80
                print("Face detected")
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1] , box[2], box[3]
                x1 = max(x1, 0)
                y1 = max(y1, 0)
                x2 = max(x2, 0)
                y2 = max(y2, 0)
                print("x1: ", x1, " y1: ", y1, " x2: ", x2, " y2: ", y2)

                face_region = self.arm_img[y1:y2, x1:x2] # start and ending coords on x and y axis
                plt.imshow(face_region)
                plt.axis('off')
                plt.show()

                #### Different face detector
                # detected_faces = self.face_detector(self.arm_img)
                # print("detected_faces: ", detected_faces)
                # print(detected_faces[0].left(), detected_faces[0].top(), detected_faces[0].right(), detected_faces[0].bottom())
                # x1, y1, x2, y2 = detected_faces[0].left(), detected_faces[0].top(), detected_faces[0].right(), detected_faces[0].bottom()
                # face_prototype = self.arm_img[y1:y2, x1:x2] # start and ending coords on x and y axis
                # plt.imshow(face_prototype)
                # plt.axis('off')
                # plt.show()
                #### ~Different face detector
                face_cheat = dlib.rectangle(x1, y1, x2, y2)
                landmarks = self.shape_predictor(self.arm_img, face_cheat)
                aligned_face = dlib.get_face_chip(self.arm_img, landmarks)

                plt.imshow(aligned_face)
                plt.axis('off')
                plt.show()
                face_descriptor = self.face_rec_model.compute_face_descriptor(aligned_face)
                print(face_descriptor)
                print(face_descriptor.shape)

                msg = Float32MultiArray()
                approach_flag = True
                start_time = rospy.Time.now()
                while (approach_flag):
                    twist = Twist()
                    twist.linear.x = -self.linear_const/1.5
                    self.twist_pub.publish(twist)
                    curr_time = rospy.Time.now()
                    # rospy.Duration.from_sec()
                    if (curr_time - start_time > rospy.Duration.from_sec(5.5)):
                        print("Reversal done!!!!")
                        approach_flag = False

                msg.data = face_descriptor
                for i in range(0,5):
                    self.face_embedding_pub.publish(msg)




def main():

    cylinder_center = Cylinder_center_detector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
