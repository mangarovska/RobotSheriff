#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher ring_marker_pub_;

tf2_ros::Buffer tf2_buffer;

// Create a ROS publisher for the output

typedef pcl::PointXYZ PointT;
using namespace std;

// camera spec
int width_ = 640;
int height_ = 480;
int dims_[] = {480, 640, 3};

/**
 * Global declarations for easier debugging and tweaking:
 *  Note: all global variables are postfixed with _.
 **/
visualization_msgs::MarkerArray marker_array_cylinder_;
visualization_msgs::MarkerArray marker_array_rings_;
visualization_msgs::Marker marker_confirmed_;
vector<vector<float>> non_maxima_supp_vector_; // for saving x, y of detected cylinder centroid
// 1) segmentation for planar model
float plan_normal_distance_weight_ = 0.4; // default 0.1 // higher seems to jump less<0.5>
int plan_max_iterations_ = 70;			  // default 100 // lower seems to offer a more stable prediction
float plan_distance_threshold_ = 0.03;	  // default 0.03
// 2) segmentation for cylinder model
float cyl_normal_distance_weight_ = 0.3;   // default 0.1 <0.2>
int cyl_max_iterations_ = 10000;		   // default 10000
float cyl_set_distance_threshold_ = 0.020; // default 0.05 <0.02, 0.01>
float cyl_rad_min_ = 0.11;				   // default 0.06
float cyl_rad_max_ = 0.13;				   // default 0.2 0.11, 0.15
										   // decent 0.11; 0.15

int marker_num_ = 0;
boost::shared_ptr<const sensor_msgs::Image> depth_image_;

bool distance_check(vector<float> new_detection, float acceptable_distance)
{
	for (int i = 0; i < non_maxima_supp_vector_.size(); i++)
	{
		float x_dist = pow(non_maxima_supp_vector_[i][0] - new_detection[0], 2);
		float y_dist = pow(non_maxima_supp_vector_[i][1] - new_detection[1], 2);
		if (sqrt(x_dist + y_dist) > acceptable_distance)
		{
			// assume outlier, empty our vector
			non_maxima_supp_vector_.clear();
			return false;
		}
	}
	return true;
}

void get_pose(cv::RotatedRect e, float dist) {
    float k_f = 525.0; // kinect focal length in pixels

    float elipse_x = dims_[1] / 2 - e.center.x;
    float elipse_y = dims_[0] / 2 - e.center.y;

    float angle_to_target = atan2(elipse_x, k_f);

    // Get the angles in the base_link relative coordinate system
    float x = dist * cos(angle_to_target);
    float y = dist * sin(angle_to_target);

    // Define a stamped message for transformation - in the "camera rgb frame"
    geometry_msgs::PointStamped point_s;
    point_s.point.x = -y;
    point_s.point.y = 0;
    point_s.point.z = x;
    point_s.header.frame_id = "camera_rgb_optical_frame";
    point_s.header.stamp = ros::Time(0);

    // Get the point in the "map" coordinate system
    geometry_msgs::PointStamped point_world;
    try {
        point_world = tf2_buffer.transform(point_s, "map");
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Error transforming point: %s", ex.what());
        return;
    }

    // Create a Pose object with the same position
    geometry_msgs::Pose pose;
    pose.position.x = point_world.point.x;
    pose.position.y = point_world.point.y;
    pose.position.z = point_world.point.z;

    // Create a marker used for visualization
	marker_num_++;
    visualization_msgs::Marker marker;
    marker.header.stamp = point_world.header.stamp;
    marker.header.frame_id = point_world.header.frame_id;
    marker.pose = pose;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.frame_locked = false;
    marker.lifetime = ros::Duration(10);
    marker.id = marker_num_;
	geometry_msgs::Vector3 scale;
	scale.x = 0.1; scale.y = 0.1; scale.z = 0.1;
    marker.scale = scale;
	std_msgs::ColorRGBA color;
	color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
    marker.color = color;
    marker_array_rings_.markers.push_back(marker);

    ring_marker_pub_.publish(marker_array_rings_);
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
	// All the objects needed
	int counter = 0;
	ros::Time time_rec, time_test;
	time_rec = ros::Time::now();

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	Eigen::Vector4f centroid;

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	// Read in the cloud data
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud);
	// std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	// std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(75);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(plan_normal_distance_weight_); // default 0.1 // higher seems to jump less<0.5>
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(plan_max_iterations_);			// default 100 // lower seems to offer a more stable prediction
	seg.setDistanceThreshold(plan_distance_threshold_); // default 0.03
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	// std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	// std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

	pcl::PCLPointCloud2 outcloud_plane;
	pcl::toPCLPointCloud2(*cloud_plane, outcloud_plane);
	pubx.publish(outcloud_plane);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	// Create a ROS publisher for the output
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(cyl_normal_distance_weight_); // default 0.1
	seg.setMaxIterations(cyl_max_iterations_);				  // default 10000
	seg.setDistanceThreshold(cyl_set_distance_threshold_);	  // default 0.05 <0.25 je videt dobro.
	seg.setRadiusLimits(cyl_rad_min_, cyl_rad_max_);		  // default 0.06, 0.2 0.11, 0.15
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	// std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty()) {
		// counter++;
		// if (counter > 10) {
		// 	non_maxima_supp_vector_.clear();
		// 	counter = 0;
		// 	}
		std::cerr << "Can't find the cylindrical component." << std::endl;
	} else
	{
		/**
		 * Detected cloud cylinder. create marker for it.
		 */
		vector<float> centroid_point { centroid[0], centroid[1], centroid[2] };
		bool is_acceptable_dist = distance_check(centroid_point, 0.6);
		if (non_maxima_supp_vector_.size() == 0 || is_acceptable_dist) {
			non_maxima_supp_vector_.push_back(centroid_point);
		}
		// std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;

		pcl::compute3DCentroid(*cloud_cylinder, centroid);
		std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

		// Create a point in the "camera_rgb_optical_frame"
		geometry_msgs::PointStamped point_camera;
		geometry_msgs::PointStamped point_map;
		visualization_msgs::Marker marker;
		geometry_msgs::TransformStamped tss;

		point_camera.header.frame_id = "camera_rgb_optical_frame";
		point_camera.header.stamp = ros::Time::now();

		point_map.header.frame_id = "map";
		point_map.header.stamp = ros::Time::now();

		point_camera.point.x = centroid[0];
		point_camera.point.y = centroid[1];
		point_camera.point.z = centroid[2];

		try
		{
			time_test = ros::Time::now();

			std::cerr << time_rec << std::endl;
			std::cerr << time_test << std::endl;
			tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
			// tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("Transform warning: %s\n", ex.what());
		}

		// std::cerr << tss ;

		tf2::doTransform(point_camera, point_map, tss);

		// std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;

		// std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();

		marker.ns = "cylinder_detection";
		marker.id = 0;

		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = point_map.point.x;
		marker.pose.position.y = point_map.point.y;
		marker.pose.position.z = point_map.point.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0f;

		marker.lifetime = ros::Duration();

		pubm.publish(marker);
		if (is_acceptable_dist && non_maxima_supp_vector_.size() >10) {

			marker_confirmed_.header.frame_id = "map";
			marker_confirmed_.header.stamp = ros::Time::now();

			marker_confirmed_.ns = "cylinder_nsm_detection";
			marker_confirmed_.id = 0;

			marker_confirmed_.type = visualization_msgs::Marker::CYLINDER;
			marker_confirmed_.action = visualization_msgs::Marker::ADD;

			marker_confirmed_.pose.position.x = point_map.point.x;
			marker_confirmed_.pose.position.y = point_map.point.y;
			marker_confirmed_.pose.position.z = point_map.point.z;
			marker_confirmed_.pose.orientation.x = 0.0;
			marker_confirmed_.pose.orientation.y = 0.0;
			marker_confirmed_.pose.orientation.z = 0.0;
			marker_confirmed_.pose.orientation.w = 1.0;

			marker_confirmed_.scale.x = 0.15;
			marker_confirmed_.scale.y = 0.15;
			marker_confirmed_.scale.z = 0.15;

			marker_confirmed_.color.r = 0.0f;
			marker_confirmed_.color.g = 0.2f;
			marker_confirmed_.color.b = 1.0f;
			marker_confirmed_.color.a = 1.0f;

			marker.lifetime = ros::Duration();
			pubm.publish(marker_confirmed_);
			marker_array_cylinder_.markers.push_back(marker_confirmed_);
			non_maxima_supp_vector_.clear();
		}

		pcl::PCLPointCloud2 outcloud_cylinder;
		pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
		puby.publish(outcloud_cylinder);
	}
}

void the_ring_cb (const sensor_msgs::ImageConstPtr& msg) {
	// TODO: implement
	printf("I got a new image, for rings");
	std::cerr << "I got a new image, for rings: " << std::endl;
	cv_bridge::CvImagePtr cv_image_ptr;
	cv::Mat gray_image;
	cv::Mat equalized_image;
	cv::Mat thresh;
	try {
		// Convert the image to OpenCV format
	cv_image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		std::cerr << "cv_bridge exception: " << e.what() << std::endl;
	}
	// Set the dimensions of the image
	// int width_ = cv_image_ptr->image.cols;
	// int height_ = cv_image_ptr->image.rows;
	// hardcoded since it's a constant 640x480

	std::cerr << "Image width: " << cv_image_ptr->image.cols << ", height: " << cv_image_ptr->image.rows << std::endl;
	// Tranform image to "gayscale", 
	// the superior image rep.
	cv::cvtColor(cv_image_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
	// Do histogram equalization
	cv::equalizeHist(gray_image, equalized_image);
	// Binarize the image, there are different ways to do it
	cv::adaptiveThreshold(equalized_image, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 25);

	// Find the contours
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
	cv::drawContours(equalized_image, contours, -1, (0, 255, 0), 3);
	cv::imshow("Contour window", equalized_image);
	cv::waitKey(1);
	
	std::vector<cv::RotatedRect>list_of_elipses;
	for (const auto& contour : contours) {
		cv::Rect boundingRect = cv::boundingRect(contour);
		int width_cont = boundingRect.width;
    	int height_cont = boundingRect.height; // looking for height
		if (height_cont >= 20 && contour.size() >= 5) {
			// Fit an ellipse to the contour
			auto ellipse = cv::fitEllipse(contour);
			list_of_elipses.push_back(ellipse);
		}
	}

	std::vector<std::vector<cv::RotatedRect>> candidates;
	for (int i = 0; i < list_of_elipses.size(); i++) {
		for (int j = i + 1; j < list_of_elipses.size(); j++) {
			cv::RotatedRect e1 = list_of_elipses[i];
			cv::RotatedRect e2 = list_of_elipses[j];
			double dist = sqrt(pow(e1.center.x - e2.center.x, 2) + pow(e1.center.y - e2.center.y, 2));
			
			if (dist < 5) {
				candidates.push_back({e1, e2});
			}
		}
	}
	std::cerr << "Processing is done! found " << candidates.size() << " candidates for rings" << std::endl;

	try {
		depth_image_ = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth/image_raw", ros::Duration(5));
	} catch (std::exception& e) {
		std::cerr << "Could not get depth image: " << e.what() << std::endl;
	} 

	for (const auto& c : candidates) {
		auto e1 = c[0];
		auto e2 = c[1];
		
		cv::ellipse(cv_image_ptr->image, e1, (255, 0, 0), 2);
		cv::ellipse(cv_image_ptr->image, e2, (255, 255, 0), 2);

		double size = (e1.size.width + e1.size.height) / 2;
		cv::Point2f center = e1.center;

		int x1 = (int)(center.x - size / 2);
		int x2 = (int)(center.x + size / 2);
		int x_min = (x1 > 0) ? x1 : 0;
		int x_max = (x2 < width_) ? x2 : width_;

		int y1 = (int)(center.y - size / 2);
		int y2 = (int)(center.y + size / 2);
		int y_min = (y1 > 0) ? y1 : 0;
		int y_max = (y2 < height_) ? y2 : height_;
		// if (x_min || y_min == y_max) {
		// 	continue;
		// }
		std::cerr << "x_min: " << x_min << ", x_max: " << x_max << ", y_min: " << y_min << ", y_max: " << y_max << std::endl;
		cv::Mat depth_image = cv_bridge::toCvCopy(depth_image_, "16UC1")->image;
		// cv::Mat mask = cv::Mat::ones(depth_image.size(), CV_8U);
		// cv::compare(depth_image, depth_image, mask, cv::CMP_EQ);

		get_pose(e1, static_cast<float>(cv::mean(depth_image(cv::Range(y_min, y_max), cv::Range(x_min, x_max)))[0]));
	}

	if(candidates.size() > 0) {
		cv::imshow("Ring window", cv_image_ptr->image);
		cv::waitKey(1);
	}
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "rings_n_cylinder"); //node name
	ros::NodeHandle nh;

	// For transforming between coordinate frames
	// already included, very nice
	tf2_ros::TransformListener tf2_listener(tf2_buffer);

	// Create a ROS subscriber for the input point cloud
    // input is remapped using launch file
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
	ros::Subscriber sub2 = nh.subscribe("/camera/rgb/image_raw", 1, the_ring_cb);

	// Create a ROS publisher for the output point cloud
	pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
	puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);
	
	ring_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("detected_ring", 1);
	pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder", 1);

	// Spin
	ros::spin();
}
