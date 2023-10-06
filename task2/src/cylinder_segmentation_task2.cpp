#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
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
#include "geometry_msgs/PointStamped.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "task2/poseWithCovarString.h"
#include <cmath>
#include <thread>
#include <chrono>
// #include <sound_play.h>
// #include <sound_play/sound_play.h>
// #include <sstream>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher pub_vocalizer;
ros::Publisher pubm_array;
ros::Publisher status_pub;
ros::Publisher pub_cylinder_pose;



tf2_ros::Buffer tf2_buffer;
// Create a ROS publisher for the output

typedef pcl::PointXYZRGB PointT;
namespace std {
template<> struct hash<std::vector<int>> {
	size_t operator()(const std::vector<int>& v) const {
		std::hash<int> hasher;
		size_t seed = 0;
		for (int i : v) {
			seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
		}
		return seed;
	}
};
}

/**
 * Global declarations for easier debugging and tweaking:
 *  Note: all global variables are postfixed with _.
 * This is for the sake of clarity for the reader.
 **/
visualization_msgs::MarkerArray marker_array_;
visualization_msgs::Marker marker_confirmed_;
int marker_id_ = 0;

std::vector<std::vector<double>> non_maxima_supp_vector_; // for saving x, y.z of detected cylinder centroid
std::vector<std::vector<int>> non_maxima_rgb_vector_; // for saving r,g,b of random sample of cylinder cloud

std::vector<std::vector<double>> confirmed_cylinder_vector_; // for saving x, y of detected cylinder centroid
std::vector<std::vector<int>> detected_rgb_vector_; // for saving x, y of detected cylinder centroid

int number_of_desired_cylinders_ = 4; // Task2 states 3 cylinders on map

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

/***
 * It would appear c++ developers do not care much about working with colors.
 * This is a map of RGB values to color names, to compensate for the lack of
 * a standard library for working with colors. I will simplify it extensively.
 * source: https://www.rapidtables.com/web/color/RGB_Color.html#color-chart
 * AFAIK we're only working with the most elementary colors, so this mildly overengineered
 * Change the map so the value better represents our needs. 
 * - We could also consider changing the color format to HSV, which is better representative
 * - of human vision
 ***/
static const std::unordered_map<std::vector<int>, std::string> colorMap_ = {
	{{0, 0, 0}, "black"},
	{{255, 255, 255}, "white"},
    {{255, 0, 0}, "red"},
    {{0, 255, 0}, "green"},
    {{0, 0, 255}, "blue"},
	{{255, 255, 0}, "yellow"},
	{{0, 255, 255}, "light blue"},
	{{255, 0, 255}, "pink"},
	// {{192, 192, 192}, "light gray"}, // overfitting 
	// {{128, 128, 128}, "gray"},
	// {{128, 0, 0}, "dark red"},
	// {{128, 128, 0}, "dark yellow"},
	// {{0, 128, 0}, "dark green"},
	// {{128, 0, 128}, "purple"},
	// {{0, 128, 128}, "teal"},
	// {{0, 0, 128}, "dark blue"},
    // add more colors here if desired
	// {{165,42,42}, "brown"}, // brown
	// {{233,150,122}, "dark salmon"} // dark salmon
};


/**
 * @brief Checks if the new detection is within acceptable euclidean distance of the previous detections
 * 
 * @param new_detection std::vector<double> in the form of {x, y, z}
 * @param acceptable_distance positive double {recommended values 0.2 - 0.5}
 * @return true 
 * @return false 
 */
bool distance_check(std::vector<double> new_detection, double acceptable_distance)
{
	for (int i = 0; i < non_maxima_supp_vector_.size(); i++)
	{
		double x_dist = pow(non_maxima_supp_vector_[i][0] - new_detection[0], 2);
		double y_dist = pow(non_maxima_supp_vector_[i][1] - new_detection[1], 2);
		if (sqrt(x_dist + y_dist) > acceptable_distance)
		{
			// assume outlier, empty our vector
			non_maxima_supp_vector_.clear();
			non_maxima_rgb_vector_.clear();
			return false;
		}
	}
	return true;
}


/**
 * @brief Calculates the square of the euclidean distance between two colors
 * 
 * @param c1 std::vector<int> color1 in rgb format
 * @param c2 std::vector<int> color2 in rgb format
 * @return squared distance double 
 */
double color_distance(const std::vector<int>& c1, const std::vector<int>& c2) {
    double d = 0;
    for (int i = 0; i < 3; i++) {
        d += std::pow(c1[i] - c2[i], 2);
    }
    return std::sqrt(d);
}

/**
 * @brief An improved color distance calculator, based on the redmean approach.
 * Makes RGB actually usable.
 * 
 * @param c1 std::vector<int> color1 in rgb format
 * @param c2 std::vector<int> color2 in rgb format
 * @return squared distance double (weighted redmean euclidean)
 */
double modified_color_distance(const std::vector<int>& c1, const std::vector<int>& c2) {
	double mean_red = (c1[0] + c2[0]) / 2;
	double delta_red = c1[0] - c2[0];
	double delta_green = c1[1] - c2[1];
	double delta_blue = c1[2] - c2[2];

	double d = std::sqrt((2+mean_red/256) * std::pow(delta_red,2) 
				+ 4 * std::pow(delta_green, 2) + 
				(2+(255-mean_red)/256) * std::pow(delta_blue, 2));
	return d;
    // return std::sqrt(d);
}

/**
 * @brief Calculates the mean point within the non_maxima_supp_vector_ and returns it as a vector
 * 
 * @return std::vector<double> mean point of our nms
 */
std::vector<double> calculate_mean() {
	double x_mean = 0; double y_mean = 0; double z_mean = 0;
	int sz = non_maxima_supp_vector_.size();
	for (int i = 0; i < sz; i++) {
		x_mean += non_maxima_supp_vector_[i][0];
		y_mean += non_maxima_supp_vector_[i][1];
		z_mean += non_maxima_supp_vector_[i][2];
	}
	x_mean /= sz; y_mean /= sz; z_mean /= sz;

	std::vector<double> mean_vec = {x_mean, y_mean, z_mean};
	return mean_vec;
}


/**
 * @brief Calculates the mean color within the non_maxima_rgb_vector_ and returns it as a vector
 * 
 * @return std::vector<int> mean color of our detections
 */
std::vector<int> rgb_mean() {
	int r_mean = 0; int g_mean = 0; int b_mean = 0;
	int sz = non_maxima_rgb_vector_.size();
	for (int i = 0; i < sz; i++) {
		r_mean += non_maxima_rgb_vector_[i][0];
		g_mean += non_maxima_rgb_vector_[i][1];
		b_mean += non_maxima_rgb_vector_[i][2];
		// std::cout<<"in LOOP: r: "<<r_mean<<" g: "<<g_mean<<" b: "<<b_mean<<std::endl;
	}
	r_mean /= sz; g_mean /= sz; b_mean /= sz;

	std::vector<int> rgb_mean_vector = {r_mean, g_mean, b_mean};
	return rgb_mean_vector;
}

/**
 * @brief Finds the closest color in the colorMap_ to the given color using euclidean distance
 * in call to color_distance()
 * 
 * @param color  std::vector<int> color in rgb format
 * @return std::string 
 */
std::string find_closest_color(const std::vector<int>& color) {

    double minDistance = std::numeric_limits<double>::max();
    std::string closestColor = "";
    for (const auto& entry : colorMap_) { // loop needed, I apologise.
        double distance = modified_color_distance(entry.first, color);
        if (distance < minDistance) {
            minDistance = distance;
            closestColor = entry.second;
        }
    }
    return closestColor;
}


/**
 * @brief Checks if the new detection is within the radius of any of the confirmed cylinders.
 * Returns true if it's a new detection
 * 
 * @param new_detection std::vector<double> a new confirmed detection in the form of {x, y, z}
 * @param radius positive double {recommended values 0.2 - 0.5}
 * @return true 
 * @return false 
 */
bool is_new(std::vector<double> new_detection, double radius)
{
	for (int i = 0; i < confirmed_cylinder_vector_.size(); i++)
	{
		double x_dist = pow(confirmed_cylinder_vector_[i][0] - new_detection[0], 2);
		double y_dist = pow(confirmed_cylinder_vector_[i][1] - new_detection[1], 2);
		if (sqrt(x_dist + y_dist) <  radius)
		{
			// assume it is an old detection, empty our queue of detections
			non_maxima_supp_vector_.clear();
			non_maxima_rgb_vector_.clear();
			return false;
		}
	}
	return true;
}

/**
 * @brief Auxiliary function to visualize a pointcloud with rgb values, for the purpose of debugging
 * and setting parameters in the passthrough filtering.
 * 
 * @param cloud pcl::PointCloud<PointT>::Ptr Cloud to be visualized
 * @param name std::string name of the window. Default: "cloud"
 */
void rgb_pointcloud_vizualisation(pcl::PointCloud<PointT>::Ptr cloud, std::string name = "cloud") {
	pcl::visualization::PCLVisualizer viewer(name);
	viewer.addPointCloud(cloud);

	// Set the color of the points to reflect their Z coordinate
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, color_handler, name);

	// Set the background color of the viewer to black
	viewer.setBackgroundColor(0, 0, 0);

	// Set the camera position to a good viewing angle
	viewer.setCameraPosition(0, 0, -2, 0, -1, 0);

	// Display the viewer and wait for the user to close it
	while(!viewer.wasStopped()) {
    	viewer.spinOnce();
    	std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay to reduce CPU usage
	}
	return;
}


/**
 * @brief Auxiliary function to randomly sample our pointcloud and return the mean color of the sampled points
 * 
 * @param cloud_blob pcl::PointCloud<PointT>::Ptr subcloud we're sampling from
 * @param max_samples int maximum number of samples we want to take, if there are fewer points, take greedily. Default: 400
 * @return std::vector<int> mean rgb vector of the sampled points
 */
std::vector<int> color_sampler(const pcl::PointCloud<PointT>::Ptr cloud_blob, int max_samples=400) {
		pcl::RandomSample<PointT> sampler;
		pcl::PointCloud<PointT>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZRGB>);
		int num_samples;

		if (cloud_blob->points.size() >= max_samples)
			num_samples = max_samples;
		else {
			num_samples = cloud_blob->points.size();
		}
		sampler.setInputCloud(cloud_blob);
		sampler.setSample(num_samples);

		sampler.filter(*cloud_sampled);

		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		int r_sum = 0, g_sum = 0,  b_sum = 0;
		for (it = cloud_sampled->begin(); it != cloud_sampled->end(); ++it) {
			// access the point's color using the iterator's RGB property
			r_sum += it->r;
			g_sum += it->g;
			b_sum += it->b;
		}
		r_sum /= num_samples;
		g_sum /= num_samples;
		b_sum /= num_samples;

		std::vector<int> rgb_vector{r_sum, g_sum, b_sum};
		return rgb_vector;
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
	// std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
	
	// internal function call to enable temporary visualization of the point cloud:
	// rgb_pointcloud_vizualisation(cloud, "initial cloud");


	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	// std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// rgb_pointcloud_vizualisation(cloud_filtered, "after z filter");

	// Build a passthrough filter to remove points that are too low
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.4, 0); // 6 million aint cutting it
	pass.filter(*cloud_filtered);

	// rgb_pointcloud_vizualisation(cloud_filtered, "after y filter");
	// Testing confirms it works, disable visualization since it's somewhat slow

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
	// std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	// std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

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
	// std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty() || cloud_cylinder->points.size() < 100)
	{
		// counter++;
		// if (counter > 10) {
		// 	non_maxima_supp_vector_.clear();
		// 	counter = 0;
		// 	}
		// std::cout << "Can't find the cylindrical component." << std::endl;
	}
	else
	{
		/***
		 * Detected cloud cylinder. create marker for it.
		 ***/


		std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;

		pcl::compute3DCentroid(*cloud_cylinder, centroid);
		// std::cout << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

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

			// std::cout << time_rec << std::endl;
			// std::cout << time_test << std::endl;
			tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
			// tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("Transform warning: %s\n", ex.what());
		}

		// std::cout << tss ;

		tf2::doTransform(point_camera, point_map, tss);

		std::vector<double> centroid_point{point_map.point.x, point_map.point.y, point_map.point.z};
		bool is_acceptable_dist = distance_check(centroid_point, 0.6);
		std::vector<int> rgb_vector = color_sampler(cloud_cylinder);

		std::cout << "RGB: [" << rgb_vector[0] << " " << rgb_vector[1] << " " << rgb_vector[2] << "]" << std::endl;
		std::cout << "Non-maxima rgb size: " << non_maxima_rgb_vector_.size() << std::endl;
		if ((non_maxima_supp_vector_.size() == 0 || is_acceptable_dist) && !std::isnan(centroid_point[0]))
		{
			// std::cout << "Pushing back centroid point " << centroid_point[0] << " " << centroid_point[1] << " " << centroid_point[2] << std::endl;
			non_maxima_supp_vector_.push_back(centroid_point);
			non_maxima_rgb_vector_.push_back(rgb_vector);
		}
		/***
		 * 
		 * After more important business is complete we can now subsample the points from PCL
		 * to ensure we get a decent representation of the detected object for color extraction.
		 * 
		 ***/
		// std::cout << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;

		// std::cout << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

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
		pubm_array.publish(marker_array_);

		// std::cout << "non_maxima_supp_vector_.size(): " << non_maxima_supp_vector_.size() << std::endl;
		// std::cout << "confirmed_cylinder_vector_.size(): " << confirmed_cylinder_vector_.size() << std::endl;
		if (is_acceptable_dist && non_maxima_supp_vector_.size() > 11)
		{
			auto mean_vect = calculate_mean();
			auto rgb_mean_vect = rgb_mean();
			if (!is_new(mean_vect, 0.6)) return; // if false we don't continue as it's already been detected.
			confirmed_cylinder_vector_.push_back(mean_vect);
			detected_rgb_vector_.push_back(rgb_mean_vect);
			std::cout << "Confirmed new cylinder: " << mean_vect[0] << " " << mean_vect[1] << " " << mean_vect[2] << std::endl;
			std::cout << "Cylinder sampled rgb vector: " << rgb_mean_vect[0] << " " << rgb_mean_vect[1] << " " << rgb_mean_vect[2] << std::endl;
			std::string name = find_closest_color(rgb_mean_vect);
			std::cout << "Cylinder color name: " << name << std::endl;

			for (int i = 0; i < 5; i++) {
				task2::poseWithCovarString msg;
				msg.pose_msg.pose.position.x = mean_vect[0];
				msg.pose_msg.pose.position.y = mean_vect[1];
				msg.pose_msg.pose.position.z = mean_vect[2];
				msg.pose_msg.pose.orientation.x = 0.0;
				msg.pose_msg.pose.orientation.y = 0.0;
				msg.pose_msg.pose.orientation.z = 0.0;
				msg.pose_msg.pose.orientation.w = 1.0;
				// std_msgs::String blyat;
				// blyat.data = name;
				msg.additional_info = name;
				pub_cylinder_pose.publish(msg);
			}

			// std::stringstream ss;
			// ss << "Detected " << name << " cylinder.";
			// soundhandle.say(name, "voice_kal_diphone", 1.0);
			// point_camera.point.x = mean_vect[0];
			// point_camera.point.y = mean_vect[1];
			// point_camera.point.z = mean_vect[2];

			std::string blya = "Detected " + name + " cylinder.";
			std_msgs::String blya2;

			// tf2::doTransform(point_camera, point_map, tss);
			blya2.data = blya;
			pub_vocalizer.publish(blya2);


			marker_confirmed_.header.frame_id = "map";
			marker_confirmed_.header.stamp = ros::Time::now();

			marker_confirmed_.ns = "cylinder_nsm_detection";
			marker_confirmed_.id = marker_id_++;

			marker_confirmed_.type = visualization_msgs::Marker::CYLINDER;
			marker_confirmed_.action = visualization_msgs::Marker::ADD;
			marker_confirmed_.pose.position.x = mean_vect[0];
			marker_confirmed_.pose.position.y = mean_vect[1];
			marker_confirmed_.pose.position.z = mean_vect[2];
			marker_confirmed_.pose.orientation.x = 0.0;
			marker_confirmed_.pose.orientation.y = 0.0;
			marker_confirmed_.pose.orientation.z = 0.0;
			marker_confirmed_.pose.orientation.w = 1.0;

			marker_confirmed_.scale.x = 0.15;
			marker_confirmed_.scale.y = 0.15;
			marker_confirmed_.scale.z = 0.15;

			marker_confirmed_.color.r = static_cast<float>(rgb_mean_vect[0])/255;
			marker_confirmed_.color.g = static_cast<float>(rgb_mean_vect[1])/255;
			marker_confirmed_.color.b = static_cast<float>(rgb_mean_vect[2])/255;
			marker_confirmed_.color.a = 1.0f;

			marker.lifetime = ros::Duration();
			// pubm.publish(marker_confirmed_);
			marker_array_.markers.push_back(marker_confirmed_);
			pubm_array.publish(marker_array_);
			non_maxima_supp_vector_.clear();
			non_maxima_rgb_vector_.clear();

			if(confirmed_cylinder_vector_.size() >= number_of_desired_cylinders_) {
				std_msgs::Int8 status;
				status.data = 2;
				for (int i = 0; i < 20; i++) {
					status_pub.publish(status);
					// std::cout << "========================= Cylinder detection is done. :DDD =========================" << std::endl;
				}
			}
		}

		pcl::PCLPointCloud2 outcloud_cylinder;
		pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
		puby.publish(outcloud_cylinder);
	}
}


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "cylinder_segment");
	ros::NodeHandle nh;

	// For transforming between coordinate frames
	tf2_ros::TransformListener tf2_listener(tf2_buffer);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
	puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);
	pub_cylinder_pose = nh.advertise<task2::poseWithCovarString>("/cylinder_position", 25);

	// Create a ROS publisher for the markers of detected cylinders and the array of all detected cylinders
	pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder", 1);
	pubm_array = nh.advertise<visualization_msgs::MarkerArray>("all_cylinders", 1);
	pub_vocalizer = nh.advertise<std_msgs::String>("/vocalizer_req", 10);

	// Create a ROS publisher for the status of the Cylinder detection.
	status_pub = nh.advertise<std_msgs::Int8>("/poster_and_cylinder_status", 1);

	// Spin
	ros::spin();
}