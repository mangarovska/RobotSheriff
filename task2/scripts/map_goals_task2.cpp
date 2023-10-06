#include "ros/ros.h"
#include <string>
#include <numeric>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "tf2_ros/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <algorithm>
#include <cmath>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <costmap_2d/costmap_2d.h>

struct Point {
    float x;
    float y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};


using namespace std;
// global_variables have _ at the end:
int status;
string status_text;

bool kill = false;

/**
 * 3 Status variables for face, cylinder, and ring search
 * Redefine how many are expected to be found by each node, within respective files.
 **/
bool face_finished_status_ = false;
bool cylinder_finished_status_ = false;
bool ring_position_received_ = false;
bool robot_position_received_ = false;

bool searching_mode_  = true; 

geometry_msgs::Pose green_ring_pose_;
geometry_msgs::Pose ring_detected_robot_pose_;

visualization_msgs::MarkerArray current_centroids_;

// Map related globals.
uint32_t map_size_x_;
uint32_t map_size_y_;
float map_resolution_;
double map_origin_x_;
double map_origin_y_;
std::vector<int8_t> map_data_;

// Costmap related globals.
nav_msgs::OccupancyGrid cost_map_info_;
uint32_t map_size_x_costmap_;
uint32_t map_size_y_costmap_;
float map_resolution_costmap_;
double map_origin_x_costmap_;
double map_origin_y_costmap_;
std::vector<int8_t> map_data_costmap_;

costmap_2d::Costmap2D* costmap_;
// costmap = new costmap_2d::Costmap2D();


// clustering related globals:
std::vector<Point> unoccupied_;
std::vector<Point> centroids_;
std::vector<Point> ordered_centroids_;


double offsets_[] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.60};

// double x_[] = {-0.34814828634262085, -0.5487848520278931, -0.4885403513908386, // 1, 2.1, 2 
//                 0.06265157461166382, 0.06657689809799194, // 3, 4
//                 0.8998000621795654, 1.078585147857666, 3.0338735580444336, // 5.1, 5, 6
//                 3.033291816711426, 1.7951998710632324, 2.4665167331695557, // 7, 8, 9
//                 2.114750385284424, 1.20090913772583, 1.220787763595581, // 10, 11, 12
//                 2.3978078365325928, 0.40399789810180664, -1.1655912399291992 // 13, 14, 15
//                 }; 


// double y_[] = {-0.40090030431747437, -0.341474711894989, -0.10002802312374115,
//                  -0.4511810541152954, -1.8849139213562012, // 3, 4
//                 -1.8857204914093018, -1.8207522630691528, -1.5461450815200806, // 5, 6
//                 -1.3517465591430664, -0.5885991454124451, -0.5846586227416992,
//                 0.10487368702888489, 0.17758110165596008, -0.39124003052711487,
//                 1.4852874279022217, 1.7814254760742188, 1.1461007595062256
//                 };

// double ang_z_[] = {0.9999975403181238, -0.9648741897335182, 0.6106669073765557, // 1, 2.1, 2 
//                     -0.8399541368012626, 0.26959706656511967, // 3, 4
//                     -0.01221677072110597, -0.1677907384344494, -0.7989332524344922, // 5.1, 5, 6
//                     0.9762405205065298, 0.42623655295553564, 0.853075655790312, // 7, 8, 9
//                     -0.7715864885037318, -0.8125374313887092, 0.6940477388940999, // 10, 11, 12
//                     0.9954018972057143, -0.07263027118776717, 0.6652667413725667
//                     };

// double ang_w_[] = {0.002217962511495981, 0.2627123864344555, 0.791887572976842, // 1, 2.1, 2 
//                     0.5426573947440924, 0.9629732196169748, // 3, 4
//                     0.99992537247194, 0.9858226352116399, 0.6014197021668345, // 5.1, 5, 6
//                     0.21668974622981088, 0.9046117404304362, 0.5217872416013343, // 7, 8, 9
//                     0.6361244302481084, 0.5829090174223064, 0.7199289799250946, // 10, 11, 12
//                     0.09578654936505764, 0.9973589342394198, 0.7466057613115014
//                     };

std::vector<double> x_ = {-0.34814828634262085, -0.5487848520278931, -0.4885403513908386, // 1, 2.1, 2 
                0.06265157461166382, 0.06657689809799194, // 3, 4
                0.8998000621795654, 1.078585147857666, 3.0338735580444336, // 5.1, 5, 6
                3.033291816711426, 1.7951998710632324, 2.4665167331695557, // 7, 8, 9
                2.114750385284424, 1.20090913772583, 1.220787763595581, // 10, 11, 12
                2.3978078365325928, 0.40399789810180664, -1.1655912399291992 // 13, 14, 15
                }; 


std::vector<double>  y_ = {-0.40090030431747437, -0.341474711894989, -0.10002802312374115,
                -0.4511810541152954, -1.8849139213562012, // 3, 4
                -1.8857204914093018, -1.8207522630691528, -1.5461450815200806, // 5, 6
                -1.3517465591430664, -0.5885991454124451, -0.5846586227416992,
                0.10487368702888489, 0.17758110165596008, -0.39124003052711487,
                1.4852874279022217, 1.7814254760742188, 1.1461007595062256
                };


std::vector<double> ang_z_ = {0.9999975403181238, -0.9648741897335182, 0.6106669073765557, // 1, 2.1, 2 
                    -0.8399541368012626, 0.26959706656511967, // 3, 4
                    -0.01221677072110597, -0.1677907384344494, -0.7989332524344922, // 5.1, 5, 6
                    0.9762405205065298, 0.42623655295553564, 0.853075655790312, // 7, 8, 9
                    -0.7715864885037318, -0.8125374313887092, 0.6940477388940999, // 10, 11, 12
                    0.9954018972057143, -0.07263027118776717, 0.6652667413725667
                    };


std::vector<double> ang_w_ = {0.002217962511495981, 0.2627123864344555, 0.791887572976842, // 1, 2.1, 2 
                    0.5426573947440924, 0.9629732196169748, // 3, 4
                    0.99992537247194, 0.9858226352116399, 0.6014197021668345, // 5.1, 5, 6
                    0.21668974622981088, 0.9046117404304362, 0.5217872416013343, // 7, 8, 9
                    0.6361244302481084, 0.5829090174223064, 0.7199289799250946, // 10, 11, 12
                    0.09578654936505764, 0.9973589342394198, 0.7466057613115014
                    };


bool sent_ = false;
int idx_ = 0;
// int coordinate_count_ = sizeof(x_)/sizeof(x_[0]);
int coordinate_count_ = x_.size();

int primitive_timer_ = 0;
bool interrupt_flag_ = false;

int cell_x_ = 0;
int cell_y_ = 0;



void customRandomShuffle(std::vector<Point>& points) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = points.size() - 1; i > 0; --i) {
        std::uniform_int_distribution<> dis(0, i);
        int j = dis(gen);

        Point temp = points[i];
        points[i] = points[j];
        points[j] = temp;
    }
}

std::vector<Point> getOccupancyFreePoints() {
    std::vector<Point> occupancy_free_points;
    
    for (uint32_t y = 0; y < map_size_y_; ++y) {
        for (uint32_t x = 0; x < map_size_x_; ++x) {
            size_t index = y * map_size_x_ + x;
            if (map_data_[index] == 0 &&  5 >= map_data_costmap_[index] >= 0) {  // Assuming 0 represents occupancy free
                Point point;
                point.x = map_origin_x_ + (x + 0.5) * map_resolution_;
                point.y = map_origin_y_ + (y + 0.5) * map_resolution_;
                occupancy_free_points.push_back(point);
            }
        }
    }
    printf("occupancy_free_points: %ld\n", occupancy_free_points.size());
    return occupancy_free_points;
}


std::vector<Point> kMeansClustering(std::vector<Point>& points, int k) {
    std::vector<Point> centroids;
    
    // Initialize centroids randomly
    customRandomShuffle(points);
    for (int i = 0; i < k; ++i) {
        centroids.push_back(points[i]);
    }
    
    while (true) {
        std::vector<std::vector<Point>> clusters(k);
        
        // Assign each point to the nearest centroid
        for (const auto& point : points) {
            int nearest_centroid_index = 0;
            float min_distance = std::numeric_limits<float>::max();
            
            for (int i = 0; i < k; ++i) {
                float distance = std::sqrt(std::pow(point.x - centroids[i].x, 2) + std::pow(point.y - centroids[i].y, 2));
                
                if (distance < min_distance) {
                    nearest_centroid_index = i;
                    min_distance = distance;
                }
            }
            
            clusters[nearest_centroid_index].push_back(point);
        }
        
        // Update centroids
        std::vector<Point> new_centroids;
        
        for (const auto& cluster : clusters) {
            if (cluster.empty()) {
                // Skip empty clusters
                continue;
            }
            
            float sum_x = 0.0;
            float sum_y = 0.0;
            
            for (const auto& point : cluster) {
                sum_x += point.x;
                sum_y += point.y;
            }
            
            Point centroid;
            centroid.x = sum_x / cluster.size();
            centroid.y = sum_y / cluster.size();
            new_centroids.push_back(centroid);
        }
        
        // Check convergence
        if (new_centroids == centroids) {
            break;
        }
        
        centroids = new_centroids;
    }
    printf("centroids: %ld\n", centroids.size());
    return centroids;
}


bool map_init(ros::NodeHandle n) {
    while (!ros::service::waitForService("/static_map", ros::Duration(-1))) {
        ROS_INFO("Waiting for service static_map to become available");
    }
    ros::ServiceClient map_service_client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_map;
    if (map_service_client.call(srv_map)) {
        ROS_INFO("Successfuly received static_map information.");
        map_origin_x_ = srv_map.response.map.info.origin.position.x;
        map_origin_y_ = srv_map.response.map.info.origin.position.y;
        map_resolution_ = srv_map.response.map.info.resolution;
        map_size_x_ = srv_map.response.map.info.width;
        map_size_y_ = srv_map.response.map.info.height;
        map_data_ = srv_map.response.map.data;
        return true;
    }
  return false;
}

// This function converts a point in the map frame to a cell index
// then immediately checks if the occupancy grid is free at that cell
// returns true if the cell is free, and it's within map bounds.
bool isCellFree(double pointx, double pointy) {
  // Compute the cell index from the x,y coordinates of the point
  cell_x_ = (int)((pointx - map_origin_x_) / map_resolution_);
  cell_y_ = (int)((pointy - map_origin_y_) / map_resolution_);
  
  // Check if the cell index is within the bounds of the map
  if (cell_x_ < 0 || cell_x_ >= map_size_x_ || cell_y_ < 0 || cell_y_ >= map_size_y_) {
    ROS_WARN("Point (%.3f, %.3f) is outside the map", pointx, pointy);
    return false;
  }
  
  // Return true if the cell is free, and a valid goal by checking costmap
  return map_data_[cell_y_ * map_size_x_ + cell_x_] == 0 && map_data_costmap_[cell_y_ * map_size_x_ + cell_x_] == 0;
}
    

void messageReceived(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {	//The parameter is the message type
    if (msg -> status_list.size() > 0) {
        status = msg -> status_list[0].status;
        status_text = msg -> status_list[0].text;
        // printf("header seq: %d\n", msg->header.seq);
    } else { // no message received
        status = 10;
        status_text = "No message received from /move_base/status yet.";
    }
}
    // cout << msg -> status_list.size() << endl;


void killSigCallback(const std_msgs::String::ConstPtr& msg){
    kill = true;
}


std::vector<double> generate_valid_marker_goal() {
    for (int i = 0; i < sizeof(offsets_) / sizeof(offsets_[0]); i++) {
        // left
        double x_position = green_ring_pose_.position.x - offsets_[i];
        double y_position = green_ring_pose_.position.y;
        // int idx = static_cast<int>(x_position) + static_cast<int>(y_position * map_size_x_);
        if (isCellFree(x_position, y_position)) {
            std::cout << "SUCCESS: Found a point that I can post goal to: pose LEFT: " << x_position << ", " << y_position << std::endl;
            return std::vector<double> {x_position, y_position};
        }

        // right
        x_position = green_ring_pose_.position.x + offsets_[i];
        y_position = green_ring_pose_.position.y;
        // idx = static_cast<int>(x_position) + static_cast<int>(y_position * map_size_x_);
        if (isCellFree(x_position, y_position)) {
            std::cout << "SUCCESS: Found a point that I can post goal to: pose RIGHT: " << x_position << ", " << y_position << std::endl;
            return std::vector<double> {x_position, y_position};
        }

        // up
        x_position = green_ring_pose_.position.x;
        y_position = green_ring_pose_.position.y - offsets_[i];
        // idx = static_cast<int>(x_position) + static_cast<int>(y_position * map_size_x_);
        if (isCellFree(x_position, y_position)) {
            std::cout << "SUCCESS: Found a point that I can post goal to: pose UP " << x_position << ", " << y_position << std::endl;
            return std::vector<double> {x_position, y_position};
        }

        // down
        x_position = green_ring_pose_.position.x;
        y_position = green_ring_pose_.position.y + offsets_[i];
        // idx = static_cast<int>(x_position) + static_cast<int>(y_position * map_size_x_);
        if (isCellFree(x_position, y_position)){
            std::cout << "SUCCESS: Found a point that I can post goal to: pose DOWN " << x_position << ", " << y_position << std::endl;
            return std::vector<double> {x_position, y_position};
        }
    }
    std::cout<< "WARNING: Failed to find a valid goal point" << std::endl;
    return std::vector<double> {green_ring_pose_.position.x, green_ring_pose_.position.y};
}

geometry_msgs::Quaternion prepare_orientation(std::vector<double> goal) {
    // ring_detected_robot_pose_;
    // green_ring_pose_
    std::vector<double> position_diff = {green_ring_pose_.position.x - goal[0], green_ring_pose_.position.y - goal[1], 0};
    double norm = sqrt(accumulate(position_diff.begin(), position_diff.end(), 0.0, [](double sum, double x) { return sum + x*x; }));
    for (auto& x : position_diff) {
        x /= norm;
    }
    double yaw = atan2(position_diff[1], position_diff[0]);

    auto desired_orientation = tf::createQuaternionMsgFromYaw(yaw);

    return desired_orientation;
}

void cost_map_init(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid) {
    printf("AYO I'M IN THE INIT\n");
    cost_map_info_ = *occupancyGrid;
    map_size_x_costmap_ = cost_map_info_.info.width;
    printf("size_x: %d\n", map_size_x_costmap_);
    map_size_y_costmap_ = cost_map_info_.info.height;
    printf("size_y: %d\n", map_size_y_costmap_);
    map_resolution_costmap_ = cost_map_info_.info.resolution;
    map_origin_x_costmap_ = cost_map_info_.info.origin.position.x;
    map_origin_y_costmap_ = cost_map_info_.info.origin.position.y;
    map_data_costmap_ = cost_map_info_.data;  
    // costmap_ = new costmap_2d::Costmap2D("global_costmap", n);
    ROS_INFO("Costmap received");
}

void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid) {
    printf("AYO I'M IN THE CALLBACK\n");
    cost_map_info_ = *occupancyGrid;
    map_size_x_costmap_ = cost_map_info_.info.width;
    printf("size_x: %d\n", map_size_x_costmap_);
    map_size_y_costmap_ = cost_map_info_.info.height;
    printf("size_y: %d\n", map_size_y_costmap_);
    map_resolution_costmap_ = cost_map_info_.info.resolution;
    map_origin_x_costmap_ = cost_map_info_.info.origin.position.x;
    map_origin_y_costmap_ = cost_map_info_.info.origin.position.y;
    map_data_costmap_ = cost_map_info_.data;
    ROS_INFO("Costmap received");
}

bool perform_line_of_sight_check(double start_x, double start_y, double end_x, double end_y)
{
  if (map_data_costmap_.empty())
  {
    ROS_WARN("Costmap not available. Cannot perform line-of-sight check.");
    return false;
  }

  // Convert start and end points to map coordinates
  double resolution = map_resolution_costmap_;
  double origin_x = map_origin_x_costmap_;
  double origin_y = map_origin_y_costmap_;
  int map_size_x = map_size_x_costmap_;
  int map_size_y = map_size_y_costmap_;

  int start_map_x = static_cast<int>((start_x - origin_x) / resolution);
  int start_map_y = static_cast<int>((start_y - origin_y) / resolution);
  int end_map_x = static_cast<int>((end_x - origin_x) / resolution);
  int end_map_y = static_cast<int>((end_y - origin_y) / resolution);

  // Check if the start and end points are within the map boundaries
  if (start_map_x < 0 || start_map_x >= map_size_x || start_map_y < 0 || start_map_y >= map_size_y ||
      end_map_x < 0 || end_map_x >= map_size_x || end_map_y < 0 || end_map_y >= map_size_y)
  {
    ROS_WARN("Start or end point is outside the costmap boundaries.");
    return false;
  }

  // Calculate the index of the start and end points in the map data array
  int start_index = start_map_y * map_size_x + start_map_x;
  int end_index = end_map_y * map_size_x + end_map_x;

  // Check if any cell along the line is an obstacle

  int8_t* map_data = map_data_.data();
  int index = start_index;

  int dx = abs(end_map_x - start_map_x);
  int dy = abs(end_map_y - start_map_y);
  int sx = (start_map_x < end_map_x) ? 1 : -1;
  int sy = (start_map_y < end_map_y) ? 1 : -1;
  int err = dx - dy;

  while (true)
  {
    // Check if the current cell is an obstacle
    if (map_data[index] >= 1)
    {
      return false;
    }

    // Break the loop if the end point is reached
    if (index == end_index)
    {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      index += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      index += sy * map_size_x;
    }
  }

  return true;
}

void face_and_cylinder_status_callback(const std_msgs::Int8::ConstPtr& msg) {
    int status_received = (int)msg->data;
    if (status_received == 1) {
        face_finished_status_ = true;
    } else if (status_received == 2) {
        cylinder_finished_status_ = true;
    }
    std::cout << "=================================== face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_ : " << cylinder_finished_status_ << " ===================================" <<std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
    std::cout << "face_finished_status_: " << face_finished_status_ << "cylinder_finished_status_: " << cylinder_finished_status_<< std::endl;
}


void ring_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // <20 for ring
    green_ring_pose_ = msg->pose;
    ring_position_received_ = true;

    // std::cout << "RING POSITION CALLBACK:--- green_ring_pose_:" << std::endl;
    std::cout << "RING POSITION CALLBACK:--- green_ring_pose_:\n" << green_ring_pose_ << std::endl; 

}


void robot_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // <20 for ring
    ring_detected_robot_pose_ = msg->pose;
    robot_position_received_ = true;

    // std::cout << "ROBOT POSITION CALLBACK:--- ring_detected_robot_pose_:" << std::endl;
    std::cout << "ROBOT POSITION CALLBACK:--- ring_detected_robot_pose_:\n" << ring_detected_robot_pose_ << std::endl;
}

geometry_msgs::PoseStamped goal_creator(uint8_t use_scenario = 0) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = ang_w_[idx_];
    goal.pose.orientation.z = ang_z_[idx_];
    goal.pose.position.x = x_[idx_];
    goal.pose.position.y = y_[idx_];
    goal.header.stamp = ros::Time::now();
    return goal;
}

/*
* If status is reached and interrupt flag has been raised
* then reset the interrupt flag, allow map_goals to take over
* and reset the timer
*/
void try_resetting_interrupt_flag() {
    if (status == 3 && interrupt_flag_) {
        interrupt_flag_ = false;
        primitive_timer_ = 0;
    }
}

/**
 * @brief Interrupt handler for the robot
 * If status is 2 and a message has been sent, then we can assume a previous move command
 * has been interrupted and we need to wait for the robot to stop moving before sending
 * this is achieved by raising interrupt_flag_ and waiting for 3 seconds. 
 **/
void interrupt_handler() {
    if (status == 2 && sent_ == true) {
        interrupt_flag_ = true;
        sent_ = false;
        primitive_timer_ = 0;

        ros::Duration(3, 0).sleep();
    }
}

/*
* If status represents a pending status (1, 2) and a message has been sent_tfBuffer
* then we can safely assume status is going to be <3 until next point is reached
* So we can reset sent_ to false and increment index to the next goal, making sure we don't
* overflow over the array bounds of x_, y_, ang_z_, ang_w_
*/
void prepare_next_goal() {
    if (sent_ && status <= 2) {
        sent_ = false;
        idx_ += 1;
        primitive_timer_ = 0;
        printf("idx_: %f\n", floor(coordinate_count_ / 10));
        printf("idx_ but int: %d\n", (int) floor(coordinate_count_ / 10));

        if(idx_ == coordinate_count_) {
            // auto receivedMessage = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap");
            // cost_map_init(receivedMessage);
            // unoccupied_ = getOccupancyFreePoints();
            // centroids_ = kMeansClustering(unoccupied_, coordinate_count_);
            idx_ = 0;
        }
    }
}


void undefined_state_resolver() {
    if (sent_ && status == 3) {
        if (primitive_timer_ >= 32) {
            printf("Status == 3 (no 1 or 2 received) and sent_ == 1, initializing logic reset");
            sent_ = false;
            primitive_timer_ = 0;
            idx_++;
        }
        primitive_timer_ += 1;
    }
}

// you know what this is :DDD
double generateRandomYaw() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-M_PI, M_PI);  // Range of -180 to 180 degrees in radians

    return dis(gen);
}


tf2::Quaternion generateRandomYawQuaternion() {
    double randomYaw = generateRandomYaw();  // Assume generateRandomYaw() is implemented as before

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, randomYaw);

    return q;
}

// Function to compute distance between two points
double distance(const Point& p1, const Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}


bool pointComparator(const Point& p1, const Point& p2) {
    if (p1.x == p2.x) {
        return p1.y < p2.y;
    }
    return p1.x < p2.x;
}


void __temp_add_me_my_clusters_pls() {
    current_centroids_.markers.clear();
    for (size_t i = 0; i < centroids_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Set the frame ID appropriately
        marker.header.stamp = ros::Time::now();
        marker.ns = "centroids";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = centroids_[i].x;
        marker.pose.position.y = centroids_[i].y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        current_centroids_.markers.push_back(marker);
    }
}

void __override_previous_list() {    
    printf("og_centroids?\n");
    for (const auto& point : centroids_) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    } // all different


    std::sort(centroids_.begin(), centroids_.end(), pointComparator);
    printf("\nsorted_centroids?\n");
    for (const auto& point : centroids_) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    } // all different

    ordered_centroids_.push_back(centroids_[0]);  // Start with the first centroid
    centroids_.erase(centroids_.begin());

    // ordered_centroids_.push_back(centroids_[0]);  // Start with the first centroid
    // centroids_.erase(centroids_.begin());  // Remove the first centroid from the original vector
    while (ordered_centroids_.size() < coordinate_count_) { // from first to last.
        float min = 9999.99;
        int min_jdx = 0;
        for(int jdx = 0; jdx < centroids_.size(); jdx++) {
            printf("jdx: %d\n", jdx);
            bool not_walled = perform_line_of_sight_check(ordered_centroids_.back().x, ordered_centroids_.back().y, centroids_[jdx].x, centroids_[jdx].y);
            float curr_distance = distance(ordered_centroids_.back(), centroids_[jdx]);

            if (!not_walled) {
                curr_distance = (curr_distance+1)*1.5;
            }
            if (curr_distance < min && curr_distance != 0.0) {
                printf("not_walled: %d\n", not_walled);
                min = curr_distance;
                min_jdx = jdx;
            }
        }
        printf("min:%f\nmin_jdx: %d\n", min, min_jdx);
        ordered_centroids_.push_back(centroids_[min_jdx]);
        centroids_.erase(centroids_.begin() + min_jdx);
    }
    printf("\nsorted????\n");
    for (const auto& point : ordered_centroids_) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }

    std::cout << std::endl;
    x_.clear();
    y_.clear();
    ang_z_.clear();
    ang_w_.clear();
    for (int i = 0; i < coordinate_count_; i++) {
        // x_[i] = ordered_centroids_[i].x;
        x_.push_back(ordered_centroids_[i].x);
        // y_[i] = ordered_centroids_[i].y;
        y_.push_back(ordered_centroids_[i].y);
        double randomYaw = generateRandomYaw();
        tf2::Quaternion randomQuaternion = generateRandomYawQuaternion();
        // ang_z_[i] = randomQuaternion.z();
        ang_z_.push_back(randomQuaternion.z());
        // ang_w_[i] = randomQuaternion.w();
        ang_w_.push_back(randomQuaternion.w());

    }
}   


int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;
    map_init(n); // This actually prepares our global map information
    ros::Subscriber costmap_sub = n.subscribe("/move_base/global_costmap/costmap", 10, &costmap_callback);
    auto receivedMessage = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap");
    cost_map_init(receivedMessage);
    // map_sub = n.subscribe("map", 10, &mapCallback);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // ros::Publisher plus_marker = n.advertise<visualization_msgs::MarkerArray>("/all_plus", 10);
    ros::Publisher detect_floor_initalize_pub = n.advertise<std_msgs::Int8>("/detect_floor/init", 10);
    ros::Publisher centroids_pub = n.advertise<visualization_msgs::MarkerArray>("/all_centroids", 30);



    ros::Subscriber robot_location_sub = n.subscribe("/ring_status/robot", 20, &robot_position_callback);
    ros::Subscriber ring_location_sub = n.subscribe("/ring_status/ring", 20, &ring_position_callback);
    ros::Subscriber face_and_cylinder_status_sub = n.subscribe("/face_and_cylinder_status", 20, &face_and_cylinder_status_callback);
    ros::Subscriber kill_receive = n.subscribe("/kill_signal", 10, &killSigCallback);
    ros::Subscriber status_sub = n.subscribe("/move_base/status", 1, &messageReceived);

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tfListener_(tf2_buffer_);


    
    ros::Rate r(2);

    int final_goal_sent_counter = 0;
    int first = 0;

    unoccupied_ = getOccupancyFreePoints();
    coordinate_count_ = 15;
    centroids_ = kMeansClustering(unoccupied_, coordinate_count_);

    __temp_add_me_my_clusters_pls();

    __override_previous_list();

    std::vector<double> new_goal;

    while(ros::ok()) {
        if (searching_mode_) {
            if (kill){
                printf("Killing map_goals node\n");
                break;
            }

            try_resetting_interrupt_flag();

            interrupt_handler();

            /*
            * If status represents a finished status (3, 4, 5, 6, 7, 8, 9) and a message has NOT been sent_, 
            * interrupt flag is not raised, then we can prepare a new goal, publish it and set sent_ to true,
            * this is because the assumption that the robot is currently not moving is a safe one.
            * Note, if there is no response from /move_base/status, we create a message locally, with a special status
            * code of 10, and a status text of "No message received from /move_base/status yet."
            * For more detail see messageReceived callback function above.
            */
           searching_mode_ = !(ring_position_received_ && robot_position_received_); // && cylinder_finished_status_);

            if (status >= 3 && !sent_ && !interrupt_flag_ && searching_mode_) { // !interrupt_flag_ 
                ros::Duration(2, 0).sleep();
                if (idx_ != 0){
                    ROS_INFO("status code: %d (%s)", int(status), status_text.c_str());
                }
                geometry_msgs::PoseStamped goal = goal_creator();
                goal_pub.publish(goal);
                ROS_INFO("sent_ goal #%d: x_=%f, y_=%f", int(idx_+1), x_[idx_], y_[idx_]);
                sent_ = true;
                primitive_timer_ = 0;
                // if (status == 3) status=10;
                // primitive_timer_ = 0;
            }

            /*
            * If status represents a pending status (1, 2) and a message has been sent_
            * then we can safely assume status is going to be <3 until next point is reached
            * So we can reset sent_ to false and increment index to the next goal, making sure we don't
            * overflow over the array bounds of x_, y_, ang_z_, ang_w_
            */
            prepare_next_goal();

            // UNDEF STATE:
            // handle undefined state that occurs due to async nature of ROS
            // default action shouldn't trigger this as status == 3 and sent_ == 1 should be impossible
            // see above, as we immediately set sent_ to false if status is pending resolvement
            // How can goal be reached without initiliazing status = 1 or status = 2?
            undefined_state_resolver();

            printf("idx_: %d, status: %d, sent_: %d\n", idx_, status, sent_);

            primitive_timer_ += 1;
            searching_mode_ =  !(ring_position_received_ && robot_position_received_); // (face_finished_status_ && cylinder_finished_status_ && ring_finished_status_);
        } else if (!searching_mode_) {
            /**
            * Success. All base functions have been completed.
            * We can now continue to try and park at our final destination, underneath the green ring.
            * We received information about robot position and ring position from the /ring_status topic callback.
            * Now we can construct a position somewhere down the middle, and send our robot there, turning it the right direction, so as to simplify
            * the discovery the ground ring.
            **/
            if (first==0) {

                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.pose.orientation.w = ring_detected_robot_pose_.orientation.w;
                goal.pose.orientation.z = ring_detected_robot_pose_.orientation.z;
                goal.pose.position.x=  ring_detected_robot_pose_.position.x;//(green_ring_pose_.position.x + ring_detected_robot_pose_.position.x)/1.8;
                goal.pose.position.y = ring_detected_robot_pose_.position.y;//(green_ring_pose_.position.y + ring_detected_robot_pose_.position.y)/1.8;
                goal.header.stamp = ros::Time::now();
                std::cout << "STOP: x_=" << goal.pose.position.x << ", y_=" << goal.pose.position.y << std::endl;
                goal_pub.publish(goal);
                sent_ = true;

                // prepare the next one
                // ring_detected_robot_pose_.position.x = goal.pose.position.x;
                // ring_detected_robot_pose_.position.y = goal.pose.position.y;
                primitive_timer_ = 0;
                first++;
                ros::Duration(3, 0).sleep();
            } else if (first < 5) {
                new_goal = generate_valid_marker_goal();
                first++;
            } else if (first < 10) {
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "map";
                auto direction = prepare_orientation(new_goal);
                // goal.pose.orientation.w = ring_detected_robot_pose_.orientation.w;
                // goal.pose.orientation.z = ring_detected_robot_pose_.orientation.z;
                goal.pose.orientation = direction;
                goal.pose.position.x=  new_goal[0]; //(green_ring_pose_.position.x + ring_detected_robot_pose_.position.x)/1.8;
                goal.pose.position.y = new_goal[1]; //(green_ring_pose_.position.y + ring_detected_robot_pose_.position.y)/1.8;
                goal.header.stamp = ros::Time::now();
                std::cout << "sent final goal: x_=" << goal.pose.position.x << ", y_=" << goal.pose.position.y << std::endl;
                goal_pub.publish(goal);
                sent_ = true;
                first++;
            } else {
                if (status == 3) {
                ros::Duration(2, 0).sleep();
                first = 10;
                std_msgs::Int8 msg;
                msg.data = 1;
                detect_floor_initalize_pub.publish(msg);
                break;
                }
            }
        }
        centroids_pub.publish(current_centroids_);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}