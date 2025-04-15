#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cmath>
#include <vector>

namespace pp_pi
{

    class PP_PI
    {

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher ctrl_pub;
        ros::Publisher path_pub;

        nav_msgs::Path path;
        nav_msgs::Odometry vehicle_odom;

        double velocity = 10;
        double i_error = 0;

        double lookahead_distance;
        double axle_length;
        double Kpp;
        double Kp;
        double Ki;
        double weight_current;
        int filter_length;
        std::vector<double> prev_steerings;

        bool ReadParameters();
        double GetYaw(geometry_msgs::Quaternion& q);
        void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        void LocalTransform(geometry_msgs::Pose& current_point_pose, geometry_msgs::Pose& target_point_pose, double transformed_vector[3]);
        int ClosestWaypointIndex(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path);
        geometry_msgs::Pose ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, int closest_index);
        double PPPIAlgorithm(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, double t_axle_length, double t_Kpp, double t_Kp, double t_Ki);
        double CalculateLookaheadError(double heading_error, double lateral_error, double hipo);
        double LowPassFilter(double steering, std::vector<double> &steering_vec, int f_length, double w_current);
        double PurePursuitAlgorithm(double target_x, double target_y, double length);
            

        public:
            PP_PI(ros::NodeHandle& nh);

    };

}