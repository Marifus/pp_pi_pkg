#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

namespace pp_pi
{

    class PP_PI
    {
        bool input_log, output_log;

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher ctrl_pub;
        ros::Publisher path_pub;
        ros::Publisher mark_pub;

        nav_msgs::Path path;
        nav_msgs::Odometry vehicle_odom;

        int marker_id;

        double velocity;
        double i_error;

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
/*         void PathCallback(const nav_msgs::Odometry::ConstPtr& msg); */
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        int ClosestWaypointIndex(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path);
        geometry_msgs::Pose ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, int closest_index);
        double PPPIAlgorithm(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, double t_axle_length, double t_Kpp, double t_Kp, double t_Ki);
        double CalculateLateralError(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int closest_idx);
        double CalculateLookaheadError(double heading_error, double lateral_error, double hipo);
        double LowPassFilter(double steering, std::vector<double> &steering_vec, int f_length, double w_current);
        double PurePursuitAlgorithm(double target_x, double target_y, double length);
        double CalculatePathYaw(const nav_msgs::Path& t_path, int closest_idx);
        geometry_msgs::Pose LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose);
            
        public:
            PP_PI(ros::NodeHandle& nh);

    };

}