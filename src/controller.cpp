#include "pp_pi_pkg/controller.hpp"

namespace pp_pi 
{

    PP_PI::PP_PI(ros::NodeHandle& nh) : nh_(nh)
    {

        if(!ReadParameters())
        {
            ROS_ERROR("Parametreler Okunamadi.");
            ros::requestShutdown();
        }

        path_sub = nh_.subscribe("/odom", 10, &PP_PI::PathCallback, this);
        odom_sub = nh_.subscribe("/odom_sim", 10, &PP_PI::OdomCallback, this);
        ctrl_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);

    }

    bool PP_PI::ReadParameters()
    {

        if (!nh_.getParam("lookahead_distance", lookahead_distance)) return false;
        if (!nh_.getParam("axle_length", axle_length)) return false;
        if (!nh_.getParam("Kpp", Kpp)) return false;
        if (!nh_.getParam("Kp", Kp)) return false;
        if (!nh_.getParam("Ki", Ki)) return false;
        if (!nh_.getParam("weight_current", weight_current)) return false;
        if (!nh_.getParam("filter_length", filter_length)) return false;

        ROS_INFO("Lookahead Distance: %f", lookahead_distance);
        ROS_INFO("PP_PI Katsayilari: [%f, %f, %f]", Kpp, Kp, Ki);
        ROS_INFO("Filtre Katsayisi: [%f]", weight_current);
        ROS_INFO("Filtre Uzunlugu: [%d]", filter_length);

        return true;
    }

    double PP_PI::GetYaw(geometry_msgs::Quaternion& q)
    {
        double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return yaw;
    }

    void PP_PI::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;
        pose_stamped.header = msg->header;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    }

    void PP_PI::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;

        geometry_msgs::Quaternion& q = vehicle_odom.pose.pose.orientation;
        double current_heading = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);

        ControlOutput();
    }

    void PP_PI::ControlOutput()
    {

        double steering_angle = LowPassFilter(PPPIAlgorithm(vehicle_odom.pose.pose, path, lookahead_distance, axle_length, Kpp, Kp, Ki), prev_steerings, filter_length, weight_current);

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 50) {

            steering_angle = 50 * (M_PI / 180);

        }

        if (steering_angle_degree < -50) {

            steering_angle = -50 * (M_PI / 180);

        }

        ROS_INFO("Donus Acisi: [%f]", steering_angle);

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity;
        ctrl_pub.publish(control_msg);

    }

    double PP_PI::PPPIAlgorithm(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, double t_axle_length, double t_Kpp, double t_Kp, double t_Ki)
    {

        geometry_msgs::Pose pp_target_pose = ChooseLookaheadPoint(current_point_pose, t_path, t_lookahead_distance);
        double transformed_pp_target_vec[3] = {0, 0, 0};
        LocalTransform(current_point_pose, pp_target_pose, transformed_pp_target_vec);

        double pp_steering = PurePursuitAlgorithm(transformed_pp_target_vec[0], transformed_pp_target_vec[1], t_axle_length);

        geometry_msgs::Pose closest_pose = t_path.poses[ClosestWaypointIndex(current_point_pose, t_path)].pose;
        double transformed_closest_pose[3] = {0, 0, 0};
        LocalTransform(current_point_pose, closest_pose, transformed_closest_pose);

        double current_yaw = GetYaw(current_point_pose.orientation);
        double path_yaw = GetYaw(closest_pose.orientation);

        double heading_error = current_yaw - path_yaw;
        double lateral_error = std::sqrt(std::pow(transformed_closest_pose[0], 2) + std::pow(transformed_closest_pose[1], 2));
        if (sin(-heading_error) < 0) lateral_error = -lateral_error;
        double lookahead_error = CalculateLookaheadError(heading_error, lateral_error, (axle_length/2 + t_lookahead_distance));

        double pppi_steering = t_Kpp * pp_steering + t_Kp * lookahead_error + t_Ki * i_error;
        i_error += lateral_error;
        return pppi_steering;
    }

    double PP_PI::CalculateLookaheadError(double heading_error, double lateral_error, double hipo)
    {
        double lookahead_error = lateral_error+hipo*sin(-heading_error);
        return lookahead_error;
    }

    void PP_PI::LocalTransform(geometry_msgs::Pose& current_point_pose, geometry_msgs::Pose& target_point_pose, double transformed_vector[3])
    {
        
        double tx = -1 * current_point_pose.position.x;
        double ty = -1 * current_point_pose.position.y;

        double current_heading_ = GetYaw(current_point_pose.orientation);

        double target_vec[3] = {target_point_pose.position.x, target_point_pose.position.y, 1};

        double TransformationMatrix[3][3] = {
            {cos(-current_heading_), -sin(-current_heading_), cos(-current_heading_) * tx - sin(-current_heading_) * ty},
            {sin(-current_heading_), cos(-current_heading_), sin(-current_heading_) * tx + cos(-current_heading_) * ty},
            {0.0, 0.0, 1.0}
        };

        for (int i=0; i<3; i++) {
        
            transformed_vector[i] = 0;

            for (int j=0; j<3; j++) {

                transformed_vector[i] += TransformationMatrix[i][j] * target_vec[j];

           }
        }

    }

    int PP_PI::ClosestWaypointIndex(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path)
    {
        int closest_point_index;
        double min_distance2;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped& waypoint = path.poses[i];

            if (i == 0)
            {
                closest_point_index = i;
                min_distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);
            }

            else 
            {
                double distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);

                if (distance2 < min_distance2)
                {
                    closest_point_index = i;
                    min_distance2 = distance2;
                }
            }
        }

        return closest_point_index;
    }

    geometry_msgs::Pose PP_PI::ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance) 
    {

        int chosen_point_index = ClosestWaypointIndex(current_point_pose, path);
        geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];

        double distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));

        while (t_lookahead_distance > distance) 
        {

            chosen_point_index++;
            geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];
            distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));

        }

        chosen_point_index--;

        geometry_msgs::Pose& chosen_point = path.poses[chosen_point_index].pose;
        return chosen_point;
    }

    double PP_PI::PurePursuitAlgorithm(double target_x, double target_y, double length)
    {
        double distance2 = (target_x * target_x) + (target_y * target_y);

        double steering = atan2((2*target_y*length), (distance2));

        return steering;
    }

    double PP_PI::LowPassFilter(double steering, std::vector<double> &steering_vec, int f_length, double w_current)
    {
        double sum = 0;

        steering_vec.push_back(steering);
        if (steering_vec.size() > f_length)
        {
            for (int i=1; i<steering_vec.size(); i++) steering_vec[i-1] = steering_vec[i];
            steering_vec.pop_back();
        }

        for (int i=0; i<(steering_vec.size()-1); i++) sum+= steering_vec[i];

        double w_prev = (1 - w_current) / (steering_vec.size()-1);
        return ((sum * w_prev) + (steering * w_current));
    }

}