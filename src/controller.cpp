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

        i_error = 0;
        marker_id = 0;

        path_sub = nh_.subscribe("/shortest_path", 10, &PP_PI::PathCallback, this);
        odom_sub = nh_.subscribe("/odom", 10, &PP_PI::OdomCallback, this);
        ctrl_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);
        mark_pub = nh_.advertise<visualization_msgs::Marker>("/waypoint", 10);

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
        if (!nh_.getParam("velocity", velocity)) return false;
        if (!nh_.getParam("input_log", input_log)) return false;
        if (!nh_.getParam("output_log", output_log)) return false;

        ROS_INFO("Lookahead Distance: [%f]", lookahead_distance);
        ROS_INFO("PP_PI Katsayilari: [%f, %f, %f]", Kpp, Kp, Ki);
        ROS_INFO("Filtre Katsayisi: [%f]", weight_current);
        ROS_INFO("Filtre Uzunlugu: [%d]", filter_length);
        ROS_INFO("Arac Hizi: [%f]", velocity);
        ROS_INFO("Girdi/Cikti Gosterme: [%d, %d]", input_log, output_log);

        return true;
    }


    double PP_PI::GetYaw(geometry_msgs::Quaternion& q)
    {
        double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return yaw;
    }

/* 
    void PP_PI::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    }
*/

    void PP_PI::PathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path = *msg;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            if (i+1<path.poses.size())
            {
                while (path.poses[i].pose.position.x == path.poses[i+1].pose.position.x && path.poses[i].pose.position.y == path.poses[i+1].pose.position.y)
                {
                    path.poses.erase(path.poses.begin()+i);
                    if (i+1>=path.poses.size()) break;
                }
            }
        }
    }


    void PP_PI::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;
        double current_heading = GetYaw(vehicle_odom.pose.pose.orientation);

        vehicle_odom.pose.pose.position.x += cos(current_heading)*axle_length*0.5;
        vehicle_odom.pose.pose.position.y += sin(current_heading)*axle_length*0.5;

        if(input_log)
        {
            ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
            ROS_INFO("Simdiki Yaw: [%f]", current_heading);
        }

        if (path.poses.size() != 0) ControlOutput();

        else
        {
            autoware_msgs::VehicleCmd ctrl_msg;
            ctrl_msg.header.stamp = ros::Time::now();
            ctrl_msg.twist_cmd.twist.linear.x = 0;
            ctrl_msg.twist_cmd.twist.angular.z = 0;
            ctrl_pub.publish(ctrl_msg);
        }
    }


    void PP_PI::ControlOutput()
    {
        double steering_angle = LowPassFilter(PPPIAlgorithm(vehicle_odom.pose.pose, path, lookahead_distance, axle_length, Kpp, Kp, Ki), prev_steerings, filter_length, weight_current);

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) steering_angle = 100 * (M_PI / 180);
        else if (steering_angle_degree < -100) steering_angle = -100 * (M_PI / 180);

        if(output_log)
        {
            ROS_INFO("Donus Acisi: [%f]", steering_angle);
        }

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity;
        ctrl_pub.publish(control_msg);
    }


    double PP_PI::PPPIAlgorithm(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, double t_axle_length, double t_Kpp, double t_Kp, double t_Ki)
    {
/*         int closest_point_index = ClosestWaypointIndex(current_point_pose, t_path); */
        int closest_point_index = 0;
        geometry_msgs::Pose pp_target_pose = ChooseLookaheadPoint(current_point_pose, t_path, t_lookahead_distance, closest_point_index);

        geometry_msgs::Pose transformed_pp_target_pose = LocalTransform(current_point_pose, pp_target_pose);

        double pp_steering = PurePursuitAlgorithm(transformed_pp_target_pose.position.x, transformed_pp_target_pose.position.y, t_axle_length);

        geometry_msgs::Pose closest_pose = t_path.poses[closest_point_index].pose;

        geometry_msgs::Pose transformed_closest_pose = LocalTransform(current_point_pose, closest_pose);

        double current_yaw = GetYaw(current_point_pose.orientation);
        double path_yaw = CalculatePathYaw(t_path, closest_point_index);

        double heading_error = current_yaw - path_yaw;

        double lateral_error = CalculateLateralError(current_point_pose, t_path, closest_point_index);

        double lookahead_error = CalculateLookaheadError(heading_error, lateral_error, (axle_length/2 + t_lookahead_distance));
        i_error += lateral_error;

        double pppi_steering = t_Kpp * pp_steering + t_Kp * lookahead_error + t_Ki * i_error;
        return pppi_steering;
    }


    double PP_PI::CalculateLateralError(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int closest_idx)
    {
        double path_yaw = CalculatePathYaw(t_path, closest_idx);

        geometry_msgs::Pose closest_point_pose = t_path.poses[closest_idx].pose;

        tf2::Transform transform, closest_point_tf;
        tf2::fromMsg(closest_point_pose, closest_point_tf);

        tf2::Vector3 translation(current_point_pose.position.x, current_point_pose.position.y, 0);
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, path_yaw);

        transform.setOrigin(translation);
        transform.setRotation(rotation);

        tf2::Transform transformed_tf = transform.inverse() * closest_point_tf;

        double lateral_error = transformed_tf.getOrigin().y();

        return lateral_error;
    }


    double PP_PI::CalculateLookaheadError(double heading_error, double lateral_error, double hipo)
    {
        double lookahead_error = lateral_error+hipo*sin(-heading_error);
        return lookahead_error;
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


    geometry_msgs::Pose PP_PI::ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, int closest_index) 
    {
        int chosen_point_index = closest_index;
        geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];

        double lookahead_distance2 = std::pow(t_lookahead_distance, 2);
        double distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);

        while (lookahead_distance2 > distance2) 
        {
            chosen_point_index++;
            geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];
            distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);
        }

        geometry_msgs::Pose& chosen_point = path.poses[chosen_point_index].pose;

        visualization_msgs::Marker marker;
        marker.header = t_path.header;
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = chosen_point.position;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
        mark_pub.publish(marker);

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


    double PP_PI::CalculatePathYaw(const nav_msgs::Path& t_path, int closest_idx)
    {
        int second_idx = closest_idx+1;
        double path_yaw;

        while (t_path.poses[second_idx].pose.position.y == t_path.poses[closest_idx].pose.position.y && t_path.poses[second_idx].pose.position.x == t_path.poses[closest_idx].pose.position.x)
        {
            if (second_idx<closest_idx) second_idx--;

            else {
                second_idx++;
                if (!(second_idx<t_path.poses.size())) second_idx = closest_idx-1;
            }
        }
        
        if (second_idx<closest_idx) path_yaw = std::atan2(t_path.poses[closest_idx].pose.position.y - t_path.poses[second_idx].pose.position.y, t_path.poses[closest_idx].pose.position.x - t_path.poses[second_idx].pose.position.x);
        else path_yaw = std::atan2(t_path.poses[second_idx].pose.position.y - t_path.poses[closest_idx].pose.position.y, t_path.poses[second_idx].pose.position.x - t_path.poses[closest_idx].pose.position.x);

        return path_yaw;
    }


    geometry_msgs::Pose PP_PI::LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose)
    {   
        tf2::Transform origin_tf, target_point_tf, g2l_transform, transformed_point_tf;

        tf2::fromMsg(origin_pose, origin_tf);
        g2l_transform = origin_tf.inverse();

        tf2::fromMsg(target_point_pose, target_point_tf);

        transformed_point_tf = g2l_transform * target_point_tf;
        
        geometry_msgs::Pose transformed_pose;
        tf2::toMsg(transformed_point_tf, transformed_pose);

        return transformed_pose;
    }

}