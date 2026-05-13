#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <numeric>
//Servicio:
#include "seg_tray_msgs/srv/corner_service.hpp"

class CorridorNavigationNode : public rclcpp::Node
{
public:
    CorridorNavigationNode() : Node("corr_nav_node")
    {
        // Load parameters
        loadParameters();

        RCLCPP_INFO(this->get_logger(), "CorridorNavigationNode started");

        // Publisher for cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
        // Subscriber for robot pose
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom", 10,
            std::bind(&CorridorNavigationNode::odomCallback, this, std::placeholders::_1));

        // Subscriber for laser scan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan", 10,
            std::bind(&CorridorNavigationNode::laserCallback, this, std::placeholders::_1));

        
        //Para poder tener los dos callbacks a la vez:
        callback_group_timer_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_service_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        //Servicio:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&CorridorNavigationNode::controlLoop, this),
            callback_group_timer_);

        corner_service_ = this->create_service<seg_tray_msgs::srv::CornerService>(
            "corner_service",
            std::bind(&CorridorNavigationNode::cornerCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_service_);

        corner_client_ = this->create_client<seg_tray_msgs::srv::CornerService>("corner_service");

        RCLCPP_INFO(this->get_logger(), "Corridor Navigation Node Initialized");
        measured_data_=false;
    }

private:
    void loadParameters()
    {
         // Declare parameters of this node (name, initial_value)
        this->declare_parameter("time_step", 25);  // in milliseconds
        this->declare_parameter("max_linear_speed", 1.2);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("wheel_base", 0.331);
        this->declare_parameter("wheel_radius", 0.097518);
        this->declare_parameter("corridor_width", 10.0);  // meters
        this->declare_parameter("look_ahead_distance", 1.0);  // meters 
        // Read parameters
        time_step_ = this->get_parameter("time_step").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        corridor_width_ = this->get_parameter("corridor_width").as_double();
        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();

        /*RCLCPP_INFO(this->get_logger(), 
            "max_linear_speed: %.2f, max_angular_speed: %.2f, wheel_base: %.2f, wheel_radius: %.2f, corridor_width: %.2f, look_ahead_distance: %.2f", 
            max_linear_speed_, max_angular_speed_, wheel_base_, wheel_radius_, corridor_width_, look_ahead_distance_);*/
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // Extract yaw from quaternion
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    double averageRangeInWindow(const sensor_msgs::msg::LaserScan::SharedPtr& msg,
        double center_angle_rad, double window_size_deg = 10.0)
    {
        double window = window_size_deg * M_PI / 180.0;
        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;
        int n = msg->ranges.size();

        double sum = 0.0;
        int count = 0;

        for (int i = 0; i < n; i++) {
            double angle = angle_min + i * angle_inc;

            if (angle > center_angle_rad - window &&
                angle < center_angle_rad + window)
            {
                double r = msg->ranges[i];
                if (std::isfinite(r)) {
                    sum += r;
                    count++;
                }
            }
        }

        if (count == 0)
            return std::numeric_limits<double>::infinity();

        return sum / count;
    } 

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data to determine the position inside the corridor
        // Lateral distances using a window of ±10°
        dist_left_  = averageRangeInWindow(msg, +M_PI/2, 1.0);   // +90°
        dist_right_ = averageRangeInWindow(msg, -M_PI/2, 1.0);   // -90°

        // Looking ahead (80º) and rear (100º) to detect if which wall is closer
        // Using closer wall to determine orientation sign (theta_sign_)
        
        double delta_angle = 15.0 * M_PI / 180.0; // 15 degrees
        //int orientation_vote = 0; // +1 left, -1 right

        if (dist_left_ < dist_right_) {
            // Use left wall for orientation sign
            d_front = averageRangeInWindow(msg, M_PI/2 - delta_angle, 2.0); // ~75 deg
            double d_back  = averageRangeInWindow(msg, M_PI/2 + delta_angle, 2.0); // ~105 deg
            
            // If frontal distance is smaller, we are facing the left wall (+Theta)
            if (std::isfinite(d_front) && std::isfinite(d_back)) {
                theta_sign_ = (d_front < d_back) ? 1.0 : -1.0;
            }
        } else {
            // Use right wall for orientation sign
            double d_front = averageRangeInWindow(msg, -M_PI/2 + delta_angle, 2.0); // ~ -75 deg
            double d_back  = averageRangeInWindow(msg, -M_PI/2 - delta_angle, 2.0); // ~ -105 deg
            
            // On the right, if front < back, we are facing the right wall (-Theta)
            if (std::isfinite(d_front) && std::isfinite(d_back)) {
                theta_sign_ = (d_front < d_back) ? -1.0 : 1.0;
            }
        }

        measured_data_=true;

        //RCLCPP_INFO(this->get_logger(), "Left=%.2f m, Right=%.2f m", dist_left_, dist_right_);
    }

    //Callback del servicio. Esto es lo que hace cuando lo llamamos:
    void cornerCallback(
        const seg_tray_msgs::srv::CornerService::Request::SharedPtr /*request*/,
        seg_tray_msgs::srv::CornerService::Response::SharedPtr response)
    {
        response->success = true;
        response->linear_velocity = 0.0;    // stop
        response->angular_velocity = -1.0; //giramos
    }

    void controlLoop()
    {
        if (!measured_data_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for laser data...");
            return;
        }
        else {
            measured_data_=false; // Reset flag

            // Compute orientation error and lateral distance
            double lateral_distance = (dist_left_ + dist_right_);
            double theta_mag = std::acos(corridor_width_ / (lateral_distance + 1e-6)); // Avoid division by zero
            if (std::isnan(theta_mag)){
                theta_mag = 0.0;
            }
            double theta = theta_mag * theta_sign_; // Apply sign to orientation error
            //RCLCPP_INFO(this->get_logger(), "Lateral Distance=%.2f m, Orientation Error=%.2f rad", lateral_distance, theta);
            //RCLCPP_INFO(this->get_logger(), "theta_sign=%.1f, theta_mag=%.4f", theta_sign_, theta_mag);

            /////////////////// PUT YOUR CONTROL CODE HERE ///////////
            // Coordenadas locales:
            //Primero sacamos la distancia a la trayectoria

            /*double dp = dist_right_*cos((theta));
            //RCLCPP_INFO(this->get_logger(), "theta es %f, vamos pa la dcha?", theta);
            if(abs(dist_left_) < abs(dist_right_))
            {
                dp = dist_left_*cos((theta));
                //RCLCPP_INFO(this->get_logger(), "no, vamos pa la izq");
            }
            double dl = corridor_width_/2 - abs(dp);*/
            double dl;
            if(abs(dist_left_) > abs(dist_right_)){
                dl = corridor_width_/2 - dist_right_*cos((theta));
                //RCLCPP_INFO(this->get_logger(), "no, vamos pa la izq");
            }else{
                dl = dist_right_*cos((theta))-corridor_width_/2;
                dl = -dl;
            }
            

            double dx = dl*sin(theta) + look_ahead_distance_*cos(theta);
            double dy = dl*cos(theta) - look_ahead_distance_*sin(theta);
            double dist_2 = dx*dx + dy*dy; //Distancia en coordenadas locales al punto objetivo al cuadrado 

            //Constantes
            double front_threshold_ = 1;
            bool robot_stopped_ = false;
            double R = wheel_radius_;
            double K = wheel_base_/2;
            double v_lin = max_linear_speed_;

            //Curvatura
            double gamma = 2*dy/dist_2;

            //Velocidades angulares de las ruedas
            double w_izq = v_lin*(1-K*gamma)/R;
            double w_dcha = v_lin*(1+K*gamma)/R;  
            
            //Velocidades linear y angular del robot
            double linear_velocity = (w_izq + w_dcha)*R/2;
            double angular_velocity = (w_dcha - w_izq)*R/(2*K);

            geometry_msgs::msg::Twist cmd_vel_msg;
             RCLCPP_INFO(this->get_logger(), "DL: %f, theta: %f", dl, theta);

            //Si el robot est,a muy cerca de la pared, llamamos al servicio:
            if ((d_front < front_threshold_)&&(d_front > 0.0)) {
               //RCLCPP_INFO(this->get_logger(), "Servicio!!");
                auto request = std::make_shared<seg_tray_msgs::srv::CornerService::Request>();
                auto future = corner_client_->async_send_request(request);
                auto response = future.get();
                cmd_vel_msg.linear.x = response->linear_velocity;
                cmd_vel_msg.angular.z = response->angular_velocity;
            }else{
                cmd_vel_msg.linear.x = linear_velocity;
                cmd_vel_msg.angular.z = angular_velocity;
            }

            cmd_vel_pub_->publish(cmd_vel_msg);
        }
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Para que los dos callbacks puedan  ir a la vez:
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;

    //Servicio
    rclcpp::Service<seg_tray_msgs::srv::CornerService>::SharedPtr corner_service_;
    rclcpp::Client<seg_tray_msgs::srv::CornerService>::SharedPtr corner_client_;

    // Parameters
    int time_step_;
    double max_linear_speed_;
    double max_angular_speed_;      
    double wheel_base_;
    double wheel_radius_;
    double corridor_width_;
    double look_ahead_distance_;

    // Actual robot position
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;  

    // Laser data
    double dist_left_;
    double dist_right_;
    double d_front; //la quiero usar para el callback del servicio tb
    double theta_sign_ = 1.0; // +1 if facing left wall, -1 if facing right wall
    double measured_data_=false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<CorridorNavigationNode>());
    auto node = std::make_shared<CorridorNavigationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}