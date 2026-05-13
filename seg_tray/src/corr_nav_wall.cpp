#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <numeric>

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

        // timer 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&CorridorNavigationNode::controlLoop, this));

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

        RCLCPP_INFO(this->get_logger(), 
            "max_linear_speed: %.2f, max_angular_speed: %.2f, wheel_base: %.2f, wheel_radius: %.2f, corridor_width: %.2f, look_ahead_distance: %.2f", 
            max_linear_speed_, max_angular_speed_, wheel_base_, wheel_radius_, corridor_width_, look_ahead_distance_);
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

    struct Wall {
        double m;  // slope
        double b;  // intercept (distance to robot)
    };

    // Estimamos las paredes
    Wall extractWall(const sensor_msgs::msg::LaserScan::SharedPtr& msg,
        double angle_min_deg, double angle_max_deg)
    {
        double angle_min_rad = angle_min_deg * M_PI / 180.0;
        double angle_max_rad = angle_max_deg * M_PI / 180.0;

        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        int n = 0;

        for (int i = 0; i < (int)msg->ranges.size(); i++) {

            //Sacamos el ángulo sumándole el incremento*índice
            double angle = msg->angle_min + i * msg->angle_increment;
            double r = msg->ranges[i]; // distancia

            //Nos aseguramos de que está dentro del rango que queremos mirar y que es un numero finito
            if (angle < angle_min_rad || angle > angle_max_rad) continue;
            if (!std::isfinite(r)) continue;

            //transformamos de radio y ángulo a distancia
            double x = r * cos(angle);
            double y = r * sin(angle);

            sum_x  += x;
            sum_y  += y;
            sum_xy += x * y;
            sum_x2 += x * x;
            n++;
        }

        //Para sacar una línea necesitamos por lo menos 2 ptos:
        Wall wall;
        if (n < 2) {
            wall.m = 0.0;
            wall.b = std::numeric_limits<double>::infinity();
            return wall;
        }

        //Mínimos cuadrados:
        wall.m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        wall.b = (sum_y - wall.m * sum_x) / n;
        return wall;
    } 

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Aquí se añadirá el nuevo código para sacar las paredes izq y dcha
        wall_l = extractWall(msg, 45, 135);
        wall_r = extractWall(msg, -135, -45);

        measured_data_=true;

        //RCLCPP_INFO(this->get_logger(), "Left=%.2f m, Right=%.2f m", dist_left_, dist_right_);
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

            double theta = -std::atan(wall_r.m); 
            // Coordenadas locales:
            double dl;
            if(abs(wall_l.b) > abs(wall_r.b)){
                dl = corridor_width_/2 - abs(wall_r.b)*cos(theta);
            }else{
                //dl = wall_r.b - corridor_width_/2;
                dl = corridor_width_/2 - abs(wall_l.b)*cos(theta);
                dl = -dl;
            }



            //dl = corridor_width_/2 - wall_r.b;
            //dl = corridor_width_/2 + wall_r.b;
            dl = corridor_width_/2 - std::abs(wall_r.b) * std::cos(theta);
                        RCLCPP_INFO(this->get_logger(), "dl=%.3f, wall_r.m=%.3f, wall_l.m=%.3f, theta=%.3f, theta_izq=%.3f", 
            dl, wall_r.m, wall_l.m, theta, std::atan(wall_l.m));

            double dx = dl*sin(theta) + look_ahead_distance_*cos(theta);
            double dy = dl*cos(theta) - look_ahead_distance_*sin(theta);
            //RCLCPP_INFO(this->get_logger(), "dl=%.3f, dx=%.3f, dy=%.3f, gamma=%.3f", dl, dx, dy, gamma);
            
            double dist_2 = dx*dx + dy*dy; //Distancia en coordenadas locales al punto objetivo al cuadrado 

            //Constantes
            double R = wheel_radius_;
            double K = wheel_base_/2;
            double v_lin = max_linear_speed_;

            //Curvatura
            //double gamma = 2*dy/dist_2;
            //Si no, gira mucho y pierde la pared
            double gamma = std::clamp(2*dy/dist_2, -0.3, 0.3);

            //Velocidades angulares de las ruedas
            double w_izq = v_lin*(1-K*gamma)/R;
            double w_dcha = v_lin*(1+K*gamma)/R;  
            
            //Velocidades linear y angular del robot
            double linear_velocity = (w_izq + w_dcha)*R/2;
            double angular_velocity = (w_dcha - w_izq)*R/(2*K);

            geometry_msgs::msg::Twist cmd_vel_msg;

            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = angular_velocity;

            cmd_vel_pub_->publish(cmd_vel_msg);
        }
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;



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
    Wall wall_l;
    Wall wall_r;
    double measured_data_=false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}