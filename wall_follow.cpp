#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include <cmath>

#include <limits>
#include <cstddef>
#include <std_msgs/String.h>
#include <string.h>
#define KP 1.00
#define KD 0.00
#define KI 0.00

#define ANGLE_RANGE 270 // degrees
#define OPTIMAL_DISTANCE 0.75 // distance from target wall
#define VELOCITY 2.00
#define CAR_LENGTH 0.50
#define LOOKAHEAD 0.80 // distance traveled at t+1
// TODO: Try to reduce number of branches

enum Side {
    Left,
    Right,
};
class WallFollower
{
public:
    WallFollower()
    {
        pub_nav_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        sub_stay_right_ = n_.subscribe("/side", 1, &WallFollower::side_callback, this);
        sub_lidar_ = n_.subscribe("/scan", 1, &WallFollower::lidar_callback, this);
        side = Left;
        angle = 0.0;
        c_time = ros::Time::now();
    }
    void side_callback(std_msgs::String side_msg) {
       if (side_msg.data == "right") {
           side = Right;
       } else if (side_msg.data == "left") {
           side = Left;
       } else {
           ROS_ERROR_ONCE("wall_follow.cpp: 51, invalid side");
       }
    }
    void lidar_callback(sensor_msgs::LaserScan data) {
        double a_angle = 45.0/180.0 * M_PI;
        double b_angle = 90.0/180.0 * M_PI;

        std::vector<double> ranges(std::begin(data.ranges), std::end(data.ranges));
        int index_a;
        int index_b;
        index_a = (a_angle - data.angle_min) / data.angle_increment;
        index_b = (b_angle - data.angle_min) / data.angle_increment;
        if (side == Side::Right) {
            index_a = (data.angle_max - a_angle) / data.angle_increment;
            index_b = (data.angle_max - b_angle) / data.angle_increment;
        }
        double a = data.ranges[index_a];
        if (std::isinf(a) || std::isnan(a)) {
            index_a++;
            a = data.ranges[index_a];
        }
        double b = data.ranges[index_b];
        if (std::isinf(b) || std::isnan(b)) {
            index_b++;
            b = data.ranges[index_b];
        }
        a_angle = index_a * data.angle_increment + data.angle_min;
        b_angle = index_b * data.angle_increment + data.angle_min;
        if (side == Side::Right) {
            a_angle = data.angle_max - index_a * data.angle_increment;
            b_angle = data.angle_max - index_b * data.angle_increment;
        }
        double theta = b_angle - a_angle;
        double alpha = atan((a * cos(theta) - b) / (a * sin(theta)));
        double B = b * cos(alpha);
        double B_proj = B + LOOKAHEAD * sin(alpha); // projected distance from wall at t+1

        p_error = c_error;
        c_error = B_proj - target_distance;
        angle = pid_control();

        follow();
    }

    double pid_control() {
        p_time = c_time;
        c_time = ros::Time::now();
        double dt = (c_time - p_time).toSec();
        integral += p_error*dt;
        derivative = (p_error - c_error)/dt;
        return kp * c_error + ki * integral + kd * derivative;
    }

    void follow() {
        ackermann_msgs::AckermannDriveStamped driveStamped;
        driveStamped.drive.steering_angle = angle;
        if (abs(angle) < (5.0 / 180.0 * M_PI)) {
           speed = 2.0;
        } else if (abs(angle) < (15)) {
           speed = 1.5;
        } else {
            speed = 0.75;
        }
        driveStamped.drive.speed = speed;
        pub_nav_.publish(driveStamped);
    }

private:
    double dt;
    ros::NodeHandle n_;
    ros::Subscriber sub_stay_right_;
    ros::Publisher pub_nav_;
    ros::Subscriber sub_lidar_;
    // PID parameters
    double kp = KP;
    double ki = KI;
    double kd = KD;
    double angle; // steering angle
    double speed;
    double integral = 0; // integral value
    double derivative = 0; // derivative term
    // current and previous error
    double c_error;
    double p_error;
    // current and previous time
    ros::Time c_time;
    ros::Time p_time;
    double target_distance = OPTIMAL_DISTANCE;
    double delta = 0.0; // distance from target
    Side side;
};

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "wall_follower");

    WallFollower wall_follower;
    ros::spin();
    return 0;
}