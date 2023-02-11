#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include "math.h"

#include <limits>
#include <cstddef>
// TODO: estimate
// v = 1.8 m/s
// a = -8.26 m/s^2
// = = v - at
#define TTC_THRESHOLD 0.5

class AutoEBrake
{
public:
    AutoEBrake()
    {
        pub_brake_bool_ = n_.advertise<std_msgs::Bool>("/brake_bool", 1000);
        pub_zero_vel_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);

        sub_lidar_ = n_.subscribe("/scan", 1000, &AutoEBrake::scan_callback, this);
        sub_odom_ = n_.subscribe("/odom", 1000, &AutoEBrake::odom_callback, this);
    }
    void odom_callback(const nav_msgs::Odometry &odom_msgs)
    {
        odom_info_ = odom_msgs;
    }
    void scan_callback(const sensor_msgs::LaserScan &lidar_msgs)
    {
        lidar_info_ = lidar_msgs;
    }
    void pub()
    {

        geometry_msgs::Vector3 tmp = odom_info_.twist.twist.linear;
        double vx = tmp.x;
        double vy = tmp.y;
        std::vector<double> v = {vx, vy};
        double angle = lidar_info_.angle_min;
        double current_ttc = std::numeric_limits<double>::max();
        for (unsigned int i = 0; i < lidar_info_.ranges.size(); i++)
        {
            double range = lidar_info_.ranges[i];
            if (std::isinf(range) || std::isnan(range) || (range > lidar_info_.range_max))
            {
                continue;
            }
            double distance = lidar_info_.ranges[i];
            angle += lidar_info_.angle_increment;
            double dx = cos(angle) * distance;
            double dy = sin(angle) * distance;
            std::vector<double> d = {dx, dy};
            std::vector<double> vproj = AutoEBrake::elem_wise_mul_(AutoEBrake::dot_(v, d) / AutoEBrake::dot_(d, d), d);
            double proj = AutoEBrake::dot_(vproj, vproj);
            double ttc = distance / (std::max(proj, 0.0));
            if (ttc < current_ttc)
            {
                current_ttc = ttc;
            }
        }
        if (current_ttc <= TTC_THRESHOLD)
        {
            std_msgs::Bool brake_bool;
            brake_bool.data = true;
            pub_brake_bool_.publish(brake_bool);

            ackermann_msgs::AckermannDriveStamped drive_data;
            drive_data.drive.speed = 0.0;
            pub_zero_vel_.publish(drive_data);
        }
        else
        {
            std_msgs::Bool brake_bool;
            brake_bool.data = false;
            pub_brake_bool_.publish(brake_bool);
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_brake_bool_;
    ros::Publisher pub_zero_vel_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_odom_;
    sensor_msgs::LaserScan lidar_info_;
    nav_msgs::Odometry odom_info_;
    static double dot_(std::vector<double> a, std::vector<double> b)
    {
        double c = 0;
        for (int i = 0; i < a.size(); i++)
        {
            c += a[i] * b[i];
        }
        return c;
    }
    static std::vector<double> elem_wise_mul_(double scalar, std::vector<double> v)
    {
        std::vector<double> out = std::vector<double>();
        for (double elem : v)
        {
            out.push_back(elem * scalar);
        }
        return out;
    }
};

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "aeb");

    AutoEBrake AEBObj;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        AEBObj.pub();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}