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

#define VELOCITY_MAX 5.00
#define CAR_LENGTH 0.50
#define CAR_WIDTH 0.50
#define DISPARITY_THRESHOLD 0.2
#define MAX_ANGLE 0.4189
#define VELOCITY_SCALAR (1.0/8.0)
class DisparityExtender
{
public:
    DisparityExtender()
    {
        pub_nav_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        sub_lidar_ = n_.subscribe("/scan", 1000, &DisparityExtender::lidar_callback, this);
        angle = 0.0;
        threshold = DISPARITY_THRESHOLD;
        angle_range = (M_PI) / 2;
    }

    void lidar_callback(sensor_msgs::LaserScan data) {
        min_idx = (-angle_range - data.angle_min) / data.angle_increment;
        max_idx = (angle_range - data.angle_min) / data.angle_increment;
        ranges = std::vector<double>(std::begin(data.ranges), std::end(data.ranges));
        clean_ranges(data.range_min, data.range_max);
        filter_disparities(data.angle_increment);
        size_t i = get_target_index();
        target_distance = ranges[i];
        angle = data.angle_min + i * data.angle_increment;
        publish();
    }
    size_t get_target_index() {
        double c_max = 0.0;
        size_t i_max = 0;
        for (size_t i = min_idx; i < max_idx; i++) {
            double r = ranges[i];
            if (r > c_max) {
                c_max = r;
                i_max = i;
            }
        }
        return i_max;
//        return std::distance(ranges.begin(), std::max_element(std::begin(ranges), std::end(ranges)));
    }
    void filter_disparities(double angle_inc) {
       for (size_t i = min_idx; i < max_idx-1; i++) {
                double close = std::min(ranges[i], ranges[i+1]);
                size_t d_idx = mask_from_distance(close, angle_inc);
                size_t tmp;
                if (abs(ranges[i] - ranges[i+1]) > threshold) {
                    // if disparity found, backtrack to create bubble around disparity
                    // radius approx 1 car width
                    tmp = i + d_idx;
                    i -= d_idx;
                    for (; i < tmp; i++) {
                        ranges[i] = std::min(ranges[i], close);
                    }
                }
       }
    }
    void clean_ranges(double r_min, double r_max) {
//        for (size_t i = 0; i < min_idx; i++) {
//            ranges[i] = 0.0;
//        }
//        for (size_t i = max_idx; i < ranges.size(); i++) {
//            ranges[i] = 0.0;
//        }
        for (size_t i = min_idx; i < max_idx; i++) {
            double *r = &ranges[i];
            if (std::isnan(*r) || *r < r_min) {
               *r = 0.0;
            } else if (std::isinf(*r) || *r > r_max) {
                *r = r_max;
            }
        }
    }
    size_t mask_from_distance(double distance, double angle_inc) {
        // at a distance, find angle corresponding to car width
        /*
         * Need number of lidar points covering car width at some distance
         *  theta = arc len / radius
         */
        double theta = CAR_WIDTH / distance;
        return theta / angle_inc;
    }
    void publish() {
        ackermann_msgs::AckermannDriveStamped driveStamped;
        double s_angle = std::min(abs(angle), MAX_ANGLE);
        if (angle < 0) {
            s_angle *= -1;
        }
        driveStamped.drive.steering_angle = s_angle;
//        speed = std::min(VELOCITY_MAX,(VELOCITY_SCALAR * target_distance * VELOCITY_MAX) - abs(2.0 * s_angle));
        if (abs(s_angle) < 0.10) {
           speed = 6.0;
        } else if (abs(s_angle) < 0.20) {
           speed = 3.0;
        } else {
           speed = 1.5;
        }
        ROS_INFO_STREAM(speed);
        driveStamped.drive.speed = speed;
        pub_nav_.publish(driveStamped);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_nav_;
    ros::Subscriber sub_lidar_;
    double angle; // steering angle
    double speed;
    double threshold;
    double angle_range; // +- angle_range is max angle of lidar readings used
    std::vector<double> ranges;
    size_t min_idx;
    size_t max_idx;
    double target_distance;
};

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "disparity_extender");

    DisparityExtender disparityExtender;
    ros::spin();
    return 0;
}