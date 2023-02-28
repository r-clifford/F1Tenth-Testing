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
#define DISPARITY_THRESHOLD 0.3
#define SIDE_THRESHOLD 1.0// threshold for a disparity to be considered in side calc
#define TURN_THRESHOLD 0.1 // distance from min disparity to begin turn TODO:
#define MAX_ANGLE 0.4189
#define VELOCITY_SCALAR (1.0/8.0)
#define WALL_FOLLOW false

/* TODO: Disparity extender (DE) is working seemingly fully
 * Integration with wall following (WF) is partially implemented, but performing much
 * worse than DE alone
 * WF issues
 *  - non uniform track width
 *  - if enabled while angle to target wall is > ~45deg
 * Tune:
 *  DISPARITY_THRESHOLD
 *  SIDE_THRESHOLD
 *  TURN_THRESHOLD
 *  MAX_ANGLE
 *  VELOCITY:
 *      velocity should likely be a function of distance from target and steering angle
 *  WF is disabled for now
 *
 *  Notes:
 *      Prefer DE over WF?
 *          Maintain DE for longer during turn, currently switches control nodes
 *          mid corner, leading to poor behavior as WF seeks outer edge, often
 *          turning the incorrect way
 *      Non optimal path under WF on lower part of levine blocked
 *          Follows wall (as intended) instead of shorter path
 *          This alone makes the DE+WF performance substantially worse than pure DE
 *          on this map
 *      DE+WF requires much more parameter tuning compared to DE
 *          Most likely will require tuning per map
 *      DE only needs DISPARITY_THRESHOLD, MAX_ANGLE?, velocity function
 *
*/
class DisparityExtender {
public:
    DisparityExtender() {
        pub_nav_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
        pub_side_ = n_.advertise<std_msgs::String>("/side", 1);
        pub_follow_wall_ = n_.advertise<std_msgs::Bool>("/follow_wall", 1);
        sub_lidar_ = n_.subscribe("/scan", 1000, &DisparityExtender::lidar_callback, this);
        angle = 0.0;
        disparity_threshold = DISPARITY_THRESHOLD;
        turn_threshold = TURN_THRESHOLD;
        angle_range = 75.0 / 180.0 * M_PI;
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

        size_t closest = closest_disparity();
        double closest_distance = ranges[closest];
        double closest_angle = closest * data.angle_increment + data.angle_min;
        // tan theta = (h/f)
        // f = h / (tan theta)
        double forward_distance = abs(closest_distance / tan(closest_angle));
        auto tmp = 180 / M_PI * closest_angle;
        ROS_INFO_STREAM(closest_distance);
        auto side = std_msgs::String{};

        if (closest_angle > 0) {
            // nearest disparity is to the left
            side.data = "right";
        } else {
            side.data = "left";
        }
        pub_side_.publish(side);
        auto msg = std_msgs::Bool{};
        if (WALL_FOLLOW) {

//            if (abs(forward_distance - closest_distance) < turn_threshold) {
            if (forward_distance > turn_threshold) {
                enabled = true;
                msg.data = false;
            } else {
                enabled = false;
                msg.data = true;
            }
        }
        pub_follow_wall_.publish(msg);
        publish();
    }

    size_t closest_disparity() {
        // returns index of closest disparity
        double c_min = std::numeric_limits<double>::max();
        size_t c_idx = 0;
        for (size_t i: disparity_indexes) {
            if (ranges[i] < c_min) {
                c_min = ranges[i];
                c_idx = i;
            }
        }
        disparity_indexes.clear();
        return c_idx;
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
        for (size_t i = min_idx; i < max_idx - 1; i++) {
            double close = std::min(ranges[i], ranges[i + 1]);
            size_t d_idx = mask_from_distance(close, angle_inc);
            size_t tmp;
            double disparity = abs(ranges[i] - ranges[i + 1]);
            if (disparity > disparity_threshold) {
                // if disparity found, backtrack to create bubble around disparity
                // radius approx 1 car width
                if (disparity > SIDE_THRESHOLD) {
                    disparity_indexes.push_back(i);
                }
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
        speed = 2.0;
        driveStamped.drive.speed = speed;
        if (enabled) {
            pub_nav_.publish(driveStamped);
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_nav_;
    ros::Publisher pub_side_;
    ros::Publisher pub_follow_wall_;
    ros::Subscriber sub_lidar_;
    double angle; // steering angle
    double speed;
    double disparity_threshold;
    double turn_threshold;
    double angle_range; // +- angle_range is max angle of lidar readings used
    std::vector<double> ranges;
    size_t min_idx;
    size_t max_idx;
    double target_distance;
    bool enabled;
    std::vector<size_t> disparity_indexes;
};

int main(int argc, char **argv) {
    // Initiate ROS
    ros::init(argc, argv, "disparity_extender");

    DisparityExtender disparityExtender;
    ros::spin();
    return 0;
}