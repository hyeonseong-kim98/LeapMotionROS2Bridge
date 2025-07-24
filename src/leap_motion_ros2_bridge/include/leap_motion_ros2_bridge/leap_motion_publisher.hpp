#ifndef LEAP_MOTION_PUBLISHER_HPP
#define LEAP_MOTION_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <leap_motion_interface/msg/leap_frame.hpp>
#include "LeapC.h"

class LeapMotionPublisher : public rclcpp::Node {
public:
    LeapMotionPublisher();
    ~LeapMotionPublisher();

private:
    void timer_callback();

    rclcpp::Publisher<leap_motion_interface::msg::LeapFrame>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LEAP_CONNECTION connection_;
};

#endif // LEAP_MOTION_PUBLISHER_HPP