#include "leap_motion_publisher.hpp"

LeapMotionPublisher::LeapMotionPublisher() : Node("leap_motion_publisher") {
    publisher_ = this->create_publisher<leap_motion_interface::msg::LeapFrame>("leap_motion_data", 10);

    // Init Leap Motion connection
    LeapCreateConnection(nullptr, &connection_);
    LeapOpenConnection(connection_);
    
    // Get one polling to get the policy state
    LEAP_CONNECTION_MESSAGE msg;
    LeapPollConnection(connection_, 1000, &msg);

    // Turn off the images and map points policy
    LEAP_POLICY_EVENT policy_event;
    uint64_t flagsToSet = 0;
    uint64_t flagsToClear = eLeapPolicyFlag_BackgroundFrames|eLeapPolicyFlag_Images|eLeapPolicyFlag_OptimizeHMD|eLeapPolicyFlag_AllowPauseResume|eLeapPolicyFlag_MapPoints|eLeapPolicyFlag_OptimizeScreenTop;
    LeapSetPolicyFlags(connection_, flagsToSet, flagsToClear);
    LeapPollConnection(connection_, 1000, &msg);
    while (msg.type != eLeapEventType_Policy) {
        LeapSetPolicyFlags(connection_, flagsToSet, flagsToClear);
        LeapPollConnection(connection_, 1000, &msg);
    }
    while ((msg.policy_event->current_policy & flagsToClear)) {
        LeapSetPolicyFlags(connection_, flagsToSet, flagsToClear);
        LeapPollConnection(connection_, 1000, &msg);
        RCLCPP_INFO(this->get_logger(), "Waiting for setting the policy...");
    }
    policy_event = *msg.policy_event;

    if (policy_event.current_policy == 0) {
        RCLCPP_INFO(this->get_logger(), "All policies are currently OFF.");
    } 
    else {
        // Check the Policy state
        if (policy_event.current_policy & eLeapPolicyFlag_BackgroundFrames) {
            RCLCPP_INFO(this->get_logger(), "Background frames policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Background frames policy is currently OFF.");
        }
        if (policy_event.current_policy & eLeapPolicyFlag_Images) {
            RCLCPP_INFO(this->get_logger(), "Image policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Image policy is currently OFF.");
        }
        if (policy_event.current_policy & eLeapPolicyFlag_OptimizeHMD) {
            RCLCPP_INFO(this->get_logger(), "Optimize HMD policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Optimize HMD policy is currently OFF.");
        }
        if (policy_event.current_policy & eLeapPolicyFlag_AllowPauseResume) {
            RCLCPP_INFO(this->get_logger(), "Allow pause resume policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Allow pause resume policy is currently OFF.");
        }
        if (policy_event.current_policy & eLeapPolicyFlag_MapPoints) {
            RCLCPP_INFO(this->get_logger(), "Map points policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Map points policy is currently OFF.");
        }
        if (policy_event.current_policy & eLeapPolicyFlag_OptimizeScreenTop) {
            RCLCPP_INFO(this->get_logger(), "Optimize screen top policy is currently ON.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Optimize screen top policy is currently OFF.");
        }
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(8), // 125hz ~= 120hz (maximum frequency)
        std::bind(&LeapMotionPublisher::timer_callback, this)
    );
}

LeapMotionPublisher::~LeapMotionPublisher() {
    LeapCloseConnection(connection_);
    LeapDestroyConnection(connection_);
}

void LeapMotionPublisher::timer_callback() {
    LEAP_CONNECTION_MESSAGE msg;
    LeapPollConnection(connection_, 1000, &msg);

    if (msg.type == eLeapEventType_Image) {
        RCLCPP_INFO(this->get_logger(), "Image event received");
    }
    if (msg.type == eLeapEventType_Tracking) {
        const LEAP_TRACKING_EVENT *tracking_event = msg.tracking_event;
        if (tracking_event && tracking_event->nHands > 0) {
            leap_motion_interface::msg::LeapFrame message;
            
            // Get header informations
            message.header.stamp        = this->get_clock()->now();
            message.header.frame_id     = "leap_motion_frame";

            // Get the number of hands
            message.n_hands             = tracking_event->nHands;

            // Get framerate
            message.framerate           = tracking_event->framerate;

            // Get the hands
            for (uint32_t i = 0; i < tracking_event->nHands; ++i) {
                LEAP_HAND *hand = &tracking_event->pHands[i];
                leap_motion_interface::msg::LeapHand hand_msg;
                
                // Get the hand informations
                hand_msg.id             = hand->id;
                hand_msg.type           = hand->type == eLeapHandType_Left ? "left" : "right";
                hand_msg.visible_time   = hand->visible_time;
                hand_msg.pinch_distance = hand->pinch_distance;
                hand_msg.grab_angle     = hand->grab_angle;
                hand_msg.pinch_strength = hand->pinch_strength;
                hand_msg.grab_strength  = hand->grab_strength;

                // Get the palm informations
                hand_msg.palm.position.x            = hand->palm.position.x;
                hand_msg.palm.position.y            = hand->palm.position.y;
                hand_msg.palm.position.z            = hand->palm.position.z;
                hand_msg.palm.stabilized_position.x = hand->palm.stabilized_position.x;
                hand_msg.palm.stabilized_position.y = hand->palm.stabilized_position.y;
                hand_msg.palm.stabilized_position.z = hand->palm.stabilized_position.z;
                hand_msg.palm.velocity.x            = hand->palm.velocity.x;
                hand_msg.palm.velocity.y            = hand->palm.velocity.y;
                hand_msg.palm.velocity.z            = hand->palm.velocity.z;
                hand_msg.palm.normal.x              = hand->palm.normal.x;
                hand_msg.palm.normal.y              = hand->palm.normal.y;
                hand_msg.palm.normal.z              = hand->palm.normal.z;
                hand_msg.palm.width                 = hand->palm.width;
                hand_msg.palm.direction.x           = hand->palm.direction.x;
                hand_msg.palm.direction.y           = hand->palm.direction.y;
                hand_msg.palm.direction.z           = hand->palm.direction.z;
                hand_msg.palm.orientation.x         = hand->palm.orientation.x;
                hand_msg.palm.orientation.y         = hand->palm.orientation.y;
                hand_msg.palm.orientation.z         = hand->palm.orientation.z;
                hand_msg.palm.orientation.w         = hand->palm.orientation.w;

                // Get the fingers informations (thumb, index, middle, ring, pinky)
                for (uint32_t f = 0; f < 5; f++){
                    hand_msg.digits[f].finger_id    = hand->digits[f].finger_id;
                    hand_msg.digits[f].is_extended  = hand->digits[f].is_extended;
                    // Get the bones (metacarpal, proximal, intermediate, distal)
                    for (uint32_t b = 0; b < 4; b++){
                        hand_msg.digits[f].bones[b].width           = hand->digits[f].bones[b].width;
                        hand_msg.digits[f].bones[b].prev_joint.x    = hand->digits[f].bones[b].prev_joint.x;
                        hand_msg.digits[f].bones[b].prev_joint.y    = hand->digits[f].bones[b].prev_joint.y;
                        hand_msg.digits[f].bones[b].prev_joint.z    = hand->digits[f].bones[b].prev_joint.z;
                        hand_msg.digits[f].bones[b].next_joint.x    = hand->digits[f].bones[b].next_joint.x;
                        hand_msg.digits[f].bones[b].next_joint.y    = hand->digits[f].bones[b].next_joint.y;
                        hand_msg.digits[f].bones[b].next_joint.z    = hand->digits[f].bones[b].next_joint.z;
                        hand_msg.digits[f].bones[b].rotation.x      = hand->digits[f].bones[b].rotation.x;
                        hand_msg.digits[f].bones[b].rotation.y      = hand->digits[f].bones[b].rotation.y;
                        hand_msg.digits[f].bones[b].rotation.z      = hand->digits[f].bones[b].rotation.z;
                        hand_msg.digits[f].bones[b].rotation.w      = hand->digits[f].bones[b].rotation.w;
                    }
                }
                // Get arm informations
                hand_msg.arm.width          = hand->arm.width;
                hand_msg.arm.prev_joint.x   = hand->arm.prev_joint.x;
                hand_msg.arm.prev_joint.y   = hand->arm.prev_joint.y;
                hand_msg.arm.prev_joint.z   = hand->arm.prev_joint.z;
                hand_msg.arm.next_joint.x   = hand->arm.next_joint.x;
                hand_msg.arm.next_joint.y   = hand->arm.next_joint.y;
                hand_msg.arm.next_joint.z   = hand->arm.next_joint.z;
                hand_msg.arm.rotation.x     = hand->arm.rotation.x;
                hand_msg.arm.rotation.y     = hand->arm.rotation.y;
                hand_msg.arm.rotation.z     = hand->arm.rotation.z;
                hand_msg.arm.rotation.w     = hand->arm.rotation.w;

                // Push the hand to the message
                message.hands.push_back(hand_msg);
            }
            publisher_->publish(message);
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeapMotionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}