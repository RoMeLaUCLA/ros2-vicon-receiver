#include "vicon_receiver/publisher.hpp"

/** Publisher **/
Publisher::Publisher(std::string topic_name, rclcpp::Node* node)
{
    position_publisher_ = node->create_publisher<vicon_receiver::msg::Position>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto msg = std::make_shared<vicon_receiver::msg::Position>();
    msg->header.stamp = p.frame_time;
    msg->x_trans = p.translation[0];
    msg->y_trans = p.translation[1];
    msg->z_trans = p.translation[2];
    msg->x_rot = p.rotation[0];
    msg->y_rot = p.rotation[1];
    msg->z_rot = p.rotation[2];
    msg->w = p.rotation[3];
    msg->subject_name = p.subject_name;
    msg->segment_name = p.segment_name;
    msg->frame_number = p.frame_number;
    msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}


/** Marker Publisher **/
MarkerPublisher::MarkerPublisher(std::string topic_name, rclcpp::Node* node) {
    position_publisher_ = node->create_publisher<vicon_receiver::msg::MarkerPositions>(topic_name, 10);
    is_ready = true;
}

void MarkerPublisher::publish(MarkerPositionStruct ms[], const unsigned int N, const rclcpp::Time& frame_time, const unsigned int frame_number) {
    auto msg = std::make_shared<vicon_receiver::msg::MarkerPositions>();
    msg->header.stamp = frame_time;
    msg->frame_number = frame_number;

    for(unsigned int i=0; i < N; ++i) {
        auto m = std::make_unique<vicon_receiver::msg::MarkerPosition>();
        m->x_trans = ms[i].translation[0];
        m->y_trans = ms[i].translation[1];
        m->z_trans = ms[i].translation[2];
        m->marker_name = ms[i].marker_name;
        m->subject_name = ms[i].subject_name;
        m->segment_name = ms[i].segment_name;
        m->occluded = ms[i].occluded;
        msg->markers.push_back(*m);
    }
    position_publisher_->publish(*msg);
}

