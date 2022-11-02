#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>
#include <unistd.h>
#include <boost/thread.hpp>

using namespace std;

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;
    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, Publisher> pub_map;
    map<string, MarkerPublisher> pub_map_marker;
    boost::mutex mutex;
    rclcpp::Time now_time;
    unsigned int lastFrameNumber;
    unsigned int frameCount;
    unsigned int droppedFrameCount;

public:
    Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the Publisher class.
    bool get_frame();

    void get_subjects(const rclcpp::Time& frame_time, unsigned int vicon_frame_num);
    void get_markers(const rclcpp::Time& frame_time, unsigned int vicon_frame_num);

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name);
    void create_publisher_thread(const string subject_name, const string segment_name);
//    void create_publisher(const string subject_name, const string segment_name, const string marker_name);
//    void create_publisher_thread(const string subject_name, const string segment_name, const marker_name);
};

#endif // COMMUNICATOR_HPP
