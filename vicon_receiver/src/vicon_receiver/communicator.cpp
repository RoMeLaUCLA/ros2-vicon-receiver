#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

Communicator::Communicator() : Node("vicon")
{
    // get parameters
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);
}

bool Communicator::connect()
{
    // connect to server
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;
    int counter = 0;
    while (!vicon_client.IsConnected().Connected)
    {
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
            sleep(1);
        }
    }
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    // perform further initialization
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    msg = "Initialization complete";
    cout << msg << endl;

    return true;
}

bool Communicator::disconnect()
{
    if (!vicon_client.IsConnected().Connected)
        return true;
    sleep(1);
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;
    vicon_client.Disconnect();
    msg = "Successfully disconnected";
    cout << msg << endl;
    if (!vicon_client.IsConnected().Connected)
        return true;
    return false;
}

bool Communicator::get_frame() {
    while(vicon_client.GetFrame().Result != Result::Success && rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "getFrame returned false\n");
        usleep( 1.0e6/300.0); // TODO: Configurable
    }
    now_time = now();


    static rclcpp::Time lastTime;
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    int frameDiff = 0;
    if (lastFrameNumber != 0) {
        frameDiff = frame_number.FrameNumber - lastFrameNumber;
        frameCount += frameDiff;

        if ((frameDiff) > 1) {
            droppedFrameCount += frameDiff;
            double droppedFramePct = (double)droppedFrameCount / frameCount * 100;
            RCLCPP_DEBUG_STREAM(get_logger(), "" << frameDiff << " more (total " << droppedFrameCount << "/" << frameCount << ", "
                                                              << droppedFramePct << "%) frame(s) dropped. Consider adjusting rates.");
        }
    }

    lastFrameNumber = frame_number.FrameNumber;

    if (frameDiff == 0) {
        return false;
    } else {
        rclcpp::Duration vicon_latency(vicon_client.GetLatencyTotal().Total);

        get_subjects(now_time - vicon_latency, lastFrameNumber);
        get_markers(now_time - vicon_latency, lastFrameNumber);

        lastTime = now_time;
        return true;
    }
}

void Communicator::get_subjects(const rclcpp::Time& frame_time, unsigned int vicon_frame_num)
{

    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // count the number of segments
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            // get the segment name
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            PositionStruct current_position;
            current_position.frame_time = frame_time;

            // get position of segment
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion rot =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
            
            for (size_t i = 0; i < 4; i++)
            {
                if (i < 3)
                    current_position.translation[i] = trans.Translation[i];
                current_position.rotation[i] = rot.Rotation[i];
            }
            current_position.segment_name = segment_name;
            current_position.subject_name = subject_name;
            current_position.translation_type = "Global";
            current_position.frame_number = vicon_frame_num;

            // send position to publisher
            boost::mutex::scoped_try_lock lock(mutex);

            if (lock.owns_lock())
            {
                // get publisher
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        pub.publish(current_position);
                    }
                }
                else
                {
                    // create publisher if not already available
                    lock.unlock();
                    create_publisher(subject_name, segment_name);
                }
            }
        }
    }
}

void Communicator::get_markers(const rclcpp::Time& frame_time, unsigned int vicon_frame_num) {
    map<string, MarkerPublisher>::iterator pub_it;
    unsigned int SubjectCount = vicon_client.GetSubjectCount().SubjectCount;
    for (unsigned int SubjectIndex=0; SubjectIndex < SubjectCount; ++ SubjectIndex) {
        std::string this_subject_name = vicon_client.GetSubjectName(SubjectIndex).SubjectName;
        unsigned int num_subject_markers = vicon_client.GetMarkerCount(this_subject_name).MarkerCount;

        MarkerPositionStruct* markers = new MarkerPositionStruct[num_subject_markers];
        for (unsigned int MarkerIndex = 0; MarkerIndex < num_subject_markers; ++MarkerIndex) {
            MarkerPositionStruct* current_marker = markers + MarkerIndex;
//            current_marker->frame_time = frame_time;
//            current_marker->frame_number = vicon_frame_num;

            current_marker->marker_name = vicon_client.GetMarkerName(this_subject_name, MarkerIndex).MarkerName;
            current_marker->subject_name = this_subject_name;
            current_marker->segment_name = vicon_client.GetMarkerParentName(this_subject_name, current_marker->marker_name).SegmentName;

            Output_GetMarkerGlobalTranslation _Output_GetMerkerGlobalTranslation =
                    vicon_client.GetMarkerGlobalTranslation(this_subject_name, current_marker->marker_name);

            current_marker->translation[0] = _Output_GetMerkerGlobalTranslation.Translation[0];
            current_marker->translation[1] = _Output_GetMerkerGlobalTranslation.Translation[1];
            current_marker->translation[2] = _Output_GetMerkerGlobalTranslation.Translation[2];
            current_marker->occluded = _Output_GetMerkerGlobalTranslation.Occluded;
        }



            // send position to publisher
        boost::mutex::scoped_try_lock lock(mutex);

        if (lock.owns_lock())
        {
            // get publisher
            pub_it = pub_map_marker.find(this_subject_name + "/markers");
            if (pub_it != pub_map_marker.end())
            {
                MarkerPublisher & pub = pub_it->second;

                if (pub.is_ready)
                {
                    pub.publish(markers, num_subject_markers, frame_time, vicon_frame_num);
                }
            }
            else
            {
                // create publisher if not already available
                lock.unlock();
                create_publisher(this_subject_name, "");
            }
        }

    }


}

void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

void Communicator::create_publisher_thread(const string subject_name, const string segment_name)
{

//    std::string key = subject_name + "/" + segment_name;
//    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
//    std::string key_markers = subject_name + "/markers";
//    std::string topic_name_markers = ns_name + "/" + subject_name + "/markers";
//
//    cout << "Creating publisher for " << topic_name << "(" << key << ")" << endl;
//    cout << "Creating publisher for " << topic_name_markers << "(" << key_markers << ")" << endl;
//
//    // create publisher
//    boost::mutex::scoped_lock lock(mutex);
//    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
//    pub_map_marker.insert(std::map<std::string, MarkerPublisher>::value_type(key_markers, MarkerPublisher(topic_name_markers, this)));
//
//    // we don't need the lock anymore, since rest is protected by is_ready
//    lock.unlock();


    if(!segment_name.empty()) {
        std::string key = subject_name + "/" + segment_name;
        std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;

        cout << "Creating publisher for segment " + segment_name + " from subject " + subject_name << ": " + topic_name + "(" + key + ")" << endl;

        // create publisher
        boost::mutex::scoped_lock lock(mutex);
        pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));

        // we don't need the lock anymore, since rest is protected by is_ready
        lock.unlock();
    } else {
        std::string key_markers = subject_name + "/markers";
        std::string topic_name_markers = ns_name + "/" + subject_name + "/markers";

        cout << "Creating publisher for markers of subject " + subject_name << ": " + topic_name_markers + " (" + key_markers + ")" << endl;

        // create publisher
        boost::mutex::scoped_lock lock(mutex);
        pub_map_marker.insert(std::map<std::string, MarkerPublisher>::value_type(key_markers, MarkerPublisher(topic_name_markers, this)));

        // we don't need the lock anymore, since rest is protected by is_ready
        lock.unlock();
    }

}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();
    node->connect();

    while (rclcpp::ok()){
        bool was_new_frame = node->get_frame();
        RCLCPP_WARN_EXPRESSION(node->get_logger(), !was_new_frame, "grab frame return");
    }

    node->disconnect();
    rclcpp::shutdown();
    return 0;
}
