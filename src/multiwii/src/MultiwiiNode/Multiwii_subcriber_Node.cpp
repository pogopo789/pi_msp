#include <ros/ros.h>
#include <MultiwiiNode/Multiwii_subscriber_Node.h>

void MultiWiiSubscriberNode::setup() {
    // subscriber
    rc_sub = multiWii_node.nh.subscribe("/PC_input_topic", 100, &MultiWiiSubscriberNode::rc_override_raw, this); // raw channel order
}

////////////////////////////////////////////////////////////////////////////
/// callbacks for subscribed messages

void MultiWiiSubscriberNode::rc_override_raw(const mavros_msgs::OverrideRCIn &rc) {
    std::vector<uint16_t> channels;
    for(const uint16_t c : rc.channels) { channels.push_back(c); }
    multiWii_node.fcu->setRc(channels);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Multiwii_subcriber");

    MultiWiiSubscriberNode node;

    // setup FCU, register publisher
    // node.setup();

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    // register callback for dynamic configuration
    // - update rates for MSP subscriber
    // - main ROS node loop rate
}
