#include <ros/ros.h>
#include <MultiwiiNode/Multiwii_publisher_Node.h>

void MultiWiiPublisherNode::setup() {
    // publisher
    battery_pub = multiWii_node.nh.advertise<sensor_msgs::BatteryState>("battery",1);
}

/**
 * @brief setDynamicConfigureCallback set the callback
 * This will call the callback once for initialisation
 */
void MultiWiiPublisherNode::setDynamicConfigureCallback() {
    // dynamic configure
    dyn_conf_srv.setCallback(boost::bind(&MultiWiiPublisherNode::dynconf_callback, this, _1, _2));
}

void MultiWiiPublisherNode::dynconf_callback(multiwii::UpdateRatesConfig &config, uint32_t /*level*/) {
    // define map with matching update rate per message ID
    const std::map<msp::ID, double> msp_rates = {
        {msp::ID::MSP2_INAV_DEBUG, config.MSP2_INAV_DEBUG},
        {msp::ID::MSP_ATTITUDE, config.MSP_ATTITUDE},
        {msp::ID::MSP_ANALOG, config.MSP_ANALOG},
    };
    // apply update
    for(const auto& r : msp_rates) {
        if(multiWii_node.fcu->hasSubscription(r.first)) {
            multiWii_node.fcu->getSubscription(r.first)->setTimerFrequency(r.second);
        }
    }
}

////////////////////////////////////////////////////////////////////////////
    /// callbacks for published messages

void MultiWiiPublisherNode::onAttitude(const msp::msg::Attitude &attitude){
    tf->transformStamped.header.stamp = ros::Time::now();

    tf->q.setRPY(deg2rad(attitude.roll), deg2rad(attitude.pitch), deg2rad(attitude.yaw));
    tf->transformStamped.transform.rotation.x = tf->q.x();
    tf->transformStamped.transform.rotation.y = tf->q.y();
    tf->transformStamped.transform.rotation.z = tf->q.z();
    tf->transformStamped.transform.rotation.w = tf->q.w();

    // tf->BoardcasterObject.sendTransform(tf->transformStamped);
}

void MultiWiiPublisherNode::onAnalog(const msp::msg::Analog &analog) {
    sensor_msgs::BatteryState battery;
    battery.header.stamp = ros::Time::now();
    battery.voltage = analog.vbat;
    battery.current = analog.amperage;
    battery_pub.publish(battery);
}

void MultiWiiPublisherNode::onDebug(const msp::msg::INAVDebug &debug){
    tf->transformStamped.header.stamp = ros::Time::now();

    tf->transformStamped.transform.translation.x = (double)(debug.debug1/1000);
    tf->transformStamped.transform.translation.y = (double)(debug.debug0/1000);
    if (debug.debug2<0)
        tf->transformStamped.transform.translation.z = (double)(0);
    else
        // tf->transformStamped.transform.translation.z = (double)(debug.debug2/1000);
        tf->transformStamped.transform.translation.z = (double)(0);
    tf->BoardcasterObject.sendTransform(tf->transformStamped);

}

void MultiWiiPublisherNode::getAllData(){

    multiWii_node.fcu->subscribe(&MultiWiiPublisherNode::onAnalog, this);
    multiWii_node.fcu->subscribe(&MultiWiiPublisherNode::onDebug, this);
    multiWii_node.fcu->subscribe(&MultiWiiPublisherNode::onAttitude, this);

    setDynamicConfigureCallback();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiWii_publisher");

    MultiWiiPublisherNode node;

    // setup FCU, register publisher
    // node.setup();
    node.getAllData();

    ROS_INFO("MSP PUB IS READY!");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    // register callback for dynamic configuration
    // - update rates for MSP subscriber
    // - main ROS node loop rate

}
