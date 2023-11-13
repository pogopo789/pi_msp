#pragma once
#include <MultiwiiNode/Multiwii_Node.h>

class MultiWiiSubscriberNode {
private:
    MultiWiiNode multiWii_node;

    ros::Subscriber rc_sub;

public:
    MultiWiiSubscriberNode(){
        setup();
    };

    void setup();
    void rc_override_raw(const mavros_msgs::OverrideRCIn &rc);
};