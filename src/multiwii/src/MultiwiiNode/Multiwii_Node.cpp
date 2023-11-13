#include <MultiwiiNode/Multiwii_Node.h>

double deg2rad(const double deg) {
    return deg/180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad/M_PI * 180.0;
}

void MultiWiiNode::setup() {
    std::string device;
    int baudrate = 115200;
    if(nh.getParam("device", device)) {
        if(!nh.getParam("baudrate", baudrate)) {
            ROS_ERROR("Parameter 'baudrate' not set. Using default baudrate of %i", baudrate);
        }
        else {
            if(baudrate<=0) {
                ROS_ERROR("'baudrate' must be positive!");
                baudrate = 115200;
            }
        }
        ROS_INFO("Connected to FCU at %s", device.c_str());
    }
    else {
        ROS_ERROR("Parameter 'device' not set.");
    }
    fcu->connect(device, uint(baudrate));
}