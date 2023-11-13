#pragma once

#include <MultiwiiNode/Multiwii_Node.h>
#include <MultiwiiNode/TF_Broadcaster.h>

class MultiWiiPublisherNode {
private:
    MultiWiiNode multiWii_node;
    
    float acc_1g;
    float gyro_unit;
    float magn_gain;
    float si_unit_1g;
    std::string tf_base_frame;

    dynamic_reconfigure::Server<multiwii::UpdateRatesConfig> dyn_conf_srv;

    ros::Publisher battery_pub;

    TF_Boardcaster *tf;
    
public:
    MultiWiiPublisherNode() {
        // configure
        float std_grav;
        if(multiWii_node.nh.getParam("standard_gravity", std_grav))
            this->si_unit_1g = std_grav;
        else
            ROS_ERROR("Parameter 'standard_gravity' not set.");

        float acc_1g;
        if(multiWii_node.nh.getParam("acc_1g", acc_1g))
            this->acc_1g = acc_1g;
        else
            ROS_ERROR("Parameter 'acc_1g' not set.");

        float gyro_unit;
        if(multiWii_node.nh.getParam("gyro_unit", gyro_unit))
            this->gyro_unit = gyro_unit;
        else
            ROS_ERROR("Parameter 'gyro_unit' not set.");

        float magn_gain;
        if(multiWii_node.nh.getParam("magn_gain", magn_gain))
            this->magn_gain = magn_gain;
        else
            ROS_ERROR("Parameter 'magn_gain' not set.");

        // Get the base frame to which the TF is published
        multiWii_node.nh.param<std::string>("base_link", this->tf_base_frame, "odom");

        tf = new TF_Boardcaster("odom","base_link");

        setup();
    }

    ~MultiWiiPublisherNode(){
        delete tf;
    }

    void setup();

    void setDynamicConfigureCallback();

    void dynconf_callback(multiwii::UpdateRatesConfig &config, uint32_t /*level*/);

    void onAttitude(const msp::msg::Attitude &attitude);

    void onAnalog(const msp::msg::Analog &analog);

    void onDebug(const msp::msg::INAVDebug &debug);

    void getAllData();

};