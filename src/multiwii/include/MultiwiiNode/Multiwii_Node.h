#pragma once
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <multiwii/UpdateRatesConfig.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>

#include <Eigen/Geometry>

double deg2rad(const double deg);

double rad2deg(const double rad);

class MultiWiiNode
{
public:
    ros::NodeHandle nh;
    fcu::FlightController *fcu;

    MultiWiiNode() {
        nh = ros::NodeHandle("~");
        // configure
        fcu = new fcu::FlightController();
        setup();
    }
    ~MultiWiiNode() {
        delete fcu;
    }
    
    fcu::FlightController& fc() const {
        return *fcu;
    }

    void setup();
};