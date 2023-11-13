#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <string>

class TF_Boardcaster
{
public:
    TF_Boardcaster(const std::string &frame_id, const std::string &child_frame_id);
    void initBroadcasterObject(const std::string &frame_id, const std::string &child_frame_id);

    tf2_ros::TransformBroadcaster BoardcasterObject;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
};

TF_Boardcaster::TF_Boardcaster(const std::string &frame_id, const std::string &child_frame_id){
    initBroadcasterObject(frame_id,child_frame_id);
}

void TF_Boardcaster::initBroadcasterObject(const std::string &frame_id, const std::string &child_frame_id)
{
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    BoardcasterObject.sendTransform(transformStamped);
}