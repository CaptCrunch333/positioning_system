#include "ROSUnit_UpdateReference.hpp"

//TODO rename the topics and class
ROSUnit_UpdateReference* ROSUnit_UpdateReference::_instance_ptr = NULL;
UpdatePoseMessage ROSUnit_UpdateReference::_pose_ref_msg;

ROSUnit_UpdateReference::ROSUnit_UpdateReference(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_pose_reference", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_UpdateReference::~ROSUnit_UpdateReference() {

}

void ROSUnit_UpdateReference::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateReference::callbackSetpoint( positioning_system::Update_Pose_Reference::Request  &req, 
                                                positioning_system::Update_Pose_Reference::Response &res){

    //TODO change to receive only one reference at a time
    float x, y, z, yaw;
    x = req.setpoint_pose.x;
    y = req.setpoint_pose.y;
    z = req.setpoint_pose.z;
    yaw = req.setpoint_pose.yaw;

    _pose_ref_msg.setPoseMessage(x, y, z, yaw);

    _instance_ptr->emit_message((DataMessage*) &_pose_ref_msg);

    return true;

}