#include "UpdatePoseMessage.hpp"

UpdatePoseMessage::UpdatePoseMessage(float t_x, float t_y, float t_z, float t_yaw) {
    _type = msg_type::UPDATEPOSEREFERENCE;
    _x = t_x;
    _y = t_y;
    _z = t_z;
    _yaw = t_yaw;
}

UpdatePoseMessage::UpdatePoseMessage() {
    _type = msg_type::UPDATEPOSEREFERENCE;
}

UpdatePoseMessage::~UpdatePoseMessage() {

}

float UpdatePoseMessage::getX(){
    return _x;
}
float UpdatePoseMessage::getY(){
    return _y;
}
float UpdatePoseMessage::getZ(){
    return _z;
}
float UpdatePoseMessage::getYaw(){
    return _yaw;
}
msg_type UpdatePoseMessage::getType(){
    return _type;
}
void UpdatePoseMessage::setPoseMessage(float t_x, float t_y, float t_z, float t_yaw){
    _type = msg_type::UPDATEPOSEREFERENCE;
    _x = t_x;
    _y = t_y;
    _z = t_z;
    _yaw = t_yaw;
}

const int UpdatePoseMessage::getSize()
{
    return sizeof(this);
}