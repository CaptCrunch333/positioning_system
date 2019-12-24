#include "X_UserReference.hpp"

X_UserReference::X_UserReference() {

}

X_UserReference::~X_UserReference() {

}

void X_UserReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::UPDATEPOSEREFERENCE){   

        UpdatePoseMessage* user_msg = (UpdatePoseMessage*)t_msg;

        m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, user_msg->getX());
        this->emit_message((DataMessage*) &m_output_msg);
    }
}
