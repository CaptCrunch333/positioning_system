#include "ProcessVariableReference.hpp"

ProcessVariableReference::ProcessVariableReference(block_id t_name, block_type t_type) : Reference(t_name, t_type) {
    
    _reference_type = reference_type::process_variable_ref;
    _reference_value = 0.0;
}

ProcessVariableReference::~ProcessVariableReference() {

}

reference_type ProcessVariableReference::getReferenceType(){
    return _reference_type;
}

DataMessage* ProcessVariableReference::receive_msg_internal(DataMessage* t_msg){
    // std::cout << " My current Reference Variable Value: " << _reference_value << std::endl;

    Vector3DMessage* pos_msg = (Vector3DMessage*)t_msg;
    // std::cout << "Data received (Process Variable): " << pos_msg->getData().x << std::endl;
    Vector3D<float> error;

    error.x = _reference_value - pos_msg->getData().x;
    error.y = 0.0 - pos_msg->getData().y ;
    error.z = 0.0 - pos_msg->getData().z;
    //TODO for testing
    // if(error.x < 0.05 && error.x > -0.05){
    //     error.x = 0;
    // }
  
    m_error_msg.setVector3DMessage(error);

    return (DataMessage*) &m_error_msg;
}

void ProcessVariableReference::setReferenceValue(float t_reference_value){
    _reference_value = t_reference_value;
}