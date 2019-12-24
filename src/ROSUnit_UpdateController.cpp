#include "ROSUnit_UpdateController.hpp"

//TODO rename the topics and class
ROSUnit_UpdateController* ROSUnit_UpdateController::_instance_ptr = NULL;
PID_parameters ROSUnit_UpdateController::_update_controller_msg;
control_system ROSUnit_UpdateController::_name;

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_update_controller = t_main_handler.advertiseService("update_controller", callbackUpdateController);
    _instance_ptr = this;
}   

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateController::callbackUpdateController(positioning_system::Update_Controller::Request  &req, 
                                                        positioning_system::Update_Controller::Response &res){

    PID_parameters data;
    data.kp = req.parameters.kp;
    data.ki = req.parameters.ki;
    data.kd = req.parameters.kd;
    data.kdd = req.parameters.kdd;
    data.anti_windup = req.parameters.anti_windup;
    data.en_pv_derivation = req.parameters.en_pv_derivation;
    data.id = static_cast<block_id>((int)req.parameters.id);

    _update_controller_msg.setPIDParam(data);
    _instance_ptr->emit_message((DataMessage*) &_update_controller_msg);
    
    return true;
}