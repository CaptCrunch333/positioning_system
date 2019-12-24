#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include <geometry_msgs/PoseStamped.h>
#include "UpdatePoseMessage.hpp"
#include "Vector3D.hpp"
#include "PID_values.hpp"
#include <positioning_system/PID_param.h>
#include <positioning_system/Update_Controller.h>
#include "common_types.hpp"

class ROSUnit_UpdateController :  public ROSUnit{

    private:
        static control_system _name;
        static ROSUnit_UpdateController* _instance_ptr;
        static PID_parameters _update_controller_msg; 
        ros::ServiceServer _srv_update_controller;
        static bool callbackUpdateController(positioning_system::Update_Controller::Request  &req, 
                                             positioning_system::Update_Controller::Response &res);
        void receive_msg_data(DataMessage* t_msg);  

    public:
        ROSUnit_UpdateController(ros::NodeHandle&);
        ~ROSUnit_UpdateController();
};