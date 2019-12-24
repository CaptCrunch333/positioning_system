#pragma once
#include "DataMessage.hpp"
#include "Block.hpp"
#include "PID_values.hpp"
#include "Vector3D.hpp"

class ControlSystemMessage : public DataMessage{

private:
    Block* _to_remove;
    Block* _to_add;
    msg_type _type;
    control_system_msg_type _control_system_msg_type;
    PID_parameters* _pid_para;
    control_system _source;
    control_system _destination;
    Vector3D<float> _v3d_data;
    float _data;

public:
    msg_type getType();
    const int getSize();
    Block* getBlockToRemove();
    Block* getBlockToAdd();
    control_system_msg_type getControlSystemMsgType();
    PID_parameters* getPIDSettings();
    control_system getSource();
    control_system getDestination();
    float getData();
    Vector3D<float> getV3DData();

    ControlSystemMessage();

    void setControlSystemMessage(control_system_msg_type, Block*, Block*);
    void setControlSystemMessage(control_system_msg_type, Block*);
    void setControlSystemMessage(control_system_msg_type, PID_parameters*);
    void setControlSystemMessage(control_system, control_system_msg_type, Vector3D<float>);
    void setControlSystemMessage(control_system, control_system_msg_type, float);

    ~ControlSystemMessage();
};