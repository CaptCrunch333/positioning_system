#pragma once
#include "common_types.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"
#include "FlightScenarioMessage.hpp"

class UserReference : public msg_emitter, public msg_receiver{

private:
    UpdatePoseMessage _user_msg;

public:

    virtual void receive_msg_data(DataMessage*) = 0;

    UserReference();
    ~UserReference();
};