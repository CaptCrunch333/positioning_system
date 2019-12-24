#pragma once
#include "Switcher.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include <vector>
#include "ControlSystemMessage.hpp"
#include "PID_values.hpp"
#include "UpdatePoseMessage.hpp"
#include "PVProvider.hpp"
#include "ReferenceMessage.hpp"
#include "TimedBlock.hpp"
#include "ROSMsg.hpp"

class ControlSystem : public TimedBlock, public msg_emitter, public msg_receiver{

    private:
        control_system _control_system;
        Switcher* controllerSwitcher;
        Switcher* referenceSwitcher;
        PVProvider* _providerProcessVariable;
        std::vector<Switcher*> _switchers;
        block_frequency _frequency;
        // ControlSystem();
        ReferenceMessage m_ref_msg_x;
        ReferenceMessage m_ref_msg_y;
        ReferenceMessage m_ref_msg_z;
        ReferenceMessage m_ref_msg_yaw;

        ControlSystemMessage m_output_msg;

        ROSMsg m_ros_msg;

        ReferenceMessage m_ref_out_msg;

        ControlSystemMessage m_provider_data_msg;
        ControlSystemMessage m_switch_msg;
        ControlSystemMessage m_add_block_msg;
        ControlSystemMessage m_change_PID_msg;
        float _dt;

    public:
        void receive_msg_data(DataMessage* t_msg);
        void getStatus();
        void switchBlock(Block* t_from, Block* t_to);
        void addBlock(Block* t_block);
        void changePIDSettings(PID_parameters* t_pid_para);
        control_system getControlSystemType();
        void loopInternal();
                
        Switcher* getControllerSwitcher();
        Switcher* getReferenceSwitcher();
        // Switcher* getProviderSwitcher();

        ControlSystem(control_system, PVProvider*, block_frequency);
        ~ControlSystem(); //TODO prevent automatic storage

    
};