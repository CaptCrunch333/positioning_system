#include "ROSUnit_BroadcastData.hpp"
ROSUnit_BroadcastData* ROSUnit_BroadcastData::_instance_ptr = NULL;

ROSUnit_BroadcastData::ROSUnit_BroadcastData(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _pos_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("body_position", 10);
    _ori_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("body_orientation", 10);
    _xpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("x_provider", 10);
    _ypv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("y_provider", 10);
    _zpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("z_provider", 10);
    _rollpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("roll_provider", 10);
    _pitchpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("pitch_provider", 10);
    _yawpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("yaw_provider", 10);
    _cs_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("control_systems_output", 10);
    _act_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("actuation_output", 10);

    _att.roll = 0;
    _head.yaw = 0;

    _instance_ptr = this;
}

ROSUnit_BroadcastData::~ROSUnit_BroadcastData() {

}


void ROSUnit_BroadcastData::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::ROS){
        ROSMsg* ros_msg = (ROSMsg*)t_msg;

        if(ros_msg->getROSMsgType() == ros_msg_type::POSITION){
            PositionMsg pos = ros_msg->getPosition();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_pos;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "body";
            msg.point.x = pos.x;
            msg.point.y = pos.y;
            msg.point.z = pos.z;
            _pos_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::ORIENTATION){
            if(ros_msg->getAttitude().roll != 0){
                _att = ros_msg->getAttitude();
            }
            //TODO refactor this. only for test.
            if(abs(ros_msg->getHeading().yaw) > 0.00001){
                _head = ros_msg->getHeading();
            }
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_ori;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "body";
            msg.point.x = _att.roll;
            msg.point.y = _att.pitch;
            msg.point.z = _head.yaw;
            _ori_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::X_PV){
            Vector3D<float> xpv = ros_msg->getX_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_xpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = xpv.x;
            msg.point.y = xpv.y;
            msg.point.z = xpv.z;
            _xpv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::Y_PV){
            Vector3D<float> ypv = ros_msg->getY_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_ypv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = ypv.x;
            msg.point.y = ypv.y;
            msg.point.z = ypv.z;
            _ypv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::Z_PV){
            Vector3D<float> zpv = ros_msg->getZ_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_zpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = zpv.x;
            msg.point.y = zpv.y;
            msg.point.z = zpv.z;
            _zpv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::ROLL_PV){
            Vector3D<float> rollpv = ros_msg->getRoll_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_rollpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = rollpv.x;
            msg.point.y = rollpv.y;
            msg.point.z = rollpv.z;
            _rollpv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::PITCH_PV){
            Vector3D<float> pitchpv = ros_msg->getPitch_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_pitchpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = pitchpv.x;
            msg.point.y = pitchpv.y;
            msg.point.z = pitchpv.z;
            _pitchpv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::YAW_PV){
            Vector3D<float> yawpv = ros_msg->getYaw_PV();
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_yawpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = yawpv.x;
            msg.point.y = yawpv.y;
            msg.point.z = yawpv.z;
            _yawpv_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::CONTROLSYSTEM){
            int i = (int)ros_msg->getSource();
            _cs_outputs[i] = ros_msg->getControlSystem();

            std_msgs::Float64MultiArray msg;
            msg.data = _cs_outputs;
            _cs_prov_pub.publish(msg);

        }else if(ros_msg->getROSMsgType() == ros_msg_type::ACTUATION){
            float* pointer = ros_msg->getActuation();

            for(int i=0;i<6;i++){
                _act_outputs[i] = *pointer++;
            }
            
            std_msgs::Float64MultiArray msg;
            msg.data = _act_outputs;
            _act_prov_pub.publish(msg);
        }

    }

}