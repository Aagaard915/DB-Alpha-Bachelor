/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

/* Edit by Mathias Thor --> 2018 - July 09 */

#include "dynamixel_workbench_controllers/position_control.h"

PositionControl::PositionControl()
        : node_handle_("") {
    std::string device_name = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
    uint32_t dxl_baud_rate = node_handle_.param<int>("baud_rate", 4000000);

    uint8_t scan_range = node_handle_.param<int>("scan_range", 200);

    uint32_t profile_velocity = node_handle_.param<int>("profile_velocity", 200);
    uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

    dxl_wb_ = new DynamixelWorkbench;

    dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

    if (!dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range)) {
        ROS_ERROR("Not found Motors, Please check scan range or baud rate");
        ros::shutdown();
        return;
    }

    initMsg();

    for (int index = 0; index < dxl_cnt_; index++) {
        dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);
        dxl_id_vector.push_back(dxl_id_[index]);
    }

    dxl_wb_->initBulkRead();
    dxl_wb_->addSyncWrite("Goal_Position");
    dxl_wb_->addSyncRead("ID");

    initPublisher();
    initSubscriber();
    initServer();
//    Incomment for loose joints when reading motorpositions:
//    for (int index = 0; index < dxl_cnt_; index++)
//        dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
}

PositionControl::~PositionControl() {
    for (int index = 0; index < dxl_cnt_; index++)
        dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

    ros::shutdown();
}

void PositionControl::initMsg() {
    printf("--------------------------------------------------------------------------\n");
    printf("\n"
           "  ____                              _          _  \n"
           " |  _ \\ _   _ _ __   __ _ _ __ ___ (_)_  _____| | \n"
           " | | | | | | | '_ \\ / _` | '_ ` _ \\| \\ \\/ / _ \\ | \n"
           " | |_| | |_| | | | | (_| | | | | | | |>  <  __/ | \n"
           " |____/ \\__, |_| |_|\\__,_|_| |_| |_|_/_/\\_\\___|_| \n"
           "  ____  |___/ ____    ____       _                \n"
           " |  _ \\ / _ \\/ ___|  |  _ \\ _ __(_)_   _____ _ __ \n"
           " | |_) | | | \\___ \\  | | | | '__| \\ \\ / / _ \\ '__|\n"
           " |  _ <| |_| |___) | | |_| | |  | |\\ V /  __/ |   \n"
           " |_| \\_\\\\___/|____/  |____/|_|  |_| \\_/ \\___|_|   \n"
           "                                                  \n");
    printf("--------------------------------------------------------------------------\n");
    printf("\n");

    for (int index = 0; index < dxl_cnt_; index++) {
        printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
        printf("ID      : %d\n", dxl_id_[index]);
        printf("\n");
    }
    printf("--------------------------------------------------------------------------\n");
}

void PositionControl::initPublisher() {
    //joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_ID_pub_ = node_handle_.advertise<std_msgs::Int32MultiArray>("joint_IDs", 1);
    joint_position_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_positions", 1);
    joint_torque_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_torques", 1);
    joint_velocity_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_velocities", 1);
    joint_errorStatus_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_errorStates", 1);
    joint_inputVoltage_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_inputVoltage", 1);
    joint_temperature_pub_ = node_handle_.advertise<std_msgs::Float32MultiArray>("joint_temperature", 1);

}

void PositionControl::initSubscriber() {
    multi_joint_command_server_ = node_handle_.subscribe("multi_joint_command", 1, &PositionControl::multiJointCommandMsgCallback, this);
    joint_reboot_ = node_handle_.subscribe("joint_reboot", 1, &PositionControl::jointRebootCallback, this);

}

void PositionControl::initServer() {
    joint_command_server_ = node_handle_.advertiseService("joint_command", &PositionControl::jointCommandMsgCallback, this);

}

void PositionControl::jointStatePublish() {
    //sensor_msgs::JointState dynamixel_;
    //dynamixel_.header.stamp = ros::Time::now();

    // IDs Vector:
    std_msgs::Int32MultiArray IDs;
    // Position Vector:
    std_msgs::Float32MultiArray positions;
    // Velocities Vector:
    std_msgs::Float32MultiArray velocities;
    // Torques Vector:
    std_msgs::Float32MultiArray torques;
    // Error States Vector:
    std_msgs::Float32MultiArray errorStates;
    // Input Voltages Vector:
    std_msgs::Float32MultiArray inputVoltages;
    // Input Voltages Vector:
    std_msgs::Float32MultiArray temperatures;

    // SYNC READ METHOD
    std::vector<std::vector<int32_t>> id = dxl_wb_->syncRead("ID");

    for (int index = 0; index < dxl_cnt_; index++) {
        positions.data.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], id[0][index])); // Converted to rad
        velocities.data.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], id[1][index])); // Converted to rad/s
        torques.data.push_back(dxl_wb_->convertValue2Torque(dxl_id_[index]*2.69, id[2][index])); // Converted to Nm
        errorStates.data.push_back(id[3][index]);
        inputVoltages.data.push_back(id[4][index]*0.1); // Converted to Volt
        temperatures.data.push_back(id[5][index]); // Already in degree

        IDs.data.push_back(dxl_id_[index]);
    }

    //joint_states_pub_.publish(dynamixel_);
    joint_ID_pub_.publish(IDs);
    joint_position_pub_.publish(positions);
    joint_velocity_pub_.publish(velocities);
    joint_torque_pub_.publish(torques);
    joint_errorStatus_pub_.publish(errorStates);
    joint_inputVoltage_pub_.publish(inputVoltages);
    joint_temperature_pub_.publish(temperatures);

    IDsLocal = IDs.data;
    errorStatesLocal = errorStates.data;
}

void PositionControl::controlLoop() {
    jointStatePublish();
}

bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res) {
    int32_t goal_position = 0;

    if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
    } else {
        goal_position = req.goal_position;
    }

    bool ret = dxl_wb_->goalPosition(req.id, goal_position);
    res.result = static_cast<unsigned char>(ret);
}

void PositionControl::jointRebootCallback(const std_msgs::Int32 &_ID){
    // RESET JOINTS WITH ERRORS (-1)
    if(_ID.data == -1){
        for (int i = 0; i < errorStatesLocal.size(); ++i) {
            if(errorStatesLocal[i] != 0) {
                if (!dxl_wb_->reboot(IDsLocal[i])) {
                    ROS_ERROR("COULD NOT REBOOT JOINT ID_%i", IDsLocal[i]);
                } else
                    ROS_INFO("JOINT ID_%i REBOOTED", IDsLocal[i]);

            }
        }
    }
    // RESET ALL (-2)
    else if(_ID.data == -2){
        for (int i = 0; i < errorStatesLocal.size(); ++i) {
            if (!dxl_wb_->reboot(IDsLocal[i])) {
                ROS_ERROR("COULD NOT REBOOT JOINT ID_%i", IDsLocal[i]);
            } else
                ROS_INFO("JOINT ID_%i REBOOTED", IDsLocal[i]);
        }
    }
    // RESET SPECIFIC JOINT (ID)
    else {
        if (!dxl_wb_->reboot(_ID.data)){
            ROS_ERROR("COULD NOT REBOOT JOINT ID_%i", _ID.data);
        } else
            ROS_INFO("JOINT ID_%i REBOOTED", _ID.data);
    }

}

void PositionControl::multiJointCommandMsgCallback(const std_msgs::Float32MultiArray &_IDnPOS){

    // Expecting array to be like: ID, POS, ID, POS, ID, POS, ID, POS....
    std::vector<float> IDnPOS = _IDnPOS.data;
    int32_t goal_dxl_position[IDnPOS.size()] = {0,};

    for (int index = 0; index < IDnPOS.size(); index+=2) {
        if(std::find(dxl_id_vector.begin(), dxl_id_vector.end(), IDnPOS[index]) != dxl_id_vector.end()) {
            goal_dxl_position[index] = IDnPOS[index];
            goal_dxl_position[index+1] = dxl_wb_->convertRadian2Value(IDnPOS[index], IDnPOS[index + 1]);
        } else
            ROS_ERROR("ID WAS NOT FOUND");
    }

    if (!dxl_wb_->syncWrite("Goal_Position", goal_dxl_position, IDnPOS.size())) {
        ROS_ERROR("ERROR IN SYNCWRITE. WRONG SET-POINT OR SAME ID TWICE?");
    }

}

int main(int argc, char **argv) {
    // Init ROS node

    ros::init(argc, argv, "dynamixel_ROS_driver");
    PositionControl pos_ctrl;
    ros::Rate loop_rate(60);
    while (ros::ok()) {
        pos_ctrl.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
