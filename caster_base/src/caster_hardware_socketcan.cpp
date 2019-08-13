#include "caster_base/caster_hardware_socketcan.h"

#include <math.h>
#include <errno.h>

const uint16_t kCanOpenSendHeader = 0x600;
const uint16_t kCanOpenRecvHeader = 0x580;

/**
* Initialize Caster hardware
*/
iqr::CasterHardware::CasterHardware() {

}

std::string iqr::CasterHardware::ToBinary(size_t data, uint8_t length) {
  std::string data_binary = "";
  for(int i=0; i<length*8; i++) {
    data_binary.append(std::to_string((data>>i)&0x01));
  }

  return data_binary;
}

void iqr::CasterHardware::ControllerTimerCallback(const ros::TimerEvent&) {
  UpdateHardwareStatus();
  controller_manager_->update(ros::Time::now(), ros::Duration(0.1));
  WriteCommandsToHardware();
}

bool iqr::CasterHardware::SetDigitalOutputCB(caster_base::SetDigitalOutput::Request &req, caster_base::SetDigitalOutput::Response &res) {
  ROS_INFO("set digital output %d to %d", req.io, req.active);

  if(req.io>=1 && req.io<=4) {
    if(req.active == true) {
      Command(kSetIndividualDO, 0x00, static_cast<uint8_t>(req.io), 1);
    } else if (req.active == false) {
      Command(kResetIndividualDO, 0x00, static_cast<uint8_t>(req.io), 1);
    }

    res.result = true;
  } else {
    ROS_WARN("digital pin should be in 1-4, now is %d", req.io);
    res.result = false;
  }

  return true;
}

void iqr::CasterHardware::Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node_name_ = node_name;
  nh_ = nh;
  private_nh_ = private_nh;

  private_nh_.param<std::string>("can_send_topic", send_topic_, "sent_messages");
  private_nh_.param<std::string>("can_receive_topic", receive_topic_, "received_messages");
  private_nh_.param<int>("can_id", can_id_, 1);
  private_nh_.param<std::string>("left_wheel_joint", left_wheel_joint_, "drive_wheel_left_joint");
  private_nh_.param<std::string>("right_wheel_joint", right_wheel_joint_, "drive_wheel_right_joint");

  can_pub_ = nh_.advertise<can_msgs::Frame>(send_topic_, 1000);
  can_sub_ = nh_.subscribe<can_msgs::Frame>(receive_topic_, 10, &iqr::CasterHardware::CanReceiveCallback, this);

  set_io_service_ = private_nh_.advertiseService("set_digital_output", &iqr::CasterHardware::SetDigitalOutputCB, this);

  controller_manager_ = new controller_manager::ControllerManager(this, nh);
  timer_ = nh.createTimer(ros::Duration(0.025), &iqr::CasterHardware::ControllerTimerCallback, this);

  RegisterControlInterfaces();

  diagnostic_updater_.setHardwareID("caster_robot");
  diagnostic_updater_.add("Left motor", this, &iqr::CasterHardware::LeftMotorCheck);
  diagnostic_updater_.add("Right motor", this, &iqr::CasterHardware::RightMotorCheck);
  diagnostic_updater_.add("Status", this, &iqr::CasterHardware::StatusCheck);
  diagnostic_updater_.add("Controller", this, &iqr::CasterHardware::ControllerCheck);

  // Clear();

  ROS_INFO("can_pub: %s, can_sub: %s, can_id: %d", send_topic_.c_str(), receive_topic_.c_str(), can_id_);
  ROS_INFO("caster base initialized");
}

void iqr::CasterHardware::LeftMotorCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  /* request motor flags 
   * f1 = Amps Limit currently active
   * f2 = Motor stalled
   * f3 = Loop Error detected
   * f4 = Safety Stop active
   * f5 = Forward Limit triggered
   * f6 = Reverse Limit triggered
   * f7 = Amps Trigger activated
  */

  status.add("current (A)", motor_status_[kLeftMotor].current);
  status.add("speed (RPM)", motor_status_[kLeftMotor].rpm);

  // ROS_INFO("motor %s", ToBinary(motor_status_[kLeftMotor].status, 1).c_str());

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if((motor_status_[kLeftMotor].status)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Limit currently active");
  }
  if((motor_status_[kLeftMotor].status>>1)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor stalled");
  }
  if((motor_status_[kLeftMotor].status>>2)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Loop Error detected");
  }
  //if(motor_status_[kLeftMotor].status&0x08 == 0x08) {
  if((motor_status_[kLeftMotor].status>>3)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Safety Stop active");
  }
  if((motor_status_[kLeftMotor].status>>4)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Forward Limit triggered");
  }
  if((motor_status_[kLeftMotor].status>>5)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Reverse Limit triggered");
  }
  if((motor_status_[kLeftMotor].status>>6)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Trigger activated");
  }
}

void iqr::CasterHardware::RightMotorCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  /* request motor flags 
   * f1 = Amps Limit currently active
   * f2 = Motor stalled
   * f3 = Loop Error detected
   * f4 = Safety Stop active
   * f5 = Forward Limit triggered
   * f6 = Reverse Limit triggered
   * f7 = Amps Trigger activated
  */

  status.add("current (A)", motor_status_[kRightMotor].current);
  status.add("speed (RPM)", motor_status_[kRightMotor].rpm);
  // status.add("counter", motor_status_[kRightMotor].counter);

  // ROS_INFO("motor %s", ToBinary(motor_status_[kLeftMotor].status, 1).c_str());

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if((motor_status_[kRightMotor].status)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Limit currently active");
  }
  if((motor_status_[kRightMotor].status>>1)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor stalled");
  }
  if((motor_status_[kRightMotor].status>>2)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Loop Error detected");
  }
  if((motor_status_[kRightMotor].status>>3)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Safety Stop active");
  }
  if((motor_status_[kRightMotor].status>>4)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Forward Limit triggered");
  }
  if((motor_status_[kRightMotor].status>>5)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Reverse Limit triggered");
  }
  if((motor_status_[kRightMotor].status>>6)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Trigger activated");
  }
}

void iqr::CasterHardware::StatusCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  /* request status flags 
   * f1 = Serial mode
   * f2 = Pulse mode
   * f3 = Analog mode
   * f4 = Power stage off
   * f5 = Stall detected
   * f6 = At limit
   * f7 = Unused
   * f8 = MicroBasic script running
  */

  std::string contorl_mode = "unknown";

  if((status_flags_)&0x01 == 0x01) {
    contorl_mode = "Serial mode";
  }
  if((status_flags_>>1)&0x01 == 0x01) {
    contorl_mode = "Pulse mode";
  }
  if((status_flags_>>2)&0x01 == 0x01) {
    contorl_mode = "Analog mode";
  }

  status.add("Control mode", contorl_mode);

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if(contorl_mode == "unknown") {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "unknown control mode");
  }
  if((status_flags_>>3)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Power stage off");
  }
  if((status_flags_>>4)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Stall detected");
  }
  if((status_flags_>>5)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "At limit");
  }
  if((status_flags_>>6)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unused");
  }
  if((status_flags_>>7)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "MicroBasic script running");
  }
}

void iqr::CasterHardware::ControllerCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  /* request fault flags
   * f1 = Overheat
   * f2 = Overvoltage
   * f3 = Undervoltage
   * f4 = Short circuit
   * f5 = Emergency stop
   * f6 = Brushless sensor fault
   * f7 = MOSFET failure
   * f8 = Default configuration loaded at startup
  */

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if((fault_flags_)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Overheat");
  }
  if((fault_flags_>>1)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Overvoltage");
  }
  if((fault_flags_>>2)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Undervoltage");
  }
  if((fault_flags_>>3)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Short circuit");
  }
  if((fault_flags_>>4)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Emergency stop");
  }
  if((fault_flags_>>5)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Brushless sensor fault");
  }
  if((fault_flags_>>6)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "MOSFET failure");
  }
  if((fault_flags_>>7)&0x01 == 0x01) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Default configuration loaded at startup");
  }
}

void iqr::CasterHardware::CanReceiveCallback(const can_msgs::Frame::ConstPtr& msg) {
  // ROS_INFO_STREAM("get msg: " << msg->id);
  if(msg->id == kCanOpenRecvHeader+can_id_) {
    // ROS_INFO("data length: %d", msg->dlc);

    if((msg->data[0]&0xF0) == kQuery<<4) {
      uint16_t index = (msg->data[2]<<8) + msg->data[1];
      uint8_t sub_index = msg->data[3];
      
      // ROS_INFO("Response: query %02X,%02X successful", index, sub_index);

      switch (index) {
        case kReadMotorAmps: {
          int16_t temp_current_x10;
          memcpy(&temp_current_x10, msg->data.data()+4, 2);
          motor_status_[sub_index-1].current = temp_current_x10 / 10.0;
          // ROS_INFO("Query response, motor current %d: %f", sub_index, motor_status_[sub_index-1].current);
          break;
        }
        case kReadAbsBLCounter: {
          memcpy(&motor_status_[sub_index-1].counter, msg->data.data()+4, 4);
          if(motor_status_[sub_index-1].counter_reset == false) {
            motor_status_[sub_index-1].counter_offset = motor_status_[sub_index-1].counter;
            motor_status_[sub_index-1].counter_reset = true;
          }
          // ROS_INFO("Query response, counter %d: %d", sub_index, motor_status_[sub_index-1].counter);
          break;
        }
        case kReadBLMotorRPM: {
          memcpy(&motor_status_[sub_index-1].rpm, msg->data.data()+4, 2);
          // ROS_INFO("Query response, rpm %d: %d", sub_index, motor_status_[sub_index-1].rpm);
          break;
        }
        case kReadStatusFlags: {
          status_flags_ = msg->data[4];
          // ROS_INFO("Query response, status flags %s", ToBinary(status_flags_, 1).c_str());
          break;
        }
        case kReadFaultFlags: {
          fault_flags_ = msg->data[4];
          // ROS_INFO("Query response, fault flags %s", ToBinary(fault_flags_, 1).c_str());
          break;
        }
        case kReadMotorStatusFlags: {
          /* TODO: the index of data is different from doc */
          motor_status_[kLeftMotor].status = msg->data[4];
          motor_status_[kRightMotor].status = msg->data[6];
          // ROS_INFO("Query response, motor status flags %s: %s", ToBinary(motor_status_[kLeftMotor].status, 1).c_str(), ToBinary(motor_status_[kRightMotor].status, 1).c_str());
          break;
        }
        default: {

        }
      }
    } else if((msg->data[0]&0xF0) == kResponseCommandSuccess<<4) {
      uint16_t index = (msg->data[2]<<8) + msg->data[1];
      uint8_t sub_index = msg->data[3];
      // ROS_INFO("Response: command %02X,%02X successful", index, sub_index);
    }
  }
}

void iqr::CasterHardware::Clear() {
  /* Clear BL Counter */
  // Command(kSetBLCounter, static_cast<uint8_t>(kLeftMotor+1), 0, 4);
  // Command(kSetBLCounter, static_cast<uint8_t>(kRightMotor+1), 0, 4);
  // motor_status_[kLeftMotor].counter_offset = motor_status_[kLeftMotor].counter;
  // motor_status_[kRightMotor].counter_offset = motor_status_[kRightMotor].counter;
}

void iqr::CasterHardware::UpdateHardwareStatus() {
  bool success = false;
  uint32_t data;

  /* request motor speed */
  // int16_t l_rpm=-1, r_rpm=-1;
  // uint32_t left_rpm=-1, right_rpm=-1;
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor+1), 2);
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor+1), 2);
  // l_rpm = static_cast<int16_t>(left_rpm);
  // r_rpm = static_cast<int16_t>(right_rpm);

  /* request motor counter */
  // int32_t l_count=-1, r_count=-1;
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor+1), 4);
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kRightMotor+1), 4);

  // uint8_t status_flag;
  success = Query(kReadStatusFlags, 0x00, 1);
  // status_flag = static_cast<uint8_t>(data);

  // uint8_t fault_flag;
  success = Query(kReadFaultFlags, 0x00, 1);
  // fault_flag = static_cast<uint8_t>(data);

  /* TODO: strange rules for opencan id */
  // uint8_t left_motor_flag, right_motor_flag;
  success = Query(kReadMotorStatusFlags, 0x01, 4);
  // left_motor_flag = data;
  // right_motor_flag = data >> 16;

  success = Query(kReadMotorAmps, 0x01, 4);
  success = Query(kReadMotorAmps, 0x02, 4);

  joints_[kLeftMotor].velocity = motor_status_[kLeftMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].velocity = motor_status_[kRightMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  joints_[kLeftMotor].position = (motor_status_[kLeftMotor].counter-motor_status_[kLeftMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].position = (motor_status_[kRightMotor].counter-motor_status_[kRightMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  diagnostic_updater_.update();

  // ROS_INFO("motor counter: %d, %d, %d, %d", motor_status_[kLeftMotor].counter, motor_status_[kRightMotor].counter, motor_status_[kLeftMotor].rpm, motor_status_[kRightMotor].rpm);
  // ROS_INFO("motor counter: %f, %f, %d, %d", joints_[0].position, joints_[1].position, l_rpm, r_rpm);
  // ROS_INFO("status: %s, fault: %s, left: %s, right: %s", \
            ToBinary(status_flag, sizeof(status_flag)).c_str(), ToBinary(fault_flag, sizeof(fault_flag)).c_str(), \
            ToBinary(left_motor_flag, sizeof(left_motor_flag)).c_str(), ToBinary(right_motor_flag, sizeof(right_motor_flag)).c_str());
}

/**
* Get current encoder travel offsets from MCU and bias future encoder readings against them
*/
void iqr::CasterHardware::ResetTravelOffset() {

}

/**
* Register interfaces with the RobotHW interface manager, allowing ros_control operation
*/
void iqr::CasterHardware::RegisterControlInterfaces() {
  hardware_interface::JointStateHandle left_wheel_joint_state_handle(left_wheel_joint_, &joints_[0].position, &joints_[0].velocity, &joints_[0].effort);
  joint_state_interface_.registerHandle(left_wheel_joint_state_handle);

  hardware_interface::JointHandle left_wheel_joint_handle(left_wheel_joint_state_handle, &joints_[0].velocity_command);
  velocity_joint_interface_.registerHandle(left_wheel_joint_handle);

  hardware_interface::JointStateHandle right_wheel_joint_state_handle(right_wheel_joint_, &joints_[1].position, &joints_[1].velocity, &joints_[1].effort);
  joint_state_interface_.registerHandle(right_wheel_joint_state_handle);

  hardware_interface::JointHandle right_wheel_joint_handle(right_wheel_joint_state_handle, &joints_[1].velocity_command);
  velocity_joint_interface_.registerHandle(right_wheel_joint_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void iqr::CasterHardware::WriteCommandsToHardware() {
  int32_t speed[2];

  speed[0] = static_cast<int32_t>(joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // int16_t s_v = ntohl(speed);
  // memcpy(buf+5, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kLeftMotor+1), static_cast<uint32_t>(speed[0]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);

  speed[1] = static_cast<int32_t>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60) * -1.0;
  // s_v = ntohl(speed);
  // memcpy(buf+9, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kRightMotor+1), static_cast<uint32_t>(speed[1]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);

  // ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}


void iqr::CasterHardware::SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  uint8_t buf[8];
  bzero(buf, 8);

  can_msgs::Frame frame;
  frame.header.stamp = ros::Time::now();

  frame.id = kCanOpenSendHeader + node_id;
  frame.is_rtr = 0;
  frame.is_extended = 0;
  frame.is_error = 0;
  frame.dlc = 8;

  /* set command type and data length */
  buf[0] = (type<<4) + ((4-data_length) << 2);

  /* set index and sub index*/
  memcpy(buf+1, &index, 2);
  memcpy(buf+3, &sub_index, 1);

  /* set data */
  memcpy(buf+4, &data, data_length);

  for(int i=0; i<8; i++) {
    frame.data[i] = buf[i];
  }

  /* send */
  // ROS_INFO("sending");
  can_pub_.publish(frame);
  // ROS_INFO("Sent");
}

bool iqr::CasterHardware::Query(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint8_t data_length) {
  SendCanOpenData(can_id_, kQuery, query, sub_index, 0x0000, data_length);

  return true;
}

bool iqr::CasterHardware::Command(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  SendCanOpenData(can_id_, kCommand, query, sub_index, data, data_length);

  return true;
}