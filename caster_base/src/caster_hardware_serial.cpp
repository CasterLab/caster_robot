#include "caster_base/caster_hardware_serial.h"

#include <math.h>
#include <errno.h>

/**
* Initialize Caster hardware
*/
iqr::CasterHardware::CasterHardware() {

}

iqr::CasterHardware::~CasterHardware() {
  if(serial_port_driver_.isOpen()) {
    serial_port_driver_.close();
  }
  
  if(serial_port_body_.isOpen()) {
    serial_port_body_.close();
  }
}

uint16_t iqr::CasterHardware::CRC16 (const uint8_t *data, uint16_t length) {
  static const uint16_t crc_table[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

  uint8_t temp;
  uint16_t crc_word = 0xFFFF;

  while(length--) {
    temp = *data++ ^ crc_word;
    crc_word >>= 8;
    crc_word ^= crc_table[temp];
  }
  return crc_word;
}

std::string iqr::CasterHardware::ToBinary(size_t data, uint8_t length) {
  std::string data_binary = "";
  for(int i=0; i<length*8; i++) {
    data_binary.append(std::to_string((data>>i)&0x01));
  }

  return data_binary;
}

void iqr::CasterHardware::ControllerTimerCallback(const ros::TimerEvent&) {
  static ros::Time start_time = ros::Time::now();

  UpdateHardwareStatus();
  SerialReadUpadata();

  ros::Duration delta = ros::Time::now() - start_time;
  controller_manager_->update(ros::Time::now(), delta);
  // controller_manager_->update(ros::Time::now(), ros::Duration(0.025));

  // ROS_INFO("delta: %lf", delta.toSec()*1000.0);
  start_time = ros::Time::now();
  
  WriteCommandsToHardware();
}

bool iqr::CasterHardware::SetDigitalOutputCB(caster_base::SetDigitalOutput::Request &req, caster_base::SetDigitalOutput::Response &res) {
  ROS_INFO("set digital output %d to %d", req.io, req.active);

  if(req.io>=1 && req.io<=4) {
    if(req.active == true) {
      std::string setIndividual = "!D0 " + std::to_string(req.io) + "\r";
      serial_port_driver_.write(setIndividual);
    } else if (req.active == false) {
      std::string resetIndividual = "!D1 " + std::to_string(req.io) + "\r";
      serial_port_driver_.write(resetIndividual);
    }

    res.result = true;
  } else {
    ROS_WARN("digital pin should be in 1-4, now is %d", req.io);
    res.result = false;
  }

  return true;
}

bool iqr::CasterHardware::SerialPortInit(serial::Serial &serial_option, std::string port, int baudrate) {
	try {
	  serial_option.setPort(port);
	  serial_option.setBaudrate(baudrate);
	  serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(100);
	  serial_option.setTimeout(serial_timeout);
	  serial_option.open();
	  serial_option.setRTS(false);
	  serial_option.setDTR(false);
	} catch (serial::IOException& e) {
	  ROS_ERROR_STREAM("Unable to open serial port:" + port);
	  // return false;
	}
}

void iqr::CasterHardware::Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node_name_ = node_name;
  nh_ = nh;
  private_nh_ = private_nh;

  private_nh_.param<int>("base_baudrate", baudrate_driver_, 115200);
  private_nh_.param<std::string>("base_port", port_driver_, "/dev/caster_base");
  private_nh_.param<int>("body_baudrate", baudrate_body_, 115200);
  private_nh_.param<std::string>("body_port", port_body_, "/dev/caster_body");

  private_nh_.param<std::string>("body_joint", body_joint_name_, "");
  private_nh_.param<std::string>("left_wheel_joint", left_wheel_joint_, "drive_wheel_left_joint");
  private_nh_.param<std::string>("right_wheel_joint", right_wheel_joint_, "drive_wheel_right_joint");

  set_io_service_ = private_nh_.advertiseService("set_digital_output", &iqr::CasterHardware::SetDigitalOutputCB, this);

  controller_manager_ = new controller_manager::ControllerManager(this, nh);
  timer_ = nh.createTimer(ros::Duration(0.04), &iqr::CasterHardware::ControllerTimerCallback, this);

  RegisterControlInterfaces();

  if(body_joint_name_ != "") {
    ROS_INFO_STREAM("Start body control");
	  SerialPortInit(serial_port_body_, port_body_, baudrate_body_);
  }
  SerialPortInit(serial_port_driver_, port_driver_, baudrate_driver_);

  diagnostic_updater_.setHardwareID("caster_robot");
  diagnostic_updater_.add("Left motor", this, &iqr::CasterHardware::LeftMotorCheck);
  diagnostic_updater_.add("Right motor", this, &iqr::CasterHardware::RightMotorCheck);
  diagnostic_updater_.add("Status", this, &iqr::CasterHardware::StatusCheck);
  diagnostic_updater_.add("Controller", this, &iqr::CasterHardware::ControllerCheck);
  
  // Clear();

  // ROS_INFO("can_pub: %s, can_sub: %s, can_id: %d", send_topic_.c_str(), receive_topic_.c_str(), can_id_);
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

  status.add("Speed (RPM)", motor_status_[kLeftMotor].rpm);
  status.add("Current (A)", motor_status_[kLeftMotor].current);
  status.add("Temperature (℃)", motor_status_[kLeftMotor].temperature);
  status.add("Temperature MCU (℃)", motor_status_[kLeftMotor].temperature_MCU);

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

  status.add("Speed (RPM)", motor_status_[kRightMotor].rpm);
  status.add("Current (A)", motor_status_[kRightMotor].current);
  status.add("Temperature (℃)", motor_status_[kRightMotor].temperature);
  status.add("Temperature MCU (℃)", motor_status_[kRightMotor].temperature_MCU);
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
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Unknown control mode");
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
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Emergency stop");
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

int16_t* iqr::CasterHardware::BufferSpilt(std::string buf_driver) {
  
  // datacheck
  // find
  int datacheck = 1;
  int index_data = buf_driver.find("?T 1");

  if (index_data != -1)
  {
    char *data_n = new char[20];
    char *buf_c = new char[1000];
    char const *delim1 = "\r";
    char const *delim2 = "=";
    std::vector<std::string> buf_spilt(27);

    buf_driver = buf_driver.substr(index_data);
    strcpy(buf_c, buf_driver.c_str());
    buf_spilt[0] = strtok(buf_c, delim1); 

    for (int i = 1; i < 26; ++i)
    {
      // ROS_INFO("%s", buf_spilt[i-1].c_str());
      data_n = strtok(NULL, delim1);
      if (data_n != NULL)
      {
        buf_spilt[i] = data_n;
      }
      else {
        datacheck = 0;
        i = 26;
      }
    }
    // ROS_INFO("%s", buf_spilt[25].c_str());
    if (datacheck == 1)
    {
      for (int i = 1; i < 26; i+=2)
      {
        char *buf_n = new char[20];
        strcpy(buf_n, buf_spilt[i].c_str());
        char *temp2 = strtok(buf_n, delim2);
        if (temp2 == data_type[(i-1)/2])
        {
          temp2 = strtok(NULL, delim2);
          num[(i-1)/2] = atoi(temp2);
        }
      }
    }
  }
  return num;
}

void iqr::CasterHardware::SerialReadUpadata() {

  std::string buf_driver; 
  if (serial_port_driver_.available())
  {
    serial_port_driver_.read(buf_driver, serial_port_driver_.available());
    // for (int i = 0; i < buf_driver.size(); i++)
    // {
    //   ROS_INFO("%c", *(buf_driver.c_str()+i));
    // }

    if (buf_driver.size() != 0)
    {  
      int16_t *data = BufferSpilt(buf_driver);

      for (int index_cc = 0; index_cc < 2; ++index_cc)
      {
        motor_status_[index_cc].temperature = data[1+index_cc];
        motor_status_[index_cc].temperature_MCU = data[0];
        motor_status_[index_cc].counter = data[5+index_cc];
        if(motor_status_[index_cc].counter_reset == false) {
          motor_status_[index_cc].counter_offset = motor_status_[index_cc].counter;
          motor_status_[index_cc].counter_reset = true;
        }
        motor_status_[index_cc].current = data[3+index_cc] / 10.0;
        motor_status_[index_cc].rpm = data[7+index_cc];
        motor_status_[index_cc].status = data[9+index_cc];
      } 
      status_flags_ = data[11];
      fault_flags_ = data[12];  
      // ROS_INFO("Response: querysuccessful");
      // ROS_INFO("Query response, motor current 0: %f", motor_status_[0].current);
      // ROS_INFO("Query response, motor current 1: %f", motor_status_[1].current);
      // ROS_INFO("Query response, motor Tem MCU: %i", motor_status_[0].temperature_MCU);
      // ROS_INFO("Query response, motor Tem 1: %i", motor_status_[0].temperature);
      // ROS_INFO("Query response, motor Tem 2: %i", motor_status_[1].temperature);
      // ROS_INFO("Query response, motor counter 1: %i", motor_status_[0].counter);
      // ROS_INFO("Query response, motor counter 2: %i", motor_status_[1].counter);
      // ROS_INFO("Query response, motor rpm 1: %i", motor_status_[0].rpm);
      // ROS_INFO("Query response, motor rpm 2: %i", motor_status_[1].rpm);
      // ROS_INFO("Query response, motor status_flags: %i", status_flags_);
      // ROS_INFO("Query response, motor fault_flags: %i", fault_flags_);
      // ROS_INFO("Query response, motor status 1: %i", motor_status_[1].status);
      // ROS_INFO("Query response, motor status 2: %i", motor_status_[1].status);
    } 
  }
}

void iqr::CasterHardware::Clear() {
  /* Clear BL Counter */
  // serial_port_driver_.write("!CB 1 0\r");
  // serial_port_driver_.write("!CB 2 0\r");
  // motor_status_[kLeftMotor].counter_offset = motor_status_[kLeftMotor].counter;
  // motor_status_[kRightMotor].counter_offset = motor_status_[kRightMotor].counter;
}

void iqr::CasterHardware::UpdateHardwareStatus() {
  bool success = false;
  uint32_t data;
  if(body_joint_name_ != "") {
    body_joint_.position = body_joint_.position_command;
  }
  serial_port_driver_.write("?T 1_?T 2_?T 3_?A 1_?A 2_?CB 1_?CB 2_?BS 1_?BS 2_?FM 1_?FM 2_?FS_?FF\r");

  if(body_joint_name_ != "") {

    uint8_t buf[13];
    bzero(buf, 13);

    // send request
    buf[0] = 0x01;                    // ID
    buf[1] = 0x03;                    // write multi register
    buf[2] = 0x00;                    // Starting Address Hi
    buf[3] = 0x06;                    // Starting Address Lo
    buf[4] = 0x00;                    // Quantity of Registers Hi
    buf[5] = 0x01;                    // Quantity of Registers Lo

    // crc
    uint16_t crc = CRC16(buf, 6);
    memcpy(buf+6, &crc, 2);

    serial_port_body_.write(buf, 8);

    // ros::Duration(0.005).sleep();

    bzero(buf, 13);
    serial_port_body_.read(buf, 7);

    uint16_t position;
    uint8_t t_data[2];
    t_data[0] = buf[4];
    t_data[1] = buf[3];
    memcpy(&position, t_data, 2);
    body_joint_.position = static_cast<double>(position) / 100000.0f;

    // ROS_INFO("rdata: %02x, %02x, %02x, %02x, %02x, %02x, %02x", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
  }

  joints_[kLeftMotor].velocity = motor_status_[kLeftMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].velocity = motor_status_[kRightMotor].rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0;

  joints_[kLeftMotor].position = (motor_status_[kLeftMotor].counter-motor_status_[kLeftMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor].position = (motor_status_[kRightMotor].counter-motor_status_[kRightMotor].counter_offset) / 30.0 / REDUCTION_RATIO * M_PI * 2.0;

  diagnostic_updater_.update();

  // ROS_INFO("Body: %d, %lf", position, body_joint_.position);
  // ROS_INFO("motor counter: %d, %d, %d, %d, %lf, %lf",
  //           motor_status_[kLeftMotor].counter, motor_status_[kRightMotor].counter,
  //           motor_status_[kLeftMotor].rpm, motor_status_[kRightMotor].rpm,
  //           joints_[kLeftMotor].velocity, joints_[kRightMotor].velocity);
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
  if(body_joint_name_ != "") {
    hardware_interface::JointStateHandle body_joint_state_handle(body_joint_name_, &body_joint_.position, &body_joint_.velocity, &body_joint_.effort);
    joint_state_interface_.registerHandle(body_joint_state_handle);
    
    hardware_interface::JointHandle body_joint_handle(body_joint_state_handle, &body_joint_.position_command);
    position_joint_interface_.registerHandle(body_joint_handle);

    registerInterface(&position_joint_interface_);
  }

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
  std::string buf_clear;
  if(body_joint_name_ != "") {
    // body_joint_.position = body_joint_.position_command;
  }
  // ROS_INFO("Velocity left motor : %f", joints_[0].velocity_command);

  // ROS_INFO("Velocity right motor : %f", joints_[1].velocity_command);

  speed[0] = joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60;
  std::string command_vel_left = "!S 1 " + std::to_string(speed[0]) + "\r";
  serial_port_driver_.write(command_vel_left);
  speed[1] = joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60;
  std::string command_vel_right = "!S 2 " + std::to_string(speed[1]) + "\r";
  serial_port_driver_.write(command_vel_right);
  // serial_port_driver_.read(buf_clear, serial_port_driver_.available());

  if(body_joint_name_ != "" and abs(body_joint_.position-body_joint_.position_command)>0.001) {
    uint8_t buf[13];
    bzero(buf, 13);

    // send request
    buf[0] = 0x01;                    // ID
    buf[1] = 0x10;                    // write multi register
    buf[2] = 0x00;                    // Starting Address Hi
    buf[3] = 0x07;                    // Starting Address Lo
    buf[4] = 0x00;                    // Quantity of Registers Hi
    buf[5] = 0x02;                    // Quantity of Registers Lo
    buf[6] = 0x04;                    // Byte Count

    // set speed 80
    buf[7] = 0x00;
    buf[8] = 0x50;

    // set position
    uint8_t t_data[2];
    uint16_t position = static_cast<uint16_t>(body_joint_.position_command * 100000.0f);
    memcpy(t_data, &position, 2);
    buf[9] = t_data[1];
    buf[10] = t_data[0];

    // crc
    uint16_t crc = CRC16(buf, 11);
    memcpy(buf+11, &crc, 2);

    serial_port_body_.write(buf, 13);

    // ros::Duration(0.05).sleep();

    bzero(buf, 13);
    serial_port_body_.read(buf, 8);

    // ROS_INFO("wdata: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  }

  // ROS_INFO("body command: %lf, %lf, delta %lf", body_joint_.position_command, body_joint_.position, abs(body_joint_.position-body_joint_.position_command));
  // ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}