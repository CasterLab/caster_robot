#ifndef CASTER_HARDWARE_SOCKETCAN_H_
#define CASTER_HARDWARE_SOCKETCAN_H_

#include <string>
#include <math.h>
#include <errno.h>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <typeinfo>
#include <sys/types.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <caster_base/SetDigitalOutput.h>

#define REDUCTION_RATIO                 14.0

namespace iqr {
/**
* Class representing Caster hardware, allows for ros_control to modify internal state via joint interfaces
*/
class CasterHardware : public hardware_interface::RobotHW {
  public:
    enum MotorIndex {
      kLeftMotor = 0,
      kRightMotor = 1,
    };

    struct RoboteqCanOpenObjectDictionary
    {
      /* runtime commands */
      std::string kSetVelocity = "!S";
      std::string kSetBLCounter = "!CB";
      std::string kSetIndividualDO = "!D1";
      std::string kResetIndividualDO = "!D0";

      /* runtime queries */
      std::string kReadMotorAmps = "?A";
      std::string kReadAbsBLCounter = "?CB";
      std::string kReadBLMotorRPM = "?BS";
      std::string kReadStatusFlags = "?FS";
      std::string kReadFaultFlags = "?FF";
      std::string kReadMotorStatusFlags = "?FM";
    };

    struct MotorStatus {
      float current;
      int16_t temperature;
      int16_t temperature_MCU;
      int16_t rpm;
      int32_t counter;
      int32_t counter_offset;
      uint8_t status;
      bool counter_reset;

      MotorStatus() :
        rpm(0), counter(0), status(0), counter_reset(false)
      { }
    };

    struct Joint {
      double effort;
      double position;
      double position_command;
      double position_offset;
      double velocity;
      double velocity_command;

      Joint() :
        effort(0), position(0), position_command(0), position_offset(0), velocity(0) , velocity_command(0)
      { }
    };

    int16_t *num = new int16_t[14];
    std::string data_type[13] = {"T", "T", "T", "A", "A", "CB", "CB", "BS", "BS", "FM", "FM", "FS","FF"};
    CasterHardware();

    void Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    void UpdateHardwareStatus();
    void WriteCommandsToHardware();

    void Clear();

  private:
    void ResetTravelOffset();
    void RegisterControlInterfaces();

    std::string ToBinary(size_t data, uint8_t length);
    uint16_t CRC16 (const uint8_t *data, uint16_t length);
    int16_t* BufferSpilt(std::string buf_driver);

    void ControllerTimerCallback(const ros::TimerEvent&);

    void LeftMotorCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void RightMotorCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void StatusCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void ControllerCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void SerialReadUpadata();
    bool SetDigitalOutputCB(caster_base::SetDigitalOutput::Request &req, caster_base::SetDigitalOutput::Response &res);
		bool SerialPortInit(serial::Serial &serial_option, std::string port, int baudrate);
    std::string node_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer timer_;
    controller_manager::ControllerManager *controller_manager_;

    std::string send_topic_, receive_topic_;
    
    ros::Publisher can_pub_;
    ros::Subscriber can_sub_;

    std::string left_wheel_joint_, right_wheel_joint_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // diagnostic  update
    diagnostic_updater::Updater diagnostic_updater_;

    // digital output service
    ros::ServiceServer set_io_service_;

    int can_id_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    int8_t fault_flags_;
    int8_t status_flags_;
    MotorStatus motor_status_[2];

    Joint joints_[2];
    MotorIndex motor_index;
    RoboteqCanOpenObjectDictionary query;
    // RoboteqClientCommandType commands;
    // serial port params
    int baudrate_body_, baudrate_driver_;
    std::string port_body_, port_driver_;
    serial::Serial serial_port_body_, serial_port_driver_;

    // ros controller joints
    std::string body_joint_name_;

    // body joint
    Joint body_joint_;
};
}  // namespace iqr
#endif  // CASTER_HARDWARE_SOCKETCAN_H_
