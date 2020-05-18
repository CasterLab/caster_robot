#include <random>

#include <ros/ros.h>
#include <serial/serial.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <rhdlc.h>

enum StateType {
  kNormal = 0,
  kEStop,
  kHalt,
};

struct DCState {
  int32_t power_mw;
  int16_t current_ma;
  int16_t voltage_mv;

  DCState():
    power_mw(0), current_ma(0), voltage_mv(0) {
  }
};

struct HardwareState {
  bool halt;
  bool e_stop;
  bool charger_detect;
  DCState dc_05v;
  DCState dc_12v;
  DCState dc_19v;
  DCState dc_24v;

  HardwareState(): halt(0), e_stop(0), charger_detect(0) {
  }
};

HardwareState hardware_state;

void SendBuffer(const uint8_t *data_buffer, uint16_t buffer_length) {
  ROS_INFO("buffer sent");
}

void FrameHandler(const uint8_t *frame_buffer, uint16_t frame_length) {

  hardware_state.halt = static_cast<bool>(frame_buffer[0]);
  hardware_state.e_stop = static_cast<bool>(frame_buffer[1]);
  hardware_state.charger_detect = static_cast<bool>(frame_buffer[2]);

  memcpy(&hardware_state.dc_05v.power_mw, frame_buffer+3, 4);
  memcpy(&hardware_state.dc_05v.current_ma, frame_buffer+7, 2);
  memcpy(&hardware_state.dc_05v.voltage_mv, frame_buffer+9, 2);

  memcpy(&hardware_state.dc_12v.power_mw, frame_buffer+11, 4);
  memcpy(&hardware_state.dc_12v.current_ma, frame_buffer+15, 2);
  memcpy(&hardware_state.dc_12v.voltage_mv, frame_buffer+17, 2);

  memcpy(&hardware_state.dc_19v.power_mw, frame_buffer+19, 4);
  memcpy(&hardware_state.dc_19v.current_ma, frame_buffer+23, 2);
  memcpy(&hardware_state.dc_19v.voltage_mv, frame_buffer+25, 2);

  memcpy(&hardware_state.dc_24v.power_mw, frame_buffer+27, 4);
  memcpy(&hardware_state.dc_24v.current_ma, frame_buffer+31, 2);
  memcpy(&hardware_state.dc_24v.voltage_mv, frame_buffer+33, 2);

  // ROS_INFO("get data");
}

void MCUCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.add("Charger Detect", hardware_state.charger_detect);

  status.addf("DC-05V Current (A)", "%.3f", hardware_state.dc_05v.current_ma / 1000.0);
  status.addf("DC-05V Voltage (V)", "%.2f", hardware_state.dc_05v.voltage_mv / 1000.0);
  status.addf("DC-05V Power (W)", "%.4f", hardware_state.dc_05v.power_mw / 1000.0);

  status.addf("DC-12V Current (A)", "%.3f", hardware_state.dc_12v.current_ma / 1000.0);
  status.addf("DC-12V Voltage (V)", "%.2f", hardware_state.dc_12v.voltage_mv / 1000.0);
  status.addf("DC-12V Power (W)", "%.4f", hardware_state.dc_12v.power_mw / 1000.0);

  status.addf("DC-19V Current (A)", "%.3f", hardware_state.dc_19v.current_ma / 1000.0);
  status.addf("DC-19V Voltage (V)", "%.2f", hardware_state.dc_19v.voltage_mv / 1000.0);
  status.addf("DC-19V Power (W)", "%.4f", hardware_state.dc_19v.power_mw / 1000.0);

  status.addf("DC-24V Current (A)", "%.3f", hardware_state.dc_24v.current_ma / 1000.0);
  status.addf("DC-24V Voltage (V)", "%.2f", hardware_state.dc_24v.voltage_mv / 1000.0);
  status.addf("DC-24V Power (W)", "%.4f", hardware_state.dc_24v.power_mw / 1000.0);

  status.addf("DC-ALL Power (W)", "%.4f", (hardware_state.dc_05v.power_mw + hardware_state.dc_12v.power_mw + hardware_state.dc_19v.power_mw + hardware_state.dc_24v.power_mw) / 1000.0);

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if(hardware_state.halt == true) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Halt triggered");
  }
  if(hardware_state.e_stop == true) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "E-Stop active");
  }

  // DC 5V
  if(hardware_state.dc_05v.voltage_mv < 4500 || hardware_state.dc_05v.voltage_mv > 5500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-5V Voltage too low / high");
  }
  if(hardware_state.dc_05v.current_ma < 0 || hardware_state.dc_05v.current_ma > 2500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-5V Current too low / high");
  }
  if(hardware_state.dc_05v.power_mw < 0 || hardware_state.dc_05v.power_mw > 15000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-5V Power too low / high");
  }

  // DC-12V
  if(hardware_state.dc_12v.voltage_mv < 11500 || hardware_state.dc_12v.voltage_mv > 12500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-12V Voltage too low / high");
  }
  if(hardware_state.dc_12v.current_ma < 0 || hardware_state.dc_12v.current_ma > 5500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-12V Current too low / high");
  }
  if(hardware_state.dc_12v.power_mw < 0 || hardware_state.dc_12v.power_mw > 65000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-12V Power too low / high");
  }

  // DC-19V
  if(hardware_state.dc_19v.voltage_mv < 18500 || hardware_state.dc_19v.voltage_mv > 19500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-19V Voltage too low / high");
  }
  if(hardware_state.dc_19v.current_ma < 0 || hardware_state.dc_19v.current_ma > 5000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-19V Current too low / high");
  }
  if(hardware_state.dc_19v.power_mw < 0 || hardware_state.dc_19v.power_mw > 100000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-19V Power too low / high");
  }

  // DC-24V
  if(hardware_state.dc_24v.voltage_mv < 23500 || hardware_state.dc_24v.voltage_mv > 24500) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-24V Voltage too low / high");
  }
  if(hardware_state.dc_24v.current_ma < 0 || hardware_state.dc_24v.current_ma > 10000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-24V Current too low / high");
  }
  if(hardware_state.dc_24v.power_mw < 0 || hardware_state.dc_24v.power_mw > 240000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-24V Power too low / high");
  }

  // DC-ALL
  int32_t power_mw = hardware_state.dc_05v.power_mw + hardware_state.dc_12v.power_mw + hardware_state.dc_19v.power_mw + hardware_state.dc_24v.power_mw;
  if(power_mw < 0 || power_mw > 240000) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-ALL Power too low / high");
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_mcu_node");
  ros::NodeHandle private_nh("~");

  int baudrate;
  std::string port;

  private_nh.param<int>("mcu_baudrate", baudrate, 115200);
  private_nh.param<std::string>("mcu_port", port, "/dev/ttyUSB0");

  serial::Serial serial_port;

  try {
    serial_port.setPort(port);
    serial_port.setBaudrate(baudrate);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(50);
    serial_port.setTimeout(serial_timeout);
    serial_port.open();
    serial_port.setRTS(false);
    serial_port.setDTR(false);
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open serial port:" + port);
  }

  diagnostic_updater::Updater diagnostic;
  diagnostic.setHardwareID("caster_robot");
  diagnostic.add("MCU", MCUCheck);

  // ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

  RHDLC hdlc(&SendBuffer, &FrameHandler, 300);

  size_t available_data;
  uint8_t receive_byte[100];
  while(ros::ok()) {
    available_data = serial_port.available();
    available_data = available_data>100 ? 100 : available_data;
    if(available_data > 0) {
      serial_port.read(receive_byte, available_data);
      for(int i=0; i<available_data; i++) {
        hdlc.charReceiver(receive_byte[i]);
      }
    }
    diagnostic.update();
    ros::spinOnce();
  }

  ROS_INFO("All finish");

  return 0;
}
