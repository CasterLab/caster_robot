#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/format.hpp>

#include "modbus_rtu_master.h"
#include "caster_msgs/CasterState.h"

const std::string error_info[14] = {
  "Monomer Overvoltage Protection", "Single undervoltage protection",
  "Overvoltage protection of whole group", "Overall undervoltage protection", 
  "Charging Overtemperature Protection", "Charging cryogenic protection", 
  "Discharge Overtemperature Protection", "Discharge cryogenic protection", 
  "Charging Overcurrent Protection", "Discharge Overcurrent Protection", 
  "Short circuit protection", "Front-end IC error detection", "Software Lock-in MOS", "Unable to open port"};  

class CasterMCUNode {
  public:
    CasterMCUNode();
    ~CasterMCUNode();

    void loop();
    void CallBack(const std_msgs::Bool &msg);

  private:
    int baudrate_;
    std::string port_name_;

    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber estop_sub_;

    caster_msgs::CasterState state_msg_;
    diagnostic_updater::Updater diagnostic_;

    ModBusRTUMaster *modbus_handler_;

    void MCUCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void PDBCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void BMSCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
};

CasterMCUNode::CasterMCUNode(): nh_("~") {
  nh_.param<int>("baudrate", baudrate_, 115200);
  nh_.param<std::string>("port", port_name_, "/dev/caster_mcu");

  modbus_handler_ = new ModBusRTUMaster(port_name_, baudrate_);

  estop_sub_ = nh_.subscribe("estop", 1, &CasterMCUNode::CallBack, this);
  state_pub_ = nh_.advertise<caster_msgs::CasterState>("state", 10);

  diagnostic_.setHardwareID("caster_robot");
  diagnostic_.add("PDB", this->MCUCheck);
  diagnostic_.add("PDB", this->PDBCheck);
  diagnostic_.add("PDB", this->BMSCheck);
}

CasterMCUNode::~CasterMCUNode() {
  free(modbus_handler_);
  modbus_handler_ = nullptr;
}

void CasterMCUNode::MCUCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.add("Hardware Version", state_msg_.hardware_version);
  status.add("Firmware Version", state_msg_.firmware_version);
  status.add("Serial Number", state_msg_.serial_number);

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  if(state_msg_.pdb_error == true) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "PDB Error");
  }
  if(state_msg_.bms_error == true) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "BMS Error");
  }
}

void CasterMCUNode::PDBCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.addf("DC-12V Current (A)", "%.3f", state_msg_.pdb.dc_12_current);
  status.addf("DC-12V Voltage (V)", "%.2f", state_msg_.pdb.dc_12_voltage);

  status.addf("DC-19V Current (A)", "%.3f", state_msg_.pdb.dc_19_current);
  status.addf("DC-19V Voltage (V)", "%.2f", state_msg_.pdb.dc_19_voltage);

  status.addf("DC-24V Current (A)", "%.3f", state_msg_.pdb.dc_24_current);
  status.addf("DC-24V Voltage (V)", "%.2f", state_msg_.pdb.dc_24_voltage);

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");

  // DC-12V
  if(state_msg_.pdb.dc_12_voltage < 11.5 || state_msg_.pdb.dc_12_voltage > 12.5) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-12V Voltage too low / high");
  }
  if(state_msg_.pdb.dc_12_current < -0.1 || state_msg_.pdb.dc_12_current > 5.5) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-12V Current too low / high");
  }

  // DC-19V
  if(state_msg_.pdb.dc_19_voltage < 18.5 || state_msg_.pdb.dc_19_voltage > 19.5) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-19V Voltage too low / high");
  }
  if(state_msg_.pdb.dc_19_current < -0.1 || state_msg_.pdb.dc_19_current > 5.0) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-19V Current too low / high");
  }

  // DC-24V
  if(state_msg_.pdb.dc_24_voltage < 23.5 || state_msg_.pdb.dc_24_voltage > 24.5) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-24V Voltage too low / high");
  }
  if(state_msg_.pdb.dc_24_current < -0.1 || state_msg_.pdb.dc_24_current > 10.0) {
    status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "DC-24V Current too low / high");
  }
}

void CasterMCUNode::BMSCheck(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.add("Voltage (V)", state_msg_.bms.voltage);
  status.add("Current (A)", state_msg_.bms.current);
  status.add("Residual Capacity (mAh)", state_msg_.bms.residual_capacity);
  status.add("Design Capacity (mAh)", state_msg_.bms.design_capacity);
  status.add("RSOC (%)", state_msg_.bms.rsoc);
  status.add("Cycle Count", state_msg_.bms.cycle_count);
  status.add("Balance State", state_msg_.bms.balance_state);
  status.add("Safty State", state_msg_.bms.safty_state);
  status.add("FET State", state_msg_.bms.fet_state);
  status.add("Firmware Version", state_msg_.bms.firmware_version);

  status.add("Cell Count", state_msg_.bms.cell_count);
  for(int i=0; i<state_msg_.bms.cell_count; i++) {
    key_format = boost::format("Cell %1% Voltage (V)") % i;
    status.add(key_format.str(), state_msg_.bms.cell_voltage[i]);
  }

  status.add("NTC Count", state_msg_.bms.ntc_count);
  for(int i=0; i<state_msg_.bms.ntc_count; i++) {
    key_format = boost::format("Ntc %1% (℃)") % i;
    status.add(key_format.str(), state_msg_.bms.Ntntc_datacTem[i]);
  }

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  for(int i = 0; i < 13; ++i) {
    if((state_msg_.bms.safty_state & (0x0001>>i)>>i)==1) {
      status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_info[i]);
    }
  }
}

void CasterMCUNode::loop() {
  uint16_t data[256] = {0};

  uint8_t ret = modbus_handler_->GetMultipleRegisters(0x01, 0x0000, 0x0028, data);

  if (!ret) {
    ROS_WARN("Read Multiple Registers faild!!");
    return;
  }

  state_msg_.header.stamp = ros::Time::now();

  state_msg_.hardware_version = str(boost::format("v%1%.%2%") % int((data[0] & 0xff00) >> 8) % int(data[0] & 0x00ff));
  state_msg_.firmware_version = str(boost::format("v%1%.%2%") % int((data[1] & 0xff00) >> 8) % int(data[1] & 0x00ff));
  state_msg_.serial_number = str(boost::format("SN%1%%2%.%3%") % int((data[2] & 0xf000) >> 12) % int((data[2] & 0x0f00) >> 8) % int(data[2] & 0x00ff));

  state_msg_.estop_state = data[4] & 0x0002;
  state_msg_.charge_state = data[4] & 0x0001;
  state_msg_.low_power_state = data[4] & 0x0004;

  state_msg_.pdb_error = (data[5] && data[6]) ? true : false;
  state_msg_.pdb.dc_12_voltage = data[7] / 1000.0;  //#12V voltage V
  state_msg_.pdb.dc_12_current = data[8] / 1000.0;  //#12V current A
  state_msg_.pdb.dc_19_voltage = data[9] / 1000.0;  //#19V voltage V
  state_msg_.pdb.dc_19_current = data[10] / 1000.0; //#19V current A
  state_msg_.pdb.dc_24_voltage = data[11] / 1000.0; //#24V voltage V
  state_msg_.pdb.dc_24_current = data[12] / 1000.0; //#24V current A

  state_msg_.bms_error = (data[13]) ? true : false;
  state_msg_.bms.voltage = data[14] / 100.0;
  state_msg_.bms.current = int16_t(data[15]) / 100.0;
  state_msg_.bms.residual_capacity = data[16] / 100.0;
  state_msg_.bms.design_capacity = data[17] / 100.0;
  state_msg_.bms.rsoc = data[18];
  state_msg_.bms.cycle_count = data[19];
  state_msg_.bms.balance_state = uint32_t((data[20] << 16) | data[21]);
  state_msg_.bms.safty_state = data[22];
  state_msg_.bms.cell_count = data[23];
  state_msg_.bms.ntc_count = data[24];
  state_msg_.bms.fet_state = data[25];
  state_msg_.bms.firmware_version = str(boost::format("v%1%.%2%") % int((data[26] & 0xff00) >> 8) % int(data[26] & 0x00ff));

  for (size_t i = 0; i < msg.bms.NTCCount; i++) {
    float ntc = (data[27 + i] - 2731) / 10.0;
    msg.bms.ntc_data.push_back(ntc);
  }

  for (size_t i = 0; i < msg.bms.cellCount; i++) {
    float vol = data[29 + i] / 1000.0;
    msg.bms.cell_voltage.push_back(vol);
  }

  state_pub_.publish(msg);
  diagnostic_.update();
}

void CasterMCUNode::CallBack(const std_msgs::Bool &msg) {
  uint16_t data[2] = {0x0001, 0x0000};
  if (msg.data) {
    uint8_t ret = modbus_handler_->SetMultipleRegisters(0x01, 0x0003, 0x0001, data);
    if (!ret) {
      ROS_WARN("Write Multiple Registers faild!!");
    }
  } else {
    uint8_t ret = modbus_handler_->SetMultipleRegisters(0x01, 0x0003, 0x0001, data + 1);
    if (!ret) {
      ROS_WARN("Write Multiple Registers faild!!");
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_mcu_node");

  CasterMCUNode node;

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    node.loop();
    r.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}
