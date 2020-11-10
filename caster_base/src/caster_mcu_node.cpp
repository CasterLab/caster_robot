#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/format.hpp>
#include "ModBusRTUMaster.h"
#include "caster_msgs/CasterStatus.h"

class CasterMCUNode
{
public:
  CasterMCUNode();
  ~CasterMCUNode();
  void loop();
  void callBack(const std_msgs::Bool &msg);

private:
  ros::NodeHandle nh;
  ros::Publisher statusPub;
  ros::Subscriber estopSub;

  int baudRate;
  std::string portName;

  ModBusRTUMaster *master;
};

CasterMCUNode::CasterMCUNode()
    : nh("~")
{
  nh.param<int>("mcu_baudrate", baudRate, 115200);
  nh.param<std::string>("mcu_port", portName, "/dev/caster_mcu");

  master = new ModBusRTUMaster(portName, baudRate);

  estopSub = nh.subscribe("EStop", 1, &CasterMCUNode::callBack, this);
  statusPub = nh.advertise<caster_msgs::CasterStatus>("caster_status", 10);
}

CasterMCUNode::~CasterMCUNode()
{
  free(master);
  master = nullptr;
}

void CasterMCUNode::loop()
{
  uint16_t data[256] = {
      0,
  };

  uint8_t ret = master->getMultipleRegisters(0x01, 0x0000, 0x0028, data);

  if (!ret)
  {
    ROS_WARN("Read Multiple Registers faild!!");
    return;
  }

  caster_msgs::CasterStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";

  msg.hardVersion = str(boost::format("V%1%.%2%") % int((data[0] & 0xff00) >> 8) % int(data[0] & 0x00ff));
  msg.softVersion = str(boost::format("V%1%.%2%") % int((data[1] & 0xff00) >> 8) % int(data[1] & 0x00ff));
  msg.serialNumber = str(boost::format("SN%1%%2%.%3%") % int((data[2] & 0xf000) >> 12) % int((data[2] & 0x0f00) >> 8) % int(data[2] & 0x00ff));

  msg.EStopStatus = data[4] & 0x0002;
  msg.ChargeStatus = data[4] & 0x0001;
  msg.LowPowerStatus = data[4] & 0x0004;

  msg.PDBError = (data[5] && data[6]) ? true : false;
  msg.pdb.voltage12V = data[7] / 1000.0;  //#12V voltage V
  msg.pdb.current12V = data[8] / 1000.0;  //#12V current A
  msg.pdb.voltage19V = data[9] / 1000.0;  //#19V voltage V
  msg.pdb.current19V = data[10] / 1000.0; //#19V current A
  msg.pdb.voltage24V = data[11] / 1000.0; //#24V voltage V
  msg.pdb.current24V = data[12] / 1000.0; //#24V current A

  msg.BMSError = (data[13]) ? true : false;
  msg.bms.voltage = data[14] / 100.0;                                                                            //#总电压V
  msg.bms.current = int16_t(data[15]) / 100.0;                                                                   //#电流A
  msg.bms.remCap = data[16] / 100.0;                                                                             //#剩余容量Ah
  msg.bms.nomCap = data[17] / 100.0;                                                                             //#标称容量Ah
  msg.bms.RSOC = data[18];                                                                                       //#剩余容量%
  msg.bms.capCount = data[19];                                                                                   //#循环次数
  msg.bms.balStatus = uint32_t((data[20] << 16) | data[21]);                                                     //#平衡状态
  msg.bms.proStatus = data[22];                                                                                  //#保护状态
  msg.bms.cellCount = data[23];                                                                                  //#串数
  msg.bms.NTCCount = data[24];                                                                                   //#NTC个数
  msg.bms.FETStatus = data[25];                                                                                  //#FET控制状态
  msg.bms.softVersion = str(boost::format("V%1%.%2%") % int((data[26] & 0xff00) >> 8) % int(data[26] & 0x00ff)); //#软件版本

  for (size_t i = 0; i < msg.bms.NTCCount; i++)
  {
    float ntc = (data[27 + i] - 2731) / 10.0;
    msg.bms.NTCData.push_back(ntc);
  }

  for (size_t i = 0; i < msg.bms.cellCount; i++)
  {
    float vol = data[29 + i] / 1000.0;
    msg.bms.cellVoltage.push_back(vol);
  }

  statusPub.publish(msg);
}

void CasterMCUNode::callBack(const std_msgs::Bool &msg)
{
  uint16_t data[2] = {0x0001, 0x0000};
  if (msg.data)
  {
    uint8_t ret = master->setMultipleRegisters(0x01, 0x0003, 0x0001, data);
    if (!ret)
    {
      ROS_WARN("Write Multiple Registers faild!!");
    }
  }
  else
  {
    uint8_t ret = master->setMultipleRegisters(0x01, 0x0003, 0x0001, data + 1);
    if (!ret)
    {
      ROS_WARN("Write Multiple Registers faild!!");
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "caster_mcu_node");

  CasterMCUNode node;

  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();
    node.loop();
    r.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}
