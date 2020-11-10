#ifndef MODBUSRTUMASTER_H
#define MODBUSRTUMASTER_H

#include <iostream>
#include <thread>
#include <string>
#include <mutex>

//OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

class ModBusRTUMaster
{
public:
  ModBusRTUMaster(const std::string portNmae, const uint32_t baudRate);
  ~ModBusRTUMaster();

  uint8_t setMultipleRegisters(const uint8_t slaveId, const uint16_t startAdd, const uint16_t length, uint16_t *data); //0x10 = 16
  uint8_t getMultipleRegisters(const uint8_t slaveId, const uint16_t startAdd, const uint16_t length, uint16_t *data); //0x03 = 03

protected:
  static uint16_t ModBusCRC(const uint8_t *data, const uint8_t length);

private:
  std::mutex mtx;
  serial::Serial com;
};

#endif // !MODBUSRTUMASTER_H

