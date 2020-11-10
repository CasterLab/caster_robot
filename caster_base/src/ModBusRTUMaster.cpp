#include "ModBusRTUMaster.h"

#define REBACK_SLEEP_MS 1

ModBusRTUMaster::ModBusRTUMaster(const std::string portNmae, const uint32_t baudRate)
{
  try
  {
    com.setPort(portNmae);
    com.setBaudrate(baudRate);
    com.setBytesize(serial::eightbits);
    com.setParity(serial::parity_none);
    com.setStopbits(serial::stopbits_one);
    com.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(50);
    com.setTimeout(serial_timeout);
    com.open();
    //com.setRTS(false);
    //com.setDTR(false);
  }
  catch (serial::IOException &e)
  {
    std::cout << "Unable to open serial port:" << portNmae << std::endl;
    return;
  }

  std::cout << "open serial port:" << portNmae << " successful!!"<< std::endl;
    
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

ModBusRTUMaster::~ModBusRTUMaster()
{
  com.close();
}

uint8_t ModBusRTUMaster::setMultipleRegisters(const uint8_t slaveId, const uint16_t startAdd, const uint16_t length, uint16_t *data)
{
  if (!com.isOpen())
  {
    std::cout << "serial not open!!" << std::endl;
    return 0;
  }

  std::lock_guard<std::mutex> lck(mtx);

  com.flush();

  const uint8_t fuctionNum = 0x10;
  uint8_t sendBuf[256] = {
      0,
  };

  sendBuf[0] = slaveId;
  sendBuf[1] = fuctionNum;
  sendBuf[2] = uint8_t((startAdd & 0xff00) >> 8);
  sendBuf[3] = uint8_t((startAdd & 0x00ff));
  sendBuf[4] = uint8_t((length & 0xff00) >> 8);
  sendBuf[5] = uint8_t((length & 0x00ff));
  sendBuf[6] = uint8_t((length & 0x00ff) * 2);
  uint8_t bufIndex = 6;
  for (uint16_t i = 0; i < length; i++)
  {
    sendBuf[7 + i * 2] = uint8_t((data[i] & 0xff00) >> 8);
    sendBuf[8 + i * 2] = uint8_t((data[i] & 0x00ff));
    bufIndex += 2;
  }

  uint16_t crc = ModBusCRC(sendBuf, bufIndex + 1);
  sendBuf[bufIndex + 1] = uint8_t((crc & 0x00ff));
  sendBuf[bufIndex + 2] = uint8_t((crc & 0xff00) >> 8);

  size_t bufLen = bufIndex + 3;
  size_t writeLen = com.write(sendBuf, bufLen);
  if (!(bufLen == writeLen))
  {
    std::cout << "Failed to send message!!" << std::endl;
    com.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t recBuf[256] = {
      0,
  };
  size_t recLen = 8;

  size_t readLen = com.read(recBuf, recLen);

  if (!(recLen == readLen))
  {
    std::cout << "Failed to read message!!" << std::endl;
    com.flush();
    return 0;
  }
  if (!(recBuf[0] == slaveId))
  {
    std::cout << "Message ID error!!" << std::endl;
    com.flush();
    return 0;
  }
  if (!(recBuf[1] == fuctionNum))
  {
    std::cout << "Message fuction number error!!" << std::endl;
    com.flush();
    return 0;
  }

  uint16_t startAdd_ = uint16_t((recBuf[2] << 8) | recBuf[3]);
  uint16_t length_ = uint16_t((recBuf[4] << 8) | recBuf[5]);

  if (!(startAdd_ == startAdd))
  {
    std::cout << "Message start address error!!" << std::endl;
    com.flush();
    return 0;
  }

  if (!(length_ == length))
  {
    std::cout << "Message read length error!!" << std::endl;
    com.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(recBuf, recLen - 2);
  uint16_t rec_crc_ = uint16_t((recBuf[recLen - 1] << 8) | (recBuf[recLen - 2]));

  if (!(rec_crc == rec_crc_))
  {
    std::cout << "Message crc check error!!" << std::endl;
    com.flush();
    return 0;
  }

  com.flush();
  return 1;
}

uint8_t ModBusRTUMaster::getMultipleRegisters(const uint8_t slaveId, const uint16_t startAdd, const uint16_t length, uint16_t *data)
{
  if (!com.isOpen())
  {
    std::cout << "serial not open!!" << std::endl;
    return 0;
  }

  std::lock_guard<std::mutex> lck(mtx);

  com.flush();

  const uint8_t fuctionNum = 0x03;
  uint8_t sendBuf[8] = {
      0,
  };

  sendBuf[0] = slaveId;
  sendBuf[1] = fuctionNum;
  sendBuf[2] = uint8_t((startAdd & 0xff00) >> 8);
  sendBuf[3] = uint8_t((startAdd & 0x00ff));
  sendBuf[4] = uint8_t((length & 0xff00) >> 8);
  sendBuf[5] = uint8_t((length & 0x00ff));
  uint16_t crc = ModBusCRC(sendBuf, 6);
  sendBuf[6] = uint8_t((crc & 0x00ff));
  sendBuf[7] = uint8_t((crc & 0xff00) >> 8);

  size_t writeLen = com.write(sendBuf, 8);

  if (!(writeLen == 8))
  {
    std::cout << "Failed to send message!!" << std::endl;
    com.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t recBuf[256] = {
      0,
  };
  size_t recLen = (length * 2) + 5;
  size_t readLen = com.read(recBuf, recLen);

  if (!(recLen == readLen))
  {
    std::cout << "Failed to read message!!" << std::endl;
    com.flush();
    return 0;
  }
  if (!(recBuf[0] == slaveId))
  {
    std::cout << "Message ID error!!" << std::endl;
    com.flush();
    return 0;
  }
  if (!(recBuf[1] == fuctionNum))
  {
    std::cout << "Message fuction number error!!" << std::endl;
    com.flush();
    return 0;
  }
  if (!(recBuf[2] == (length * 2)))
  {
    std::cout << "Message data length error!!" << std::endl;
    com.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(recBuf, recLen - 2);
  uint16_t rec_crc_ = uint16_t((recBuf[recLen - 1] << 8) | (recBuf[recLen - 2]));

  if (!(rec_crc == rec_crc_))
  {
    std::cout << "Message crc check error!!" << std::endl;
    com.flush();
    return 0;
  }

  for (uint16_t i = 0; i < length; i++)
  {
    data[i] = uint16_t((recBuf[3 + i * 2] << 8) | (recBuf[4 + i * 2]));
  }

  com.flush();
  return 1;
}

uint16_t ModBusRTUMaster::ModBusCRC(const uint8_t *data, const uint8_t length)
{
  uint8_t j;
  uint16_t crc;
  uint8_t len = length;
  crc = 0xFFFF;

  while (len--)
  {
    crc = crc ^ *data++;
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }

  return crc;
  //return (crc << 8 | crc >> 8);
}
