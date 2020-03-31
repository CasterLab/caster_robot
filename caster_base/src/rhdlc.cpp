#include "rhdlc.h"

/* HDLC Asynchronous framing */
/* The frame boundary octet is 01111110, (7E in hexadecimal notation) */
#define FRAME_BOUNDARY_OCTET 0x7E

/* A "control escape octet", has the bit sequence '01111101', (7D hexadecimal) */
#define CONTROL_ESCAPE_OCTET 0x7D

/* If either of these two octets appears in the transmitted data, an escape octet is sent, */
/* followed by the original data octet with bit 5 inverted */
#define INVERT_OCTET 0x20

/* The frame check sequence (FCS) is a 16-bit CRC-CCITT */
#define CRC16_CCITT_INIT_VAL 0xFFFF

RHDLC::RHDLC (SendCharType send_char, FrameHandlerType frame_handler, uint16_t max_frame_length)
    : SendChar(send_char), SendBuffer(nullptr), FrameHandler(frame_handler) {
  frame_position_ = 0;
  max_frame_length_ = max_frame_length;

  /* data_length*2+4(crc_length*2)+1(header)+1(end) */
  receive_frame_buffer_ = new uint8_t[max_frame_length_*2+6];
  frame_checksum_ = CRC16_CCITT_INIT_VAL;
  escape_character_ = false;
}

RHDLC::RHDLC (SendBufferType send_buffer, FrameHandlerType frame_handler, uint16_t max_frame_length)
    : SendChar(nullptr), SendBuffer(send_buffer), FrameHandler(frame_handler) {
  frame_position_ = 0;
  max_frame_length_ = max_frame_length;

    /* data_length*2+4(crc_length*2)+1(header)+1(end) */
  receive_frame_buffer_ = new uint8_t[max_frame_length_*2+6];

  frame_checksum_ = CRC16_CCITT_INIT_VAL;
  escape_character_ = false;
}

uint16_t RHDLC::CRCUpdate (uint16_t crc, uint8_t data) {
  data ^= crc & 0xFF;
  data ^= data << 4;

  return ((((uint16_t)data << 8) | (crc>>8)&0xFF) ^ (uint8_t)(data >> 4) 
          ^ ((uint16_t)data << 3));
}

uint16_t RHDLC::CheckSum(const uint8_t *data, uint16_t length) {
  uint16_t checksum = 0;
  for(uint16_t i=0; i<length; i++) {
    checksum += data[i];
  }

  return checksum;
}

void RHDLC::Send(const uint8_t *data_buffer, uint16_t buffer_length) {
  (*SendBuffer)(data_buffer, buffer_length);

  if(SendChar != nullptr) {
    for(uint16_t i=0; i<buffer_length; i++) {
      SendChar(data_buffer[i]);
    }
  } else if (SendBuffer != nullptr) {
    SendBuffer(data_buffer, buffer_length);
  }
}

void RHDLC::charReceiver(uint8_t data) {
  /* FRAME FLAG */
  if(data == FRAME_BOUNDARY_OCTET) {
    if(escape_character_ == true) {
      escape_character_ = false;
    }
    /* If a valid frame is detected */
    else if(frame_position_ > 2){
      frame_checksum_ = (receive_frame_buffer_[frame_position_-1] << 8 ) | (receive_frame_buffer_[frame_position_-2] & 0xff);
      if(frame_checksum_ == CheckSum(receive_frame_buffer_, frame_position_-2)) {
        (*FrameHandler)(receive_frame_buffer_, static_cast<uint8_t>((frame_position_-2)));
      }
    }
    frame_position_ = 0;
    frame_checksum_ = CRC16_CCITT_INIT_VAL;
    return;
  }

  if(escape_character_) {
    escape_character_ = false;
    data ^= INVERT_OCTET;
  } else if(data == CONTROL_ESCAPE_OCTET) {
    escape_character_ = true;
    return;
  }

  receive_frame_buffer_[frame_position_] = data;

  frame_position_ += 1;

  if(frame_position_ == max_frame_length_) {
    frame_position_ = 0;
    frame_checksum_ = CRC16_CCITT_INIT_VAL;
  }
}

/* Wrap given data in HDLC frame and send it out byte at a time*/
void RHDLC::frameDecode(const uint8_t *frame_buffer, uint8_t frame_length) {
  uint8_t data, data_buffer[frame_length*2+6];
  uint16_t fcs = CRC16_CCITT_INIT_VAL, data_pointer = 0;;

  // sendchar(FRAME_BOUNDARY_OCTET);
  data_buffer[data_pointer++] = FRAME_BOUNDARY_OCTET;

  for(uint16_t i=0; i<frame_length; i++) {
    data = frame_buffer[i];//*frame_buffer++;
    // fcs = CRCUpdate(fcs, data);
    if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET)) {
      // sendchar(CONTROL_ESCAPE_OCTET);
      data_buffer[data_pointer++] = CONTROL_ESCAPE_OCTET;
      data ^= INVERT_OCTET;
    }
    // sendchar(data);
    data_buffer[data_pointer++] = data;
    // frame_length--;
  }

  fcs = CheckSum(frame_buffer, frame_length);

  data = fcs & 0xFF;
  if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET)) {
    // sendchar(CONTROL_ESCAPE_OCTET);
    data_buffer[data_pointer++] = CONTROL_ESCAPE_OCTET;
    data ^= INVERT_OCTET;
  }
  // sendchar(data);
  data_buffer[data_pointer++] = data;

  data = (fcs>>8) & 0xFF;
  if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET)) {
    // sendchar(CONTROL_ESCAPE_OCTET);
    data_buffer[data_pointer++] = CONTROL_ESCAPE_OCTET;
    data ^= INVERT_OCTET;
  }
  // sendchar(data);
  data_buffer[data_pointer++] = data;

  // sendchar(FRAME_BOUNDARY_OCTET);
  data_buffer[data_pointer++] = FRAME_BOUNDARY_OCTET;

  Send(data_buffer, data_pointer);
}
