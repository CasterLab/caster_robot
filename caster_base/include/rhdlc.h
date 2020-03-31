#ifndef RHDLC_H_
#define RHDLC_H_

#include <stdint.h>
#include <stdbool.h>

class RHDLC {
  public:
    typedef void (* SendCharType) (uint8_t);
    typedef void (* SendBufferType) (const uint8_t *data_buffer, uint16_t buffer_length);
    typedef void (* FrameHandlerType)(const uint8_t *frame_buffer, uint16_t frame_length);

    RHDLC(SendCharType, FrameHandlerType, uint16_t max_frame_length);
    RHDLC(SendBufferType, FrameHandlerType, uint16_t max_frame_length);
    void charReceiver(uint8_t data);
    void frameDecode(const uint8_t *frame_buffer, uint8_t frame_length);

  private:
    /* User must define a function, that sends a 8bit char over the chosen interface, usart, spi, i2c etc. */
    SendCharType SendChar;

    /* User must define a function, that sends a data buffer over the chosen infterface, usart, spi, i2c etc. */
    SendBufferType SendBuffer;

    /* User must define a function, that will process the valid received frame */
    /* This function can act like a command router/dispatcher */
    FrameHandlerType FrameHandler;

    void Send(const uint8_t *data_buffer, uint16_t buffer_length);

    uint16_t CRCUpdate (uint16_t crc, uint8_t data);
    uint16_t CheckSum(const uint8_t *data, uint16_t length);

    bool escape_character_;
    uint8_t frame_position_;
    uint8_t *receive_frame_buffer_;

    uint16_t frame_checksum_;

    uint16_t max_frame_length_;
};

#endif // RHDLC_H_
