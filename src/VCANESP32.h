/** \mainpage
@copyright Copyright © Duncan Greenwood 2026 (duncan_greenwood@hotmail.com)
 
 This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
 Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
 The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0/

# Introduction to VCANESP32
This library provides a software CAN interface on an ESP32 device when used in conjunction with the VLCB_Arduino library
suite.  The main VLCB-Arduino library can be found at [VLCB](https://github.com/SvenRosvall/VLCB-Arduino).

See [Design documents](https://github.com/SvenRosvall/VLCB-Arduino/blob/main/docs/Design.md) for how this library is structured.

## Examples
There are two versions of the example that, from the users perspective, are functionally identical.
The first uses a single core in the Pico and is identified as VLCB_4in4out_Pico_s, where 's' stands
for single core.  The other core will be dormant in a low power state.

The second version makes use of both cores in the processer and is identifed as VLCB_4in4out_Pico_d,
where 'd' stands for dual core.  This is organised such that the VLCB library runs in core 0 and the
application runs in core 1.

The docs folder in the VCANESP32 repository provides notes on how to use this library with the 4in4out examples.
It also holds a circuit schematic suitable for the examples.

## Hardware

Currently supports the arduino-esp32 Arduino core
[arduino-esp32](https://github.com/espressif/arduino-esp32) Full instructions on how to do this in the associated
[documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/)

*/

#pragma once

// header files

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include <Controller.h>
#include <CanTransport.h>

namespace VLCB
{
/// # VCANESP32

// constants

static const uint32_t tx_qsize = 32;
static const uint32_t rx_qsize = 32;

static const uint32_t CANBITRATE = 125000UL;                // 125Kb/s - fixed for VLCB
static const size_t MAX_CAN_DATA_LEN = 8;                   // max CAN data bytes

//
/// an implementation of the Transport interface class
/// to support the ESP32 TWAI driver
//

class VCANESP32 : public CanTransport
{
public:
/// \cond
  VCANESP32();
  VCANESP32(byte gpio_tx, byte gpio_rx);
  virtual ~VCANESP32();

  // these methods are declared virtual in the base class and must be implemented by the derived class
  bool begin();
  bool available() override;
  CANFrame getNextCanFrame() override;
  bool sendCanFrame(CANFrame * msg) override;
  void reset() override;
  byte getHardwareType() override { return CAN_HW_ESP32_TWAI; };
/// \endcond

  /// define here the connections to the CAN bus transceiver
  /// ensure that the ESP32 GPIO number is specified and NOT the physical device pin number
  void setPins(byte tx_pin, byte rx_pin);
  
  /// there are two buffer queues, one for transmit and one for receive. The larger the buffer, the
  /// more memory it uses. This function allows the user to specify the size of the buffers.
  /// the default size is 32 for transmit and 32 for receive.
  void setNumBuffers(unsigned int num_rx_buffers, unsigned int num_tx_buffers);
  
/// \cond

  void printStatus(void);
  
  virtual unsigned int receiveCounter() override { return _numMsgsRcvd; }
  virtual unsigned int transmitCounter() override { return _numMsgsSent; }
  virtual unsigned int receiveErrorCounter() override { return 0; }
  virtual unsigned int transmitErrorCounter() override { return _numSendErr; }
  virtual unsigned int receiveBufferUsage() override { return 0; };
  virtual unsigned int transmitBufferUsage() override { return 0; };
  virtual unsigned int receiveBufferPeak() override { return _hwmRx; };
  virtual unsigned int transmitBufferPeak() override { return _hwmTx; };
  virtual unsigned int errorStatus() override { return 0; }

  QueueHandle_t rx_queue_handle;    // received message queue, using FreeRTOS queue API

/// \endcond
private:
  unsigned int _numMsgsSent, _numMsgsRcvd, _numSendErr, _hwmRx, _hwmTx;
  unsigned int _num_rx_buffers, _num_tx_buffers;
  byte _gpio_tx, _gpio_rx;
  twai_node_handle_t twai_node_handle;
};

}