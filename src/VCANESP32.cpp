// Copyright (C) Duncan Greenwood 2026 (duncan_greenwood@hotmail.com)
// This file is part of VLCB-Arduino project on
// https://github.com/SvenRosvall/VLCB-Arduino Licensed under the Creative
// Commons Attribution-NonCommercial-ShareAlike 4.0 International License. The
// full licence can be found at:
// http://creativecommons.org/licenses/by-nc-sa/4.0/

/// notes:
/// the twai_frame_t structure does not contain space for the data payload
/// a buffer must be allocated when creating a message
/// and freed when a message goes out of scope
/// this reduces memory use and improves performance by minimising data copying

// 3rd party libraries

#include <Streaming.h>
#include <VCANESP32.h>

///
/// VCANESP32 VLCB CAN transport driver
///

namespace VLCB {

/// these free functions are used as callbacks from the TWAI driver to the
/// application the VCANESP32 object instance pointer (this) is passed as the
/// user data context arg 'user_ctx'

//
/// message receive callback
/// called by the TWAI driver in interrupt context whenever a new CAN message is
/// received from the bus pinned in IRAM for better performance place the
/// message in the receive queue
//

static bool IRAM_ATTR twai_rx_callback(twai_node_handle_t handle,
                                       const twai_rx_done_event_data_t* edata,
                                       void* user_ctx) {
  VCANESP32* vcanesp32_instance_ptr = (VCANESP32*)user_ctx;
  twai_frame_t rx_frame;

  // allocate the frame data buffer

  rx_frame.buffer = (uint8_t*)calloc(MAX_CAN_DATA_LEN, sizeof(uint8_t));
  rx_frame.buffer_len = MAX_CAN_DATA_LEN;

  // receive the frame from the TWAI driver and add to the receive queue

  if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
    if (xQueueSendFromISR(vcanesp32_instance_ptr->rx_queue_handle, &rx_frame,
                          NULL) != pdPASS) {
      Serial.printf("error: twai_rx_callback: unable to queue message");
    }
  } else {
    Serial.printf(
        "error: twai_rx_callback: error receiving message from TWAI driver");
  }

  return false;
}

//
/// message transmit callback
/// called by the TWAI driver in interrupt context whenever a CAN message has
/// been successfully transmitted to the bus pinned in IRAM for better
/// performance free the dynamically allocated data buffer
//

static bool IRAM_ATTR twai_tx_callback(twai_node_handle_t handle,
                                       const twai_tx_done_event_data_t* edata,
                                       void* user_ctx) {
  if (!edata->is_tx_success) {
    Serial.printf("error: twai_tx_callback: error sending message");
  }

  free(edata->done_tx_frame->buffer);
  return false;
}

//
/// error callback
/// called by the TWAI driver in interrupt context if an error occurs
/// pinned in IRAM for better performance
//

static bool IRAM_ATTR twai_error_callback(twai_node_handle_t handle,
                                          const twai_error_event_data_t* edata,
                                          void* user_ctx) {
  Serial.printf("error twai_error_callback, error = %lu\n",
                edata->err_flags.val);
  return false;
}

//
/// state change callback
/// called by the TWAI driver in interrupt context whenever a state change
/// occurs pinned in IRAM for better performance
//

static bool IRAM_ATTR twai_state_change_callback(
    twai_node_handle_t handle, const twai_state_change_event_data_t* edata,
    void* user_ctx) {
  Serial.printf("info: twai_state_change_callback, from %u to %u\n",
                edata->old_sta, edata->new_sta);
  return false;
}

//
/// constructors
/// with or without IO pins specified
//

VCANESP32::VCANESP32() {
  _num_rx_buffers = rx_qsize;
  _num_tx_buffers = tx_qsize;
}

VCANESP32::VCANESP32(byte gpio_tx, byte gpio_rx)
    : _gpio_tx(gpio_tx), _gpio_rx(gpio_rx) {
  _num_rx_buffers = rx_qsize;
  _num_tx_buffers = tx_qsize;
}

//
/// destructor
//

VCANESP32::~VCANESP32() {
  twai_node_disable(twai_node_handle);
  twai_node_delete(twai_node_handle);
}

//
/// explicitly set the IO pins to be used for CAN TX and RX
//

void VCANESP32::setPins(byte gpio_tx, byte gpio_rx) {
  _gpio_tx = gpio_tx;
  _gpio_rx = gpio_rx;
}

//
/// set the size of CAN frame transmit and receive queues
/// these can be tuned according to bus load and available memory
/// default size is 32
//

void VCANESP32::setNumBuffers(unsigned int num_rx_buffers,
                              unsigned int num_tx_buffers) {
  _num_rx_buffers = num_rx_buffers;
  _num_tx_buffers = num_tx_buffers;
}

//
/// initialise the TWAI driver, message queues and receive callback
//

bool VCANESP32::begin() {
  // initialise counters

  _numMsgsSent = 0;
  _numMsgsRcvd = 0;
  _numSendErr = 0;
  _hwmRx = 0;
  _hwmTx = 0;

  // the tx queue is provided by the TWAI driver
  // the rx queue is provided by this library, using the FreeRTOS queue API

  rx_queue_handle = xQueueCreate(_num_rx_buffers, sizeof(twai_frame_t));

  /// initialise the TWAI driver

  twai_onchip_node_config_t twai_node_config;

  bzero(&twai_node_config, sizeof(twai_onchip_node_config_t));

  twai_node_config.io_cfg.tx = (gpio_num_t)_gpio_tx;
  twai_node_config.io_cfg.rx = (gpio_num_t)_gpio_rx;
  twai_node_config.bit_timing.bitrate = CANBITRATE;
  twai_node_config.tx_queue_depth = _num_tx_buffers;

  // create a new TWAI controller driver instance

  uint32_t ret = 0;
  twai_node_handle_t twai_node_handle = NULL;

  if ((ret = twai_new_node_onchip(&twai_node_config, &twai_node_handle)) ==
      ESP_OK) {
    // register the receive event callback (before starting the controller)

    twai_event_callbacks_t user_cbs;
    bzero(&user_cbs, sizeof(twai_event_callbacks_t));
    user_cbs.on_rx_done = twai_rx_callback;
    user_cbs.on_tx_done = twai_tx_callback;
    user_cbs.on_state_change = twai_state_change_callback;
    user_cbs.on_error = twai_error_callback;

    // we pass a pointer to this object instance as user context data
    // this gives the message receive callback access to the receive queue
    // handle

    twai_node_register_event_callbacks(twai_node_handle, &user_cbs, this);

    // start the TWAI driver instance

    if ((ret = twai_node_enable(twai_node_handle)) != ESP_OK) {
      Serial.printf("error: twai_node_enable returns %lu\n", ret);
    }

  } else {
    Serial.printf("error: twai_new_node_onchip returns %lu\n", ret);
  }

  return (ret == ESP_OK);
}

//
/// this method is called by the CanService process function on a regular basis
/// check for one or more messages in the receive queue
//

bool VCANESP32::available() {
  static unsigned long last_stats_captured_at = 0;

  if (millis() - last_stats_captured_at >= 1000) {
    last_stats_captured_at = millis();
    captureTWAIStats();
  }

  return (uxQueueMessagesWaiting(rx_queue_handle) > 0);
}

//
/// get next CAN message, if a message is available in the queue
//

CANFrame VCANESP32::getNextCanFrame(void) {
  twai_frame_t rx_msg;
  CANFrame frame;

  if (xQueueReceive(rx_queue_handle, &rx_msg, 0) == pdPASS) {
    frame.id = rx_msg.header.id;
    frame.len = rx_msg.header.dlc;
    frame.ext = rx_msg.header.ide;
    frame.rtr = rx_msg.header.rtr;
    memcpy(frame.data, rx_msg.buffer, rx_msg.buffer_len);

    // free the frame data buffer - this was allocated in the rx callback
    // function
    free(rx_msg.buffer);

    ++_numMsgsRcvd;
  }

  return frame;
}

//
/// send a CAN message
//

bool VCANESP32::sendCanFrame(CANFrame* frame) {
  esp_err_t ret;
  twai_frame_t tx_frame;

  // allocate the tx frame data buffer
  // this will be freed in the tx complete callback

  tx_frame.buffer = (uint8_t*)calloc(frame->len, sizeof(uint8_t));

  // populate the TWAI frame from from the VLCB message frame

  tx_frame.header.id = frame->id;
  tx_frame.header.ide = frame->ext;
  tx_frame.header.rtr = frame->rtr;
  tx_frame.header.dlc = frame->len;
  memcpy(tx_frame.buffer, frame->data, frame->len);
  tx_frame.buffer_len = frame->len;

  // send the frame - allow up to 500ms for tx queue space to become available

  if ((ret = twai_node_transmit(twai_node_handle, &tx_frame, 500)) == ESP_OK) {
    ++_numMsgsSent;
  }

  return (ret == ESP_OK);
}

//
/// display the CAN bus status instrumentation
//

void VCANESP32::printStatus() {
  // removed so that no libraries produce serial output
  // can be implemented in user's sketch
}

//
/// reset the CAN transceiver
//

void VCANESP32::reset() {
  twai_node_recover(twai_node_handle);
  return;
}

//
/// capture TWAI stats
//

void VCANESP32::captureTWAIStats() {
  twai_node_status_t node_status;
  twai_node_record_t node_statistics;
  twai_error_state_t node_error_state;
  uint32_t node_bus_err_num;

  if (twai_node_get_info(twai_node_handle, &node_status, &node_statistics) ==
      ESP_OK) {
    _numSendErr = node_status.tx_error_count;
    _numRecvErr = node_status.rx_error_count;

    node_error_state = node_status.state;
    node_bus_err_num = node_statistics.bus_err_num;

    if (node_error_state != TWAI_ERROR_ACTIVE) {
      Serial.printf("error: bus state = %u, num errors = %lu\n",
                    node_error_state, node_bus_err_num);
    }
  }

  return;
}

}  // namespace VLCB
