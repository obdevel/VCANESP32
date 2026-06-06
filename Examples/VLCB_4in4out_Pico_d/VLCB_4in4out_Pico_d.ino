// VLCB4IN4OUT
// Version for use with ESP32 on-chip TWAI peripheral
// Uses both cores of the ESP32

/*
  Copyright (C) 2026 Duncan Greenwood
  Copyright (C) 2023 Martin Da Costa
  This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
  Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0

  3rd party libraries needed for compilation:

  Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
*/

///////////////////////////////////////////////////////////////////////////////////
//
// Node variables:
//  NV1-4 - Behaviour of switch 1-4: 0) None 1) On/Off 2) On only 3) Off only 4) Toggle
//
// Event variables:
//  EV1 - Produce event for switch N where N is EV1 value in the range 1-4. Must be unique.
//  EV2-5 - Change LED 1-4: 0) No change 1) Normal 2) Slow blink 3) Fast blink
#define DEBUG 1  // set to 0 for no serial debug

#if DEBUG
#define DEBUG_PRINT(S) Serial << "Core " << xTaskGetCoreID(xTaskGetHandle(NULL)) << F(" ") << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// 3rd party libraries
#include <Streaming.h>

// VLCB library header files
#include <VLCB.h>       // Controller class
#include <VCANESP32.h>  // CAN controller

// forward function declarations
void eventhandler(byte, const VLCB::VlcbMessage *);
byte eventValidator(int nn, int en, byte evNum, byte evValue);
void printConfig();
void processSwitches();

// constants
const byte VER_MAJ = 1;              // code major version
const char VER_MIN = 'c';            // code minor version
const byte VER_BETA = 0;             // code beta sub-version
const byte MANUFACTURER = MANU_DEV;  // Module Manufacturer set to Development
const byte MODULE_ID = 82;           // CBUS module type

const byte LED_GRN = 14;  // VLCB green Unitialised LED pin
const byte LED_YLW = 15;  // VLCB yellow Normal LED pin
const byte SWITCH0 = 13;  // VLCB push button switch pin

// module name, must be 7 characters, space padded.
char mname[] = "4IN4OUT";

// Module objects
const byte LED[] = { 22, 26, 27, 28 };     // LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = { 18, 19, 20, 21 };  // Module Switch takes input to 0V.

const bool active = 0;  // 0 is for active low LED drive. 1 is for active high

const byte NUM_LEDS = sizeof(LED) / sizeof(LED[0]);
const byte NUM_SWITCHES = sizeof(SWITCH) / sizeof(SWITCH[0]);

// module objects
VLCB::Switch moduleSwitch[NUM_SWITCHES];  //  switch as input
VLCB::LED moduleLED[NUM_LEDS];            //  LED as output
byte state[NUM_SWITCHES];

VLCB::VCANESP32 vcanesp32;  // CAN transport object

// Service objects
VLCB::LEDUserInterface ledUserInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface;
VLCB::MinimumNodeService mnService;
VLCB::CanService canService(&vcanesp32);
VLCB::NodeVariableService nvService;
VLCB::ConsumeOwnEventsService coeService;
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;

// FreeRTOS task
TaskHandle_t task_handle;
QueueHandle_t queue_0_to_1, queue_1_to_0;

//
///  setup VLCB - runs once at power on called from setup()
//

void setupVLCB() {
  VLCB::checkStartupAction(LED_GRN, LED_YLW, SWITCH0);

  VLCB::setServices({ &mnService, &ledUserInterface, &serialUserInterface, &canService, &nvService,
                      &ecService, &epService, &etService, &coeService });
  // set config layout parameters
  VLCB::setNumNodeVariables(NUM_SWITCHES);
  VLCB::setMaxEvents(64);
  VLCB::setNumEventVariables(1 + NUM_LEDS);

  // set module parameters
  VLCB::setVersion(VER_MAJ, VER_MIN, VER_BETA);
  VLCB::setModuleId(MANUFACTURER, MODULE_ID);

  // set module name
  VLCB::setName(mname);

  // register the VLCB event handler, to receive event messages of learned events
  ecService.setEventHandler(loadrcvdmess);
  // register the VLCB request event handler to receive event status requests.
  epService.setRequestEventHandler(loadrcvdmess);

  // configure and start CAN bus and VLCB message processing
  //vcanesp32.setNumBuffers(16, 4);  // more buffers = more memory used, fewer = less
  //vcanesp32.setPio(0);             // PIO 0 is the default so this line commented out.  Alternative is value of 1
  vcanesp32.setPins(1, 0);  // select pins for CAN Tx & Rx

  if (!vcanesp32.begin()) {
    Serial << F("> error starting VLCB") << endl;
  } else {
    Serial << F("> VLCB started") << endl;
  }

  // initialise and load configuration
  VLCB::begin();

  Serial << F("> mode = (") << _HEX(VLCB::getCurrentMode()) << ") " << VLCB::Configuration::modeString(VLCB::getCurrentMode());
  Serial << F(", CANID = ") << VLCB::getCANID();
  Serial << F(", NN = ") << VLCB::getNodeNum() << endl;
}

//
///  setup Module - runs once at power on called from setup()
//

void setupModule() {
  // configure the module switches, active low
  for (byte i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].setPin(SWITCH[i], INPUT_PULLUP);
    state[i] = false;
  }

  // configure the module LEDs
  for (byte i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].setPin(LED[i], LOW);  //Second arguement sets 0 = active low, 1 = active high. Default if no second arguement is active high.
  }

  Serial << "> Module has " << NUM_LEDS << " LEDs and " << NUM_SWITCHES << " switches." << endl;
}

void setup() {

  uint32_t stimer = millis(), temp_val;

  Serial.begin(115200);
  while (!Serial && millis() - stimer < 3000)
    ;
  Serial << endl
         << F("> ** VLCB 4 in 4 out ESP32 dual core ** ") << __FILE__ << endl;

  // show code version and copyright notice
  printConfig();

  // create the two inter-task queues
  queue_0_to_1 = xQueueCreate(20, sizeof(uint32_t));
  queue_1_to_0 = xQueueCreate(20, sizeof(uint32_t));

  // create the 2nd task
  xTaskCreate(task_func,
              "task",
              2000,
              (void *)1,
              20,
              &task_handle);

  // give core 1 the go ahead
  xQueueSend(queue_0_to_1, (void *)0, portMAX_DELAY);

  // wait for core 1 to initialise EEPROM
  xQueueReceive(queue_0_to_1, &temp_val, portMAX_DELAY);

  setupModule();

  Serial << xTaskGetCoreID(xTaskGetHandle(NULL)) << F("> Module ready") << endl;
  // end of setup0
}

void loop() {

  uint32_t temp_val;

  if (uxQueueMessagesWaiting(queue_1_to_0) > 0) {

    xQueueReceive(queue_1_to_0, &temp_val, portMAX_DELAY);
    byte msgLen = (byte)temp_val;
    // Ensure Core 0 has loaded complete message into FIFO
    while ((msgLen + 1) != rp2040.fifo.available()) {}
    receivedData(msgLen);  // Action FIFO contents
  }

  // Run the LED code
  for (byte i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();

  // end of loop()
}

/// task function for 2nd task

void task_func(void *pvParameters) {

  uint32_t temp_val;

  // wait for go ahead from main task
  xQueueReceive(queue_0_to_1, &temp_val, portMAX_DELAY);

  setupVLCB();

  Serial << xTaskGetCoreID(xTaskGetHandle(NULL)) << F("> VLCB ready") << endl;

  //  BaseType_t xQueueSend(
  //                       QueueHandle_t xQueue,
  //                       const void * pvItemToQueue,
  //                       TickType_t xTicksToWait
  //                     );

  xQueueSend(queue_1_to_0, (void *)1, portMAX_DELAY);

  for (;;) {

    // do VLCB message processing
    VLCB::process();

    if (uxQueueMessagesWaiting(queue_0_to_1) == 3) {
      // uint8_t event = (uint8_t)rp2040.fifo.pop();
      // uint8_t _state = (uint8_t)rp2040.fifo.pop();
      // uint16_t sendData = (uint16_t)rp2040.fifo.pop();

      xQueueReceive(queue_0_to_1, &temp_val, portMAX_DELAY);
      uint8_t event = (uint8_t)temp_val;
      xQueueReceive(queue_0_to_1, &temp_val, portMAX_DELAY);
      uint8_t _state = (uint8_t)temp_val;
      xQueueReceive(queue_0_to_1, &temp_val, portMAX_DELAY);
      uint16_t sendData = (uint16_t)temp_val;

      if (event) {
        byte eventIndex = VLCB::findExistingEventByEv(1, sendData);
        if (!VLCB::isEventIndexValid(eventIndex)) {
          DEBUG_PRINT(F(" sk> No event for this button."));
          eventIndex = createEvent(VLCB::getNodeNum(), sendData);
          if (!VLCB::isEventIndexValid(eventIndex)) {
            DEBUG_PRINT(F(" sk> Could not create default event"));
            // Could not create default event. Ignore it and don't send an event.
          }
          // Created a valid event, now set EV1 to the switch number.
          VLCB::writeEventVariable(eventIndex, 1, sendData);
          DEBUG_PRINT(F(" sk> Wrote event variable 1 value=") << sendData);
        }
        epService.sendEventAtIndex(_state, eventIndex);
      } else {
        epService.sendEventResponse(_state, sendData);
      }
    }
  }
}

/////////////////////////////////////////////////////////
// Main Task Functions

void processSwitches(void) {
  for (byte i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].run();
    if (moduleSwitch[i].stateChanged()) {
      byte nv = i + 1;
      byte nvval = VLCB::readNV(nv);
      byte swNum = i + 1;
      bool event = true;

      DEBUG_PRINT(F(" sk> Button ") << i << F(" state change detected. NV Value = ") << nvval);

      switch (nvval) {
        case 1:
          // ON and OFF
          state[i] = (moduleSwitch[i].isPressed());
          DEBUG_PRINT(F(" sk> Button ") << i << (state[i] ? F(" pressed, send state: ") : F(" released, send state: ")) << state[i]);
          rp2040.fifo.push(event);
          rp2040.fifo.push(state[i]);
          rp2040.fifo.push(swNum);
          break;

        case 2:
          // Only ON
          if (moduleSwitch[i].isPressed()) {
            state[i] = true;
            DEBUG_PRINT(F(" sk> Button ") << i << F(" pressed, send state: ") << state[i]);
            rp2040.fifo.push(event);
            rp2040.fifo.push(state[i]);
            rp2040.fifo.push(swNum);
          }
          break;

        case 3:
          // Only OFF
          if (moduleSwitch[i].isPressed()) {
            state[i] = false;
            DEBUG_PRINT(F(" sk> Button ") << i << F(" pressed, send state: ") << state[i]);
            rp2040.fifo.push(event);
            rp2040.fifo.push(state[i]);
            rp2040.fifo.push(swNum);
          }
          break;

        case 4:
          // Toggle button
          if (moduleSwitch[i].isPressed()) {
            state[i] = !state[i];
            DEBUG_PRINT(F(" sk> Button ") << i << (state[i] ? F(" pressed, send state: ") : F(" released, send state: ")) << state[i]);
            rp2040.fifo.push(event);
            rp2040.fifo.push(state[i]);
            rp2040.fifo.push(swNum);
          }
          break;

        default:
          DEBUG_PRINT(F(" sk> Button ") << i << F(" do nothing."));
          break;
      }
    }
  }
}

void receivedData(byte length) {
  byte rcvdData[length];
  byte index = rp2040.fifo.pop();
  for (byte n = 0; n < length; n++) {
    rcvdData[n] = rp2040.fifo.pop();
  }

  byte opc = rcvdData[0];
  delay(50);
  DEBUG_PRINT(F(" sk> received data popped: index = ") << index << F(", opcode = 0x") << _HEX(rcvdData[0]));
  DEBUG_PRINT(F(" sk> received data popped: length = ") << length);

#if DEBUG
  unsigned int node_number = (rcvdData[1] << 8) + rcvdData[2];
  unsigned int event_number = (rcvdData[3] << 8) + rcvdData[4];
#endif

  DEBUG_PRINT(F(" sk> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(F(" sk> op_code = 0x") << _HEX(opc));

  switch (opc) {
    case OPC_ACON:
    case OPC_ASON:
      DEBUG_PRINT(F(" k> case is opCode ON"));
      for (byte i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 2;
        byte evval = VLCB::getEventEVval(index, ev);
        //DEBUG_PRINT(F(" sk> EV = ") << ev << (" Value = ") << evval);

        switch (evval) {
          case 1:
            moduleLED[i].on();
            break;

          case 2:
            moduleLED[i].blink(500);
            break;

          case 3:
            moduleLED[i].blink(250);
            break;

          default:
            break;
        }
      }
      break;

    case OPC_ACOF:
    case OPC_ASOF:
      DEBUG_PRINT(F(" sk> case is opCode OFF"));
      for (byte i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 2;
        byte evval = VLCB::getEventEVval(index, ev);

        if (evval > 0) {
          moduleLED[i].off();
        }
      }
      break;

    case OPC_AREQ:
    case OPC_ASRQ:
      bool event = false;
      byte evval = VLCB::getEventEVval(index, 1) - 1;
      DEBUG_PRINT(F(" sk> Handling request op =  ") << _HEX(opc) << F(", request input = ") << evval << F(", state = ") << state[evval]);
      rp2040.fifo.push(event);
      rp2040.fifo.push(state[evval]);
      rp2040.fifo.push(index);
      break;
  }
}

////////////////////////////////////////////////////////////////////
//Core 1 Functions

byte eventValidator(int nn, int en, byte evNum, byte evValue) {
  // EV#1 is for produced events. It specifies which switch triggers this event.
  // There can only be one event with EV#1 set to a specific switch.
  if (evNum == 1) {
    // Search for an event where EV#1 has the same value.
    byte index = VLCB::findExistingEventByEv(evNum, evValue);
    if (VLCB::isEventIndexValid(index)) {
      // Yes, one such event does exist.
      return CMDERR_INV_EV_VALUE;
    }
  }
  return GRSP_OK;
}

byte createEvent(unsigned int nn, byte preferredEN) {
  DEBUG_PRINT(F(" sk> Will create event nn=") << nn << F(" en=") << preferredEN);
  byte eventIndex = VLCB::findExistingEvent(nn, preferredEN);
  if (VLCB::isEventIndexValid(eventIndex)) {
    DEBUG_PRINT(F(" sk> Preferred event already exists. Try to find a free event number"));
    // Find an unused EN
    for (unsigned int en = 1; en < 65535; ++en) {
      DEBUG_PRINT(F(" sk> Trying en=") << en);
      eventIndex = VLCB::findExistingEvent(nn, en);
      if (!VLCB::isEventIndexValid(eventIndex)) {
        DEBUG_PRINT(F(" sk> is free, create it."));
        eventIndex = VLCB::findEmptyEventSpace();
        if (VLCB::isEventIndexValid(eventIndex)) {
          VLCB::createEventAtIndex(eventIndex, nn, en);
          DEBUG_PRINT(F(" sk> Created event at index=") << eventIndex);
          return eventIndex;
        } else {
          DEBUG_PRINT(F(" sk> No empty space for event. index=") << eventIndex);
          return 0xFF;
        }
      }
    }
    DEBUG_PRINT(F(" sk> No free event number"));
    return 0xFF;
  } else {
    DEBUG_PRINT(F(" sk> Preferred event does not exist."));
    // Find an empty slot to create an event.
    eventIndex = VLCB::findEmptyEventSpace();
    if (VLCB::isEventIndexValid(eventIndex)) {
      DEBUG_PRINT(F(" sk> Creating preferred event"));
      VLCB::createEventAtIndex(eventIndex, nn, preferredEN);
      return eventIndex;
    } else {
      DEBUG_PRINT(F(" sk> No empty space for event. index=") << eventIndex);
      return 0xFF;
    }
  }
}

//
/// called from the VLCB library when a learned event is received
//

void loadrcvdmess(byte index, const VLCB::VlcbMessage *msg) {
  rp2040.fifo.push(msg->len);
  rp2040.fifo.push(index);
  for (byte n = 0; n < msg->len; n++) {
    rp2040.fifo.push(msg->data[n]);
  }
  DEBUG_PRINT(F(" sk> received message put: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));
  DEBUG_PRINT(F(" sk> received message put: length = ") << msg->len);
}


void printConfig(void) {
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Martin Da Costa (MERG M6237) 2025") << endl;
}
