// VLCB4IN4OUT
// Version for use with ESP32 on-chip TWAI peripheral
// Uses a single core of the ESP32

// there is no default board for this project so you should change the IO pin assigmments to
// match your specific hardware

/*
  Copyright (C) 2026 Duncan Greenwood
  Copyright (C) 2023 Martin Da Costa
  This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
  Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0

  3rd party libraries needed for compilation:

  Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
*/

//////////////////////////////////////////////////////////////////////////
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

// Instantiate module objects
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
  ecService.setEventHandler(eventhandler);
  // register the VLCB request event handler to receive event status requests.
  epService.setRequestEventHandler(eventhandler);
  // register a validator for taught VLCB events.
  etService.setEventValidator(eventValidator);

  // configure and start CAN bus and VLCB message processing
  // vcanesp32.setNumBuffers(16, 4);  // more buffers = more memory used, fewer = less
  vcanesp32.setPins(1, 0);  // select pins for CAN TX & RX

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

  // show code version and copyright notice
  printConfig();
}

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

  uint32_t stimer = millis();
  Serial.begin(115200);
  while (!Serial && millis() - stimer < 3000)
    ;

  Serial << endl
         << F("> ** VLCB 4 in 4 out ESP32 single core ** ") << __FILE__ << endl;

  setupVLCB();
  setupModule();

  // end of setup
  Serial << F("> ready") << endl
         << endl;
}

void loop() {

  // do VLCB message, switch and LED processing
  VLCB::process();

  // Run the LED code
  for (byte i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();

  // end of loop()
}

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
  DEBUG_PRINT(F("sk> Will create event nn=") << nn << F(" en=") << preferredEN);
  byte eventIndex = VLCB::findExistingEvent(nn, preferredEN);
  if (VLCB::isEventIndexValid(eventIndex)) {
    DEBUG_PRINT(F("sk> Preferred event already exists. Try to find a free event number"));
    // Find an unused EN
    for (unsigned int en = 1; en < 65535; ++en) {
      DEBUG_PRINT(F("sk> Trying en=") << en);
      eventIndex = VLCB::findExistingEvent(nn, en);
      if (!VLCB::isEventIndexValid(eventIndex)) {
        DEBUG_PRINT(F("sk> is free, create it."));
        eventIndex = VLCB::findEmptyEventSpace();
        if (VLCB::isEventIndexValid(eventIndex)) {
          VLCB::createEventAtIndex(eventIndex, nn, en);
          DEBUG_PRINT(F("sk> Created event at index=") << eventIndex);
          return eventIndex;
        } else {
          DEBUG_PRINT(F("sk> No empty space for event. index=") << eventIndex);
          return 0xFF;
        }
      }
    }
    DEBUG_PRINT(F("sk> No free event number"));
    return 0xFF;
  } else {
    DEBUG_PRINT(F("sk> Preferred event does not exist."));
    // Find an empty slot to create an event.
    eventIndex = VLCB::findEmptyEventSpace();
    if (VLCB::isEventIndexValid(eventIndex)) {
      DEBUG_PRINT(F("sk> Creating preferred event"));
      VLCB::createEventAtIndex(eventIndex, nn, preferredEN);
      return eventIndex;
    } else {
      DEBUG_PRINT(F("sk> No empty space for event. index=") << eventIndex);
      return 0xFF;
    }
  }
}

void processSwitches(void) {
  for (byte i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].run();
    if (moduleSwitch[i].stateChanged()) {
      byte nv = i + 1;
      byte nvval = VLCB::readNV(nv);
      byte swNum = i + 1;
      DEBUG_PRINT(F("sk> Button ") << i << F(" state change detected. NV Value = ") << nvval);

      byte eventIndex = VLCB::findExistingEventByEv(1, swNum);
      if (!VLCB::isEventIndexValid(eventIndex)) {
        DEBUG_PRINT(F("sk> No event for this button."));
        eventIndex = createEvent(VLCB::getNodeNum(), swNum);
        if (!VLCB::isEventIndexValid(eventIndex)) {
          DEBUG_PRINT(F("sk> Could not create default event"));
          // Could not create default event. Ignore it and don't send an event.
          continue;
        }
        // Created a valid event, now set EV1 to the switch number.
        VLCB::writeEventVariable(eventIndex, 1, swNum);
        DEBUG_PRINT(F("sk> Wrote event variable 1 value=") << swNum);
      }

      DEBUG_PRINT(F("sk> Using event at index=") << eventIndex);

      switch (nvval) {
        case 1:
          // ON and OFF
          state[i] = (moduleSwitch[i].getState());
          DEBUG_PRINT(F("sk> Button ") << i << (state[i] ? F(" pressed, send state: ") : F(" released, send state: ")) << state[i]);
          epService.sendEventAtIndex(state[i], eventIndex);
          break;

        case 2:
          // Only ON
          if (moduleSwitch[i].isPressed()) {
            state[i] = true;
            DEBUG_PRINT(F("sk> Button ") << i << F(" pressed, send state: ") << state[i]);
            epService.sendEventAtIndex(state[i], eventIndex);
          }
          break;

        case 3:
          // Only OFF
          if (moduleSwitch[i].isPressed()) {
            state[i] = false;
            DEBUG_PRINT(F("sk> Button ") << i << F(" pressed, send state: ") << state[i]);
            epService.sendEventAtIndex(state[i], eventIndex);
          }
          break;

        case 4:
          // Toggle button
          if (moduleSwitch[i].isPressed()) {
            state[i] = !state[i];
            DEBUG_PRINT(F("sk> Button ") << i << (state[i] ? F(" pressed, send state: ") : F(" released, send state: ")) << state[i]);
            epService.sendEventAtIndex(state[i], eventIndex);
          }
          break;

        default:
          DEBUG_PRINT(F("sk> Button ") << i << F(" do nothing."));
          break;
      }
    }
  }
}

//
/// called from the VLCB library when a learned event is received
//
void eventhandler(byte index, const VLCB::VlcbMessage *msg) {
  byte opc = msg->data[0];

  DEBUG_PRINT(F("sk> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));

  unsigned int node_number = (msg->data[1] << 8) + msg->data[2];
  unsigned int event_number = (msg->data[3] << 8) + msg->data[4];
  DEBUG_PRINT(F("sk> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(F("sk> op_code = ") << _HEX(opc));

  switch (opc) {
    case OPC_ACON:
    case OPC_ASON:
      DEBUG_PRINT(F("sk> case is opCode ON"));
      for (byte i = 0; i < NUM_LEDS; i++) {
        byte ev = i + 2;
        byte evval = VLCB::getEventEVval(index, ev);
        //DEBUG_PRINT(F("sk> EV = ") << ev << (" Value = ") << evval);

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
      DEBUG_PRINT(F("sk> case is opCode OFF"));
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
      byte evval = VLCB::getEventEVval(index, 1) - 1;
      DEBUG_PRINT(F("> Handling request op =  ") << _HEX(opc) << F(", request input = ") << evval << F(", state = ") << state[evval]);
      epService.sendEventResponse(state[evval], index);
  }
}

void printConfig(void) {
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Martin Da Costa (MERG M6237) 2025, Duncan Greenwood (MERG 5767) 2026") << endl;
}
