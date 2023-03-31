/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <PinChangeInterrupt.h>
#include <ServoInput.h>
#include <USIWire.h>

#include <avr/wdt.h>

#define COMMAND_DISARM (0x68)
#define COMMAND_ARM    (0x69)

#define SOFTWARE_TIMEOUT (1000)
#define REMOTE_TIMEOUT   (100)

#define DISARM_TIME      (5000)

#define REMOTE_THRESHOLD (1600)

enum class State : uint8_t {
  Disarmed           = 0,
  DisarmingRequested = 1,
  DisarmingRemote    = 2,
  DisarmingSoftware  = 3,
  Armed              = 4,
};

struct StatePacket {
  bool servoInputSteeringUpdate : 1;
  bool servoInputThrottleUpdate : 1;
  bool armable                  : 1;
  uint8_t unused1               : 2;
  State state                   : 3;
  uint16_t servoInputSteeringValue;
  uint16_t servoInputThrottleValue;

  // handy shortcuts to the state
  State operator=(const State state_) {
    state = state_;
  }
} __attribute__((packed));

constexpr int peripheralAddress = 0x08;

constexpr int servoInputSteeringPin = 3;
constexpr int servoInputThrottlePin = 4;
constexpr int disableSteeringPin = 5;
constexpr int enableThrottlePin = 1;

ServoInputPin<servoInputSteeringPin> servoInputSteering;
ServoInputPin<servoInputThrottlePin> servoInputThrottle;

static volatile StatePacket state = {
  .servoInputSteeringUpdate = false,
  .servoInputThrottleUpdate = false,
  .armable = false,
  .unused1 = 0,
  .state = State::Disarmed,

  .servoInputSteeringValue = 1500,
  .servoInputThrottleValue = 1500
};

volatile unsigned long armTime = 0;
volatile unsigned long disarmTime = 0;
volatile unsigned long remoteTime = 0;

void setup() {
  pinMode(disableSteeringPin, OUTPUT);
  pinMode(enableThrottlePin, OUTPUT);
  digitalWrite(disableSteeringPin, HIGH);
  digitalWrite(enableThrottlePin, LOW);
  wdt_enable(WDTO_1S);

  Wire.begin(peripheralAddress);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

bool remoteArmed() {
  return ((millis() - remoteTime) < REMOTE_TIMEOUT)
      && (state.servoInputThrottleValue > REMOTE_THRESHOLD);
}

void arm() {
  armTime = millis();
  digitalWrite(disableSteeringPin, LOW);
  digitalWrite(enableThrottlePin, HIGH);
}

void disarm() {
  disarmTime = millis();
  digitalWrite(enableThrottlePin, LOW);
  digitalWrite(disableSteeringPin, HIGH);
}

void loop() {
  // update state packet with new servo values, if available.
  if (servoInputSteering.available()) {
    auto value = servoInputSteering.getPulse();

    noInterrupts();
    state.servoInputSteeringUpdate = true;
    state.servoInputSteeringValue = value;
    interrupts();
  }

  if (servoInputThrottle.available()) {
    auto value = servoInputThrottle.getPulse();

    noInterrupts();
    state.servoInputThrottleUpdate = true;
    state.servoInputThrottleValue = value;
    remoteTime = millis();
    interrupts();
  }

  // update armable flag
  state.armable = remoteArmed();

  // Check if we need to make a state transistion
  noInterrupts();
  switch (state.state) {
    case State::Armed:
      if (!remoteArmed()) {
        disarm();
        state = State::DisarmingRemote;
      } else if ((millis() - armTime) >= SOFTWARE_TIMEOUT) {
        disarm();
        state = State::DisarmingSoftware;
      }
      break;

    case State::Disarmed:
      break;
    
    case State::DisarmingRequested:
    case State::DisarmingRemote:
    case State::DisarmingSoftware:
      if ((millis() - disarmTime) >= DISARM_TIME) {
        state = State::Disarmed;        
      }
      break;
  }
  interrupts();
  wdt_reset();
}

void receiveEvent(int length) {
  auto command = Wire.read();
  switch (command) {
    case COMMAND_ARM:
      if ((state.state == State::Armed || state.state == State::Disarmed)
          && remoteArmed()) {
        arm();
        state = State::Armed;
      }
      break;
    
    case COMMAND_DISARM:
      disarm();
      if (state.state == State::Armed) {
        state = State::DisarmingRequested;
      }
      break;

    default:
      break;
      /* do nothing */
  }

  // dump remainder of rx buffer
  while (Wire.available()) {
    (void) Wire.read();
  }
}

void requestEvent() {
  Wire.write(reinterpret_cast<volatile uint8_t*>(&state), sizeof state);
  state.servoInputSteeringUpdate = false;
  state.servoInputThrottleUpdate = false;
}
