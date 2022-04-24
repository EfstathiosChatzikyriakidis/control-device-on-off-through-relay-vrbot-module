/*
 *  Control Device (On/Off) Through Relay & VRBot Module.
 *
 *  Copyright (C) 2010 Efstathios Chatzikyriakidis (stathis.chatzikyriakidis@gmail.com)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// include VRBot communication protocol commands.
#include "vrbot_protocol.h"

// the serial communication pin number for receiving data.
const uint8_t receivePin = 12;

// the serial communication pin number for transmitting data.
const uint8_t transmitPin = 13;

// the serial communication baud rate.
const long baudRate = 9600;

// the period of a bit.
const int bitPeriod = 1000000 / baudRate;

// the timeout for each communication (secs).
const unsigned char timeOut = 5;

// the pin number of the relay switch.
const int relayPin = 7;

// the pin number of the status led.
const int ledPin = 8;

// the delay time (ms) status led stay off.
const long statusLedOffDelay = 2000;

// startup point entry (runs once).
void
setup() {
  // set the relay pin as an output.
  pinMode(relayPin, OUTPUT);

  // set the led pin as an output.
  pinMode(ledPin, OUTPUT);

  // PC UART pins initialization.
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);

  // VRBot UART pins initialization.
  pinMode(receivePin, INPUT);
  pinMode(transmitPin, OUTPUT);
  
  // VRBot mode pins initialization.
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // connect digital pin 2 to digital pin 3 to
  // enter normal mode - digital pin 2 is high.

  // connect digital pin 2 to digital pin 4 to
  // enter bridge mode - digital pin 2 is low.

  // set VRBot mode pins to HIGH/LOW according to datasheet.
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  
  // if digital pin 2 is LOW enter bridge mode.
  if (digitalRead(2) == LOW) {
    // bridge mode allow direct communication between
    // the VRBot module and the VRBot GUI application.
    while (true) {
      int pc2vr = digitalRead(0);
      digitalWrite(transmitPin, pc2vr);
    
      int vr2pc = digitalRead(receivePin);
      digitalWrite(1, vr2pc);
    }
  } 

  // if digital pin 2 is HIGH enter normal mode.

  // setup the serial line for communication.
  Serial.begin(baudRate);
  
  // delay some time before trying to setup VRBot.
  delay(200);

  // print verbose welcome messages.
  Serial.println("Arduino Duemilanove VRBot Control Program.");
  Serial.println("Enter Normal Mode.");

  // setup the VRBot.
  Serial.println("Try to setup VRBot.");
  VRBotSetup();

  // detect the VRBot.
  Serial.println("Try to detect VRBot.");
  if (!VRBotDetect())
    Serial.println("VRBot NA.");
  else {
    Serial.println("VRBot detected.");

    // set VRBot timeout (in secs).
    Serial.print("Setting timeout to: ");
    Serial.print(timeOut, DEC);
    Serial.println(" seconds.");
    VRBotSetTimeout(timeOut);

    // set VRBot language to English.
    Serial.println("Setting Language to: English.");
    VRBotSetLanguage(0);  
  } 
}

// loop the main sketch.
void
loop() {
  // handle speaker dependent recognition.
  SDRecognition();   
}

// speaker dependent recognition handler.
void
SDRecognition() {
  // speaker recognized command.
  int cmd;

  // print verbose message.
  Serial.println("Say Trigger!");

  // status led on (light).
  statusLedOn(0);

  // start SD trigger message recognition and wait for trigger.
  VRBotRecognizeSD(0);

  // check recognition result.
  cmd = VRBotCheckResult();

  // status led off (dark).
  statusLedOff(statusLedOffDelay);

  // check for timeout result.
  if (cmd == -1) {
    Serial.println("Trigger Timeout.");
    return;
  }
  
  // check for error result.
  if (cmd == -2) {
    Serial.println("Trigger Error.");
    return;
  } 
  
  // print verbose messages.
  Serial.println("Select Group 1.");
  Serial.println("Say Base Command!");
  
  // status led on (light).
  statusLedOn(0);

  // start SD recognition group 1 and wait for a command.
  VRBotRecognizeSD(1);

  // check recognition result.
  cmd = VRBotCheckResult();

  // status led off (dark).
  statusLedOff(statusLedOffDelay);

  // check the base command.
  switch (cmd) {
    case -2:
      Serial.println("Base Command Error.");
      break;

    case -1:
      Serial.println("Base Command Timeout.");
      break;

    case 0:
      // print verbose messages.
      Serial.println("Base Command: DEVICE.");
      Serial.println("Say Sub Command!");

      // status led on (light).
      statusLedOn(0);

      // start SD recognition group 2 and wait for a command.
      VRBotRecognizeSD(2);

      // check recognition result.
      cmd = VRBotCheckResult();

      // status led off (dark).
      statusLedOff(statusLedOffDelay);

      // check the sub command.
      switch (cmd) {
        case -2:
          Serial.println("Sub Command Error.");
          break;

        case -1:
          Serial.println("Sub Command Timeout.");
          break;

        case 0:
          // switch on the relay.
          Serial.println("Sub Command: OPEN.");
          digitalWrite(relayPin, HIGH);
          break;

        case 1:
          // switch off the relay.
          Serial.println("Sub Command: CLOSE.");
          digitalWrite(relayPin, LOW);
          break;

        default: // other sub command.
          Serial.println("Other Sub Command.");
          break;
      }
      break;

    default: // other base command.
      Serial.println("Other Base Command.");
      break;
  }
}

// light the status led pin with delay time.
void
statusLedOn(int msTime) {
  digitalWrite(ledPin, HIGH);
  delay(abs(msTime));
}

// dark the status led pin with delay time.
void
statusLedOff(int msTime) {
  digitalWrite(ledPin, LOW);
  delay(abs(msTime));
}

// setup the VRBot.
void
VRBotSetup() {
  digitalWrite(transmitPin, HIGH);
  delayMicroseconds(bitPeriod); 
}

// read data from the VRBot.
unsigned char
VRBotRead() {
  // the data that will be read.
  uint8_t val = 0;

  // digitalRead delay is about 100 cycles.
  int bitDelay = bitPeriod - clockCyclesToMicroseconds(100);
  
  // one byte of serial data (LSB first).
  // ...--\    /--\/--\/--\/--\/--\/--\/--\/--\/--...
  //   \--/\--/\--/\--/\--/\--/\--/\--/\--/
  //  start  0   1   2   3   4   5   6   7 stop

  while (digitalRead (receivePin));

  // confirm that this is a real start bit, not line noise.
  if (digitalRead(receivePin) == LOW) {
    // frame start indicated by a falling edge and low start bit.

    // jump to the middle of the low start bit.
    delayMicroseconds(bitDelay / 2 - clockCyclesToMicroseconds(50));
  
    // offset of the bit in the byte: from 0 (LSB) to 7 (MSB).
    for (int offset = 0; offset < 8; offset++) {
      // jump to middle of next bit.
      delayMicroseconds(bitDelay);

      // read bit.
      val |= digitalRead(receivePin) << offset;
    }
  
    delayMicroseconds(bitPeriod);

    // return the data.    
    return val;
  }
  
  return -1;
}

// write data to the VRBot.
void
VRBotWrite (uint8_t b) {
  // defensive programming issue.
  if (baudRate == 0) return;

  // digitalWrite delay is about 50 cycles.
  int bitDelay = bitPeriod - clockCyclesToMicroseconds(50);

  byte mask;

  digitalWrite(transmitPin, LOW);
  delayMicroseconds(bitDelay);

  for (mask = 0x01; mask; mask <<= 1) {
    if (b & mask) // choose bit.
      digitalWrite(transmitPin, HIGH); // send 1.
    else
      digitalWrite(transmitPin, LOW); // send 0.

    delayMicroseconds(bitDelay);
  }

  digitalWrite(transmitPin, HIGH);
  delayMicroseconds(bitDelay);
}

// detect the VRBot.
unsigned char
VRBotDetect() {
  unsigned char i;

  for (i = 0; i < 5; ++i) {
    delay(100);    
    VRBotWrite(CMD_BREAK);        
    if (VRBotRead() == STS_SUCCESS)
      return 255;
  }

  return 0;
}

// set the language for the VRBot.
unsigned char
VRBotSetLanguage(unsigned char lang) {
  VRBotWrite(CMD_LANGUAGE);
  delay(5);
  VRBotWrite(ARG_ZERO + lang);

  if (VRBotRead() == STS_SUCCESS)
    return 255;

  return 0;
}

// try to recognize a speaker
// dependent group from VRBot.
void
VRBotRecognizeSD(unsigned char group) {
  VRBotWrite(CMD_RECOG_SD);
  delay(5);
  VRBotWrite(ARG_ZERO + group);
}

// set the timeout with the VRBot communication.
void
VRBotSetTimeout(unsigned char s) {
  VRBotWrite(CMD_TIMEOUT);
  delay(5);

  VRBotWrite(ARG_ZERO + s);
  delay(5);
}

// check the result from a possible VRBot recognition.
signed char
VRBotCheckResult() {
  unsigned char rx;

  rx = VRBotRead();
  if (rx == STS_SIMILAR || rx == STS_RESULT) {
    delay(5);
    VRBotWrite(ARG_ACK);

    // return command recognized.
    return (VRBotRead() - ARG_ZERO);
  }

  // timeout return code.
  if (rx == STS_TIMEOUT)
    return -1;
  
  // error return code.
  return -2;
}
