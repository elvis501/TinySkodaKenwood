/*
 * TinySender.cpp
 *
 *  Example for sending using TinyIR. By default sends simultaneously using all supported protocols
 *  To use a single protocol, simply delete or comment out all unneeded protocols in the main loop
 *  Program size is significantly reduced when using a single protocol
 *  For example, sending only 8 bit address and command NEC codes saves 780 bytes program memory and 26 bytes RAM compared to SimpleSender,
 *  which does the same, but uses the IRRemote library (and is therefore much more flexible).
 *
 *
 * The FAST protocol is a proprietary modified JVC protocol without address, with parity and with a shorter header.
 *  FAST Protocol characteristics:
 * - Bit timing is like NEC or JVC
 * - The header is shorter, 3156 vs. 12500
 * - No address and 16 bit data, interpreted as 8 bit command and 8 bit inverted command,
 *     leading to a fixed protocol length of (6 + (16 * 3) + 1) * 526 = 55 * 526 = 28930 microseconds or 29 ms.
 * - Repeats are sent as complete frames but in a 50 ms period / with a 21 ms distance.
 *
 *
 *  This file is part of IRMP https://github.com/IRMP-org/IRMP.
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2022-2024 Armin Joachimsmeyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 * ATmega16m1 support from
 * https://github.com/thomasonw/ATmegaxxM1-C1
 * 
 ************************************************************************************
 */
#include <Arduino.h>
#include<avr/wdt.h> /* Header for watchdog timers in AVR */

#include "PinDefinitionsAndMore.h" // Set IR_SEND_PIN for different CPU's

#define DEBUGLOG

#ifdef DEBUGLOG
# define LLOG(...) Serial.print(__VA_ARGS__)
# define LLOGln(...) Serial.println(__VA_ARGS__)
#else
# define LLOG(...)
# define LLOGln(...)
#endif

#define digitalPinToInterrupt(p)    ((p) == 16 ? 0 : ((p) == 17 ? 1 : ((p) == 21 ? 2 : ((p) == 12 ? 3 : NOT_AN_INTERRUPT))))

#define IR_SEND_PIN 6
#define IR_RECEIVE_PIN  12

#define USE_EXTENDED_NEC_PROTOCOL

#include "TinyIRReceiver.hpp" // include the code
#include "TinyIRSender.hpp"



void setup() {
#ifdef DEBUGLOG
    Serial.begin(115200);
#endif
    wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
    delay(3000);
    // Just to know which program is running on my Arduino
    LLOGln(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TINYIR));
    
    // Enables the interrupt generation on change of IR input signal
    if (!initPCIInterruptForTinyReceiver()) {
        LLOGln(F("No interrupt available for pin " STR(IR_RECEIVE_PIN))); // optimized out by the compiler, if not required :-)
    }
    LLOGln(F("Ready to receive NEC IR signals at pin " STR(IR_RECEIVE_PIN)));
    LLOGln(F("Send NEC IR signals at pin " STR(IR_SEND_PIN)));
    wdt_enable(WDTO_2S);  /* Enable the watchdog with a timeout of 2 seconds */
}

/*
 * Set up the data to be sent.
 * The compiler is intelligent and removes the code for 16 bit address handling if we call it with an uint8_t address :-).
 * Using an uint16_t address or data requires additional 28 bytes program memory for NEC and 56 bytes program memory for FAST.
 */
uint16_t  sSkodaSWC = 0x1782;
uint8_t   sAddress  = 0xb9;  // Kenwood 
uint8_t   sCommand  = 0x00;
uint8_t   sRepeats  = 0;

void loop() {
    /*
     * Print current send values
     */

    if (TinyIRReceiverData.justWritten) {
        TinyIRReceiverData.justWritten = false;

    if ( TinyIRReceiverData.Address == sSkodaSWC ) {
      switch ( TinyIRReceiverData.Command ){
        case 0x01:
          sCommand = 0x14;  // Vol+
          break;
       case 0x00:
          sCommand = 0x15;  // Vol-
          break;
       case 0x0B:
          sCommand = 0x0B;  // Next
          break;
       case 0x0A:
          sCommand = 0x0A;  // Prev
          break;
       default:
          sCommand = 0x00;
      }

      if ( sCommand ){
        sRepeats = ( TinyIRReceiverData.Flags == IRDATA_FLAGS_IS_REPEAT );
        sendNEC(IR_SEND_PIN, sAddress, sCommand, sRepeats );
      }
      
    }
    
    //LLOGln();
    Serial.print(F("Send now:"));
    Serial.print(F(" address=0x"));
    Serial.print( TinyIRReceiverData.Address , HEX);
    Serial.print(F(" command=0x"));
    LLOGln(sCommand, HEX);
    //Serial.print(F(" repeats="));
    //Serial.print(TinyIRReceiverData.Flags);
    //LLOGln();
    
    }
    
    wdt_reset();  /* Reset the watchdog */    
    delay(10);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}
