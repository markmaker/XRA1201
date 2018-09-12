/* 

Copyright (c) 2015 Markus Kuehni, mailto:mark@makr.zone, http://makr.zone

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/


#ifndef XRA1201_h
#define XRA1201_h

#include <Arduino.h>
#include <Wire.h>

#ifndef OUTPUT_OPENDRAIN
#define OUTPUT_OPENDRAIN 4
#endif

#define XRA1201_REGISTER_COUNT 0x16

class XRA1201  {
   public:
     XRA1201(uint8_t address = 0x20, TwoWire &wire = Wire);
     void begin(bool initialize = true);
     
     uint8_t getAddress();
     void setInterrupt(uint8_t pin);
     void beginTransaction();
     uint8_t endTransaction();

     void digitalWrite(uint8_t pin, uint8_t val);
     uint8_t digitalRead(uint8_t pin);
     void pinMode(uint8_t pin, uint8_t mode);
     
     uint8_t writeSettings();
     
   private:
     TwoWire &wire;
     uint8_t address;
     uint8_t interruptPin;
     bool retainedMode;
     uint32_t modified;
     uint8_t controlRegister[XRA1201_REGISTER_COUNT];
     uint8_t opendrain[2];

     void initialize();
     void readAll();
     uint8_t readInputs();
     uint8_t update();
     void touch(uint8_t cr);
     inline void setRegisterBits(uint8_t cr, uint8_t bits) {
       uint8_t reg = this->controlRegister[cr] | bits;
       if (reg != this->controlRegister[cr]) {
         this->controlRegister[cr] = reg;
         touch(cr);
       }
     }
     inline void clearRegisterBits(uint8_t cr, uint8_t bits) {
       uint8_t reg = this->controlRegister[cr] & ~bits;
       if (reg != this->controlRegister[cr]) {
         this->controlRegister[cr] = reg;
         touch(cr);
       }
     }
};


class XRA1201Pin {
  public:
    XRA1201 &iox;
    uint8_t pin;
  XRA1201Pin(XRA1201 &iox, uint8_t pin)  
  : iox(iox), pin(pin)  {}
};

void digitalWrite(XRA1201Pin pin, uint8_t val);
uint8_t digitalRead(XRA1201Pin pin);
void pinMode(XRA1201Pin pin, uint8_t mode);

#endif
