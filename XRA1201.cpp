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

/*
Description of the XRA1201 device © 2016 Exar Corporation: 

The XRA1201/1201P is a 16-bit GPIO expander with an I2C/SMBus interface. 
After power-up, the XRA1201 has internal 100K ohm pull-up resistors on 
each I/O pin that can be individually enabled. The XRA1201P has the internal 
pull-up resistors enabled upon power-up in case it is necessary for the 
inputs to be in a known state.
In addition, the GPIOs on the XRA1201/1201P can individually be controlled 
and configured. As outputs, the GPIOs can be outputs that are high, low or 
in three-state mode. The three-state mode feature is useful for applications 
where the power is removed from the remote devices, but they may still be 
connected to the GPIO expander.
As inputs, the internal pull-up resistors can be enabled or disabled and the 
input polarity can be inverted. The interrupt can be programmed for different 
behaviors. The interrupts can be programmed to generate an interrupt on the 
rising edge, falling edge or on both edges. The interrupt can be cleared if 
the input changes back to its original state or by reading the current state 
of the inputs.
The XRA1201/1201P are enhanced versions of other 16-bit GPIO expanders with 
an I2C/SMBus interface. The XRA1201 is pin and software compatible with the 
PCA9535, TCA9535 and MAX7312. The XRA1201P is pin and software compatible 
with the CAT9555, PCA9555, TCA9555, MAX7311 and MAX7318.

https://www.exar.com/product/interface/bridges/bridging-solutions/i2c-gpio-expanders/xra1201

*/

/*

Library description:

This library supports the main functionality of the XRA1201 (with the 
exception of the Polarity Inversion Register).

The I/O pins are used "Arduino style" with very similar functions.

  XRA1201 iox;
  iox.begin();
  iox.pinMode(pin2, OUTPUT);
  val = iox.digitalRead(pin);
  iox.digitalWrite(pin2, val);

Setting pin modes and outputs can either be done synchonously (default) or 
in a "retained" mode, where changes for multiple pins are buffered and then 
efficiently sent to the device using one I2C transmission. 

  iox.beginTransaction();
  iox.pinMode(pin2, OUTPUT);
  iox.digitalWrite(pin2, LOW);
  iox.pinMode(pin3, OUTPUT_OPENDRAIN);
  iox.digitalWrite(pin3, HIGH);
  iox.endTransaction();

Reading pins is currently done using single synchronous read through I2C. 

There is also a "pin abstraction", a small struct acting as a "pin number" 
(normally just an int) that can syntactically be used like a normal pin: 

    XRA1201 Meteo::IOxMeteo(0x22);
    
    #define PIN_METEO_WIND_PROBE    XRA1201Pin(IOxMeteo, 0)
    #define PIN_METEO_WIND_SPEED    XRA1201Pin(IOxMeteo, 1)
    #define PIN_METEO_RAIN          XRA1201Pin(IOxMeteo, 2)

    ...
    pinMode(PIN_METEO_WIND_PROBE, OUTPUT);
    digitalWrite(PIN_METEO_WIND_PROBE, LOW);
    pinMode(PIN_METEO_WIND_SPEED, INPUT_PULLUP);
    pinMode(PIN_METEO_RAIN, INPUT_PULLUP);

Of course these pins can only be used in your own code, as they are not simple ints that
can be passed to third party libraries etc.

Also be aware that these expander GPIOs are much slower than built-in ones!

*/

#include "XRA1201.h"

#define XRA1201_CR_INPUT          0x00
#define XRA1201_CR_OUTPUT         0x02
#define XRA1201_CR_POLARITY       0x04
#define XRA1201_CR_CONFIG         0x06
#define XRA1201_CR_PULLUP         0x08
#define XRA1201_CR_INT_EN         0x0A
#define XRA1201_CR_3STATE         0x0C
#define XRA1201_CR_INT_STATUS     0x0E
#define XRA1201_CR_INT_RAISING    0x10
#define XRA1201_CR_INT_FALLING    0x12
#define XRA1201_CR_FILTER         0x14

#define XRA1201_PORT_A            0
#define XRA1201_PORT_B            1

#define XRA1201_PIN_PORT(pin)     (((pin)>>3)&0x1)
#define XRA1201_PIN_BIT(pin)      (1<<((pin)&0x7))

// Description
//   Construct a device using the given I2C address and bus instance.
//
// Syntax
//   XRA1201 iox()
//   XRA1201 iox(address)
//   XRA1201 iox(address, wire)
//
// Parameters
//   address : the address of the I2C device, either 0x20 (default) or 0x21 (see ADDR pin of XRA1201)
//   wire : the I2C bus instance, if you have multiple (defaults to the Wire global)
// 
XRA1201::XRA1201(uint8_t address, TwoWire &wire)
  : wire(wire), address(address), 
  interruptPin(0), retainedMode(0), modified(0) {
  this->initialize();
}

void  XRA1201::initialize() {
  // control register power on reset defaults 
  this->controlRegister[XRA1201_CR_INPUT + XRA1201_PORT_A] = 0;    // actually unknown
  this->controlRegister[XRA1201_CR_INPUT + XRA1201_PORT_B] = 0;    // actually unknown
  this->controlRegister[XRA1201_CR_OUTPUT + XRA1201_PORT_A] = 0xFF;
  this->controlRegister[XRA1201_CR_OUTPUT + XRA1201_PORT_B] = 0xFF;
  this->controlRegister[XRA1201_CR_POLARITY + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_POLARITY + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_CONFIG + XRA1201_PORT_A] = 0xFF;
  this->controlRegister[XRA1201_CR_CONFIG + XRA1201_PORT_B] = 0xFF;
  this->controlRegister[XRA1201_CR_PULLUP + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_PULLUP + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_INT_EN + XRA1201_PORT_A] = this->interruptPin ? 0xFF : 0x00; 
  this->controlRegister[XRA1201_CR_INT_EN + XRA1201_PORT_B] = this->interruptPin ? 0xFF : 0x00;
  this->controlRegister[XRA1201_CR_3STATE + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_3STATE + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_INT_STATUS + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_INT_STATUS + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_INT_RAISING + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_INT_RAISING + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_INT_FALLING + XRA1201_PORT_A] = 0x00;
  this->controlRegister[XRA1201_CR_INT_FALLING + XRA1201_PORT_B] = 0x00;
  this->controlRegister[XRA1201_CR_FILTER + XRA1201_PORT_A] = 0xFF;
  this->controlRegister[XRA1201_CR_FILTER + XRA1201_PORT_B] = 0xFF;
  this->opendrain[0] = 0;
  this->opendrain[1] = 0;
  // mark all as modified (except read only registers INPUT and INT_STATUS)
  this->modified = ((1 << XRA1201_REGISTER_COUNT) - 1) & ~((0x03 << XRA1201_CR_INPUT) | (0x03 << XRA1201_CR_INT_STATUS));
}
 

// Description 
//   Initialization. This assumes the device is in a unknown state.
//
// Syntax
//   iox.begin()
//   
// Parameters
//   None
// 
// Returns
//   None
// 
void XRA1201::begin(bool initialize) {
  // NOTE: we do this as per Arduino convention. It yould be better to do this 
  // once only on the Wire object.
  this->wire.begin();
  if (initialize) {
    this->initialize();
    // make sure the chip reflects the internal settings allright
    this->writeSettings();
  }
  else {
    // TDOD: discover the current settings
    //this->readAll();
  }

}

// Description
//   Get the I2C address back.
//
// Syntax
//   iox.getAddress()
//
// Parameters
//   None
// 
// Returns
//   None
// 
uint8_t XRA1201::getAddress() {
  return this->address;   
}

// Description 
//   Set the IO expander interrupt pin or 0 if none. This interrupt pin on the Micro Controller Unit 
//   (MCU) must be connected to the !INT pin on the XRA1201. It will signal changes on the IO 
//   expander's input pins, so the MCU knows when it has to read the changed input pins through the 
//   I2C bus. 
//   Otherwise (if the interupt pin is set to 0) the MCU must read the input pins each time a
//   digitalRead() is requested. This can be slow and/or congest the I2C bus and/or waste precious 
//   battery power. 
//
// Syntax
//   setInterrupt(pin)
//
// Parameters
//    pin : the interrupt pin 
// 
// Returns
//    None
// 
void XRA1201::setInterrupt(uint8_t pin) {
  this->interruptPin = pin;
  // TODO:  attachInterrupt(pin, ); for real interrupt handling
  if (pin) { 
    this->setRegisterBits(XRA1201_CR_INT_EN + XRA1201_PORT_A, 0xFF);
    this->setRegisterBits(XRA1201_CR_INT_EN + XRA1201_PORT_B, 0xFF);
  }
  else {
    this->clearRegisterBits(XRA1201_CR_INT_EN + XRA1201_PORT_A, 0xFF);
    this->clearRegisterBits(XRA1201_CR_INT_EN + XRA1201_PORT_B, 0xFF);
  } 
}

// Description
//   Begins a new transaction where multiple digitalWrite() and pinMode() calls are buffered
//   and only sent to the XRA1201 when you call endTransaction().
//   This may prevent delays between pins, prevent glitches and save a lot of I2C bus bandwidth, 
//   time and perhaps battery power.
//
// Syntax
//   iox.beginTransaction()
//
// Parameters
//   None
//
// Returns
//   None
// 
void XRA1201::beginTransaction() {
  this->retainedMode = true;
}

// Description
//   Ends a transaction, see beginTransaction()
//
// Syntax
//   iox.endTransaction()
//
// Parameters
//   None
//
// Returns
//   Wire write return code.
// 
uint8_t XRA1201::endTransaction() {
  uint8_t ret = this->writeSettings();
  this->retainedMode = false;
  return ret;
}

// Description
//   Write a HIGH or a LOW value to a digital pin.
//   If the pin has been configured as an OUTPUT with pinMode(), its voltage will be set to the 
//   corresponding value : The VCCP voltage for HIGH, 0V (ground) for LOW.
//   If the pin is configured as an INPUT, digitalWrite() will enable (HIGH) or disable (LOW) 
//   the internal pullup on the input pin. It is recommended to set the pinMode() to 
//   INPUT_PULLUP to enable the internal pull-up resistor.   
//
// Syntax
//   digitalWrite(pin, value)
//
// Parameters
//   pin : the pin number
//   value : HIGH or LOW
//
// Returns
//   None
//
// See also: https://www.arduino.cc/en/Reference/DigitalWrite
//
void XRA1201::digitalWrite(uint8_t pin, uint8_t val) {
  uint8_t port = XRA1201_PIN_PORT(pin);
  uint8_t crConfig = XRA1201_CR_CONFIG + port;
  uint8_t pinBits = XRA1201_PIN_BIT(pin);
  uint8_t cr;
  // TODO: carry over pin state on mode change to mimick Arduino such as the pullup to the output an change from input to output mode

  if (this->controlRegister[crConfig] & pinBits) {
    // INPUT mode - digitalWrite actually sets the pullup
    cr = XRA1201_CR_PULLUP + port;
  }
  else if (this->opendrain[port] & pinBits) {
    // OUTPUT_OPENDRAIN mode - digitalWrite actually sets the 3state
    cr = XRA1201_CR_3STATE + port;
  }
  else {
    // normal OUPUT mode - digitalWrite sets the output pin
    cr = XRA1201_CR_OUTPUT + port;
  }
  if (val) {
    this->setRegisterBits(cr, pinBits);
  }
  else  {
    this->clearRegisterBits(cr, pinBits);
  }
}

// Description
//   Reads the value from a specified digital pin, either HIGH or LOW.
//
// Syntax
//   digitalRead(pin)
//
// Parameters
//   pin : the number of the digital pin you want to read(int)
//
// Returns
//   HIGH or LOW
//
// Se also: https://www.arduino.cc/en/Reference/DigitalRead
//
uint8_t XRA1201::digitalRead(uint8_t pin) {
  uint8_t bit = XRA1201_PIN_BIT(pin);
  uint8_t cri = XRA1201_CR_INT_EN + XRA1201_PIN_PORT(pin);
  uint8_t cr = XRA1201_CR_INPUT + XRA1201_PIN_PORT(pin);
  // TODO: read when interrupt actually occurs
  if ((this->interruptPin == 0)
    || !(this->controlRegister[cri] & bit))  { 
    // conventional read
    this->readInputs();
  }
  else if (::digitalRead(this->interruptPin) == LOW) { 
    this->readInputs();
    //Serial.print(" [interrupt IOx] ");
  }
  
  // TODO: consider mode and read other registers i.e. output register when in mode OUTPUT
  return (this->controlRegister[cr] & bit) ? HIGH : LOW;
}



// Description
//   Configures the specified pin to behave either as an input or an output.
//
// Syntax
//   pinMode(pin, mode)
//
// Parameters
//   pin : the number of the pin whose mode you wish to set
//   mode : INPUT, INPUT_PULLUP, OUTPUT
// 
// Returns
//    None
//
// See https://www.arduino.cc/en/Reference/PinMode
//
void XRA1201::pinMode(uint8_t pin, uint8_t mode) {
  uint8_t port = XRA1201_PIN_PORT(pin);
  uint8_t crConfig = XRA1201_CR_CONFIG + port;
  uint8_t crPullup = XRA1201_CR_PULLUP + port;
  uint8_t cr3State = XRA1201_CR_3STATE + port;
  uint8_t pinBits = XRA1201_PIN_BIT(pin);
  
  if (mode == INPUT) {
    this->setRegisterBits(crConfig, pinBits);
    this->clearRegisterBits(crPullup, pinBits);
    this->clearRegisterBits(cr3State, pinBits);
    this->opendrain[port] &= ~pinBits;
  }
  else if (mode == INPUT_PULLUP) {
    this->setRegisterBits(crConfig, pinBits);
    this->setRegisterBits(crPullup, pinBits);
    this->clearRegisterBits(cr3State, pinBits);
    this->opendrain[port] &= ~pinBits;
  }
  else if (mode == OUTPUT) {
    this->clearRegisterBits(crConfig, pinBits);
    this->clearRegisterBits(crPullup, pinBits);
    this->clearRegisterBits(cr3State, pinBits);
    this->opendrain[port] &= ~pinBits;
  }
  else if (mode == OUTPUT_OPENDRAIN) {
    this->clearRegisterBits(crConfig, pinBits);
    this->clearRegisterBits(crPullup, pinBits);
    if (! (this->opendrain[port] & pinBits)) { 
      // opendrain newly set
      this->opendrain[port] |= pinBits;
      // opendrain is open by default 
      // NOTE this is different from the other modes where setting the mode does not change the state
      this->setRegisterBits(cr3State, pinBits);
    }
    // set output value to LOW so when the 3state is cleared, it will drain 
    uint8_t crOutput = XRA1201_CR_OUTPUT + port;
    this->clearRegisterBits(crOutput, pinBits);
  }
  // else TODO: error diagnostics
}

// Description
//   Writes modified configurations registers to the XRA1201 device.
//
// Syntax
//   writeSettings()
//
// Parameters
//   None
// 
// Returns
//   return code of Wire.endTransmission()
//
uint8_t XRA1201::writeSettings() {
  if (this->modified) { 
    uint8_t ret = 0;

    // TODO: order 3state before output (and check for other dependencies)

    for (int cr = 0; cr < XRA1201_REGISTER_COUNT; cr++) {
      if (this->modified & (1<<cr)) {
        // register is modified
        this->wire.beginTransmission(this->address);
        this->wire.write(cr);
        this->wire.write(this->controlRegister[cr]);
        if ((!(cr&1)) && (this->modified & (1 << (cr+1)))) {
          // first port register (even number) and next one is modified too
          // -> write second port too
          cr++;
          this->wire.write(this->controlRegister[cr]);
        }
        ret = this->wire.endTransmission();
      }
    }     
    if (this->modified & (0x3<<XRA1201_CR_CONFIG)) {
      // configuration was changed - make sure the new inputs are read
      this->readInputs();
    }
    if (ret == 0) {
      // reset modification 
      this->modified = 0;
    }

    return ret;
  }
  else { 
    return 0;
  }
}


// Internal --------------------------------------------------------------------------------------

// Internal helper to read the input pin registers from the XRA1201
uint8_t XRA1201::readInputs() {

  // send command byte to address the input control register
  this->wire.beginTransmission(this->address);
  this->wire.write(XRA1201_CR_INPUT);
  uint8_t ret = this->wire.endTransmission();
  if (ret != 0) {
    Serial.print("XRA1201::readInputs() i2c write failure, ret=");
    Serial.println(ret);
    return ret;
  }

  // read registers
  //??this->wire.setTimeout(100);
  ret = this->wire.requestFrom(this->address, (uint8_t)2);
  if (ret != 2) {
    Serial.print("XRA1201::readInputs() i2c read failure, ret=");
    Serial.println(ret);
    return 10+ret;
  }
  // always read both registers
  this->controlRegister[XRA1201_CR_INPUT + XRA1201_PORT_A] = this->wire.read();
  this->controlRegister[XRA1201_CR_INPUT + XRA1201_PORT_B] = this->wire.read();
  return ret;
}

// Internal helper to mark one configuration register as modified.
void XRA1201::touch(uint8_t cr){
  this->modified |= (1<<cr);
  this->update();
}

// Internal helper to update the XRA1201 configurations registers. Does nothing in retained mode.
uint8_t XRA1201::update() {
  if (! this->retainedMode) {
    return this->writeSettings();
  }
  return 0;
}


// Overrides of Arduino static functions with pin abstraction

void digitalWrite(XRA1201Pin pin, uint8_t val) {
  pin.iox.digitalWrite(pin.pin, val);
}

uint8_t digitalRead(XRA1201Pin pin) {
  return pin.iox.digitalRead(pin.pin);
}
void pinMode(XRA1201Pin pin, uint8_t mode){
  pin.iox.pinMode(pin.pin, mode);
}

