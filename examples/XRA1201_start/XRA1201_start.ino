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

This is only a starting point for your sketch as it is hardware dependant. 

It is untested because I don't have a breadboard hardware setup available 
(personally I use the library in a complex custom PCB circuit).

Please contribute a working breadoard example back, if you have one. :-)

*/

// include the library
#include <XRA1201.h>

// create the IO expander object, setting the I2C address which in turn is 
// programmable on the IC using pins (so you can use multiple).
XRA1201 IOx(0x22);

// define pin abstraction "pin numbers" to use the GPIOs like a built-in pin.
#define PIN_EXAMPLE_1           XRA1201Pin(IOx, 0)
#define PIN_EXAMPLE_2           XRA1201Pin(IOx, 1)
#define PIN_EXAMPLE_3           XRA1201Pin(IOx, 3)


void setup() {
  
  // initialize the IO expander
  IOx.begin();
  // demonstrate retained mode where multiple GPIO setup changes 
  // are sent as consolidated register changes to the expander.
  IOx.beginTransaction();
  pinMode(PIN_EXAMPLE_1, OUTPUT);
  digitalWrite(PIN_EXAMPLE_1, LOW);
  pinMode(PIN_EXAMPLE_2, INPUT_PULLUP);
  pinMode(PIN_EXAMPLE_3, OUTPUT_OPENDRAIN);
  digitalWrite(PIN_EXAMPLE_1, HIGH);
  IOx.endTransaction();

  // you can also use the IOx object if you prefer OOP style
  IOx.pinMode(4, OUTPUT);
  IOx.digitalWrite(4, HIGH);
}

void loop() {
  // Blink with IO expander
  digitalWrite(PIN_EXAMPLE_1, HIGH);
  delay(1000);
  digitalWrite(PIN_EXAMPLE_1, LOW);
  delay(1000);
}
