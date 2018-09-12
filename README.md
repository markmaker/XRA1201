# XRA1201
Arduino Library for the XRA1201 IO-Expander

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
