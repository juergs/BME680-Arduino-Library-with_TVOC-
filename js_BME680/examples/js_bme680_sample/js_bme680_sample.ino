/* ***********************************************************************
    File: js_bme680_sample.ino
    Tested with ESP8266 in  Wemos D1 mini 
    for test of js_BME680-Library functions.
    Makes use of static declared JS_BME680-object (needs no declaration).
    Watch Serial-Monitor @115KB for Output
    Further Infos @ github.com/juergs
**************************************************************************/

#include <js_bme680.h>

unsigned long prevMillis  = 0;  

void setup() 
{
    // put your setup code here, to run once:
    JS_BME680.set_bme680_device_address(0x76);  // may be ommitted in case of default-address 0x76 (SDO = GND), declare in case of address 0x77 (SDO = High)      
      
    JS_BME680.do_begin(); 
}

void loop() 
{
    //--- put your main code here, to run repeatedly:
    static unsigned long baseIntervall = JS_BME680.get_bme680Interval(); 
    unsigned long currentMillis = millis();
 
    //---bme680 data screen added
    if (currentMillis - prevMillis > baseIntervall) 
    {   
      // --- ca. alle 10 Sekunden eine Messung aller Messgroessen      
      JS_BME680.do_bme680_measurement();
      prevMillis = currentMillis; 
    }
}