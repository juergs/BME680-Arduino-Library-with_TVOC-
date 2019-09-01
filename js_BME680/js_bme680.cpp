/* ----------------------------------
    File: js_bme680.c
----------------------------------*/

#include "js_bme680.h"

  uint8_t         _I2C_BME680_ADDRESS   = BME680_I2C_ADDR_PRIMARY;  // 0x76 should be set as default address 

  float       	  temperature = 0.0;    
	float  			    humidity    = 0.0;    
	float 			    pressure 	  = 0.0;
	float      		  tvoc     	  = 0.0; 

	unsigned long   prevBme680Millis  = millis();           	// counter main loop for BME 680
	unsigned long   intervalBme680    = 10000;              	// 10 sec update interval default
	float           resFiltered;                            	// low pass
	float           aF                = 0;
	float           tVoc              = 0;
	bool            bme680VocValid    = false;              	// true if filter is initialized, ramp-up after start
	char            bme680Msg[128];                         	// payload
  bool            _isValidIaq       = false; 
	
	//--- automatic baseline correction
	uint32_t        bme680_baseC        = 0;                	// highest adjusted r (lowest voc) in current time period
	float           bme680_baseH        = 0;                	// abs hum (g/m3)
	unsigned long   prevBme680AbcMillis = 0;      			      // ts of last save to nv 
	unsigned long   intervalBme680NV    = 604800000; 			    // 7 days of ms
	uint8_t         bDelay              = 0;  
  uint16_t        pressureSeaLevel    = 1013.25;            //default value of 1013.25 hPa  

	struct 
	{ 
		float         t_offset  = -.5;                 // offset temperature sensor
		float         h_offset  = 1.5;                 // offset humitidy sensor	
		uint32_t      vocBaseR  = 0;                   // base value for VOC resistance clean air, abc 
		uint32_t      vocBaseC  = 0;                   // base value for VOC resistance clean air, abc  
		float         vocHum    = 0;                   // reserved, abc
		uint32_t      signature = 0x49415143;          // 'IAQC'
	} preload, param;     							        //--- stable new baseline counter (avoid short-term noise)    

	typedef struct 
	{ 
		String  temperature   = "";
    float   fTemp         = 0.0; 
		String  humidity      = "";
    float   fHum          = 0.0; 
		String  abshum        = "";
    float   fAbsHum       = 0.0;
		String  pressure      = "";
    float   fPress        = 0.0;
		String  tvoc          = "";    
    float   fTvoc         = 0.0;
		String  dewpoint      = ""; 
    float   fDew          = 0.0;
		String  gas           = "";
    float   fGas          = 0.0;
		String  altitude      = "";      
    float   fAlt          = 0.0;
    String  altitudeCalib = "";      
    float   fAltCalib     = 0.0;
    float   fSeaLevel     = 0.0; 
	} GDATA_TYP; 
	
	GDATA_TYP gdata;   

//----------------------------------------------------------------------
void JS_BME680Class::set_bme680_offset_temp(float toffset) 
{
  param.t_offset = toffset; 
}
//----------------------------------------------------------------------
void JS_BME680Class::set_bme680_offset_hum(float hoffset) 
{
  param.h_offset = hoffset; 
}
//----------------------------------------------------------------------
void JS_BME680Class::set_bme680_device_address(uint8_t addr) 
{
  _I2C_BME680_ADDRESS = addr; 
}
//----------------------------------------------------------------------
JS_BME680Class::JS_BME680Class()
{
    do_begin(); 
}
//----------------------------------------------------------------------
bool   JS_BME680Class::isIAQValid()
{
 return(_isValidIaq); 
}
//----------------------------------------------------------------------	
float JS_BME680Class::getTemp(void)
{
  return (gdata.fTemp) ; 
}
//----------------------------------------------------------------------      
float JS_BME680Class::getPress(void)
{
  return (gdata.fPress) ; 
}
//----------------------------------------------------------------------      
float JS_BME680Class::getHum(void)
{
  return (gdata.fHum) ; 
}
//----------------------------------------------------------------------
float JS_BME680Class::getAlt(void)
{
  //return (gdata.fAlt) ; 
  return (1.0 - pow((float)gdata.fPress / 100.0f / BME680_SEALEVEL, 0.190284)) * 287.15 / 0.0065;
}
//----------------------------------------------------------------------
float JS_BME680Class::getCalibAlt()
{
  //return (gdata.fAltCalib) ; 
  return (1.0 - pow((float) gdata.fPress / BME680_SEALEVEL, 0.190284)) * 287.15 / 0.0065;
}
//----------------------------------------------------------------------
float JS_BME680Class::getGasRes(void)
{
  return (gdata.fGas) ;   
}
//----------------------------------------------------------------------
float JS_BME680Class::getSeaLevel()
{
  return ((float) BME680_SEALEVEL) ; 
}
//----------------------------------------------------------------------
float 	JS_BME680Class::getTVoc(void)
{
  return (gdata.fTvoc) ; 
}
//----------------------------------------------------------------------
unsigned long JS_BME680Class::get_bme680Interval()
{
  return (intervalBme680); 
}
//----------------------------------------------------------------------
float JS_BME680Class::absHum(float temp, float hum)
{
    double sdd, dd;
    sdd=6.1078 * pow(10,(7.5*temp)/(237.3+temp));
    dd=hum/100.0*sdd;
    return (float)216.687*dd/(273.15+temp);
}
//----------------------------------------------------------------------
float JS_BME680Class::dewPoint(float temp, float hum)
{
    double A = (hum/100) * pow(10, (7.5*temp / (237+temp)));
    return (float) 237*log10(A)/(7.5-log10(A));
}
//----------------------------------------------------------------------
int64_t JS_BME680Class::get_timestamp_us()
{
    return (int64_t)millis() * 1000;
}
//----------------------------------------------------------------------
int64_t JS_BME680Class::serial_timestamp()
{
    //--- a jumper to GND on D10 does the job, when needed! 
    /* if (digitalRead(PIN_ENABLE_TIMESTAMP_IN_OUTPUT) == LOW){    */
        DEBUG_OUT("["); DEBUG_OUT(get_timestamp_us() / 1e6); DEBUG_OUT("] ");
        return (get_timestamp_us() / 1e6); 
    //}
}
//----------------------------------------------------------------------
void JS_BME680Class::do_bme680_measurement()
{
   //serial_timestamp(); DEBUG_PRINT(F(" BME680-I2C: call do_bme680_measurement"));
   getBme680Readings() ;  //--- with handling of prevBme680Millis
}
//----------------------------------------------------------------------
  void JS_BME680Class::getBme680Readings() 
  {
    prevBme680Millis  = millis(); // counter main loop for BME 680
    
    if (! bme680.performReading()) 
    {
      DEBUG_PRINT(F("BME680-I2C: Failed perform reading :("));
      return;
    };

    //DEBUG_OUT( get_timestamp_us() ); 
    //DEBUG_OUT("  "); 
    serial_timestamp(); 
    DEBUG_OUT("  ");
    DEBUG_OUT(F("Temperature = "));
    DEBUG_OUT(bme680.temperature);
    DEBUG_OUT(F(" *C  "));

    DEBUG_OUT(F("Pressure = "));
    DEBUG_OUT(bme680.pressure / 100.0);
    DEBUG_OUT(" hPa  ");
    
    DEBUG_OUT(F("Humidity = "));
    DEBUG_OUT(bme680.humidity);
    DEBUG_OUT(F(" %  "));
    
    DEBUG_OUT(F("Gas = "));
    DEBUG_OUT(bme680.gas_resistance / 1000.0);
    DEBUG_OUT(F(" KOhms  "));

    #define SEALEVELPRESSURE_HPA (1013.25)
    DEBUG_OUT(F("Approx. Altitude = "));
    DEBUG_OUT(bme680.readAltitude(SEALEVELPRESSURE_HPA));
    DEBUG_OUT(F(" m  "));
    
    //DEBUG_PRINT();

    char str_temp[6];
    char str_hum[6];
    char str_absHum[6];
    char str_dewPoint[6];
    char str_pressure[16];
    //char str_altitude[8];
    char str_tVoc[8];
    char str_gas[8];

    float     t = bme680.temperature + param.t_offset;
    float     h = bme680.humidity + param.h_offset;
    float     a = absHum(t, h);
    aF = (aF == 0 || a < aF)?a:aF + 0.2 * (a - aF);
    float     d = dewPoint(t, h);
    float     p = bme680.pressure /100.0F;
    uint32_t  r = bme680.gas_resistance; // raw R VOC

    if (!bme680VocValid )
    {

      _isValidIaq = false; 

      //--- get first readings without tvoc              
      dtostrf(t, 4, 2, str_temp);
      dtostrf(h, 4, 2, str_hum);
      dtostrf(a, 4, 2, str_absHum);
      dtostrf(d, 4, 2, str_dewPoint);
      dtostrf(p, 3, 1, str_pressure);
      dtostrf(r, 3, 1, str_gas);   

      gdata.fTemp   = t; 
      gdata.fHum    = h; 
      gdata.fPress  = p; 
      gdata.fTvoc   = 0.0 ; 
      gdata.fGas    = r;  

      gdata.temperature = str_temp;
      gdata.humidity    = str_hum;
      gdata.abshum      = str_absHum;
      gdata.dewpoint    = str_dewPoint;
      gdata.pressure    = str_pressure;
      gdata.gas         = str_gas;      
      gdata.tvoc        = "0";            //--- not enough data yet
      gdata.altitude    = "-.-"; 
    }

    if (r == 0) return;      //--- first reading !=0 accepted, 0 is invalid
    
    uint32_t base = bme680Abc(r, a);       // update base resistance 
    
    if (!bme680VocValid && (millis() > 300000)) 
    { 
      
      //--- allow 300 sec warm-up for stable voc resistance (300000ms)
      resFiltered = r;        // preload low pass filter
      bme680VocValid = true;           
    }
    else
    {
         _isValidIaq = false;
    }
    

    DEBUG_PRINT(""); 
    
    if (!bme680VocValid ) return;
    
    //--- ab hier gehts los mit tVoc-Ausgaben nach ca. 30 Sekunden.  
    _isValidIaq = true;   

    resFiltered += 0.1 * (r - resFiltered);
    
    //float ratio = (float)base / (resFiltered * a * 7.0F);   // filter removed
    float ratio = (float)base / (r * aF * 7.0F);
    float tV = (1250 * log(ratio)) + 125;                     // approximation
    tVoc = (tVoc == 0)?tV:tVoc + 0.1 * (tV - tVoc);
        
    dtostrf(t, 4, 2, str_temp);
    dtostrf(h, 4, 2, str_hum);
    dtostrf(a, 4, 2, str_absHum);
    dtostrf(d, 4, 2, str_dewPoint);
    dtostrf(p, 3, 1, str_pressure);
    dtostrf(r, 3, 1, str_gas);
    dtostrf(tVoc, 1, 0, str_tVoc); 
      
    String str_press_val = String(p,2); 
    
    gdata.temperature = str_temp;
    gdata.humidity    = str_hum;
    gdata.abshum      = str_absHum;
    gdata.dewpoint    = str_dewPoint;
    gdata.pressure    = str_press_val;
    gdata.gas         = str_gas;
    gdata.tvoc        = String(tVoc, 0); 
    gdata.altitude    = "0"; 

    gdata.fTvoc = tVoc; 
    
    serial_timestamp(); 
    DEBUG_OUT (F("Taupunkt: "));  DEBUG_OUT (str_dewPoint); DEBUG_OUT ("  ");
    DEBUG_OUT (F("tVOC: ")); DEBUG_PRINT (str_tVoc);

    //--- DEBUG
    char str_filtered[16];
    dtostrf(resFiltered, 4, 3, str_filtered);
    char str_ratio[16];
    dtostrf(ratio, 4, 4, str_ratio);
    
    /* prepare to send UDP-message to fhem
    snprintf(bme680Msg
           , sizeof(bme680Msg)
           , "F:THPV;T:%s;H:%s;AH:%s;D:%s;P:%s;V:%s;R:%lu;DB:%lu;DF:%s;DR:%s;"
           , str_temp
           , str_hum
           , str_absHum
           , str_dewPoint
           , str_pressure
           , str_tVoc
           , r
           , base
           , str_filtered
           , str_ratio);
           
    DEBUG_PRINT(bme680Msg);    
    */
    DEBUG_PRINT(F("BME680-Reading done."));
  };  
//----------------------------------------------------------------------
  uint32_t JS_BME680Class::bme680Abc(uint32_t r, float a) 
  {   
    //--- automatic baseline correction
    uint32_t b = r * a * 7.0F;
    if (b > bme680_baseC && bDelay > 5) 
    {     
      // ensure that new baseC is stable for at least >5*10sec (clean air)
      bme680_baseC = b;
      bme680_baseH = a;
    } else if (b > bme680_baseC) 
    {
      bDelay++;
      //return b;
    } else 
    {
      bDelay = 0;
    };

    //--- store baseline to nv
    unsigned long currentMillis = millis();
    if (currentMillis - prevBme680AbcMillis > intervalBme680NV) 
    {
      //    prevBme680AbcMillis = currentMillis;    
      //    //---store baseline
      //    param.vocBase = bme680CurrentHigh;
      //    bme680CurrentHigh = 0;
    };
    return (param.vocBaseC > bme680_baseC)?param.vocBaseC:bme680_baseC;
  };
//----------------------------------------------------------------
uint32_t JS_BME680Class::i2c_scan() 
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  DEBUG_PRINT(F("Scanning I2C..."));

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      DEBUG_OUT(F("I2C device found at address 0x"));
      if (address<16) 
          DEBUG_OUT("0");
      DEBUG_OUT2(address,HEX);
      DEBUG_PRINT("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      DEBUG_OUT(F("Unknown error at address 0x"));
      if (address<16) 
        DEBUG_OUT("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  {
    DEBUG_PRINT("No I2C devices found\n");
    return(255);
  }
  else
  {
    DEBUG_PRINT("done\n");
    return(0);
   }
}
//----------------------------------------------------------------
void JS_BME680Class::do_begin()
{
    DEBUG_BEGIN;    //Serial.begin(115200);  
    
    delay(3000);   // wait debug console to settle 

    DEBUG_PRINT(""); 

    DEBUG_PRINT(F("*** Started!"));

/*  
    //--- enable I2C for ESP8266 NodeMCU boards [VDD to 3V3, GND to GND, SDA to D2, SCL to D1]
    Wire.begin(SDA, SCL);
    Wire.setClockStretchLimit(1000); // Default is 230us, see line78 of https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_si2c.c

    //--- nicht für STM32
    #ifdef ESP8266
      Wire.setClock(400000);
    #endif 
*/
    //--- set I2C-Clock
    #ifdef ESP8266
      Wire.setClockStretchLimit(1000); //--- Default is 230us, see line78 of https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_si2c.c
      Wire.setClock(400000);
      //--- (SDA,SCL) D1, D2 enable I2C for Wemos D1 mini SDA:D2 / SCL:D1 
      Wire.begin(D2, D1);
      DEBUG_PRINT(F("Enabled: I2C for Wemos D1 mini SDA:D2 / SCL:D1 "));
    #else
      Wire.begin();
    #endif 

   //--- (SDA,SCL) D1, D2
   //--- enable I2C for Wemos D1 mini SDA:D2 / SCL:D1 
   //Wire.begin(D2, D1);

   i2c_scan(); 
   

   // #define BME680_DEFAULT_ADDRESS  (0x77)     ///< The default I2C address
   // #define BME680_SECONDARY_ADDRESS (0x76) 

   // if (!bme680.begin()) //--- no use of Adafruits default address, which is also 0x76 
   if (!bme680.begin(_I2C_BME680_ADDRESS)) 
    {
        DEBUG_PRINT(F("I2C-Error: Could not find a valid BME680 sensor, check wiring!"));
        delay(3000);
        while (1);
    }     
    else
    {
        DEBUG_PRINT(F("I2C: ok BME680 sensor found! :-) "));
    }

    //--- set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);    // 320*C for 150 ms

    DEBUG_PRINT(F("BME680 sensor initialized."));

}


//---------------------------------
//---  declare the static instance 
JS_BME680Class JS_BME680;
