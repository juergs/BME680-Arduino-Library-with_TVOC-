/*
    File:   js_bme680.h
    Sensor is using @8266: I2C Pin D1= SCL  D2=SDA 
*/

#ifndef JS_BME680_H
    #define JS_BME680_H

    #include "Arduino.h"
    #include "SPI.h"
    #include "Wire.h"

    //--- include Adafruit lib, which could be found at Github
    #include <Adafruit_BME680.h>      //--- Adafruit BME680 library  

    #define DEBUG_SERIAL    1

    #ifdef DEBUG_SERIAL
    #define DEBUG_BEGIN       Serial.begin(115200)
    #define DEBUG_PRINT(x)    Serial.println(x)
    #define DEBUG_OUT(y)      Serial.print(y)
    #define DEBUG_OUT2(x,y)   Serial.print(x,y) 
    #else
    #define DEBUG_PRINT(x) 
    #define DEBUG_OUT(x)
    #define DEBUG_OUT2(x,y)
    #define DEBUG_BEGIN
    #endif

    //#define ESP8266
    #define BME680_DEBUG  
    #define HAS_BME680MCU       false   //--- serial interfacing
    #define HAS_BME680I2C       true    //--- hard wired i2c interface, not spi 
    #define BME680_SEALEVEL     1015
    /** BME680 I2C addresses */
    #define BME680_I2C_ADDR_PRIMARY		UINT8_C(0x76)   // SDO = GND  pemue-adapter-board
    #define BME680_I2C_ADDR_SECONDARY	UINT8_C(0x77)   // SDO = open or high
    

    /**\name C standard macros */
    #ifndef NULL
        #ifdef __cplusplus
            #define NULL   0
        #else
            #define NULL   ((void *) 0)
        #endif
    #endif  
    

    class JS_BME680Class
    {
        public:
            //--- global                        
            // Constructor
            JS_BME680Class();
            //~JS_BME680Class();

            void            do_begin();

            void            do_bme680_measurement(); 
            
            unsigned long   get_bme680Interval(); 

            void            set_bme680_device_address(uint8_t addr) ;

            bool            isIAQValid(); 
            
            float           getTemp(void);

            float           getPress(void);

            float           getHum(void);

            float           getAlt(void);   

            float           getCalibAlt();

            float           getGasRes(void);

            float           getSeaLevel();

            float           getTVoc(void);      

        private:        
            //--- internal
            Adafruit_BME680  bme680;         //--- internal BME680 sensor object                                

            // --- prototypes 

            void            getBme680Readings() ;
            
            uint32_t        bme680Abc(uint32_t r, float a); 
            
            uint32_t        i2c_scan() ; 
            
            float           absHum(float temp, float hum);
            
            float           dewPoint(float temp, float hum);

            int64_t         get_timestamp_us();

            int64_t         serial_timestamp();                 

    };

    extern JS_BME680Class JS_BME680;


#endif