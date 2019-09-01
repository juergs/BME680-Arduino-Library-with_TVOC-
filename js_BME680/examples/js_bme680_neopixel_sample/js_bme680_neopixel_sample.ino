/* ***********************************************************************
    File: js_bme680_sample.ino
    Tested with ESP8266 in  Wemos D1 mini 
    for test of js_BME680-Library functions.
    Makes use of static declared JS_BME680-object (needs no declaration).
    Watch Serial-Monitor @115KB for Output
    Further Infos @ github.com/juergs
**************************************************************************/
#include <Adafruit_NeoPixel.h>
#include <js_bme680.h>

#define STATE_SAVE_PERIOD  UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define SCROLL_DUR  6000 // Milliseconds
#define SCROLL_LOCK_BUTTON  2
#define PIXEL_PIN   13
#define NUMPIXELS   2

// Create a Neopixel object
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRBW + NEO_KHZ400);

unsigned long prevMillis  = 0;

int brightness = 50;
int fadeAmount = 2;
uint32_t lastColorTime = 0;
uint32_t lastOledTime = 0;
volatile bool scroll = true;
volatile bool scrollPrint = false;



void setup() 
{
    // put your setup code here, to run once:
     pixels.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
     pixels.show();            // Turn OFF all pixels ASAP
    pixels.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    
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

      flash_led_iaq(80); //Enable when IAQ starts to work
      
      prevMillis = currentMillis; 
    }
}


void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<pixels.numPixels(); i++) { // For each pixel in strip...
    pixels.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    pixels.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

//--- color Neopixel LEDs for TVOC/ppm indication
void flash_led_iaq(float lum)
{
  float red = 0, green = 0, blue = 0;
  //float iaq = iaqSensor.iaqEstimate, red = 0, green = 0, blue = 0;  
  float iaq = JS_BME680.getTVoc();      
  Serial.print("IAQ-reading: ");Serial.println(iaq);
  int scale = 1;
  if (iaq < 125) { // RGB=(0,255,0)
    red = 0.0;
    green = 255.0;
    blue = 0.0;    
  } 
  else if ((iaq >= 125) && (iaq < 275)) 
  { // RGB=(127-255,255-127,0)
    red = 127.0 + ((iaq - 25.0) * 127.0 / 50.0);
    green = 255.0 - ((iaq - 25.0) * 127.0 / 50.0);
    blue = 0.0;    
  } 
  else if ((iaq >= 275) && (iaq < 475)) 
  { // RGB=(255,127-0,0)
    red = 255.0;
    green = 127.0 - ((iaq - 75.0) * 127.0 / 100.0);
    blue = 0.0;    
  } 
  else if ((iaq >= 475) && (iaq < 600)) 
  { // RGB=(255-127,0,0-127)
    red = 255.0 - ((iaq - 175.0) * 127.0 / 125.0);
    green = 0.0;
    blue = ((iaq - 175.0) * 127.0 / 125.0);    
  } 
  else if (iaq > 600) 
  { // RGB=(127,0,127)
    red = 127.0;
    green = 0.0;
    blue = 127.0;    
  }
  
  red = red * lum / 100;
  green = green * lum / 100;
  blue = blue * lum / 100;
  
  for (int i = 0; i < NUMPIXELS; i++) 
  {  
    pixels.setPixelColor(i, pixels.Color(red, green, blue, 0));
  }
  pixels.show();
}
