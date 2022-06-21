// Wemos d1 mini
// Used: D1-8, A0
// free: 9-11

// RTC connections:
// DS1307 SDA --> SDA (D2)
// DS1307 SCL --> SCL (D1)
// DS1307 VCC --> 5v

#include "RTClib.h"     // Adafruit
RTC_DS1307 rtc;

// SD connections:
// * MOSI    - (D7)
// * MISO    - (D6)
// * CLK/SCK - (D5)
// * CS/SS   - (D8)
// * VCC     - 3.3v
const int chipSelect = D8;  // used for ESP8266
#include <SPI.h>
#include <SD.h>
File myFile;
String filename;

// Voltage sensor (A0)
// number of analog samples to take per reading
#define NUM_SAMPLES 10
int sum_voltage = 0;             // sum of samples taken
unsigned char sample_count = 0;  // current sample number
float adc_voltage = 0.0;  // Floats for ADC voltage & Input voltage
float in_voltage = 0.0;
float R1 = 210000.0;      // Floats for onboard wemos d1 mini resistor values in divider (in ohms)
float R2 =  99000.0; 
float R1_ext = 30000.0;   // Floats for external shield resistor values in divider (in ohms)
float R2_ext =  7500.0; 

// Temperature DHT22 
// DHT22 connection - (D3)
#include <SimpleDHT.h>    // from arduino libraries
#define DHTPIN D3         // Pin which is connected to the DHT sensor.
SimpleDHT22 dht(DHTPIN);
float humi,temp;

// Timers and led (LED - D4)
uint32_t timerLED, timerRTC;
bool LEDmode;

//**********************************************************************************

//**********************************************************************************
void setup ()
{
  // initialize digital pin LED_BUILTIN as an output (D4)
  pinMode(LED_BUILTIN, OUTPUT);
  LEDmode=0;
  digitalWrite(LED_BUILTIN, LEDmode);
  
  // Serial port
  Serial.begin(9600);
  delay(1000);  // wait for pc can connect to serial after scetch upload;
  Serial.println("SimpleDataLoggerWemos.");

  // SD init start.
  DateTime compiled = DateTime(__DATE__, __TIME__);
  filename=String(compiled.year()) + "_" + String(compiled.month()) + "_" + String(compiled.day()) + ".csv";

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    LEDsos();
  }
  else {
  Serial.println("initialization done.");
  }
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(filename, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to " + filename );
    myFile.println("Starting.");
    // close the file:
    myFile.close();
    Serial.println(" done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening " + filename );
    LEDsos();
  }
  // SD init end
  
  // RTC init start.
  if (!rtc.begin()) {
    print2file("Couldn't find RTC");
    LEDsos();
  }
  
  if (!rtc.isrunning())
  {
    print2file("RTC lost power, set time from compiled.");
    rtc.adjust(compiled);
  }

  DateTime now = rtc.now();
  print2file("Compiled "+ date2String(compiled) );
  print2file("RTC      "+ date2String(now) );
  if (now.unixtime() < compiled.unixtime() )
  {
    print2file("RTC is older than compile time! (RTC not work? Bad battery? Set time from compiled)");
    rtc.adjust(compiled);
  }
  else if (now.unixtime() > compiled.unixtime() )
  {
    print2file("RTC is newer than compile time. (Good. RTC works)");
  }
  else if (now.unixtime() == compiled.unixtime() )
  {
    print2file("RTC is the same as compile time! (Fine if we just upload scetch)");
  }
  filename=String(now.year()) + "_" + String(now.month()) + "_" + String(now.day()) + ".csv";

  // Temperature init start
  Serial.println("DHT22 Sensor. No init required.");
  // Temperature init end

  // Voltage start
  Serial.println("Voltage Sensor. Switch pin A0 to input mode.");
  pinMode(A0, INPUT);

  // Log header
  print2file ("Date Time,unixtime (sec),uptime (sec),voltage (V),temperature (C),humidity (%)");

}
//**********************************************************************************
void loop ()
{
  // LED blink
  if (millis() - timerLED >= 1000) {  // таймер 1 second)
    timerLED = millis();              // сброс таймера
    LEDmode = !LEDmode;
    digitalWrite(LED_BUILTIN, LEDmode);  // turn the LED on and off
  }
  // end LED blink
  
  // Active tests: Read RTC, Get sensors, Write SD
  if (millis() - timerRTC >= 60000) {  // таймер 1 minutes)
    timerRTC = millis();              // сброс таймера

    // get RTC
    if (!rtc.isrunning())
    {
      print2file("RTC lost confidence in the DateTime!");
    }
    DateTime now = rtc.now();
    filename=String(now.year()) + "_" + String(now.month()) + "_" + String(now.day()) + ".csv";
    
    // sensors

    // voltage
    // take 10 analog samples and make an average value
    sample_count = 0;
    sum_voltage = 0;
    while (sample_count < NUM_SAMPLES) {
        sum_voltage += analogRead(A0);
        sample_count++;
        delay(10);
    }
    adc_voltage  = ((float)sum_voltage / (float)NUM_SAMPLES ) / 1024.0; 
    in_voltage = adc_voltage / (R2/(R1+R2)) / (R2_ext/(R1_ext+R2_ext) ) ;

    // Temperature dht22
    dht.read2(&temp, &humi, NULL);
   
    // write to log
    print2file ( date2String(now) + "," + 
                 String(now.unixtime()) + "," + 
                 String(millis()/1000) + "," + 
                 String(in_voltage) + "," + 
                 String(temp) + "," + 
                 String(humi) );
  }
  // end read rtc
  delay(1);
}
//**********************************************************************************
String date2String(DateTime dt)
{
  char datestring[20];
  sprintf(datestring, "%04d/%02d/%02d %02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second() ); 
  return(datestring);
} 
void LEDsos () {
  Serial.println("SOS. Stopped on error");
  LEDmode=1;
  digitalWrite(LED_BUILTIN, LEDmode);
  while (1) {
    for ( int i=0; i < 6; i++) {
      delay (150);
      LEDmode = !LEDmode;
      digitalWrite(LED_BUILTIN, LEDmode);
    }
    for ( int i=0; i < 6; i++) {
      delay (500);
      LEDmode = !LEDmode;
      digitalWrite(LED_BUILTIN, LEDmode);
    }
    for ( int i=0; i < 6; i++) {
      delay (150);
      LEDmode = !LEDmode;
      digitalWrite(LED_BUILTIN, LEDmode);
    }
    delay (2000);
  }
}
void print2file (String data) {
  Serial.println(data);
  File myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.println(data);
    myFile.close();
  }   
}
