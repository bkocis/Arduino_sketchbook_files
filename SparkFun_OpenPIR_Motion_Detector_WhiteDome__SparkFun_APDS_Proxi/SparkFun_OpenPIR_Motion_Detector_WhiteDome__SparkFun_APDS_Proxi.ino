//////////////////////////
// Hardware Definitions //
//////////////////////////
//  SparkFun Gesture sensor - short ragne proximity
//  SparkFun OpenPIR sensor - long range proximity 
//
//

#include <Wire.h>
#include <SparkFun_APDS9960.h>
#define PIR_AOUT A0  // PIR analog output on A0
#define PIR_DOUT 2   // PIR digital output on D2
//#define LED_PIN  13  // LED to illuminate on motion
#define PRINT_TIME 10 // Rate of serial printouts

// Global Variables
unsigned long lastPrint = 0; // Keep track of last serial out
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint8_t proximity_data = 0;



void setup() 
{
    // Initialize Serial port
  Serial.begin(9600);
  Serial.println();
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
  
  //Serial.begin(115200);  // Serial is used to view Analog out
  // Analog and digital pins should both be set as inputs:
  pinMode(PIR_AOUT, INPUT);
  pinMode(PIR_DOUT, INPUT);

}

void loop() 
{
  String s1;
  
  // Read A pin, print that value to serial port:
  if ( (lastPrint + PRINT_TIME) < millis() )
  {
    lastPrint = millis();
    // Read in analog value:
    unsigned int analogPIR = analogRead(PIR_AOUT);
    unsigned int digitalPIR = 5 * digitalRead(PIR_DOUT);

    // Convert 10-bit analog value to a voltage
    // (Assume high voltage is 5.0V.)
    float voltage = (float) analogPIR / 1024.0 * 5.0;


    s1 += String(digitalPIR,6);
    s1 += "  ";    
    s1 += String(voltage,4);
    s1 +="  ";
  }

  

  // Read the proximity value
  if ( !apds.readProximity(proximity_data) ) {
    Serial.println("Error reading proximity value");
  } 
  else {
    int prx = proximity_data;
    s1 += String(prx,4);
     }

   // Print out all in one line  
   Serial.println(s1); 
}
