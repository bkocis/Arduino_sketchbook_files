//////////////////////////
// Hardware Definitions //
//////////////////////////
#define PIR_AOUT A0  // PIR analog output on A0
#define PIR_DOUT 2   // PIR digital output on D2
//#define LED_PIN  13  // LED to illuminate on motion

#define PRINT_TIME 50 // Rate of serial printouts

unsigned long lastPrint = 0; // Keep track of last serial out


#include "Adafruit_APDS9960.h"

//the pin that the interrupt is attached to
#define INT_PIN 3

//create the APDS9960 object
Adafruit_APDS9960 apds;





void setup() 
{
  Serial.begin(115200);  // Serial is used to view Analog out
  // Analog and digital pins should both be set as inputs:
  pinMode(PIR_AOUT, INPUT);
  pinMode(PIR_DOUT, INPUT);

 // Serial.begin(115200);
  pinMode(INT_PIN, INPUT_PULLUP);

  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");

  //enable proximity mode
  apds.enableProximity(true);

  //set the interrupt threshold to fire when proximity reading goes above 175
  apds.setProximityInterruptThreshold(0, 175);

  //enable the proximity interrupt
  apds.enableProximityInterrupt();


  // Configure the motion indicator LED pin as an output
//  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, LOW); // Turn the LED off
}

void loop() 
{
  // Read OUT pin, and set onboard LED to mirror output
  readDigitalValue(); 

  // Read A pin, print that value to serial port:
  printAnalogValue();
}

void readDigitalValue()
{
  // The OpenPIR's digital output is active high
  int motionStatus = digitalRead(PIR_DOUT);
/*
  // If motion is detected, turn the onboard LED on:
  if (motionStatus == HIGH)
    digitalWrite(LED_PIN, HIGH);
  else // Otherwise turn the LED off:
    digitalWrite(LED_PIN, LOW);
    */

      if(!digitalRead(INT_PIN)){
    Serial.println(apds.readProximity());

    //clear the interrupt
    apds.clearInterrupt();
  }
}

void printAnalogValue()
{
  if ( (lastPrint + PRINT_TIME) < millis() )
  {
    lastPrint = millis();
    // Read in analog value:
    unsigned int analogPIR = analogRead(PIR_AOUT);
    // Convert 10-bit analog value to a voltage
    // (Assume high voltage is 5.0V.)
    float voltage = (float) analogPIR / 1024.0 * 5.0;
    // Print the reading from the digital pin.
    // Mutliply by 5 to maintain scale with AOUT. 
    Serial.print(5 * digitalRead(PIR_DOUT));
    Serial.print(',');    // Print a comma
    Serial.print(2.5);    // Print the upper limit
    Serial.print(',');    // Print a comma
    Serial.print(1.7);    // Print the lower limit
    Serial.print(',');    // Print a comma
    Serial.print(voltage); // Print voltage
    Serial.println();
  }
}
