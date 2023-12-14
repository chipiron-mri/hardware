/**************************************************************************/
/*!
    @file     sinewave.pde
    @author   Adafruit Industries
    @license  BSD (see license.txt)

    This example will generate a sine wave with the MCP4725 DAC.

    This is an example sketch for the Adafruit MCP4725 breakout board
    ----> http://www.adafruit.com/products/935

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

long period = 2000;
unsigned long time_now = 0;
int x =0;
int i =0;
int val1,val2;


// Filter variables
const float alpha = 0.5;
//double data_filtered1[] = {0, 0};
//double data_filtered2[] = {0, 0};
const int n = 1;
const int analog_pin = 0;

float voltage1 = 0;
float voltage2 = 0;

const int numReadings = 50;  // Number of readings to average
float readings1[numReadings];    // Array to store readings
float readings2[numReadings];    // Array to store readings
int index1 = 0;                // Index for the readings array
float total1 = 0;               // Running total of readings
float total2 = 0;               // Running total of readings
float average1 = 0;
float average2 = 0;
float current = 0;
float dac_current = 0;
void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  for (int i = 0; i < numReadings; i++) {
    readings1[i] = 0;   // Initialize the array
    readings2[i] = 0;   // Initialize the array
  }
}


void loop(void) {


    
    
        dac_current = 0.8 ;  // amp
        int dac_equivalent = 451 * dac_current + 2046;
        dac.setVoltage(dac_equivalent, false);
        x= 0;
      
      
      time_now = millis();

      while(millis() < time_now + period){
      // Iload = Vsense / G * R12
      voltage1 = analogRead(0) * (5.0 / 1023.0);
      voltage2 = analogRead(1) * (5.0 / 1023.0);
      current = (voltage1-voltage2) * 4 * 0.225; 
      
      //Fix limits on scale
 

            Serial.println(current);
      
      //readings1[index1] = voltage1;
      //readings2[index1] = voltage2;
      //total1 = 0;
      //total2 = 0;
      //for ( i = 0; i < numReadings; i++)
        //total1 = total1 + readings1[i];
        //total2 = total2 + readings2[i];
      //index1 = (index1 + 1) % numReadings;  // Move to the next position in the array

      // Calculate the average DC value
      //average1 = total1 / numReadings;
      //average2 = total2 / numReadings;


      
      //Serial.print(",");
      //Serial.println(average2);

      // Low Pass Filter
      //data_filtered1[n] = alpha * voltage1 + (1 - alpha) * data_filtered1[n-1];
      // Store the last filtered data in data_filtered[n-1]
      //data_filtered1[n-1] = data_filtered1[n];
      // Print Data
      //Serial.print(data_filtered1[n]);

      //Serial.print(",");

      // Low Pass Filter
      //data_filtered2[n] = alpha * voltage2 + (1 - alpha) * data_filtered2[n-1];
      // Store the last filtered data in data_filtered[n-1]
      //data_filtered2[n-1] = data_filtered2[n];
      // Print Data
      //Serial.print(data_filtered2[n]);
      //Serial.print(",");
      //Serial.println(data_filtered1[n]-data_filtered2[n]);
      }
    
    
}
      
              

      
      

