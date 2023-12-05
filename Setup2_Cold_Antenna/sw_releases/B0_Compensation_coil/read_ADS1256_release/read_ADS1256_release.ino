/*When used with Arduino Due board, we have the following mapping:
-standards SPI ports of the Due for serial communication with the ADS1256
-digital pin 51 for DRDY (data ready)
-digital pin 53 for CS (chip select)
-plug switch for turning on compensation between any GND pin and digital pin 22
-LED+ for acquisition state on digital pin 23
-LED+ for compensation state on digital pin 25
*/
#include "ADS1256.h"

//---define when debugging---
// #define DEBUG_READ_ADS1256_PAUSE    
// #define DEBUG_READ_ADS1256

#define CH1_P   ADS1256_MUX_AIN0
#define CH1_N   ADS1256_MUX_AIN1
#define CH2_P   ADS1256_MUX_AIN2
#define CH2_N   ADS1256_MUX_AIN3

// #define DUAL_PROBE            

#define PIN_COMPENSATION   22
#define LED_COMPENSATION   25
#define LED_ACQUISITION    23

#define BIT_COMPENSATION   0
#define BIT_ACQUISITION    1

void stop ();
void start_stop ();
void pin_compensation_ISR();
void single_probe_acquisition();
void dual_probe_acquisition();
void print_ADC_state();

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.5; // voltage reference

// ADS1256 object has to be constructed in setup() 
ADS1256* adc;

static long sensor1, sensor2;
static uint32_t count_sent = 0;
static uint32_t sample_time, start_time;
byte status_register = 0x00;   //multi purpose byte to transmit to serial interface 
byte ADC_status, ADC_mux, ADC_adcon, ADC_adrate;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_COMPENSATION, INPUT_PULLUP);  
  pinMode(LED_COMPENSATION, OUTPUT);
  pinMode(LED_ACQUISITION, OUTPUT);
  pin_compensation_ISR ();
  attachInterrupt(digitalPinToInterrupt(PIN_COMPENSATION), pin_compensation_ISR, CHANGE);

  SerialUSB.begin(0);     // Initialize Native USB port
  Serial.begin(115200);   // 115200 good, doesn't slow down the loop up to 500 sps, arduino Due's programming port cannot go beyond that speed
  
  // --- setup the ADC ---
  adc = new ADS1256 (clockMHZ,vRef,false);    // RESETPIN is permanently tied to 3.3v
  adc->begin(ADS1256_DRATE_1000SPS,ADS1256_GAIN_1,true, true);   // bufferenable = true, clock out disable = true
  adc->setChannel(CH1_P,CH1_N);                   // Set ADC_mux Register 
  
  #ifdef DEBUG_READ_ADS1256
  Serial.println("\n\nInit done\n\n");
  print_ADC_state();
  #endif

  delay (1000);
  stop ();

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_ACQUISITION, HIGH);

  bitSet (status_register, BIT_ACQUISITION);
  start_time = micros();
}

void loop() 
{ 
  start_stop ();

  #ifdef DUAL_PROBE
  dual_probe_acquisition();
  #else
  single_probe_acquisition();
  #endif
}

//program stuck in infinite loop until 's' inputed
void stop (){
  #ifdef DEBUG_READ_ADS1256_PAUSE
  bitClear (status_register, BIT_ACQUISITION);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_ACQUISITION, LOW);
  
  while (Serial.read()!='s' && SerialUSB.read()!='s');

  bitSet (status_register, BIT_ACQUISITION);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_ACQUISITION, HIGH);
  #endif
}

//change state by inputing a 's'
void start_stop (){
  #ifdef DEBUG_READ_ADS1256_PAUSE
  if (Serial.read()=='s' || SerialUSB.read()=='s'){
    stop();
  }
  #endif
}

void pin_compensation_ISR(){
  bool state = !digitalRead (PIN_COMPENSATION); //PIN_COMPENSATION is pulled up by default, goes LOW as swith is turned on
  bitWrite (status_register, BIT_COMPENSATION, state);
  digitalWrite (LED_COMPENSATION, state);
}

void single_probe_acquisition(){
  adc->waitDRDY();
  sensor1 = adc->readCurrentChannelRaw ();

  //send data to PC 
  sample_time = micros()-start_time;
  SerialUSB.write ((uint8_t*) &sensor1, 3);  
  SerialUSB.write ((uint8_t*) &count_sent, 3);
  SerialUSB.write ((uint8_t*) &sample_time, 4);
  SerialUSB.write ((uint8_t*) &status_register, 1);

  count_sent++;
}

void dual_probe_acquisition(){
  adc->waitDRDY();
  adc->setChannel(CH2_P,CH2_N);
  sensor1 = adc->readCurrentChannelRaw ();

  print_ADC_state ();

  #ifdef ADS_1256_LOW_FREQUENCY
  //send data to PC 
  sample_time = micros()-start_time;
  SerialUSB.write ((uint8_t*) &sensor1, 3);  
  SerialUSB.write ((uint8_t*) &sensor2, 3);  
  SerialUSB.write ((uint8_t*) &count_sent, 3);
  SerialUSB.write ((uint8_t*) &sample_time, 4);
  SerialUSB.write ((uint8_t*) &status_register, 1);
  count_sent++;
  #endif

  adc->waitDRDY();
  adc->setChannel(CH1_P,CH1_N);
  sensor2 = adc->readCurrentChannelRaw ();

  print_ADC_state();

  //send data to PC 
  sample_time = micros()-start_time;
  SerialUSB.write ((uint8_t*) &sensor1, 3);  
  SerialUSB.write ((uint8_t*) &sensor2, 3);  
  SerialUSB.write ((uint8_t*) &count_sent, 3);
  SerialUSB.write ((uint8_t*) &sample_time, 4);
  SerialUSB.write ((uint8_t*) &status_register, 1);

  count_sent++;
}

void print_ADC_state (){
  #ifdef DEBUG_READ_ADS1256
  ADC_status = (uint8_t) adc->readRegister(ADS1256_RADD_STATUS);
  ADC_mux = (uint8_t) adc->readRegister(ADS1256_RADD_MUX);
  ADC_adcon = (uint8_t) adc->readRegister(ADS1256_RADD_ADCON);
  ADC_adrate = (uint8_t) adc->readRegister(ADS1256_RADD_DRATE);
  
  Serial.println ("ADC_status register: ");
  Serial.println (ADC_status, BIN);
  Serial.println ("ADC_mux register: ");
  Serial.println (ADC_mux, BIN);
  Serial.println ("A/D control register: ");
  Serial.println (ADC_adcon, BIN);
  Serial.println ("A/D data rate register: ");
  Serial.println (ADC_adrate, BIN);
  Serial.flush();
  #endif
}
