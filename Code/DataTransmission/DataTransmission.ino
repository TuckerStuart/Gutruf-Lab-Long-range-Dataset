
//LoRaFinalCode//

/*This code collects heart rate and temperature data every 30 seconds and sends it to receiver. All parameters of LoRa communication are modular, and other options are listed in comments. 
Heart rate collection size is also modular and is set at 5. Timing, collection sizes, and parameters can be easily altered. 
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#define timetillsleep 5  //works best for no errors
#define timetillwakeup 30000 //length of deep sleep time in milliseconds
#ifndef LoraWan_RGB 
#define LoraWan_RGB 0   
#endif

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             25        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]

#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]

#define LORA_CODINGRATE                             4         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx

#define LORA_SYMBOL_TIMEOUT                         0         // Symbols

#define LORA_FIX_LENGTH_PAYLOAD_ON                  false

#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000

#define BUFFER_SIZE                                 30        // Define the payload size here

#define collectionsize                              5         // Heart rate collection size

#define txenable                                    GPIO4     // Define TX enable pin

#define debug Serial

MAX30105 particleSensor;

// Declare data packet for transmission
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

//Heart rate variables
long lastBeat = 0; 
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
float beatsPerMinute;
int beatAvg;
int previousBeat;
int beatDiff;
int BPMs[collectionsize];
long irValue;
int k;
byte ledBrightness = 0;

//Analog read temperature variables
char temp[5];
int analogPin = ADC;
int val = 0; 

//LoRa module variables
static RadioEvents_t RadioEvents;
int16_t rssi, rxSize;
static TimerEvent_t sleep;
static TimerEvent_t wakeUp;
uint8_t lowpower = 1;

//Function for when module enters sleep mode
void onSleep()
{
  lowpower = 1;                                 //sets module to low power mode
  TimerSetValue( &wakeUp, timetillwakeup );     //sets a value for a timer to wake the module up after sleep. Can be changed above
  TimerStart( &wakeUp );                        //timer starts  
}

//Function for when module wakes up from sleep mode
void onWakeUp()
{
  lowpower = 0;            //sets module to active/awake mode
  particleSensor.wakeUp(); //wake up the MAX30102
  
  //Set up sensor. All values below were found to be optimal for heart rate collection on wrist. Heart rate data was compared with gold standard finger PPG
  byte ledBrightness = 90; //set brightness of LED. 90 was found to be optimal for wrist.
  byte sampleAverage = 8;  //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green.
  int sampleRate = 1000;   //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 118;    //Options: 69, 118, 215, 411
  int adcRange = 16384;    //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  Serial.print("Woke up\n"); //for debugging purposes to let user know the device has woken from sleep mode
 
  delay(500); //makes sure settings have all been set
  
  //Heart rate algorithm
  int i = 0;
  k = 0;
  beatsPerMinute = 0;

  while (i < collectionsize) {
  long irValue = particleSensor.getIR();   //get raw IR value from particle sensor
  if (checkForBeat(irValue) == true)       //built in function for MAX30102 that checks for a beat
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    previousBeat = beatsPerMinute;
    beatsPerMinute = 60 / (delta / 1000.0);
    beatDiff = abs(previousBeat - beatsPerMinute);      //finds difference in current beat and previous beat as measured by MAX30102

       if (beatDiff < 35){                                  //only beats that count are less than 35 BPM away from previous beat
          if (beatsPerMinute < 200 && beatsPerMinute > 30)    //eliminate unrealistic heart beats that sometimes show up
          {
            rates[rateSpot++] = (byte)beatsPerMinute;         //Store this reading in the array
            rateSpot %= RATE_SIZE;                            //Wrap variable
      
            //Take average of readings. Note that MAX30102 built in reading of heart rate collects many heart rate data points in short time
            beatAvg = 0;
              for (byte x = 0 ; x < RATE_SIZE ; x++) 
              {
                  beatAvg += rates[x];
                  beatAvg /= RATE_SIZE;
                  BPMs[i] = beatAvg;   //store average into array of readings
                  i = i + 1;  
                  k = 0;               //used to keep track of bad readings. Sets bad readings to 0
              }
         }
       }
  }
  else {
     if (k > 1800) {    
       sprintf(txpacket, "%s ", "-1");  //if 1800 bad IR readings occur, device will indicate no valid reading for heart rate
       break;
      }
      k = k + 1;  //acts as a counter for bad readings
    }
  }

  particleSensor.shutDown();   //once data is collected, shut down sensor to save power
  
  if (i == collectionsize) {   //if 5 good readings exist
      if ((abs(BPMs[0] - BPMs[1]) < 15)) {  //used to check problem of first value sometimes being extremely different from the rest
        //find the average of the beats in the BPM array
        int j = 0;
        float sumBPM = 0;
            while (j < collectionsize) {
            sumBPM = sumBPM + BPMs[j];
            j = j + 1;
            }
        beatAvg = sumBPM / (collectionsize);  
      }
      else {
        //if the first value is bad, find the average of the remaining 4
        int j = 1;
        float sumBPM = 0;
            while (j < collectionsize) {
              sumBPM = sumBPM + BPMs[j];
              j = j + 1;
            }
        beatAvg = sumBPM / (collectionsize-1);
      }
      sprintf(txpacket, "%d ", beatAvg);  //put the beat value in the data packet
  }

  //Temperature reading
  int z = 0;
  int sumval = 0;
  int avgval = 0;
  //read from analog pin 8 times. Number can be altered easily
  while (z < 8) {
    val = analogRead(analogPin);
    sumval = sumval + val;
    z = z + 1;
  }
  //find the average value 
  avgval = sumval/z;
  sprintf(temp, "%d", avgval);
  strcat(txpacket, temp);  //add temperature value to end of data packet
  
  digitalWrite(GPIO4, HIGH); //pull TX enable pin high
  
  Serial.println(txpacket);  //print the data packet to serial monitor for debugging
 
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));  //data packet transmission

  digitalWrite(GPIO4, LOW);  //pull TX enable pin low

  TimerSetValue( &sleep, timetillsleep );  //send device into sleep mode after 5 milliseconds
  TimerStart( &sleep );
}

//setup code
void setup() {
  debug.begin(115200);

  pinMode(txenable, OUTPUT);      //set TX enable to output pin

  // Initialize sensor
  if (particleSensor.begin() == false)
  {
    debug.printf("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //LoRa module setup
  rssi = 0;
  digitalWrite(txenable, HIGH); 
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
  Radio.Sleep( );
  digitalWrite(txenable, LOW); 
  
  //send to sleep first 
  TimerInit( &sleep, onSleep );
  TimerInit( &wakeUp, onWakeUp );
  onSleep();
}

void loop() {
  if (lowpower) {    
    lowPowerHandler();  //for low power operation 
  }
}
