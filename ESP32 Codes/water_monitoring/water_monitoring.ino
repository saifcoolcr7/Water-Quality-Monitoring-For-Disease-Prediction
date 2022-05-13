#include <WiFi.h>
#include "ThingSpeak.h"
#include "Arduino.h"
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"

#define ADCPIN 32
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35 34 // ESP32 pin GIOP36 (ADC0) connected to LM35

// TDS 
#define TdsSensorPin 25
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

#define SensorPin 33            //pH meter Analog output to Arduino Analog Input 0
#define Offset 41.02740741      //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
#define uart  Serial

int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
int adcValue;
float voltValue;
float voltage, ecValue;
DFRobot_ESP_EC ec;

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(35);//needed EEPROM.begin to store calibration k in eeprom
  ec.begin(0.989);//by default lib store calibration k since 10 change it by set ec.begin(30); to start from 30
  pinMode(TdsSensorPin,INPUT);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
   Serial.println("");
   Serial.println("WiFi connected\n");
   delay(500);
   ThingSpeak.begin(client);
}

void loop()
{

//pH
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = -19.18518519 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    uart.print("Voltage:");
    uart.print(voltage, 2);
    uart.print("    pH value: ");
    uart.println(pHValue, 2);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    printTime = millis();
  }

//Ec 
  Serial.print("voltage:");
  static unsigned long timepoint = millis();
  
    timepoint = millis();
    voltage = analogRead(35)/1024.0*5000;
    Serial.print("voltage:");
    Serial.println(voltage, 4);


    ecValue = ec.readEC(voltage, 35); // convert voltage to EC with temperature compensation
    Serial.print("EC:");
    Serial.print(ecValue,2);
    Serial.println("ms/cm");
  ec.calibration(voltage, 32); // calibration process by Serail CMD
  delay(500);

//TURBIDITY
  int turbidityValue = 0;
    for (int i=0; i<10; i++){
      turbidityValue += analogRead(ADCPIN);
      delay(10);
    }
    turbidityValue /= 10;
    float turbidityV = turbidityValue/100;
    //turbidityV = round_to_dp(turbidityV, 1);
    turbidityValue = turbidityV;
    //Serial.print("Turbidity level: ");
    //Serial.println(turbidityV);
    if( turbidityV > 9){
        Serial.print("Turbidity Level: ");
        Serial.println(turbidityV); 
       // Serial.println("NTU");
        Serial.println("Very Clean");
      //  delay(500);
    }
    if( turbidityV >= 8.5 && turbidityV <= 9 ){
        Serial.print("Turbidity Level: ");
        Serial.println(turbidityV); 
       // Serial.println("NTU");
        Serial.println("Clean");
    }
  
    if(turbidityV >= 6 && turbidityV < 8.5){
        Serial.print("Turbidity Level: ");
        Serial.println(turbidityV); 
       // Serial.println("NTU");
        Serial.println("Dirty");
      // delay(500);
     }
     if( turbidityV < 6){
        Serial.print("Turbidity Level: ");
        Serial.println(turbidityV); 
       // Serial.println("NTU");
        Serial.println("Very Dirty");
       // delay(500);
     }
     int ntu = -2572.2*turbidityV*turbidityV + 8700.5*turbidityV - 4352.9;
     Serial.print("NTU: ");
     Serial.println(ntu);
     delay(1000);

//TEMPERATURE
// read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 32;

  // print the temperature in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(tempC);   // print the temperature in °C
  Serial.print("°C");
  Serial.print("  ~  "); // separator between °C and °F
  Serial.print(tempF);   // print the temperature in °F
  Serial.println("°F");

  delay(500);


// TDS Sensor


static unsigned long analogSampleTimepoint = millis();
if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
{
analogSampleTimepoint = millis();
analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
analogBufferIndex++;
if(analogBufferIndex == SCOUNT)
analogBufferIndex = 0;
}
static unsigned long printTimepoint = millis();
if(millis()-printTimepoint > 800U)
{
printTimepoint = millis();
for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
tdsValue=((133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5)/100; //convert voltage value to tds value
//Serial.print("voltage:");
//Serial.print(averageVoltage,2);
//Serial.print("V ");
Serial.print("TDS Value:");
Serial.print(tdsValue,0);
Serial.println("ppm");


//THINKSPEAK

  ThingSpeak.setField(1, pHValue);
  ThingSpeak.setField(2, ecValue);
  ThingSpeak.setField(3, tdsValue);
  ThingSpeak.setField(4,turbidityValue );
  ThingSpeak.setField(5, tempC);
  ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);

  delay(7000); // 15 seconds
}
}
int getMedianNum(int bArray[], int iFilterLen)
{
int bTab[iFilterLen];
for (byte i = 0; i<iFilterLen; i++)
bTab[i] = bArray[i];
int i, j, bTemp;
for (j = 0; j < iFilterLen - 1; j++)
{
for (i = 0; i < iFilterLen - j - 1; i++)
{
if (bTab[i] > bTab[i + 1])
{
bTemp = bTab[i];
bTab[i] = bTab[i + 1];
bTab[i + 1] = bTemp;
}
}
}
if ((iFilterLen & 1) > 0)
bTemp = bTab[(iFilterLen - 1) / 2];
else
bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
return bTemp;
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    uart.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}
  


//float readTemperature()
//{
  //add your code here to get the temperature from your temperature sensor
//}
