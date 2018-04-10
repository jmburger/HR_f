// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// MAX30100 sensor:
#include <MAX30100.h>
MAX30100 sensor;
#define IR_current      MAX30100_LED_CURR_50MA          //define IR led current (50mA)
#define RED_current     MAX30100_LED_CURR_50MA          //define Red led current (50mA)
#define Sample_Rate     MAX30100_SAMPRATE_100HZ         //define sample rate (100Hz)
#define Pulse_width     MAX30100_SPC_PW_1600US_16BITS   //define led pulse width (1600)
#define Highres_mode    true      //High resolution mode
//Filter parameters:
#define ALPHA_VAL_DCR     0.9      //DC filter alpha value
#define SAMPLE_SIZE  100           //Mean difference filter sample size used to calculate the running mean difference
#define SSF_WINDOW_SIZE  10        //Slope sum function window size
#define ATF_WINDOW_SIZE  150       //Adaptive threshold function window size
#define ATF_ALPHA  0.5             //Adaptive threshold function window size
#define PEAK_NUMBER 10             //Number of peaks used tto find average timestamp

//------------------------------------------
//DC remover:
float output_DCR = 0;
float prev_output = 0;
//------------------------------------------
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     // Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//------------------------------------------
//Low pass filter (butterworth filter) variables:
float Val[3];
float BWF_output = 0;
//------------------------------------------
// Beat Detection:
//----------------Stage 1:------------------
//Slope sum function (SSF):
float SSF_input[SSF_WINDOW_SIZE];
float SSF_prev_input = 0;
float SSF_output[SSF_WINDOW_SIZE];
uint8_t SSF_index = 0;
float SSF_TV = 0;      //SSF trailling measurements
float Delta_input = 0;
//----------------Stage 2:------------------
//Adaptive threshold function:
float ATF_input = 0;
uint8_t ATF_index = 0;
float ATF_TV = 0;
float ATF_output[3];
float ATF = 0;
int Count = 0;
float AVG = 0;
//----------------Stage 3:------------------
//Peak Detection function:
uint8_t PD_index = 0;
float x[3];
float delta_signal = 0;
int BPM = 0;
int time_current = 0;
int time_prev = 0;
int timestamp_current = 0;
int timestamp_prev = 0;
int timestamp_vec[PEAK_NUMBER];
int timestamp_TV = 0;
int sum_timestamp = 0;
int mean_timestamp = 0;
bool Beat_detected = false;
bool False_beat_detected = false;
uint8_t TS_index = 0;
//-------------------------------------------   
//Variables store raw RED and IR values
uint16_t raw_IR_Val = 0;
uint16_t raw_RED_Val = 0;
float IR_Val = 0;
float RED_Val = 0;
float Threshold_val = 0;
//--------------------------------------------

// Temperature sensor:
#include <SparkFun_MLX90632_Arduino_Library.h>
MLX90632 myTempSensor;
float objectTemp = 0;

void setup() {

  Serial.begin(115200);

  //Temperature sensor:----------------------------
  //Serial.println("MLX90632 Tempurature Sensor");
  //Wire.begin();
  //myTempSensor.begin();
  //-----------------------------------------------

  //MAX30100 (HR and SpO2) senesor:----------------
  Serial.print("Initializing MAX30100..");
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  if (!sensor.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  Serial.println("MAX30100 HR and SpO2 Sensor");
  sensor.setMode(MAX30100_MODE_SPO2_HR);          //setting sensor mode (HR and SpO2)
  sensor.setLedsCurrent(IR_current, RED_current); //Set led's current IR and Red respectively
  sensor.setLedsPulseWidth(Pulse_width);          //Set led's pulse width
  sensor.setSamplingRate(Sample_Rate);            //set sample rate
  sensor.setHighresModeEnabled(Highres_mode);     //set high resolution
  sensor.resetFifo();                             //rest fifo register
  //-----------------------------------------------
}

void loop() {
  //------------TEMP Sensor-----------------
  //objectTemp = myTempSensor.getObjectTemp();  //Get the temperature of the object (in degrees Celcuis)
  //Serial.print("Object temperature: ");       //Print objects temperature
  //Serial.print(objectTemp, 2);
  //Serial.print(" C");
  //Serial.println();
  //----------------------------------------

  //-----------HR & SpO2 Sensor-------------
  sensor.update();

  while (sensor.getRawValues(&raw_IR_Val, &raw_RED_Val)) {
    //Serial.println(raw_IR_Val);
    //Serial.print('\t');
    //Serial.println(raw_RED_Val);
    //Filtering:
    //DC Removeral:
    IR_Val = DCR_function(raw_IR_Val, ALPHA_VAL_DCR);       //Remove the DC signal with a response constant of alpha = 0.95
    IR_Val = MeanDifference_F(IR_Val);                      //Mean difference filter
    IR_Val = Butterworth_LPF(IR_Val);                       //Low pass filter (butterworth filter
    //Beat Detection:
    IR_Val = SSF_function(IR_Val);
    Threshold_val = ATF_function(IR_Val);
    BPM = PDF_function(IR_Val, Threshold_val);
    Serial.println(BPM);
    //Serial.print(IR_Val);
    //Serial.print(",");
    //Serial.println();
  }

}
//---------------------------------------------------------------------------
//DC Removeral filter
float DCR_function(float raw_input, float alpha) {
  float filtered = raw_input + alpha * prev_output;  
  output_DCR = filtered - prev_output;
  prev_output = filtered;
  return output_DCR;
}
//---------------------------------------------------------------------------
// Mean difference filter
float MeanDifference_F(float raw_input) {
  float avg = 0;
  
  sum -= MeanDiff_TV[Index];
  MeanDiff_TV[Index] = raw_input;
  sum += MeanDiff_TV[Index];
  
  Index++;
  Index = Index % SAMPLE_SIZE;

  if (count < SAMPLE_SIZE) {
    count++;
  }

  avg = sum / count;
  return avg - raw_input;
}
//---------------------------------------------------------------------------
//Low pass filter (Butterworth filter)
float Butterworth_LPF(float raw_input) {
  //------------------------------
  // First Order low pass filter:
  //------------------------------
//  Val[0] = Val[1];
//
//  //Fs = 100Hz (sample rate) and Fc = 10Hz (cut-off frequency)
//  Val[1] = (2.452372752527856026e-1 * raw_input) + (0.50952544949442879485 * Val[0]);
//
//  //Fs = 100Hz (sample rate) and Fc = 4Hz (cut-off frequency)
//  //- if sample rate is exactly 100Hz then this is very precise butterworth filter
//  //Val[1] = (1.367287359973195227e-1 * raw_input) + (0.72654252800536101020 * Val[0]);
//
//  BWF_output = Val[0] + Val[1];
  //-------------------------------
  // Second Order low pass filter:
  //-------------------------------
  Val[0] = Val[1];
  Val[1] = Val[2];

  //Fs = 100Hz (sample rate) and Fc = 10Hz (cut-off frequency)
  Val[2] = (6.745527388907189559e-2 * raw_input) + (-0.41280159809618854894 * Val[0]) + (1.14298050253990091107 * Val[1]);
  BWF_output = (Val[0] + Val[2]) + 2*Val[1];
  
  return BWF_output;
}
//---------------------------------------------------------------------------
//Beat detection:
//---------------------------------STAGE 1:----------------------------------
//Slope Sum Function (SSF):
float SSF_function(float raw_input) {

  SSF_TV = SSF_output[SSF_index];                       //Setting trailing value
  SSF_input[SSF_index] = raw_input;

  Delta_input = SSF_input[SSF_index] - SSF_prev_input;  //Delta value of input signal to previous input signal
  if (Delta_input > 0)                                  //if delta vaule is greater than zero
  {
    SSF_output[SSF_index] += Delta_input;               //Summing delta values
  }

  SSF_prev_input = SSF_input[SSF_index];                //Previous input
  SSF_output[SSF_index] -= SSF_TV;                      //Subtracting trailing values
  float output = SSF_output[SSF_index];                 //Output signal value

  SSF_index++;                                          //Increment index 1
  SSF_index = SSF_index % SSF_WINDOW_SIZE;

  return output;
}
//---------------------------------STAGE 2:----------------------------------
//Adaptive threshold function:
float ATF_function(float raw_input) {

  //Greatest vaule in window period
  if (ATF_input < raw_input)
  {
    ATF_input = raw_input;
    ATF = ATF_ALPHA*ATF_input;
  }

  //Average value of the past highest values in the previous windows
  if (ATF_index == ATF_WINDOW_SIZE - 1)
  {
    ATF_output[Count] = ATF;    
    AVG = (ATF_output[0] + ATF_output[1] + ATF_output[2])/3;
    ATF_input = 0;
    Count++;
    Count = Count % 3;  
  }   
  
  ATF_index++;                                              //Increment index 1
  ATF_index = ATF_index % ATF_WINDOW_SIZE;
  
  return AVG;
}
//---------------------------------STAGE 3:----------------------------------
// Peak dectection of SSF - AFT:
float PDF_function(float raw_input, float threshold){

  // Delta value between ssf-signal and adaptive threshold
  delta_signal = raw_input-threshold;
  if (delta_signal < 0) 
  {
    delta_signal = 0;
  }
  //Peak detection with threshold:
  if (PD_index == 0)
  {
    x[PD_index] = delta_signal;
    if (x[2] > x[PD_index] && x[2] > x[1])
    {      
      time_current = millis();                //Current time instance on microcontroller
      Beat_detected = true;                   //beat detected
    }
  }
  if (PD_index == 1)
  {
    x[PD_index] = delta_signal;
    if (x[0] > x[PD_index] && x[0] > x[2])
    {      
      time_current = millis();                //Current time instance on microcontroller
      Beat_detected = true;                   //beat detected
    }
  }
  if (PD_index == 2)
  {
    x[PD_index] = delta_signal;
    if (x[1] > x[PD_index] && x[1] > x[0])
    {      
      time_current = millis();                //Current time instance on microcontroller
      Beat_detected = true;                   //beat detected 
    }
  }  
  PD_index++;                                 //Increment index 
  PD_index = PD_index % 3;      

  if (Beat_detected == true)                  //Entre if beat was detected
  {
    timestamp_current = time_current - time_prev;     //Time difference from peak to peak detected
    time_prev = time_current;                         //Previous time
    if (timestamp_current < mean_timestamp*0.7 )      //Less than 70% smaller
    {
      False_beat_detected = true;                     //False beat was detected
    }
    if (timestamp_current > mean_timestamp*1.3 )      //More than 70% greater
    {
      False_beat_detected = true;                     //False beat was detected
    }
    timestamp_TV = timestamp_vec[TS_index];           //Timestamp trailing value
    timestamp_vec[TS_index] = timestamp_current;      //Store timestamp in vector
    sum_timestamp = sum_timestamp + timestamp_current - timestamp_TV;  //Sum current timestamp and subtract trailing timestamp

    TS_index++;                                       //Increment index 
    TS_index = TS_index % PEAK_NUMBER;

    mean_timestamp = sum_timestamp/PEAK_NUMBER;       //Mean timestamp of the detected beats

    if (False_beat_detected == true)
    {
      BPM = 60000/(timestamp_current);                  //Calculating beat per minute  
      timestamp_prev = timestamp_current;               //Previous timestamp
    }
      Beat_detected = false;                            //Beat has been detected
      False_beat_detected = false;                      //False beat has been detected
  }
  return mean_timestamp;                                         //Return the BPM 
}









