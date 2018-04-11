// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// MAX30100 sensor:
#include <MAX30100.h>
MAX30100 sensor;
#define IR_current      MAX30100_LED_CURR_50MA          //define IR led current (50mA)
#define RED_current     MAX30100_LED_CURR_50MA          //define Red led current (50mA)
#define Sample_Rate     MAX30100_SAMPRATE_100HZ         //define sample rate (100Hz)
#define Pulse_width     MAX30100_SPC_PW_1600US_16BITS   //define led pulse width (1600)
#define Highres_mode    true                            //High resolution mode

//Heart Rate
#define RECORDING_TIME  5000000         //Recording time for heart rate (5 seconds)
#define SIZE  RECORDING_TIME/10000      //Vector size equal recording time divided by 10
#define ALPHA_DCR     0.95              //DC filter alpha value
#define SAMPLE_SIZE  100                //Mean difference filter sample size used to calculate the running mean difference

//Variables store raw RED and IR values
uint16_t raw_IR_Val = 0;
uint16_t raw_RED_Val = 0;
double IR_vec[SIZE]; //IR LED Vector for x sec
double RED_vec[SIZE]; //IR LED Vector for x sec
//------------------------------------------
//DC Removal variables:
double prev_filtered = 0;
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
  //-----------HR & SpO2 Sensor-------------
  //----------------------------------------
  int a = 0;                        //Counter
  int delta_time = 0;               //zero remaining time 
  int start_time = micros();        //starting time when enter while loop
  //-----------------------Reading raw sensor values-------------------------------
  while (delta_time < RECORDING_TIME)
  {     
    sensor.update();
    if (sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    {
      // Serial.print(raw_IR_Val);  
      IR_vec[a] = raw_IR_Val;
      RED_vec[a] = raw_RED_Val;
      // Serial.print(" | ");
      // Serial.print(IR_vec[a]); 
      // Serial.print(" | ");
      // Serial.println(micros()); 
      a++;
    }
    delta_time = micros() - start_time;   //update remaining time 
  }
  sensor.shutdown();
  //-----------------------Processing raw values-----------------------------------
  double DCR_IR_vec[sizeof(IR_vec)];
  float MDF_IR_vec[sizeof(IR_vec)];
  float LPF_IR_vec[sizeof(IR_vec)];
  for (int i = 0; i < sizeof(IR_vec); i++)
    {
      DCR_IR_vec[i] = DCR_function(IR_vec[i], ALPHA_DCR);
      MDF_IR_vec[i] = MDF_function(DCR_IR_vec[i]);
      LPF_IR_vec[i] = Butterworth_LPF_function(MDF_IR_vec[i]);
    }

  delay(5000);
  Serial.println(a);
  for (int i = 0; i < sizeof(IR_vec); i++)
  {
     Serial.println(LPF_IR_vec[i]);
  }
  //Serial.println("end");
  //delay(1000);
  sensor.resume();

}

//-------------------------Functions------------------------------------------------
//DC Removeral filter
double DCR_function(double raw_input, float alpha) 
{  
  double filtered = raw_input + alpha * prev_filtered;
  double output_DCR = filtered - prev_filtered;
  prev_filtered = filtered;   
  return output_DCR;
}
// Mean difference filter
float MDF_function(float raw_input) 
{
  float avg = 0; 

  sum -= MeanDiff_TV[Index];
  MeanDiff_TV[Index] = raw_input;
  sum += MeanDiff_TV[Index];  

  Index++;
  Index = Index % SAMPLE_SIZE;

  if (count < SAMPLE_SIZE) 
  {
    count++;
  }

  avg = sum / count;
  return avg - raw_input;
}

//Low pass filter (Butterworth filter)
float Butterworth_LPF_function(float raw_input) 
{

  //Second order low pass filter
  Val[0] = Val[1];
  Val[1] = Val[2];

  //Fs = 100Hz (sample rate) and Fc = 10Hz (cut-off frequency)
  Val[2] = (6.745527388907189559e-2 * raw_input) + (-0.41280159809618854894 * Val[0]) + (1.14298050253990091107 * Val[1]);
  BWF_output = (Val[0] + Val[2]) + 2*Val[1];
  
  return BWF_output;
}
