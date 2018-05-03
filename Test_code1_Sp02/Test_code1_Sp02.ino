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
#define WARM_UP_TIME 2000000              //Warm up time for MAX30100 sensor (2 seconds)
#define RECORDING_TIME 20000000           //Recording time for heart rate (20 seconds)
#define SIZE  RECORDING_TIME/10000        //Vector size equal recording time divided by 10
#define LEARNING_TIME 5000000             //Learning phase time - start of recording 
#define LEARNING_SIZE LEARNING_TIME/10000 //Vector size of learing phase
#define ALPHA_DCR_IR    0.95                //DC filter alpha value for IR LED
#define ALPHA_DCR_RED    0.75                //DC filter alpha value for RED LED
#define SAMPLE_SIZE   100                 //Mean difference filter sample size used to calculate the running mean difference
#define PEAK_PERIOD   3                   //Length of peak period 


//Variables store raw RED and IR values
uint16_t raw_IR_Val = 0;
uint16_t raw_RED_Val = 0;
//-------------FILTER Variables-------------
//DC Removal variables:
double prev_filtered_IR = 0;     //HR
double prev_filtered_RED = 0;    //Sp02
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     // Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//Low pass filter (butterworth filter) variables:
float Val[3];
//-------Beat detection Variables------------

//------------------------------------------

void setup() {
  Serial.begin(115200);
  
  //Onboard LED:
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);
  
  //Wire.begin();

  //Temperature sensor:----------------------------
  //Serial.println("MLX90632 Tempurature Sensor");
  //Wire.begin();
  //myTempSensor.begin();
  //-----------------------------------------------
  
  //MAX30100 (HR and SpO2) senesor:----------------
  MAX30100_Startup();  //Start up MAX30100 sensor  
  //-----------------------------------------------

}

void loop() {
  //-------------------------------------------------------------------------------
  //-----------------------HR & SpO2 (MAX30100) Sensor-----------------------------
  //-------------------------------------------------------------------------------  
  //-----------------------Reading raw sensor values-------------------------------
  int i = 0;                                      //Counter
  int Total_time = WARM_UP_TIME + RECORDING_TIME; //Toatl time sensor must run
  int delta_time = 0;                             //zero remaining time 
  int start_time = micros();                      //starting time when enter while loop
  float Filtered_IR_val = 0;
  float Filtered_RED_val = 0;
  float IR_DC_val = 0;
  float RED_DC_val = 0;
  PROGMEM float Filtered_IR_vec[SIZE];                 //store filtered IR vector values in flash memory // CHRIS --> When i do this it stores empty array in flash memory
  bool Warm_up = false;                           //Warm up sensor
  
  while (delta_time <= Total_time)
  {     
    sensor.update();
    // if raw data is available 
    if (sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    {
      //HR:
      //Serial.print(raw_IR_Val);           
      //add filtering to raw values:
      Filtered_IR_val, IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR_IR);       //filter raw IR LED data through DC removal
      Filtered_IR_val = MDF_function(Filtered_IR_val);                //mean difference filter IR LED data 
      Filtered_IR_val = Butterworth_LPF_function(Filtered_IR_val);    //low pass butterworth filter IR LED data
      //Filtered_IR_vec[i] = Filtered_IR_val;         //CHRIS --> It does not store the new value in the flash memory (i dont think you can store a value by value in flash memory)
      //Serial.print(" | ");
      //Serial.println(Filtered_IR_vec[i]); 
      //Serial.print(" | ");
      //Serial.println(micros());
      //-------------------------
      //Sp02:
      Filtered_RED_val, RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR_RED);  //filter raw RED LED data through DC removal
      Serial.println(raw_RED_Val);
      //RED and IR DC current balancing:
      if (Warm_up == false)
      {

      }
      //-------------------------
      i++;
    }
    //Warm up sensor:
    if (delta_time >= WARM_UP_TIME && Warm_up == false)
    {
      i=0;
      Warm_up = true;
    }
    delta_time = micros() - start_time;   //update remaining time 
  }
  sensor.shutdown();                      // Shut down MAX30100 sensor
  //fill rest of vector vector with zeros
  Serial.println(i);
  while (i < SIZE)                       
  {
   Filtered_IR_vec[i] = 0;
   i++;
  }
  //-----------------------Processing raw values-----------------------------------
  // Slope Sum function (SSF):
  // int w = 10;                                   //length of analyzing window 
  // float SSF = 0;                                //summation in window period
  // PROGMEM float SSF_output[SIZE];               //SSF output vector stored in flash memory
  // for (int i = 0; i < SIZE; i++)
  // {
  //   if (i <= w)
  //   {
  //     SSF_output[i] = 0;      
  //   }
  //   else
  //   {
  //     for (int x = w; x >= 0; x--)
  //     {
  //       SSF = 0;
  //       float delta_input = pgm_read_float(&(Filtered_IR_vec[i-w])) - pgm_read_float(&(Filtered_IR_vec[(i-w)-1]));
  //       if (delta_input > 0)
  //       {
  //         SSF += delta_input;          
  //       }
  //     }
  //     SSF_output[i] = SSF;
  //   }    
  //   //Serial.println(SSF_output[i]);    
  // }
  // //--------------------------------------------------------------------------------
  // // Beat Detection:
  // // Learning Period:
  // //Calculating top 3 peaks in the first few secounds
  // float Peak_1 = 0;
  // int P1 = 0;
  // float Peak_2 = 0;
  // int P2 = 0;
  // float Peak_3 = 0;
  // int P3 = 0;

  // for (int i = 2; i < LEARNING_SIZE; i++)
  // {
  //   // Getting values from flash memory:
  //   float SSF_i = pgm_read_float(&(SSF_output[i]));
  //   float SSF_i_1 = pgm_read_float(&(SSF_output[i-1]));
  //   float SSF_i_2 = pgm_read_float(&(SSF_output[i-2]));
  //   if (SSF_i_2 < SSF_i_1 && SSF_i_1 >= SSF_i && Peak_1 <= SSF_i_1)
  //   {
  //     Peak_3 = Peak_2;
  //     P3 = P2;
  //     Peak_2 = Peak_1;
  //     P2 = P1;
  //     Peak_1 = SSF_i_1;
  //     P1 = i-1;
  //   }
  //   if (SSF_i_2 < SSF_i_1 && SSF_i_1 >= SSF_i && Peak_1 > SSF_i_1 && Peak_2 <= SSF_i_1)
  //   {
  //     Peak_3 = Peak_2;
  //     P3 = P2;
  //     Peak_2 = SSF_i_1;
  //     P2 = i-1;
  //   }
  //   if (SSF_i_2 < SSF_i_1 && SSF_i_1 >= SSF_i  && Peak_2 > SSF_i_1 && Peak_3 < SSF_i_1)
  //   {
  //     Peak_3 = SSF_i_1;
  //     P3 = i-1;
  //   }
  // }
  // //Serial.println(Peak_1);
  // //Serial.println(Peak_2);
  // //Serial.println(Peak_3);
  // //Calculate threshold:
  // float avg_peak = (Peak_1+Peak_2+Peak_3)/3;
  // float threshold = 0.8*avg_peak;

  // //Processing Period:
  // int Peak_count = 0;                         //Count number of peaks (Beats)
  // int P2p_time_start = 0;                     //Initializing start time for peak to peak
  // int P2p_time_end = 0;                       //Initializing end time for peak to peak 
  // int Delta_P2p_time = 0;                     //Initializing delta time for peak to peak  
  // threshold = 0;                            //(PLOTTING 1)
  // for(int i = 0; i < SIZE; i++)
  // {  
  //   // Getting values from flash memory:
  //   float SSF_i = pgm_read_float(&(SSF_output[i]));
  //   float SSF_i_1 = pgm_read_float(&(SSF_output[i-1]));
  //   float SSF_i_2 = pgm_read_float(&(SSF_output[i-2]));
  //   if (i >= LEARNING_SIZE)
  //   {
  //     threshold = 0.8*avg_peak;             //(PLOTTING 1)
  //     if (SSF_i_2 < SSF_i_1 && SSF_i_1 >= SSF_i)     //Find Peak
  //     {
  //       if (SSF_i_1 > threshold)      //Count peaks above threshold (beats) 
  //       {
  //         Peak_count++;
  //         float Peak_value = SSF_i_1;
  //         //Serial.print(Peak_value);         //(PLOTTING 1)
  //         if (Peak_count > 0)
  //         {
  //           P2p_time_end = P2p_time_start;                     //ending time of peak to peak
  //         }
  //         P2p_time_start = micros();                           //starting time of peak to peak
  //         if (Peak_count > 0)
  //         {
  //           Delta_P2p_time = P2p_time_start - P2p_time_end;    //delta time between peak to peak
  //         }
  //         //Serial.println(Delta_P2p_time);
  //       }
  //     }
  //   }
    // For plotting values (PLOTTING 1)
    // if(i == P1)
    // {
    //   Serial.print(Peak_1);
    // }
    // if(i == P2)
    // {
    //   Serial.print(Peak_2);
    // }
    // if(i == P3)
    // {
    //   Serial.print(Peak_3);
    // }
    //Serial.print(",");
    //Serial.print(SSF_i);
    //Serial.print(",");
    //Serial.println(threshold);
  //}

  // Beats per minute (BPM) calculation:
  //int BPM = (60000000/(RECORDING_TIME - LEARNING_TIME))*Peak_count;
  //Serial.println(BPM);

  //test: read from flash memory
  // for (int i = 0; i < SIZE; i++)
  // {
  //   float val = pgm_read_float(&(fil_IR_vec[i]));
  //   Serial.print(val);
  //   Serial.print(" , ");
  //   Serial.println(SSF_output[i]);
  // }

  delay(2000);
  MAX30100_Startup();     //Start up MAX30100 sensor
}

//-------------------------MAX30100 START UP-----------------------------------------
void MAX30100_Startup()
{
  //Serial.print("Initializing MAX30100..");
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply or wrong target chip
  if (!sensor.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
  //  Serial.println("SUCCESS");
  }
  //Serial.println("MAX30100 HR and SpO2 Sensor");
  sensor.setMode(MAX30100_MODE_SPO2_HR);          //setting sensor mode (HR and SpO2)
  sensor.setLedsCurrent(IR_current, RED_current); //Set led's current IR and Red respectively
  sensor.setLedsPulseWidth(Pulse_width);          //Set led's pulse width
  sensor.setSamplingRate(Sample_Rate);            //set sample rate
  sensor.setHighresModeEnabled(Highres_mode);     //set high resolution
  sensor.resetFifo();                             //rest fifo register 
}
//-------------------------Functions------------------------------------------------
//DC Removeral filter for Heart Rate (HR)
float DCR_function_IR(double raw_input, float alpha) 
{  
  float filtered = raw_input + alpha * prev_filtered_IR;
  float output_DCR = filtered - prev_filtered_IR;
  float DC_val = filtered;
  prev_filtered_IR = filtered;  
  //Serial.println(output_DCR); 
  return output_DCR, DC_val;
}
//DC Removeral filter for Sp02
float DCR_function_RED(double raw_input, float alpha) 
{  
  float filtered = raw_input + alpha * prev_filtered_RED;
  float output_DCR = filtered - prev_filtered_RED;
  float DC_val = filtered;
  prev_filtered_RED = filtered;  
  //Serial.println(output_DCR); 
  return output_DCR, DC_val;
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
  //Serial.println(avg);
  return avg - raw_input;
}
//Low pass filter (Butterworth filter)
float Butterworth_LPF_function(float raw_input) 
{
  //Second order low pass filter
  Val[0] = Val[1];
  Val[1] = Val[2];
  //Fs = 100Hz (sample rate) and Fc = 10Hz (cut-off frequency)
  //Val[2] = (6.745527388907189559e-2 * raw_input) + (-0.41280159809618854894 * Val[0]) + (1.14298050253990091107 * Val[1]);
  //Fs = 100Hz (sample rate) and Fc = 5Hz (cut-off frequency)
  Val[2] = (2.008336556421122521e-2 * raw_input) + (-0.64135153805756306422 * Val[0]) + (1.56101807580071816339 * Val[1]);
  //Fs = 100Hz (sample rate) and Fc = 3Hz (cut-off frequency)
  //Val[2] = (7.820208033497201908e-3 * raw_input) + (-0.76600660094326400440 * Val[0]) + (1.73472576880927520371 * Val[1]);
  float BWF_output = (Val[0] + Val[2]) + 2*Val[1];
  //Serial.println(BWF_output);  
  return BWF_output;
}
//----------------------------------------------------------------------------------

