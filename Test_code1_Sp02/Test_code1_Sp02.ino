// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// MAX30100 sensor:
#include <MAX30100.h>
MAX30100 sensor;
#define IR_current      MAX30100_LED_CURR_50MA          //define IR led current (50mA)
LEDCurrent RED_current = MAX30100_LED_CURR_24MA;        //define initial Red led current (50mA)
#define Sample_Rate     MAX30100_SAMPRATE_100HZ         //define sample rate (100Hz)
#define Pulse_width     MAX30100_SPC_PW_1600US_16BITS   //define led pulse width (1600)
#define Highres_mode    true                            //High resolution mode

//Heart Rate
#define WARM_UP_TIME 2000000              //Warm up time for MAX30100 sensor (2 seconds)
#define RECORDING_TIME 20000000           //Recording time for heart rate (20 seconds)
#define SIZE  RECORDING_TIME/10000        //Vector size equal recording time divided by 10
#define ALPHA_DCR    0.95                //DC filter alpha value for LED
#define SAMPLE_SIZE   100                 //Mean difference filter sample size used to calculate the running mean difference
#define ACCEPTABLE_CURRENT_DIFF   55000                   //Acceptable current difference between RED current and IR current

//LED currents:
const LEDCurrent LEDCurrent_array[] = {MAX30100_LED_CURR_0MA,         //1
                                       MAX30100_LED_CURR_4_4MA,       //2
                                       MAX30100_LED_CURR_7_6MA,       //3
                                       MAX30100_LED_CURR_11MA,        //4
                                       MAX30100_LED_CURR_14_2MA,      //5
                                       MAX30100_LED_CURR_17_4MA,      //6
                                       MAX30100_LED_CURR_20_8MA,      //7
                                       MAX30100_LED_CURR_24MA,        //8
                                       MAX30100_LED_CURR_27_1MA,      //9
                                       MAX30100_LED_CURR_30_6MA,      //10
                                       MAX30100_LED_CURR_33_8MA,      //11
                                       MAX30100_LED_CURR_37MA,        //12
                                       MAX30100_LED_CURR_40_2MA,      //13
                                       MAX30100_LED_CURR_43_6MA,      //14
                                       MAX30100_LED_CURR_46_8MA,      //15
                                       MAX30100_LED_CURR_50MA};       //16   

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
//Current balancing:
int counter = 8;
//-------Beat detection Variables------------

//------------------------------------------

void setup() {
  Serial.begin(115200);
  
  //Onboard LED:
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);

  //Temperature sensor:----------------------------
  //Serial.println("MLX90632 Tempurature Sensor");
  //Wire.begin();
  //myTempSensor.begin();
  //-----------------------------------------------
  
  //MAX30100 (HR and SpO2) senesor:----------------
  
  sensor.begin();
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
  float IR_DC_val = 0;
  float RED_DC_val = 0;
  bool Warm_up = false;                           //Warm up sensor
  float IR_Array[SIZE];                           //IR array to store IR signal value
  float RED_Array[SIZE];                          //RED array to store RED signal value
  bool IR_DC = false;                             //Return either DC value (true) or AC value (false)                 
  bool RED_DC = false;                            //Return either DC value (true) or AC value (false)
  
  while (delta_time <= Total_time)
  {     
    sensor.update();
    // if raw data is available 
    if (sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    {
      //-------------------------
      //HR:
      //-------------------------
      //Serial.print(raw_IR_Val);           
      //add filtering to raw values:
      IR_DC = false;
      IR_Array[i] = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);     //filter raw IR LED data through DC removal
      IR_Array[i] = MDF_function(IR_Array[i]);                            //mean difference filter IR LED data 
      IR_Array[i] = Butterworth_LPF_function(IR_Array[i]);                //low pass butterworth filter IR LED data
      //Get DC value from signal:
      IR_DC = true;
      IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       //Get DC value from IR signal
      //Serial.print(" | ");
      //Serial.println(IR_Array[i]); 
      //Serial.print(" | ");
      //Serial.println(micros());
      //-------------------------
      //Sp02:
      //-------------------------
      RED_DC = false;
      RED_Array[i] = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
      //Get DC value from signal
      RED_DC = true;
      RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
      //Serial.println(raw_RED_Val);
      //Serial.println(RED_Array[i]);
      //Serial.println(RED_DC_val);
      //RED and IR DC current balancing:
      if (IR_DC_val - RED_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current < MAX30100_LED_CURR_50MA)
      {
        counter++;
        RED_current = LEDCurrent_array[counter];          //change RED LED's current
        sensor.setLedsCurrent(IR_current, RED_current);   //Set led's current IR and Red respectively
        //Serial.print("hi");
      }
      if (RED_DC_val - IR_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current > 0)
      {
        counter--;
        RED_current = LEDCurrent_array[counter];          //change RED LED's current
        sensor.setLedsCurrent(IR_current, RED_current);   //Set led's current IR and Red respectively
        //Serial.print("hi");
      }
      //Serial.print(" , ");  
      //Serial.print(RED_DC_val); 
      //Serial.print(" , ");
      //Serial.println(IR_DC_val);
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
  //Test print:
  //Serial.print("-----------");
  //Serial.println(i);
  //-----------------------Processing raw values-----------------------------------
  //Slope Sum function (SSF):
  int w = 10;                                   //length of analyzing window 
  float SSF = 0;                                //summation in window period
  float SSF_output[SIZE];                       //SSF output
  for (int i = 0; i < SIZE; i++)
  {
    if (i <= w)
    {
      SSF_output[i] = 0;      
    }
    else
    {
      for (int x = w; x >= 0; x--)
      {
        SSF = 0;
        float delta_input = (IR_Array[i-w]) - (IR_Array[(i-w)-1]);
        if (delta_input > 0)
        {
          SSF += delta_input;          
        }
      }
      SSF_output[i] = SSF;
    }    
    //Serial.println(SSF_output[i]);    
  }
  //--------------------------------------------------------------------------------
  // Beat Detection:
  // Learning Period:    
  int Expected_beats = 40*RECORDING_TIME/60000000;            //Amount of expected beats in the recording with a 40BPM HR
  //Serial.println(Expected_beats);
  float Beat_array[Expected_beats];                           //Store peak values of expected beats
  float Peak = 0;                                             //Value of peak detected
  bool Peak_detected = false;                                 //When a peak is detected it becomes true otherwise false
  
  //Identifies the highest peaks of the of the number of expected peaks.
  for (int i = 1; i < SIZE; i++)
  {
    //Serial.println(SSF_output[i]);
    if(SSF_output[i-1] < SSF_output[i] && SSF_output[i] > SSF_output[i+1])
    { 
      Peak = SSF_output[i];
      Peak_detected = true;
      for (int j = Expected_beats - 1; j >= 0; j--)
      { 
        if (Peak > Beat_array[j] && Peak_detected == true)
        {
          for(int p = 1; p <= j; p++)
          {
            Beat_array[p-1] = Beat_array[p];
          }
          Beat_array[j] = Peak;
          Peak_detected = false;
        }
      }
    }
  }
  // Test result of beat array 
  //Serial.println("------------");
  //for(int x = 0; x < Expected_beats; x++)
  //{
  //  Serial.println(Beat_array[x]);
  //}

  //Processing Period:
  //Calculating threshold
  int Sum_beat_array = 0;                   //Total sum of the beat array.
  for(int i = 0; i < Expected_beats; i++)
  {
    Sum_beat_array += Beat_array[i];        //Total
  }
  int threshold = 0.7*(Sum_beat_array/Expected_beats);             //threshold value for beat detection

  // for(int i = 0; i < SIZE; i++)
  // {
  //   //Serial.print(",");
  //   Serial.print(SSF_output[i]);
  //   Serial.print(",");
  //   Serial.println(threshold);
  // }

  int Peak_count = 0;                         //Count number of peaks (Beats)
  int P2p_time_start = 0;                     //Initializing start time for peak to peak
  int P2p_time_end = 0;                       //Initializing end time for peak to peak 
  int Delta_P2p_time = 0;                     //Initializing delta time for peak to peak  
  for(int i = 0; i < SIZE; i++)
  {  
    if (SSF_output[i] > threshold)      //Count peaks above threshold (beats) 
    {
      if(SSF_output[i-1] < SSF_output[i] && SSF_output[i] > SSF_output[i+1])    //Peak detecting 
      {
        Peak_count++;
        float Peak_value = SSF_output[i];
        //Serial.print(Peak_value);         
        if (Peak_count > 0)
        {
          P2p_time_end = P2p_time_start;                     //ending time of peak to peak
        }
        P2p_time_start = micros();                           //starting time of peak to peak
        if (Peak_count > 0)
        {
          Delta_P2p_time = P2p_time_start - P2p_time_end;    //delta time between peak to peak
        }
        //Serial.println(Delta_P2p_time);
      }
    }
    //Serial.print(",");
    //Serial.print(SSF_output[i]);
    //Serial.print(",");
    //Serial.println(threshold);
  }

  // Beats per minute (BPM) calculation:
  int BPM = (60000000/RECORDING_TIME)*Peak_count;
  Serial.println(BPM);

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
float DCR_function_IR(double raw_input, float alpha, bool ret) 
{  
  float output_DCR = 0;
  float filtered = raw_input + alpha * prev_filtered_IR;
  float AC_val = filtered - prev_filtered_IR;             //AC value of signal
  float DC_val = filtered;                                //DC value of signal
  //Weather to return the ac or dc value:
  if (ret == true)
  {
    output_DCR = DC_val;
  } 
  else
  {
    prev_filtered_IR = filtered;
    output_DCR = AC_val;
  } 
  //Serial.println(output_DCR); 
  return output_DCR;
}
//DC Removeral filter for Sp02
float DCR_function_RED(double raw_input, float alpha, bool ret) 
{  
  float output_DCR = 0;
  float filtered = raw_input + alpha * prev_filtered_RED;
  float AC_val = filtered - prev_filtered_RED;            //AC value of signal
  float DC_val = filtered;                                //DC value of signal 
  //Weather to return the ac or dc value:
  if (ret == true)
  {
    output_DCR = DC_val;
  } 
  else
  {
    prev_filtered_RED = filtered; 
    output_DCR = AC_val;
  }
  //Serial.println(output_DCR); 
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

