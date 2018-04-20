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
#define WARM_UP_TIME 2000000            //Warm up time for MAX30100 sensor (2 seconds)
#define RECORDING_TIME  20000000         //Recording time for heart rate (20 seconds)
#define SIZE  RECORDING_TIME/10000      //Vector size equal recording time divided by 10
#define ALPHA_DCR     0.95              //DC filter alpha value
#define SAMPLE_SIZE   100               //Mean difference filter sample size used to calculate the running mean difference
#define PEAK_PERIOD   3                 //Length of peak period 
#define ALPHA_ATF     0.5               //Adaptive threshold function (ATF)
#define BEAT_WINDOW   10                //Number of beats in beat window
#define BEAT_ALPHA    0.7               //Reject fasle peaks time bound (Value for Andre)

//Variables store raw RED and IR values
uint16_t raw_IR_Val = 0;
uint16_t raw_RED_Val = 0;
float Filtered_IR_vec[SIZE]; //filtered IR vector for x sec
//-------------FILTER Variables-------------
//DC Removal variables:
double prev_filtered = 0;
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     // Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//Low pass filter (butterworth filter) variables:
float Val[3];
//-------Beat detection Variables------------
 //Adaptive threshold initial value:
int ATF_initial = 10;
uint8_t Peak_index = 0;                       //Adaptive threshold index 
float Peak_Hieght[PEAK_PERIOD];               //Store peak period's hieght values 
float ATF[SIZE];                              //ATF output vector  
//Time between beats: 
uint8_t Beat_index = 0;                       //Beat time index
int Delta_beat_time[BEAT_WINDOW];             //Delat time between peaks vector 
int Avg_BT = 0;                               //Average beat time over recorded time
bool Warm_up = false;                      
//------------------------------------------

void setup() {
  Serial.begin(115200);

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
  while (delta_time <= Total_time)
  {     
    sensor.update();
    // if raw data is available 
    if (sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    {
      //Serial.print(raw_IR_Val);           
      //add filtering to raw values:
      Filtered_IR_vec[i] = DCR_function(raw_IR_Val, ALPHA_DCR);
      Filtered_IR_vec[i] = MDF_function(Filtered_IR_vec[i]);
      Filtered_IR_vec[i] = Butterworth_LPF_function(Filtered_IR_vec[i]);
      //Serial.print(" | ");
      //Serial.println(Filtered_IR_vec[i]); 
      //Serial.print(" | ");
      //Serial.println(micros()); 
      i++;
    }
    //Warm up sensor:
    if (delta_time >= WARM_UP_TIME && Warm_up != true)
    {
      i=0;
      Warm_up = true;
    }
    delta_time = micros() - start_time;   //update remaining time 
  }
  sensor.shutdown();                      // Shut down MAX30100 sensor
  //fill rest of vector vector with zeros
  //Serial.println(i);
  while (i <= SIZE)                       
  {
    Filtered_IR_vec[i] = 0;
    i++;
  }
  //-----------------------Processing raw values-----------------------------------
  // Slope Sum function (SSF):
  int w = 10;                                   //length of analyzing window 
  float SSF = 0;                                //summation in window period
  float SSF_output[SIZE];                       //SSF output vector
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
        float delta_input = Filtered_IR_vec[i-w] - Filtered_IR_vec[(i-w)-1];
        if (delta_input > 0)
        {
          SSF += delta_input;          
        }
      }
      SSF_output[i] = SSF;
    }    
    //Serial.println(SSF_output[i]);    
  }
  // Beat Detection:
  int Peak_number = 0;                        //Count number of peaks(heart beats)
  int Beat_time = 0;                          //Beat time value
  int End_beat_time = 0;                      
  int Total_delta_beat_time = 0;              //Total beat time over recording period 
  float Avg_BT = 0;                           //Current average beat time 
  int Current_Delta_BT =0;  
  bool Peak_detected = false;                 //If peak is detected = true
  for (int i = 0; i < SIZE; i++)
  {
    if (Peak_number > 1 && Peak_detected == true)
    {
      //Calculating time between beats:
      Delta_beat_time[Beat_index] = (Beat_time - End_beat_time);       //Time between beats 
      Current_Delta_BT = Delta_beat_time[Beat_index];        
      Beat_index++;
      Beat_index = Beat_index % BEAT_WINDOW;
      Peak_detected = false;                                           //Peak detection finished 
    }

    if (i > 3 && SSF_output[i] > ATF[i-1] && Current_Delta_BT >= BEAT_ALPHA*Avg_BT && SSF_output[i-2] < SSF_output[i-1] && SSF_output[i-1] >= SSF_output[i])
    {
      Peak_Hieght[Peak_index] = SSF_output[i];
      Serial.print(SSF_output[i]);      
      Peak_index++;
      Peak_index = Peak_index % PEAK_PERIOD;
      Peak_number++; 
      End_beat_time = Beat_time;                                  //Ending time of beat
      Beat_time = micros();                                           //Starting time of beat
      Peak_detected = true;                                           //New peak has been detected
    }    
    if (Peak_number > BEAT_WINDOW && Peak_detected == true)
    {
      int Avg_beat_time = 0;                      //Average delat beat time of beat period      
      for (int a = 0; a < BEAT_WINDOW; a++)
      {
        Avg_beat_time += Delta_beat_time[a];                  //Sum beat windows delta beat times
      }       
      Avg_BT = Avg_beat_time/BEAT_WINDOW;                     //Average delta beat time  
      //Serial.println(Avg_BT);       
    }
    //Adaptive threshold on past three peak values:
    if (Peak_number < PEAK_PERIOD)
    {
      ATF[i] = ATF_initial;                           //Intial value of adaptive threshold      
    }
    if (Peak_number >= PEAK_PERIOD)
    {      
      float Peak_tot = 0;
      for (int a = 0; a < PEAK_PERIOD; a++)
      {
        Peak_tot += Peak_Hieght[a];
      }
      if (Peak_tot < PEAK_PERIOD)                     //Not to divide with a number smaller than the peak period
      {
        ATF[i] = ATF[i-1];                            //eliminate high values during start up
      }
      else 
      {
        ATF[i] = ALPHA_ATF*(Peak_tot/PEAK_PERIOD);    //new adaptive threshold
      }
      if (ATF[i] > ATF_initial*3)                     //Eliminate high peaks
      {
        ATF[i] = ATF_initial;
      }   
    }
    Serial.print(",");
    Serial.print(SSF_output[i]); 
    Serial.print(",");
    Serial.println(ATF[i]);
  }
    //Beats per minutes:
/*  int BPM = (60000000/RECORDING_TIME)*Peak_number;      

  //Heart rate varability: (CHECK CHANGE ALOT)
  for (int a = 0; a < Peak_number; a++)
  {
    Total_delta_beat_time += Delta_beat_time[a];      //Calculating total beat time over recording period
  }
  float HR_Varabilirty = Total_delta_beat_time/Peak_number;            //Calculating average beat time (peak to peak) over the recording period*/
  //ATF_initial = ATF[SIZE-1];   //Redefine initial adaptive threshold value
  //Serial.println(Peak_number);
  //delay(5000);  
/*  for (int i = 0; i < SIZE; i++)
  {
    Serial.println(SSF_output[i]); 
    Serial.println(",");
    Serial.println(ATF[i]);
  }
  Serial.println("end");*/
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
//DC Removeral filter
float DCR_function(double raw_input, float alpha) 
{  
  float filtered = raw_input + alpha * prev_filtered;
  float output_DCR = filtered - prev_filtered;
  prev_filtered = filtered;  
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

