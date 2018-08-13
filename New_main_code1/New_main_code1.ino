// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

//EEG Sensor 
#include <Brain.h>
// Set up the brain reader, pass it the software serial object you want to listen on.
Brain brain(Serial1);
// Brain variables:

// MAX30100 sensor
#include <MAX30100.h>
MAX30100 MAX30100_sensor;
//LED currents:
const LEDCurrent LEDCurrent_array[] = {MAX30100_LED_CURR_0MA,         //0
                                       MAX30100_LED_CURR_4_4MA,       //1
                                       MAX30100_LED_CURR_7_6MA,       //2
                                       MAX30100_LED_CURR_11MA,        //3
                                       MAX30100_LED_CURR_14_2MA,      //4
                                       MAX30100_LED_CURR_17_4MA,      //5
                                       MAX30100_LED_CURR_20_8MA,      //6
                                       MAX30100_LED_CURR_24MA,        //7
                                       MAX30100_LED_CURR_27_1MA,      //8
                                       MAX30100_LED_CURR_30_6MA,      //9
                                       MAX30100_LED_CURR_33_8MA,      //10
                                       MAX30100_LED_CURR_37MA,        //11
                                       MAX30100_LED_CURR_40_2MA,      //12
                                       MAX30100_LED_CURR_43_6MA,      //13
                                       MAX30100_LED_CURR_46_8MA,      //14
                                       MAX30100_LED_CURR_50MA};       //15 
LEDCurrent IR_current = LEDCurrent_array[15];			//define IR led current (50mA)
LEDCurrent RED_current = LEDCurrent_array[7];			//define initial Red led current (24mA)
#define Sample_Rate 	MAX30100_SAMPRATE_200HZ         //define sample rate (200Hz)
#define Pulse_width   	MAX30100_SPC_PW_800US_15BITS   //define led pulse width (800)
#define Highres_mode  true                            //High resolution mode
#define Warm_up 1500000
//Current Balancing:
#define ACCEPTABLE_CURRENT_DIFF   2500  // Acceptable current difference between RED current and IR current
int counter = 8;						            // Current array_ counter
//Filter Variables
//DC Removal variable:
#define SAMPLE_SIZE   100                 //Mean difference filter sample size used to calculate the running mean difference
#define ALPHA_DCR    0.95                 //DC filter alpha value for LED
double prev_filtered_IR = 0;     //HR
double prev_filtered_RED = 0;    //Sp02
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     // Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//Low pass filter (butterworth filter) variables:
float Val[3];


void setup() {
	// Start serial terminal:
  	Serial.begin(57600);
}

void loop() {
	
	//Current_Balancing();
	int recording_time = 10000000 + Warm_up;
	int Exp_Peaks = ((recording_time - Warm_up)/1000000)*0.5;	//Expected beat peaks if heart rate is 30 bpm
	HR_SpO2(recording_time, Exp_Peaks);
	Serial.println("end");

}

// Function to get heart rate value:
int HR_SpO2(int rec_time, int Expected_Peaks)
{
	// start-up the MAX30100 sensor
	MAX30100_Startup();					
	//Variables store raw RED and IR values
	uint16_t raw_IR_Val = 0;
	uint16_t raw_RED_Val = 0;
	// Recording required varabiles
	// Varabiles:
	int IR_array_size = rec_time/5000;				// IR's size of array_.
	int SSF_array_size = (rec_time-Warm_up)/5000;	// SSF's size of array_.
	float IR_AC_array[IR_array_size];      			// IR signal AC array_.
	float SSF_output[SSF_array_size];           	// SSF output array_.  	int i = 0;                        
	int i = 0;										//Counter
  	float IR_DC_val = 0;               				// DC value of the IR signal
  	float RED_DC_val = 0;              				// DC value of the RED signal
  	float IR_DC_val_SpO2 = 0;          				// DC value of the IR signal for SpO2 calculation
  	float RED_DC_val_SpO2 = 0;         				// DC value of the RED signal for SpO2 calculation
  	float Sum_AC_IR = 0;               				// Sum of the IR AC signal value
  	float Sum_AC_RED = 0;              				// Sum of the RED AC signal value
  	int delta_rec = 0;				      			// delta time between current and start time
  	int start_rec = micros();		  				// start record time
  	// while loop to record 5 seconds of data:
	while(delta_rec <= rec_time)
    {
      // if raw data is available 
      MAX30100_sensor.update();   // Update sensor
      if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
      {
        // IR Signal:
        bool IR_DC = false;	//Return either DC value (true) or AC value (false)  
        IR_AC_array[i] = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);         //filter raw IR LED data through DC removal
        //Calculating AC RMS value: (only after 100 iterations - remove noise)
        if (i > 300 && i <= 700)
        {
          Sum_AC_IR += pow((IR_AC_array[i]),2);                                   //Sum of the IR AC signal value
        }
        //Add filtering to raw values:
        IR_AC_array[i] = MDF_function(IR_AC_array[i]);                          //mean difference filter IR LED data 
        IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);              //low pass butterworth filter IR LED data
        //Get DC value from signal:
        IR_DC = true;
        IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);        //Get DC value from IR signal
        if (i == 700)
        {
          IR_DC_val_SpO2 = IR_DC_val;
        }
        // Test Print:
        //Serial1.print(IR_DC_val);
        //Serial1.print(" , ");
        // RED Signal:
        bool RED_DC = false;	//Return either DC value (true) or AC value (false)  
        float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
        //Calculating AC RMS value: (only after 50 iterations - remove noise)
        if (i > 300 && i <= 700)
        {
          Sum_AC_RED += pow((RED_AC_value),2);                                    //Sum of the RED AC signal value
        }
        //Get DC value from signal
        RED_DC = true;
        RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
        if (i == 700)
        {
          RED_DC_val_SpO2 = RED_DC_val;
        }
        // Test Print:
        //Serial1.println(RED_DC_val);
        if (i >= IR_array_size)
        {
          break;
        }
        // Test Print:
        //Serial.print(IR_AC_array[i]);
        //Serial.print(",");
        //Serial.println(i);
        i++;
      }
      delta_rec = micros() - start_rec;				// delta time calculation 30 seconds 
  	}
  	// Shut down max30100 sensor:
  	MAX30100_sensor.shutdown();       // Shutdown MAX30100 sensor
  	// Start data processing: 

  	// Slope Sum Function (SSF):
    int window = 20;         //length of analyzing window
    int j = 0;               //new counter
    float SSF = 0;           //summation in window period
    for (int i = 300; i < IR_array_size; i++)
    {
      if (i <= (window+300))
      {
        SSF_output[j] = 0;
      }
      else
      {
        for (int x = window; x >= 0; x--)
        {
          SSF = 0;
          float delta_input = (IR_AC_array[i-window]) - (IR_AC_array[(i-window)-1]);
          if (delta_input > 0)
          {
            SSF += delta_input;          
          }
        }
        SSF_output[j] = SSF;
      } 
      //Test print:
      //Serial.print(IR_AC_array[i]);
      //Serial.print(",");
      //Serial.println(SSF_output[j]);
      //Serial.print(",");
      //Serial.println(j);
      j++;
    }
    
    //Identifies the highest peaks of the of the number of expected peaks.
    float Beat_array[Expected_Peaks];     // Store detected peaks in array_
    float Peak = 0;                       // Vale of peak detected
    bool Peak_detected = false;           // When a peak is detected it becomes true otherwise false
    for (int i = 0; i < SSF_array_size; i++)
    {
      if(SSF_output[i-1] < SSF_output[i] && SSF_output[i] > SSF_output[i+1])
      { 
        Peak = SSF_output[i];
        Peak_detected = true;
        for (int j = Expected_Peaks - 1; j >= 0; j--)
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
    //Calculating threshold
    int Sum_beat_array = 0;                   //Total sum of the beat array_.
    int threshold = 0;
    for(int i = 0; i < Expected_Peaks; i++)
    {
      Sum_beat_array += Beat_array[i];        //Total
    }
    threshold = 0.65*(Sum_beat_array/Expected_Peaks);  //threshold value for beat detection

    // Test print: 
    // for(int i = 0; i < SSF_array_size; i++)
    // {
    //   Serial.print(SSF_output[i]);
    //   Serial.print(",");
    //   Serial.println(threshold);
    // } 

}

void Current_Balancing()
{
	//Variables store raw RED and IR values
	uint16_t raw_IR_Val = 0;
	uint16_t raw_RED_Val = 0;
	int delta_5s = 0;					// delta time between current and start time
  	int start_5s = micros();			// start 5 second current balancing
  	while(delta_5s <= 5000000)
  	{
  		delta_5s = micros() - start_5s; 				// delta time calculation 3 seconds
  		// if raw data is available 
  		MAX30100_sensor.update();		// Update sensor
    	if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    	{
  	    	//Get DC value from IR signal:
  			bool IR_DC = true;	//Return either DC value (true) or AC value (false)
  			float IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       //Get DC value from IR signal

  	    	//Get DC value from RED signal:
  			bool RED_DC = true; 	//Return either DC value (true) or AC value (false)
  	    	float RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
  	
  	    	//RED and IR DC current balancing:
  			if (IR_DC_val - RED_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current < MAX30100_LED_CURR_50MA)
  			{
    			counter++;
    			RED_current = LEDCurrent_array[counter];          			              //change RED LED's current
    			MAX30100_sensor.setLedsCurrent(IR_current, RED_current);   	          //Set led's current IR and Red respectively
    		}
  			if (RED_DC_val - IR_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current > 0)
  			{
    			counter--;
    			RED_current = LEDCurrent_array[counter];          			              //change RED LED's current
    			MAX30100_sensor.setLedsCurrent(IR_current, RED_current);   	          //Set led's current IR and Red respectively
    		}
    		// Test print: 
  			//Serial1.print(RED_DC_val); 
  			//Serial1.print(" , ");
  			//Serial1.println(IR_DC_val);
  		}
  	}
}


//-------------------------MAX30100 START UP-----------------------------------------
void MAX30100_Startup()
{
  //Serial1.print("Initializing MAX30100..");
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply or wrong target chip
  	if (!MAX30100_sensor.begin()) {
    	Serial1.println("FAILED");
    	for (;;);
  	}
  	//Serial1.println("MAX30100 HR and SpO2 Sensor");
  	MAX30100_sensor.setMode(MAX30100_MODE_SPO2_HR);          //setting sensor mode (HR and SpO2)
  	MAX30100_sensor.setLedsCurrent(IR_current, RED_current); //Set led's current IR and Red respectively
  	MAX30100_sensor.setLedsPulseWidth(Pulse_width);          //Set led's pulse width
  	MAX30100_sensor.setSamplingRate(Sample_Rate);            //set sample rate
  	MAX30100_sensor.setHighresModeEnabled(Highres_mode);     //set high resolution
  	MAX30100_sensor.resetFifo();                             //rest fifo register 

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
  //Serial1.println(output_DCR); 
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
  //Serial1.println(output_DCR); 
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
  //Serial1.println(avg);
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
  //Val[2] = (2.008336556421122521e-2 * raw_input) + (-0.64135153805756306422 * Val[0]) + (1.56101807580071816339 * Val[1]);
  //Fs = 100Hz (sample rate) and Fc = 3Hz (cut-off frequency)
  Val[2] = (7.820208033497201908e-3 * raw_input) + (-0.76600660094326400440 * Val[0]) + (1.73472576880927520371 * Val[1]);
  float BWF_output = (Val[0] + Val[2]) + 2*Val[1];
  //Serial1.println(BWF_output);  
  return BWF_output;
}
//----------------------------------------------------------------------------------