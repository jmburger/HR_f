// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// Temperature Sensor
#include <SparkFun_MLX90632_Arduino_Library.h>
MLX90632 temp_sensor;

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
#define Sample_Rate 	MAX30100_SAMPRATE_100HZ         //define sample rate (100Hz)
#define Pulse_width   MAX30100_SPC_PW_1600US_16BITS   //define led pulse width (1600)
#define Highres_mode  true                            //High resolution mode

//Variables store raw RED and IR values
uint16_t raw_IR_Val = 0;
uint16_t raw_RED_Val = 0;
//Filter Variables
//DC Removal variables:
#define SAMPLE_SIZE   100                 //Mean difference filter sample size used to calculate the running mean difference
#define ALPHA_DCR    0.95                 //DC filter alpha value for LED
bool IR_DC = false;                             //Return either DC value (true) or AC value (false)                 
bool RED_DC = false;                            //Return either DC value (true) or AC value (false)
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
#define ACCEPTABLE_CURRENT_DIFF   2500 //Acceptable current difference between RED current and IR current
int counter = 8;						// Current array counter
bool Current_balaning = true;			// Set true when current balancing needs to take place. 

// Timer variables:
int time_10s = micros(); 		// time 10 seconds
int time_60s = micros();		// time 1 minute
int time_300s = micros();		// time 5 minutes

// Flags for timer:
bool time_flag = false;
bool recording = false;        //indicate which recording is taking place true = 5's and false =30's
bool Data_available = false;   //indicate that data is available for processing
void setup() 
{
	Serial.begin(115200);

  // Temperature sensor:
	//Wire.begin();
	//temp_sensor.begin();					// Start sensor
	//temp_sensor.setMode(MODE_SLEEP);		// Put sensor in sleep mode 

	// MAX30100 sensor:
	MAX30100_sensor.begin();
	//MAX30100_Startup();		//Start-up the MAX30100 sensor
}

void loop() 
{
  // Varabiles:
  float IR_AC_array[3000];           // IR signal AC array_.
  float SSF_output[3000];            //SSF output array_.
  // Current balancing for Sp02 calculation:
  // This balancing is going to take place every 5 minutes:
  if (Current_balaning == true)
  {
  	MAX30100_Startup();					// start-up the MAX30100 sensor
  	// Warm Up:
  	int delta_warmup = 0;				// delta time of warm up
  	int start_warmup = micros();		// start time of warm up (1,5 second)
  	while(delta_warmup <= 1500000)		
  	{
      MAX30100_sensor.update();   // Update sensor
  		delta_warmup = micros() - start_warmup;		// delta time of warm up
  	}
  	int delta_3s = 0;					// delta time between current and start time
  	int start_3s = micros();			// start 3 second current balancing
  	while(delta_3s <= 3000000)
  	{
  		delta_3s = micros() - start_3s; 				// delta time calculation 3 seconds
  		// if raw data is available 
  		MAX30100_sensor.update();		// Update sensor
    	if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    	{
  	    //Get DC value from IR signal:
  			IR_DC = true;
  			float IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       //Get DC value from IR signal

  	    //Get DC value from RED signal:
  			RED_DC = true;
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
  			//Serial.print(RED_DC_val); 
  			//Serial.print(" , ");
  			//Serial.println(IR_DC_val);
  		}
  	}
  	Current_balaning = false;			// Current balancing complete
  	MAX30100_sensor.shutdown();				// Shutdown MAX30100 sensor
  }
  //Every 10 seconds record 5's of HR and Sp02:
  int time_10s_micro = micros();
  if (time_10s_micro - time_10s >= 10000000 && Current_balaning == false)
  {
    recording = true;       // data recording 5's is taking place 
  	time_10s = micros();		// redefine time
  	// insert code below:
  	MAX30100_Startup();					// start-up the MAX30100 sensor
  	// Warm Up:
  	int delta_warmup = 0;				// delta time of warm up
  	int start_warmup = micros();		// start time of warm up (1,5 second)
  	while(delta_warmup <= 1500000)		
  	{
      MAX30100_sensor.update();   // Update sensor
  		delta_warmup = micros() - start_warmup;		// delta time of warm up
  	}
  	//Serial.println("10 seconds");	  // test print
    // 5 seconds recording required varabiles
  	int i = 0;                        //Counter
  	float Sum_AC_IR = 0;				      //Sum of the IR AC signal value
  	float Sum_AC_RED = 0;				      //Sum of the RED AC signal value
  	int delta_rec_5s = 0;				      // delta time between current and start time
  	int start_rec_5s = micros();		  // start record time 5 seconds
  	// while loop to record 5 seconds of data:
  	while(delta_rec_5s <= 5300000)
  	{
  		// if raw data is available 
  	  MAX30100_sensor.update();		// Update sensor
  		if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
  		{
  		  // IR Signal:
  			IR_DC = false;
  			IR_AC_array[i] = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);     	  //filter raw IR LED data through DC removal
  			//Calculating AC RMS value:
  			Sum_AC_IR += pow((IR_AC_array[i]),2);                                   //Sum of the IR AC signal value
  			//Add filtering to raw values:
  			IR_AC_array[i] = MDF_function(IR_AC_array[i]);                         	//mean difference filter IR LED data 
  			IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);             	//low pass butterworth filter IR LED data
        //Get DC value from signal:
  			IR_DC = true;
  			float IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       	//Get DC value from IR signal

  			// RED Signal:
  			RED_DC = false;
  			float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
  			//Calculating AC RMS value:
  			Sum_AC_RED += pow((RED_AC_value),2);                                   	//Sum of the RED AC signal value
  			//Get DC value from signal
  			RED_DC = true;
  			float RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
        
        i++;
        if (i >= 550)
        {
          break;
        }
  			// Test Print:
  			//Serial.println(IR_AC_array[i]);
        //Serial.println(SSF_output[i]);
        //Serial.println(i);
  		}
      delta_rec_5s = micros() - start_rec_5s;         // delta time calculation 5 seconds
  	}
    // Test print
    //Serial.println(i);
  	//Serial.println("exit while loop 5s");	// test print
  	MAX30100_sensor.shutdown();				// Shutdown MAX30100 sensor	
    Data_available = true;            // Data needs processing
  }

  //Every minute take a temperature reading:
  // int time_60s_micro = micros();
  // if (time_60s_micro - time_60s >= 60000000 && Current_balaning == false)
  // {
  // 	time_60s = micros();		// redefine time
  // 	//insert code below:
  // 	//Serial.println("60 seconds");		// test print
  // 	//float Core_body_temp = temp_sensor.getObjectTemp();			// Get core body temperature
  // 	//Serial.println(Core_body_temp);								// print core body temperature ever 1 minute.
  // }

  // //Every 5 minutes record 30's of HR and Sp02 for RR and B2B:
  int time_300s_micro = micros();
  if (time_300s_micro - time_300s >= 300000000 && Current_balaning == false)
  {
    recording = false;       // data recording 30's is taking place
  	time_300s = micros();		// redefine time
    // insert code below:
    MAX30100_Startup();         // start-up the MAX30100 sensor
    // Warm Up:
    int delta_warmup = 0;       // delta time of warm up
    int start_warmup = micros();    // start time of warm up (1,5 second)
    while(delta_warmup <= 1500000)    
    {
      MAX30100_sensor.update();   // Update sensor
      delta_warmup = micros() - start_warmup;   // delta time of warm up
    }
  	//Serial.println("5 minute");	// test print
    // 30 seconds recording required varabiles
    int i = 0;                  //Counter
    float Sum_AC_IR = 0;        //Sum of the IR AC signal value
    float Sum_AC_RED = 0;       //Sum of the RED AC signal value
  	int delta_rec_30s = 0;			//delta time between current and start time
  	int start_rec_30s = micros();	// start record time 30 seconds
  	// While loop to record 30 seconds of data:
  	while(delta_rec_30s <= 30300000)
    {
      // if raw data is available 
      MAX30100_sensor.update();   // Update sensor
      if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
      {
        // IR Signal:
        IR_DC = false;
        IR_AC_array[i] = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);         //filter raw IR LED data through DC removal
        //Calculating AC RMS value:
        Sum_AC_IR += pow((IR_AC_array[i]),2);                                   //Sum of the IR AC signal value
        //Add filtering to raw values:
        IR_AC_array[i] = MDF_function(IR_AC_array[i]);                          //mean difference filter IR LED data 
        IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);              //low pass butterworth filter IR LED data
        //Get DC value from signal:
        IR_DC = true;
        float IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);        //Get DC value from IR signal

        // RED Signal:
        RED_DC = false;
        float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
        //Calculating AC RMS value:
        Sum_AC_RED += pow((RED_AC_value),2);                                    //Sum of the RED AC signal value
        //Get DC value from signal
        RED_DC = true;
        float RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
        
        i++;
        if (i >= 3050)
        {
          break;
        }
        // Test Print:
        //Serial.println(IR_AC_array[i]);
        //Serial.println(SSF_output[i]);
        //Serial.println(i);
      }
      delta_rec_30s = micros() - start_rec_30s;				// delta time calculation 30 seconds 
  	}
    // Test print:
    //Serial.println(i);
  	//Serial.println("exit while loop 30s");	// test print
    MAX30100_sensor.shutdown();       // Shutdown MAX30100 sensor 
  	Current_balaning = true;					// Balance current again
    Data_available = true;            // Data needs processing 
  }

  // Data processing:
  if (Data_available == true)
  {
    // Recording size:
    int size = 550;
    if (recording == false)
    {
      size = 3050;
    }
    // Slope Sum Function (SSF):
    int window = 10;         //length of analyzing window
    float SSF = 0;           //summation in window period
    for (int i = 50; i < size; i++)
    {
      if (i <= (window+50))
      {
        SSF_output[i] = 0;      
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
        SSF_output[i] = SSF;
      } 
      //Test print:
      Serial.println(SSF_output[i]);
    }
    Data_available = false;       // Data has been processed
  }
}

//-------------------------MAX30100 START UP-----------------------------------------
void MAX30100_Startup()
{
  //Serial.print("Initializing MAX30100..");
  // Initialize the sensor
  // Failures are generally due to an improper I2C wiring, missing power supply or wrong target chip
  if (!MAX30100_sensor.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
  //  Serial.println("SUCCESS");
  }
  //Serial.println("MAX30100 HR and SpO2 Sensor");
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
