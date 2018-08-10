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
int Brain_prev = 0;
uint8_t signal_strength = 0;    
uint8_t attention = 0;
uint8_t meditation = 0;

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
int BPM = 0;
float SpO2 = 0;

//Current balancing:
#define ACCEPTABLE_CURRENT_DIFF   2500  // Acceptable current difference between RED current and IR current
int counter = 8;						            // Current array_ counter
bool Current_balaning = true;			      // Set true when current balancing needs to take place. 

// Timer variables:
int time_10s = micros(); 		// time 10 seconds
int time_60s = micros();		// time 1 minute
int time_300s = micros();		// time 5 minutes

// RR variables:
int RR_count = 0;
int RR = 0;

// HRV variables
int HRV_score = 0;

// Temperature sensor (thermistor)
#define THERMISTORPIN A0            // which analog pin to connect
#define THERMISTORNOMINAL 100000    // resistance at 25 degrees C 
#define TEMPERATURENOMINAL 25       // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 50               // how many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 4036           // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 100000       // the value of the 'other' resistor
uint16_t samples[NUMSAMPLES];       // Temperature variables
float Core_body_temp = 0;

// Flags for timer:
bool time_flag = false;
bool recording = false;        //indicate which recording is taking place true = 5's and false =30's
bool Data_available = false;   //indicate that data is available for processing
bool Startup = true;           //on start up
bool Start_warm = false;       //start up warmup flag

void setup() 
{
	Serial1.begin(57600);
  Serial.begin(115200);

  // MAX30100 sensor:
  MAX30100_sensor.begin();   // Warm Up:
  MAX30100_Startup();   //Start-up the MAX30100 sensor
  int delta_warmup = 0;       // delta time of warm up
  int start_warmup = micros();    // start time of warm up (1,5 second)
  while(delta_warmup <= 3000000)    
  {
    MAX30100_sensor.update();   // Update sensor
    delta_warmup = micros() - start_warmup;   // delta time of warm up
  }
  MAX30100_sensor.shutdown();       // Shutdown MAX30100 sensor 
	
}

void loop() 
{
  if (Start_warm == false){
    Serial1.println("VITALTRAC:");
    Serial1.println("=========");
    Serial1.println("");
    Serial1.println("Warming Up.....");
    Serial1.println("");
    Start_warm = true;
  }
  
  // Varabiles:
  float IR_AC_array[3050];           // IR signal AC array_.
  float SSF_output[3050];            //SSF output array_.
  // Recording varibles:
  float IR_DC_val = 0;               // DC value of the IR signal
  float RED_DC_val = 0;              // DC value of the RED signal
  float IR_DC_val_SpO2 = 0;          // DC value of the IR signal for SpO2 calculation
  float RED_DC_val_SpO2 = 0;         // DC value of the RED signal for SpO2 calculation
  float Sum_AC_IR = 0;               // Sum of the IR AC signal value
  float Sum_AC_RED = 0;              // Sum of the RED AC signal value
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
  			IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       //Get DC value from IR signal

  	    //Get DC value from RED signal:
  			RED_DC = true;
  	    RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
  	
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
  	Current_balaning = false;			// Current balancing complete
  	MAX30100_sensor.shutdown();				// Shutdown MAX30100 sensor
  }
  //Every 10 seconds record 5's of HR and Sp02:
  int time_10s_micro = micros();
  if (time_10s_micro - time_10s >= 10000000 && Current_balaning == false && Startup == false)
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
  	//Serial1.println("10 seconds");	  // test print
    // 5 seconds recording required varabiles
  	int i = 0;                        //Counter
  	Sum_AC_IR = 0;				            //Sum of the IR AC signal value
  	Sum_AC_RED = 0;				            //Sum of the RED AC signal value
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
  			//Calculating AC RMS value: (only after 50 iterations - remove noise)
        if (i > 100 && i <= 500)
        {
          Sum_AC_IR += pow((IR_AC_array[i]),2);                                   //Sum of the IR AC signal value
        }
        //Add filtering to raw values:
  			IR_AC_array[i] = MDF_function(IR_AC_array[i]);                         	//mean difference filter IR LED data 
  			IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);             	//low pass butterworth filter IR LED data
        //Get DC value from signal:
  			IR_DC = true;
  			IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       	//Get DC value from IR signal
        if (i == 500)
        {
          IR_DC_val_SpO2 = IR_DC_val;
        }
        // Test Print:
        //Serial1.print(IR_DC_val);
        //Serial1.print(" , ");
  			// RED Signal:
  			RED_DC = false;
  			float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
  			//Calculating AC RMS value: (only after 50 iterations - remove noise)
  			if (i > 100 && i <= 500)
        {
          Sum_AC_RED += pow((RED_AC_value),2);                                   	//Sum of the RED AC signal value
  			}
        //Get DC value from signal
  			RED_DC = true;
  			RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
        if (i == 500)
        {
          RED_DC_val_SpO2 = RED_DC_val;
        }
        // Test Print:
        //Serial1.println(RED_DC_val);
        i++;
        if (i >= 550)
        {
          break;
        }
  		// Test Print:
  		//Serial1.println(IR_AC_array[i]);
        //Serial1.println(SSF_output[i]);
        //Serial1.println(i);
  		}
      delta_rec_5s = micros() - start_rec_5s;         // delta time calculation 5 seconds
  	}
    // Test print
    //Serial1.println(i);
  	//Serial1.println("exit while loop 5s");	// test print
  	MAX30100_sensor.shutdown();				// Shutdown MAX30100 sensor	
    Data_available = true;            // Data needs processing
  }

  //Every minute take a temperature reading:
  int time_60s_micro = micros();
  if (time_60s_micro - time_60s >= 60000000 && Current_balaning == false || Startup == true)
  {
    uint8_t i;
    float average;
 
    // take N samples in a row, with a slight delay
    for (i=0; i< NUMSAMPLES; i++) {
     samples[i] = analogRead(THERMISTORPIN);
     delay(10);
     //Serial.println(samples[i]);    //test print
    }
    
    // average all the samples out
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
       average += samples[i];
    }
    average /= NUMSAMPLES;
   
    //Test print:
    //Serial1.print("Average analog reading "); 
    //Serial1.println(average);
   
    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    //Test print:
    //Serial1.print("Thermistor resistance "); 
    //Serial1.println(average);
   
    Core_body_temp = average / THERMISTORNOMINAL;     // (R/Ro)
    Core_body_temp = log(Core_body_temp);                  // ln(R/Ro)
    Core_body_temp /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    Core_body_temp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    Core_body_temp = 1.0 / Core_body_temp;                 // Invert
    Core_body_temp -= 273.15;                         // convert to C 
   
    //Test print:
    //Serial1.print("Temperature "); 
    //Serial1.print(Core_body_temp);
    //Serial1.println(" *C");
  }

  // //Every 5 minutes record 30's of HR and Sp02 for RR and HRV:
  int time_300s_micro = micros();
  if (time_300s_micro - time_300s >= 300000000 && Current_balaning == false || Startup == true)
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
  	//Serial1.println("5 minute");	// test print
    // 30 seconds recording required varabiles
    int i = 0;                  //Counter
    Sum_AC_IR = 0;              //Sum of the IR AC signal value
    Sum_AC_RED = 0;             //Sum of the RED AC signal value
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
        //Calculating AC RMS value: (only after 50 iterations - remove noise)
        if (i > 100 && i <= 500)
        {
          Sum_AC_IR += pow((IR_AC_array[i]),2);                                   //Sum of the IR AC signal value
        }
        //Add filtering to raw values:
        IR_AC_array[i] = MDF_function(IR_AC_array[i]);                          //mean difference filter IR LED data 
        IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);              //low pass butterworth filter IR LED data
        //Get DC value from signal:
        IR_DC = true;
        IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);        //Get DC value from IR signal
        if (i == 500)
        {
          IR_DC_val_SpO2 = IR_DC_val;
        }
        // Test Print:
        //Serial1.print(IR_DC_val);
        //Serial1.print(" , ");
        // RED Signal:
        RED_DC = false;
        float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);  //filter raw RED LED data through DC removal
        //Calculating AC RMS value: (only after 50 iterations - remove noise)
        if (i > 100 && i <= 500)
        {
          Sum_AC_RED += pow((RED_AC_value),2);                                    //Sum of the RED AC signal value
        }
        //Get DC value from signal
        RED_DC = true;
        RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    //Get DC value from RED signal
        if (i == 500)
        {
          RED_DC_val_SpO2 = RED_DC_val;
        }
        // Test Print:
        //Serial1.println(RED_DC_val);
        i++;
        if (i >= 3050)
        {
          break;
        }
        // Test Print:
        //Serial1.println(IR_AC_array[i]);
        //Serial1.println(SSF_output[i]);
        //Serial1.println(i);
      }
      delta_rec_30s = micros() - start_rec_30s;				// delta time calculation 30 seconds 
  	}
    // Test print:
    //Serial1.println(i);
  	//Serial1.println("exit while loop 30s");	// test print
    MAX30100_sensor.shutdown();       // Shutdown MAX30100 sensor 
    // Initial startup does  not require to do current balancing again:
    if (Startup == true)
    {
      Current_balaning = false;
    }
    else
    {
      Current_balaning = true;          // Balance current again
    }
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
    int window = 5;         //length of analyzing window
    int j = 0;               //new counter
    float SSF = 0;           //summation in window period
    for (int i = 50; i < size; i++)
    {
      if (i <= (window+50))
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
      j++;
      //Test print:
      //Serial1.println(SSF_output[j]);
    }

    // Peak Detection:
    // Learning period:
    // Recording size:
    size = 500;
    float factor_ = 0.81;                 // Better the BPM accuracy
    int FT = 12;                          // Time multiply factor 5 seconds to 60 seconds
    int expected_peaks = 3;               // Expected peaks/beats if heart rate is lower than 40 BPM
    if (recording == false)
    {
      //Test print:
      //Serial1.println("Hi");
      factor_ = 0.95;                     // Better the BPM accuracy
      size = 3000;
      FT = 2;                             // Time multiply factor 30 seconds to 60 seconds
      expected_peaks = 20;                // Expected peaks/beats if heart rate is lower than 40 BPM
    }
    float Beat_array[expected_peaks];     // Store detected peaks in array_
    float Peak = 0;                       // Vale of peak detected
    bool Peak_detected = false;           // When a peak is detected it becomes true otherwise false
    //Identifies the highest peaks of the of the number of expected peaks.
    for (int i = 0; i < size; i++)
    {
      // Test print:
      //Serial1.println(SSF_output[i]);
      if(SSF_output[i-1] < SSF_output[i] && SSF_output[i] > SSF_output[i+1])
      { 
        Peak = SSF_output[i];
        Peak_detected = true;
        for (int j = expected_peaks - 1; j >= 0; j--)
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
    // Test result of beat array_:
    // Serial1.println("-----------");
    // for(int x = 0; x < expected_peaks; x++)
    // {
    //  Serial1.println(Beat_array[x]);
    // } 

    //Processing Period:
    //Calculating threshold
    int Sum_beat_array = 0;                   //Total sum of the beat array_.
    for(int i = 0; i < expected_peaks; i++)
    {
      Sum_beat_array += Beat_array[i];        //Total
    }
    int threshold = 0.65*(Sum_beat_array/expected_peaks);  //threshold value for beat detection

    // Test print: 
    // for(int i = 50; i < size; i++)
    // {
    //   Serial1.print(SSF_output[i]);
    //   Serial1.print(",");
    //   Serial1.println(threshold);
    // } 
    // Serial1.println("------------------------------");
    //Serial1.print("Threshold:              ");
    //Serial1.println(threshold);


    // Counting the peaks to calculate BPM, RR and HRV:
    int Peak_count = 0;                   // Counter to count the number of peaks
    int P2p_time_start = 0;               // Peak to peak start time
    int Start_delta = micros();           // start of the total recording time (5 seconds)
    int Sum_of_p2p_times = 0;             // sum of the times between peaks in the 5 second recording
    int Delta_p2p_time[110];              // Peak to peak delta time
    for(int i = 0; i < size; i++)
    {  
      if (SSF_output[i] > threshold)      //Count peaks above threshold (beats) 
      {
        if(SSF_output[i-1] < SSF_output[i] && SSF_output[i] > SSF_output[i+1])    //Peak detecting 
        {
          Peak_count++;                                     //increment the peak counts       
          if (Peak_count > 1)
          {
            Delta_p2p_time[Peak_count-2] = micros() - P2p_time_start;    //delta time between peak to peak
            // Test print:
            //Serial1.println(Delta_p2p_time[Peak_count-2]);
            Sum_of_p2p_times += Delta_p2p_time[Peak_count-2];
          }
          P2p_time_start = micros();                        //starting time of peak to peak
        }
      }
    }
    int End_delta = micros() - Start_delta;                 // Delta time of the for loop for the 5 seconds
    // Test print:
    //Serial1.println(Sum_of_p2p_times);
    //Serial1.println(End_delta_rec5s);

    // Calculating respiratory rate (RR):
    if (recording == false)
    {
      // Test print:
      //Serial1.println("----------");
      //Serial1.println(Delta_p2p_time[0]);
      RR_count = 0;
      for ( int i = 1; i < Peak_count-2; i++)
      {
        // Test print:
        //Serial1.println(Delta_p2p_time[i]);
        if(Delta_p2p_time[i-1] < Delta_p2p_time[i] && Delta_p2p_time[i] > Delta_p2p_time[i+1])
        {
          RR_count++;                                       // count breaths
          // Test print:
          //Serial1.println("-----");
        }
      }

      // Calculating HRV:
      float HRV = 0;            // Heart rate variablity score from 0 - 100
      int sum_of_HRV = 0;     //sum of square peak to peak values for RMSSD calculation 
      for(int i = 0; i < Peak_count-2; i++) 
      {
        // Test print:
        //Serial.println(Delta_p2p_time[i]);
        sum_of_HRV += pow((Delta_p2p_time[i] - Delta_p2p_time[i+1]), 2);
        //Serial.println(sum_of_HRV);
      }
      // Test print:
      //Serial.println(sum_of_HRV);
      //Serial.println(Peak_count);
      HRV = sqrt(sum_of_HRV/(Peak_count-1));        // RMSSD calculation to get HRV score
      float HRV_score_float = log(HRV);             // ln(RMSSD) value between 0-6.5
      HRV_score = HRV_score_float*15.385;       // ln(RMSSD0 value between 0-100
      // Test print:
      //Serial.println(HRV_score);
    }

    // Test print:
    //Serial1.print("Core Body Temperature:  "); 
    //Serial1.println(Core_body_temp);               // print core body temperature ever 1 minute.

    //BPM calculation:
    int Total_60s = End_delta*FT;                           // Taking the 5 seconds/ 30 seconds to 60 seconds
    int Avg_p2p_time = Sum_of_p2p_times/Peak_count;         // Average peak to peak time in 5 second recording 
    BPM = int(Total_60s/(Avg_p2p_time)*factor_);        // Calculating the beats per minute
    // Test print:
    //Serial1.print("BPM:                    ");
    //Serial1.println(BPM);

    //SpO2 calculation:
    float RMS_AC_IR = sqrt(Sum_AC_IR/400);                     //RMS of the IR AC signal
    float RMS_AC_RED = sqrt(Sum_AC_RED/400);                   //RMS of the RED AC signal
    float R = (RMS_AC_RED/RED_DC_val_SpO2)/(RMS_AC_IR/IR_DC_val_SpO2);    //R value used to calculate Sp02
    SpO2 = 110 - 25*R;                                    //Sp02 value
    // Test print:
    //Serial1.print("SpO2:                   ");
    //Serial1.println(SpO2);

    //RR Calculation:
    RR = RR_count*2;                                        // RR breaths per minute (count 30's times 2)
    // Test print:
    //Serial1.print("RR:                     ");
    //Serial1.println(RR);

    Data_available = false;                                     // Data has been processed
    Startup = false;                                            // Start up has is complete

    //Test print bluetooth serial:
    delay(20);
    Serial1.println("VITALTRAC:");
    Serial1.println("=========");
    Serial1.println("");
    Serial1.println("Vital Data:");
    Serial1.println("-----------");
    Serial1.print("Core Body Temperature:  "); 
    Serial1.println(Core_body_temp);               // print core body temperature ever 1 minute.
    delay(20);
    Serial1.print("BPM:                     ");
    Serial1.println(BPM);
    delay(20);
    Serial1.print("HRV:                      ");
    Serial1.println(HRV_score);
    delay(20);
    Serial1.print("SpO2:                    ");
    Serial1.println(SpO2);
    delay(20);
    Serial1.print("RR:                         ");
    Serial1.println(RR);
    delay(20);
    Serial1.println("");
    Serial1.println("EEG Data:");
    Serial1.println("-----------");
    delay(20);
    Serial1.print("Signal Strength:   ");
    Serial1.println(signal_strength);
    delay(20);
    Serial1.print("Attention:              ");
    Serial1.println(attention);
    delay(20);
    Serial1.print("Meditation:           ");
    Serial1.println(meditation);
    delay(20);

    //Vital signs combinations:
    int VS_combinations = 0;
    Serial1.println("");
    Serial1.println("Condition:");
    Serial1.println("-----------");
    delay(20);
    Serial1.print("Symptoms of:   ");
    // //Combination 1:
    // if (HR_VS == 3 && HRV_VS == 1 && RR_VS == 1 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 3)
    // {
    //   //CHECK: EEG
    //   VS_combinations = 1;
    //   Serial1.print("Infection");
    // }
    // //Combination 2:
    // if (HR_VS == 3 && HRV_VS == 2 && RR_VS == 3 && EEG_VS == * && SpO2_VS == 1 && Tb_VS == 3)
    // {
    //   //CHECK: Temperature: Slightly high, EEG
    //   VS_combinations = 2;
    //   Serial1.print("Pneumonia");
    // }
    // //Combination 3:
    // if (HR_VS == 1 && HRV_VS == 3 && RR_VS == 2 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 2)
    // {
    //   //CHECK: EEG
    //   VS_combinations = 3;
    //   Serial1.print("Tired, please rest");
    // }

    // //Combination 4:
    // if (HR_VS == 2 && HRV_VS == 1 && RR_VS == 1 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 2)
    // {
    //   //CHECK: EEG, HRV: slightly low
    //   VS_combinations = 4;
    //   Serial1.print("Drugs");
    // }

    // //Combination 5:
    // if (HR_VS == 2 && HRV_VS == 1 && RR_VS == 2 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 2)
    // {
    //   //CHECK: HR: slightly high, EEG, HRV: slightly elevated, Sp02: Slightly low/normal
    //   VS_combinations = 5;
    //   Serial1.print("Dysrythmia");
    // }

    // //Combination 6:
    // if (HR_VS == 3 && HRV_VS == 2 && RR_VS == 2 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 2)
    // {
    //   //CHECK: EEG
    //   VS_combinations = 6;
    //   Serial1.print("Convultion");
    // }

    if(VS_combinations == 0)
    {
     Serial1.println("Healthy"); 
    }
    //delay(20);
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    delay(500);

  }
  
  // Expect packets about once per second (for EEG):
  // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma"
  brain.update();
  if (Brain_prev != brain.readDelta() && brain.readDelta() != 0) 
  {
    // Signal strength:
    Brain_prev = brain.readDelta();
    signal_strength = brain.readSignalQuality();         // 0 good signal , 200 poor signal
    attention = brain.readAttention();           
    meditation = brain.readMeditation(); 
  }

  //Calculating vital signs combinations:
  //Heart Rate:
  int HR_VS = 0;
  if (BPM < 40)
  {
    HR_VS = 1;    //Low heart rate 
  }
  if (BPM >= 40 && BPM <= 100)
  {
    HR_VS = 2;    //Normal heart rate
  }
  if  (BPM > 100)
  {
    HR_VS = 3;    //High heart rate
  }

  //Heart Rate Variability:
  int HRV_VS = 0;
  if (HRV_score < 35)
  {
    HRV_VS = 1;    //Low heart rate variability
  }
  if (HRV_score >= 35 && HRV_score <= 65)
  {
    HRV_VS = 2;    //Normal heart rate variability
  }
  if  (HRV_score > 65)
  {
    HRV_VS = 3;    //High heart rate variability
  }

  //Respiratory Rate:
  int RR_VS = 0;
  if (RR < 12)
  {
    RR_VS = 1;    //Low respiratory rate 
  }
  if (RR >= 12 && RR <= 20)
  {
    RR_VS = 2;    //Normal respiratory rate
  }
  if  (RR > 20)
  {
    RR_VS = 3;    //High respiratory rate
  }

  //SpO2:
  int SpO2_VS = 0;
  if (SpO2 < 90)
  {
    SpO2_VS = 1;    //Low SpO2
  }
  if (SpO2 >= 90 && SpO2 <= 100)
  {
    SpO2_VS = 2;    //Normal SpO2
  }
  if  (SpO2 > 100)
  {
    SpO2_VS = 3;    //SpO2 ERROR
    Serial1.println("Error with SpO2!!");
  }

  //Temperature: 
  int Tb_VS = 0;
  if (Core_body_temp <= 35)
  {
    Tb_VS = 1;    //Low core body temperature
  }
  if (Core_body_temp > 35 && Core_body_temp < 39)
  {
    Tb_VS = 2;    //Normal core body temperature
  }
  if  (Core_body_temp >= 39)
  {
    Tb_VS = 3;    //High core body temperature
  }

  //EEG:

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
  } else {
  //  Serial1.println("SUCCESS");
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
