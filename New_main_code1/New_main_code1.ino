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
uint8_t Attention = 0;
uint8_t Meditation = 0;   

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
#define Sample_Rate 	MAX30100_SAMPRATE_50HZ         //define sample rate (200Hz)
#define Pulse_width   	MAX30100_SPC_PW_1600US_16BITS    //define led pulse width (1600)
#define Highres_mode  	true                            //High resolution mode
#define Warm_up 		2500000
//Current Balancing:
#define ACCEPTABLE_CURRENT_DIFF   2500  // Acceptable current difference between RED current and IR current
int counter = 8;						// Current array_ counter
//Filter Variables
//DC Removal variable:
#define SAMPLE_SIZE   100            	// Mean difference filter sample size used to calculate the running mean difference
#define ALPHA_DCR     0.95             	// DC filter alpha value for LED
double prev_filtered_IR = 0;     		// HR
double prev_filtered_RED = 0;    		// Sp02
//Mean Difference variables:
float MeanDiff_TV[SAMPLE_SIZE];     	// Trailling measurements
uint8_t Index = 0;
uint8_t count = 0;
float sum = 0;
//Low pass filter (butterworth filter) variables:
float Val[3];

// Temperature sensor (thermistor)
#define THERMISTORPIN 		A0      	// Which analog pin to connect
#define THERMISTORNOMINAL 	100000  	// Resistance at 25 degrees C 
#define TEMPERATURENOMINAL 	25      	// Temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 			50      	// How many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 		4036    	// The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 		100000  	// the value of the 'other' resistor
uint16_t samples[NUMSAMPLES];       	// Temperature variables
float Core_body_temp = 0;

// Global values:
float Tb_val = 0;
int BPM_val = 0;
float SpO2_val = 0;
int RR_val = 0;
float HRV_val = 0;

// Flags

//timer requiremnts:
int start_12s = 0;
int delta_12s = 0;
int start_2m = 0;
int delta_2m = 0;

void setup() {

	// Start serial terminal:
  	Serial.begin(57600);
  	// Start the bluetooth serial.
    Serial1.begin(57600);

  	Serial1.println("VITALTRAC:");
    Serial1.println("=========");
    Serial1.println("");
    Serial1.println("Warming Up.....");
    Serial1.println("");

  	// On start up do current balancing:
  	Current_Balancing();	
  	// On start up get body temperature:				
  	Body_temperature();	
  	// On start up get HR SpO2 RR HRV:
  	int recording_time_HR = 30000000 + Warm_up;			//Heart rate recording time
	HR_SpO2_RR_HRV(recording_time_HR, true);
	// On start up get EEG data:
	int recording_time_EEG = 4000000;					//EEG recording time
	EEG(recording_time_EEG);

	// Print vital data:
	print_data();

	//Start timers:
	start_12s = micros();
	start_2m = micros();
}

void loop() {
	
	//int recording_time_EEG = 5000000;					//EEG recording time
	//EEG(recording_time_EEG);		

	// do every 12 seconds
	if(delta_12s >= 12000000)    
	{
		start_12s = micros();
		int recording_time_HR = 8000000 + Warm_up;			//Heart rate recording time
		HR_SpO2_RR_HRV(recording_time_HR, false);
		Body_temperature();
		int recording_time_EEG = 2000000;					//EEG recording time
		EEG(recording_time_EEG);
		print_data(); 

	}
	delta_12s = micros() - start_12s; 

	// do every 2 minutes
	if(delta_2m >= 120000000)    
	{
		start_2m = micros();
		int recording_time_HR = 30000000 + Warm_up;			//Heart rate recording time
		HR_SpO2_RR_HRV(recording_time_HR, true);
		Body_temperature();
		int recording_time_EEG = 2000000;					//EEG recording time
		EEG(recording_time_EEG);
		print_data();   			
	}
	delta_2m = micros() - start_2m;   

		
}
void print_data()
{

	//Heart Rate:
	int HR_VS = 0;
	if (BPM_val < 40)
	{
		HR_VS = 1;    //Low heart rate 
	}
	if (BPM_val >= 40 && BPM_val <= 100)
	{
		HR_VS = 2;    //Normal heart rate
	}
	if  (BPM_val > 100)
	{
		HR_VS = 3;    //High heart rate
	}

	//Heart Rate Variability:
	int HRV_VS = 0;
	if (HRV_val < 35 && HRV_val >= 0)
	{
		HRV_VS = 1;    //Low heart rate variability
	}
	if (HRV_val >= 35 && HRV_val <= 65)
	{
		HRV_VS = 2;    //Normal heart rate variability
	}
	if  (HRV_val > 65 && HRV_val <= 100)
	{
		HRV_VS = 3;    //High heart rate variability
	}
	else
	{
		HRV_VS = 4;
	}

	//Respiratory Rate:
	int RR_VS = 0;
	if (RR_val < 12)
	{
		RR_VS = 1;    //Low respiratory rate 
	}
	if (RR_val >= 12 && RR_val <= 20)
	{
		RR_VS = 2;    //Normal respiratory rate
	}
	if  (RR_val > 20)
	{
		RR_VS = 3;    //High respiratory rate
	}

	//SpO2:
	int SpO2_VS = 0;
	if (SpO2_val < 90 && SpO2_val >= 0)
	{
		SpO2_VS = 1;    //Low SpO2
	}
	if (SpO2_val >= 90 && SpO2_val <= 100)
	{
		SpO2_VS = 2;    //Normal SpO2
	}
	if  (SpO2_val > 100 && SpO2_val < 0)
	{
		SpO2_VS = 3;    //SpO2 ERROR
	}

	//Temperature: 
	int Tb_VS = 0;
	if (Tb_val <= 35)
	{
		Tb_VS = 1;    //Low core body temperature
	}
	if (Tb_val > 35 && Tb_val < 39)
	{
		Tb_VS = 2;    //Normal core body temperature
	}
	if  (Tb_val >= 39)
	{
		Tb_VS = 3;    //High core body temperature
	}

	//Test print:
    Serial1.println("VITALTRAC:");
    Serial1.println("=========");
    Serial1.println("");
    Serial1.println("Vital Data:");
    Serial1.println("-----------");
    Serial1.print("Core Body Temperature:  "); 
    Serial1.println(Tb_val);               // print core body temperature ever 1 minute.
    delay(20);
    Serial1.print("BPM:                      ");
    if (BPM_val < 240 && BPM_val > 0)
    {
    	Serial1.println(BPM_val);
    }
    else
    {
    	Serial1.println("Error");
    }
    Serial1.print("HRV:                      ");
    Serial1.println(HRV_val);
    if (HRV_val == 4)
    {
    	Serial1.println("Error");
    }
    Serial1.print("SpO2:                    ");
    if (SpO2_val == 3)
    {
    	Serial1.println("Error");
    }
    Serial1.println(SpO2_val);
    delay(20);
    Serial1.print("RR:                         ");
    Serial1.println(RR_val);
    Serial1.println("");
    Serial1.println("EEG Data:");
	Serial1.println("-----------");
	delay(20);
	Serial1.print("Signal Strength:   ");
	Serial1.println(signal_strength);
	Serial1.print("Attention:              ");
	Serial1.println(Attention);
	delay(20);
	Serial1.print("Meditation:           ");
	Serial1.println(Meditation);

	//Vital signs combinations:
    int VS_combinations = 0;
    Serial1.println("");
    Serial1.println("Condition:");
    Serial1.println("-----------");
    delay(20);
    Serial1.print("Symptoms of:   ");
    //Combination 1:
    if (HR_VS == 3 && HRV_VS == 1 && RR_VS == 1 && SpO2_VS == 2 && Tb_VS == 3)
    {
      VS_combinations = 1;
      Serial1.print("Infection");
    }
    //Combination 2:
    if (HR_VS == 3 && HRV_VS == 2 && RR_VS == 3 && SpO2_VS == 1 && Tb_VS == 3)
    {
      //CHECK: Temperature: Slightly high
      VS_combinations = 2;
      Serial1.print("Pneumonia");
    }
    //Combination 3:
    //if (HR_VS == 1 && HRV_VS == 3 && RR_VS == 2 && EEG_VS == * && SpO2_VS == 2 && Tb_VS == 2)
    //{
    //  VS_combinations = 3;
    //  Serial1.print("Tired, please rest");
    //}

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
    delay(20);
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    delay(20);
}

// Function to get EEG values:
void EEG(int rec_time)
{
	// Brain variables:
	int i = 0;
	int Sum_size = rec_time/1000000;					// Size of array_.
	uint8_t Sum_Attention;								// Attention 
	uint8_t Sum_Meditation;								// Meditation
	int Brain_prev = 0; 
	int delta_EEG = 0;									// delta time of warm up
  	int start_EEG = micros();							// start time of warm up (1,5 second)
  	while(delta_EEG < rec_time)		
  	{
		// Expect packets about once per second:
	    // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma
	    brain.update();
	    if (Brain_prev != brain.readDelta() && brain.readDelta() != 0) 
	    {
	        // Signal strength:
	        Brain_prev = brain.readDelta();
	        signal_strength = brain.readSignalQuality();	// 0 good signal , 200 poor signal
	        Attention = brain.readAttention();           
	        Meditation = brain.readMeditation(); 

	        i++;

	        //Test print bluetooth serial:
	        //Serial.print(signal_strength);
	        //Serial.print(",");
	        //Serial.print(attention);
	        //Serial.print(",");
	        //Serial.println(meditation);
	    }	
	    delta_EEG = micros() - start_EEG;					// delta time of warm up

	    //Calculating average EEG value:
	    //Attention = Sum_Attention/Sum_size;
	    //Meditation = Sum_Meditation/Sum_size;
	}
}

// Function to get core body temperature:
void Body_temperature()
{
	// Required variables:
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
    //Serial.print("Average analog reading "); 
    //Serial.println(average);
    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    //Test print:
    //Serial.print("Thermistor resistance "); 
    //Serial.println(average);
   
    Core_body_temp = average / THERMISTORNOMINAL;     			// (R/Ro)
    Core_body_temp = log(Core_body_temp);                  		// ln(R/Ro)
    Core_body_temp /= BCOEFFICIENT;                   			// 1/B * ln(R/Ro)
    Core_body_temp += 1.0 / (TEMPERATURENOMINAL + 273.15); 		// + (1/To)
    Core_body_temp = 1.0 / Core_body_temp;                 		// Invert
    Core_body_temp -= 273.15;                         			// convert to C 
    Tb_val = Core_body_temp + 2.5;
   
    //Test print:
    //Serial.print("Temperature "); 
    //Serial.print(Core_body_temp);
    //Serial.println(" *C");	
}

// Function to get HR, SpO2, RR, HRV values:
void HR_SpO2_RR_HRV(int rec_time, bool RR_HRV)
{
	// start-up the MAX30100 sensor
	MAX30100_Startup();					
	//Variables store raw RED and IR values:
	uint16_t raw_IR_Val = 0;
	uint16_t raw_RED_Val = 0;
	int Expected_Peaks = ((rec_time - Warm_up)/1000000)*0.5;	//Expected beat peaks if heart rate is 30 bpm
	// Recording required varabiles:
	int IR_array_size = rec_time/20000;							// IR's size of array_.
	int SSF_array_size = (rec_time-Warm_up)/20000;				// SSF's size of array_.
	float IR_AC_array[IR_array_size];      						// IR signal AC array_.
	//float SSF_output[SSF_array_size];           				// SSF output array_.  	int i = 0;                        
	int i = 0;													// Counter
  	float IR_DC_val = 0;               							// DC value of the IR signal
  	float RED_DC_val = 0;              							// DC value of the RED signal
  	float IR_DC_val_SpO2 = 0;          							// DC value of the IR signal for SpO2 calculation
  	float RED_DC_val_SpO2 = 0;         							// DC value of the RED signal for SpO2 calculation
  	float Sum_AC_IR = 0;               							// Sum of the IR AC signal value
  	float Sum_AC_RED = 0;              							// Sum of the RED AC signal value
  	int delta_rec = 0;				      						// delta time between current and start time
  	int start_rec = micros();		  							// start record time
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
        if (i > 150 && i <= 300)
        {
          Sum_AC_IR += pow((IR_AC_array[i]),2);                                 //Sum of the IR AC signal value
        }
        //Add filtering to raw values:
        IR_AC_array[i] = MDF_function(IR_AC_array[i]);                          //mean difference filter IR LED data 
        IR_AC_array[i] = Butterworth_LPF_function(IR_AC_array[i]);              //low pass butterworth filter IR LED data
        //Get DC value from signal:
        IR_DC = true;
        IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);        		//Get DC value from IR signal
        if (i == 300)
        {
          IR_DC_val_SpO2 = IR_DC_val;
        }
        // Test Print:
        //Serial1.print(IR_DC_val);
        //Serial1.print(" , ");
        // RED Signal:
        bool RED_DC = false;	//Return either DC value (true) or AC value (false)  
        float RED_AC_value = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC); 	//filter raw RED LED data through DC removal
        //Calculating AC RMS value: (only after 50 iterations - remove noise)
        if (i > 150 && i <= 300)
        {
          Sum_AC_RED += pow((RED_AC_value),2);                               	//Sum of the RED AC signal value
        }
        //Get DC value from signal
        RED_DC = true;
        RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    		//Get DC value from RED signal
        if (i == 300)
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
      delta_rec = micros() - start_rec;						// delta time calculation 30 seconds 
  	}
  	// Shut down max30100 sensor:
  	MAX30100_sensor.shutdown();       						// Shutdown MAX30100 sensor
  	// Start data processing: 

  	// Slope Sum Function (SSF):
    int window = 6;         								//length of analyzing window
    int j = 0;               								//new counter
    float SSF = 0;           								//summation in window period
    for (int i = 125; i < IR_array_size; i++)
    {
      if (i <= (window+125))
      {
        IR_AC_array[j] = 0;
      }
      else
      {
        for (int x = window; x >= 0; x--)
        {
          SSF = 0;
          float delta_input = (IR_AC_array[i-window]) - (IR_AC_array[(i-window)-1]);
          if (delta_input > 0)
          {
            SSF += (delta_input);          
          }
          //else 
          //{
          //	IR_AC_array[i] = 0;
          //}
        }
        IR_AC_array[j] = SSF;
      } 
      //Test print:
      //Serial.print(IR_AC_array[j]);
      //Serial.print(",");
      //Serial.println(SSF_output[j]);
      //Serial.print(",");
      //Serial.println(j);
      //HERE____SSF_swopped_IR_AC_array
      j++;
    }

    // Filter signal (remove spikes in signal - by capping the signal)
    int Signal_cap = 8;											// Cap signal can't go higher than 9.
    for (int i = 1; i < IR_array_size; i++)
    {
    	// Remove sudden spikes:
    	if(IR_AC_array[i-1] <= 0.01 && IR_AC_array[i] >= 5 && IR_AC_array[i+1] <= 0.01)
    	{
    		IR_AC_array[i] = 0;
    	}
    	if(IR_AC_array[i] >= Signal_cap)
		{
			IR_AC_array[i] = Signal_cap;
			for (int j = 1; j <= 14; j++)
			{
				IR_AC_array[i+j] = 0;
			}
		}
    }
    
    //Identifies the highest peaks of the of the number of expected peaks.
    float Value_of_Peak = 0;				// Value of peak detected
    float Prev_value_of_peak = 0;			// Previous value of peak detected 
    float Sum_of_Peak = 0;                  // Sum of peak detected
    int Count_peaks = 0;					// Count peaks
    int Prev_i = 0; 						// Previous i
    for (int i = 0; i < SSF_array_size; i++)
    {
      if(IR_AC_array[i-1] < IR_AC_array[i] && IR_AC_array[i] > IR_AC_array[i+1] && IR_AC_array[i] > 1 && IR_AC_array[i] != Signal_cap)
      { 
      	Value_of_Peak = IR_AC_array[i];
        Sum_of_Peak += IR_AC_array[i];
        Count_peaks++;
        if (i < Prev_i+14)
	    {
	      	if (Value_of_Peak > Prev_value_of_peak)
	     	{
	      		Sum_of_Peak -= Prev_value_of_peak;
	      	}
	      	else
	      	{
	      		Sum_of_Peak -= Value_of_Peak;
	      	}
	      	Count_peaks--;
	    }
        Prev_i = i;
        Prev_value_of_peak = Value_of_Peak;
      }
    }

    //Calculating threshold
    float threshold = 0.55*(Sum_of_Peak/Count_peaks);  //threshold value for beat detection

    // Test print: 
	//for(int i = 0; i < SSF_array_size; i++)
	//{
	  //Serial.print(IR_AC_array[i]);
	  //Serial.print(",");
	  //Serial.println(threshold);
	  //Serial.print(",");
	  //Serial.println(i);
	  //delay(5);
	//} 

    // Counting the peaks to calculate BPM, RR and HRV:
    int Peak_count = 0;                   // Counter to count the number of peaks
    int P2p_time_start = 0;               // Peak to peak start time
    int Sum_of_p2p_times = 0;             // sum of the times between peaks in the 5 second recording
    int Delta_p2p_time[110];              // Peak to peak delta time
    int Start_delta = 0;
    Prev_i = 0;	   						  // Previous i
    bool check = false;
    for(int i = 0; i < SSF_array_size; i++)
    {  
      if (IR_AC_array[i] > threshold)      											//Count peaks above threshold (beats) 
      {
        if(IR_AC_array[i-1] < IR_AC_array[i] && IR_AC_array[i] > IR_AC_array[i+1])    	//Peak detecting 
        {
          Peak_count++;
          if (i < Prev_i+14)
          {
          	Peak_count--;
          	check = true;
          }																//increment the peak counts 
          if (Peak_count == 1) 
          {
          	Start_delta = micros();           										// start of the total recording time
          }                                          
          if (Peak_count > 1 && check == false)
          {
            Delta_p2p_time[Peak_count-2] = micros() - P2p_time_start;    			//delta time between peak to peak
            // Test print:
            //Serial1.println(Delta_p2p_time[Peak_count-2]);
            Sum_of_p2p_times += Delta_p2p_time[Peak_count-2];
          }
		  check = false;
          P2p_time_start = micros();                       		 					//starting time of peak to peak
          Prev_i = i;
        }
      }
    }
    int End_delta = micros() - Start_delta;                 						// Delta time of the for loop

    // Calculating heart rate (HR):
    float refine_factor = 1;
    if (RR_HRV == true)
    {
    	refine_factor = 1;												// Factor to refine BPM value
    }else
    {
    	refine_factor = 1;												// Factor to refine BPM value
    }
    int Total_60s = End_delta*(60000000/(rec_time-Warm_up));                        // Taking the 5 seconds/ 30 seconds to 60 seconds
    int Avg_p2p_time = Sum_of_p2p_times/Peak_count;      							// Average peak to peak time in 5 second recording 
    float BPM = (Total_60s/Avg_p2p_time)*refine_factor;         					// Calculating the beats per minute
    BPM_val = BPM;

    // Calculating SpO2:
    float RMS_AC_IR = sqrt(Sum_AC_IR/100);                     						// RMS of the IR AC signal
    float RMS_AC_RED = sqrt(Sum_AC_RED/100);                   						// RMS of the RED AC signal
    float R = (RMS_AC_RED/RED_DC_val_SpO2)/(RMS_AC_IR/IR_DC_val_SpO2);    			// R value used to calculate Sp02
    float SpO2 = 110 - 25*R; 
    SpO2_val = SpO2;

    if (RR_HRV == true)
    {
    	// Calculating respiratory rate (RR):
		int RR_count = 0;
		for ( int i = 1; i < Peak_count-2; i++)
		{
		  // Test print:
		  //Serial1.println(Delta_p2p_time[i]);
		  if(Delta_p2p_time[i-1] < Delta_p2p_time[i] && Delta_p2p_time[i] > Delta_p2p_time[i+1])
		  {
		    RR_count++;                                       	// count breaths
		    // Test print:
		    //Serial1.println("-----");
		  }
		}
	    float RR = RR_count*2;                               	// RR breaths per minute (count 30's times 2)
	    RR_val = RR;

	    // Calculating heart rate variability (HRV):
	    float HRV = 0;            								// Heart rate variablity score from 0 - 100
	    int sum_of_HRV = 0;     								//sum of square peak to peak values for RMSSD calculation 
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
	    HRV = sqrt(sum_of_HRV/(Peak_count-1));        			// RMSSD calculation to get HRV score
	    float HRV_score_float = log(HRV);             			// ln(RMSSD) value between 0-6.5
	    float HRV_score = HRV_score_float*15.385;       		// ln(RMSSD0 value between 0-100
	    HRV_val = HRV_score;
    }
}

// Balance IR and RED currents:
void Current_Balancing()
{
	//Variables store raw RED and IR values
	uint16_t raw_IR_Val = 0;
	uint16_t raw_RED_Val = 0;
	MAX30100_Startup();										// start-up the MAX30100 sensor
  	// Warm Up:
  	int delta_warmup = 0;									// delta time of warm up
  	int start_warmup = micros();							// start time of warm up (1,5 second)
  	while(delta_warmup <= Warm_up)		
  	{
     	MAX30100_sensor.update();   						// Update sensor
  		delta_warmup = micros() - start_warmup;				// delta time of warm up
  	}
	int delta_5s = 0;										// delta time between current and start time
  	int start_5s = micros();								// start 5 second current balancing
  	while(delta_5s <= 5000000)
  	{
  		delta_5s = micros() - start_5s; 					// delta time calculation 3 seconds
  		// if raw data is available 
  		MAX30100_sensor.update();																				// Update sensor
    	if (MAX30100_sensor.getRawValues(&raw_IR_Val, &raw_RED_Val))
    	{
  	    	//Get DC value from IR signal:
  			bool IR_DC = true;																					//Return either DC value (true) or AC value (false)
  			float IR_DC_val = DCR_function_IR(raw_IR_Val, ALPHA_DCR, IR_DC);       								//Get DC value from IR signal

  	    	//Get DC value from RED signal:
  			bool RED_DC = true; 																				//Return either DC value (true) or AC value (false)
  	    	float RED_DC_val = DCR_function_RED(raw_RED_Val, ALPHA_DCR, RED_DC);    							//Get DC value from RED signal
  	
  	    	//RED and IR DC current balancing:
  			if (IR_DC_val - RED_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current < MAX30100_LED_CURR_50MA)
  			{
    			counter++;
    			RED_current = LEDCurrent_array[counter];          			              						//change RED LED's current
    			MAX30100_sensor.setLedsCurrent(IR_current, RED_current);   	          							//Set led's current IR and Red respectively
    		}
  			if (RED_DC_val - IR_DC_val > ACCEPTABLE_CURRENT_DIFF && RED_current > 0)
  			{
    			counter--;
    			RED_current = LEDCurrent_array[counter];          			              						//change RED LED's current
    			MAX30100_sensor.setLedsCurrent(IR_current, RED_current);   	          							//Set led's current IR and Red respectively
    		}
    		// Test print: 
  			//Serial1.print(RED_DC_val); 
  			//Serial1.print(" , ");
  			//Serial1.println(IR_DC_val);
  		}
  	}
  	MAX30100_sensor.shutdown();								// Shutdown MAX30100 sensor
}

// MAX30100 sensor start up:
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

// DC Removeral filter for Heart Rate (HR)
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

// DC Removeral filter for Sp02
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

// Low pass filter (Butterworth filter)
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
