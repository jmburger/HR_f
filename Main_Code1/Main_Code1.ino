// Required Libraries:
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// Temperature Sensor
#include <SparkFun_MLX90632_Arduino_Library.h>
MLX90632 temp_sensor;

// Timer variables:
int time_10s = micros(); 		// time 10 seconds
int time_60s = micros();		// time 1 minute
int time_300s = micros();		// time 5 minutes

// Flags for timer:
bool time_flag = false;



void setup() 
{
	Serial.begin(115200);

  	// Temperature sensor:
	Wire.begin();
	temp_sensor.begin();					// Start sensor
	temp_sensor.setMode(MODE_SLEEP);		// Put sensor in sleep mode 

}

void loop() 
{ 	
  	//Every 10 seconds record 5's of HR and Sp02:
  	int time_10s_micro = micros();
  	if (time_10s_micro - time_10s >= 10000000)
  	{
  		time_10s = micros();		// redefine time
  		//insert code below:
  		Serial.println("10 seconds"); // test print every 10 seconds
  	}

  	//Every minute take a temperature reading:
  	int time_60s_micro = micros();
	if (time_60s_micro - time_60s >= 60000000)
	{
		time_60s = micros();		// redefine time
		//insert code below:
		float Core_body_temp = temp_sensor.getObjectTemp();			// Get core body temperature
		Serial.println(Core_body_temp);								// print core body temperature ever 1 minute.
	}

	//Every 5 minutes record 30's of HR and Sp02 for RR and B2B:
	int time_300s_micro = micros();
  	if (time_300s_micro - time_300s >= 300000000)
  	{
  		time_300s = micros();		// redefine time
  		//insert code below:
  		Serial.println("5 minutes"); // test print every 5 seconds

  	}

}
