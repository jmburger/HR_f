//timer requiremnts:
int start_30s = 0;
int delta_30s = 0;
int start_1m = 0;
int delta_1m = 0;

void setup() {

	Serial.begin(57600);
	start_30s = micros();
	start_1m = micros();


}

void loop() 
{
  if(delta_30s >= 30000000)    
  {
  	Serial.println("30sec");
  	start_30s = micros();    			
  }
  delta_30s = micros() - start_30s;   	

  if(delta_1m >= 60000000)    
  {
  	Serial.println("1min");
  	start_1m = micros();    			
  }
  delta_1m = micros() - start_1m;   	

}
