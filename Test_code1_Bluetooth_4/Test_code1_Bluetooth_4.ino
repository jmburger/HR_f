void setup() 
{
  Serial1.begin(9600);  //Start communication with bluetooth module
}

void loop() 
{
  //Print to bluetooth serial monitor
	Serial1.println("hi Jarryd");
	delay(5000);
}
