//#include <SoftwareSerial.h>
#include <Brain.h>

//SoftwareSerial mySerial(15, 7); // RX, TX

// Set up the brain reader, pass it the software serial object you want to listen on.
Brain brain(Serial1);
// Brain variables:
int Brain_prev = 0;
uint8_t signal_strength = 0;    
uint8_t attention = 0;
uint8_t meditation = 0;

void setup() {

    // Start the bluetooth serial.
    Serial1.begin(57600);

    // Start the serial port
    //Serial.begin(57600);
    
}

void loop() {
    // Expect packets about once per second:
    // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma"
    brain.update();
    if (Brain_prev != brain.readDelta() && brain.readDelta() != 0) 
    {
        // Signal strength:
        Brain_prev = brain.readDelta();
        signal_strength = brain.readSignalQuality();         // 0 good signal , 200 poor signal
        attention = brain.readAttention();           
        meditation = brain.readMeditation(); 

        //Test print bluetooth serial:
        Serial1.print(signal_strength);
        Serial1.print(",");
        Serial1.print(attention);
        Serial1.print(",");
        Serial1.println(meditation);
    }
    
}
