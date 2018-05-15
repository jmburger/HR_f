#include <Brain.h>

// Set up the brain reader, pass it the software serial object you want to listen on.
Brain brain(Serial1);
int Brain_prev = 0;

void setup() {
    // Start the software serial.
    Serial.begin(57600);
    // Start the hardware serial.
    Serial1.begin(57600);
}

void loop() {
    // Expect packets about once per second.
    // The .readCSV() function returns a string (well, char*) listing the most recent brain data, in the following format:
    // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma"
    brain.update();
    if (Brain_prev != brain.readDelta()) 
    {
    	Brain_prev = brain.readDelta();
        //Serial.println(brain.readErrors());
        Serial.println(brain.readCSV());
    }
    
}
