//#include <SoftwareSerial.h>
#include <Brain.h>

//#include <SD.h>
//#include <SPI.h>
//const int chipSelect = 10;

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
    Serial.begin(57600);
//
//    // SD card read and writer:
//    Serial.print("Initializing SD card...");
//    // see if the card is present and can be initialized:
//    if (!SD.begin(chipSelect)) {
//      Serial.println("Card failed, or not present");
//      // don't do anything more:
//      return;
//    }
//    Serial.println("card initialized.");
    
}

void loop() {
    // Expect packets about once per second:
    // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma"
    brain.update();
    //Serial.println("hi");
    if (Brain_prev != brain.readDelta() && brain.readDelta() != 0) 
    {
        // Signal strength:
        Brain_prev = brain.readDelta();
        //signal_strength = brain.readSignalQuality();         // 0 good signal , 200 poor signal
        //attention = brain.readAttention();           
        //meditation = brain.readMeditation(); 

        //Test print bluetooth serial:
          Serial.println(brain.readCSV());
        //Serial1.print(signal_strength);
        //Serial1.print(",");
        //Serial1.print(attention);
        //Serial1.print(",");
        //Serial1.println(meditation);

        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
//        File dataFile = SD.open("datalog.txt", FILE_WRITE);
//      
//        // if the file is available, write to it:
//        if (dataFile) {
//          dataFile.println(brain.readCSV());
//          dataFile.close();
//          // print to the serial port too:
//          Serial.println(brain.readCSV());
//        }  
//        // if the file isn't open, pop up an error:
//        else {
//          Serial.println("error opening datalog.txt");
//        } 
    }
    
}
