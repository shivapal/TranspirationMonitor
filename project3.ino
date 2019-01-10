/*
 * Written by Shiva Pal
 * Code for transpiration rate monitor. Uses the humidity sensor chip Si7006 to measure rate of change of mass of water vapor in a gallon-sized vessel, reporting the results to a web server
 * I referenced several example sketches that came with the arduino IDE. These sketches are : BlinkWithoutDelay, Debounce, SimpleWebServerWiFi, and SFRRangeReader
 * Experimental design inspired by the paper "Using Computers in Measuring Transpiration Rate" by Peter F. Seligmann and Steven R. Thompson.
 * */

#include <SPI.h>
#include <WiFi101.h>
#include <Wire.h>
#include <Vector.h>

char ssid[] = "";        // Network name goes here
char pass[] = "";    // Network password goes here

int status = WL_IDLE_STATUS;
WiFiServer server(80); //connect to http

const int buttonPin = 14;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin

int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int state = 0;               //overall state of the device. 0 is ready state (not yet measured), 1 is calibrated (reference established), 2 is measuring

unsigned long lastLEDUpdate = 0;        // will store last time LED was updated

const long interval = 1000;           // interval at which to blink (milliseconds)

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

float reference=0; //holds the reference value of grams of water vapor in the container prior to measurement
int minutesElapsed=0; //holds minutes elapsed since measurement began
unsigned long lastMeasure=0; // time of last measurement

float backingStore[1]; //backing storage for vector
Vector<float> measurements; //holds measurements in grams of water vapor

String output="";  //String to be printed

void setup() {
  Wire.begin();  //start I2C communication
  
  WiFi.setPins(8, 7, 4, 2);  //enable wifi capability

  Serial.begin(9600);      // initialize serial communication for debugging

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(10000); // wait 10 seconds for connection:
  }
  server.begin();                           // start the web server on port 80
  printWiFiStatus();                        // you're connected now, so print out the status
  
  //set up GPIO
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState); // set initial LED state

  //set up vector for holding measurement data
  measurements.setStorage(backingStore);
  measurements.clear();
}

void loop() {
  int reading = digitalRead(buttonPin); //read in state of button pin

  //checks to see if button pressed
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); //update the last time button read value changed to now
  }

  if ((millis() - lastDebounceTime) > debounceDelay) { // if enough time has passed, the button is likely not bouncing

    if (reading != buttonState) { //if button state has changed
      buttonState = reading;  //update button state

      if (buttonState == LOW) { //if button is pressed
        state++; //move the state forward
      }
      if(state==0){ //if in ready state
        measurements.clear(); //reset vector holding measurements
        minutesElapsed=0; //reset for next round of measurements
      }
      if (state==1){ //take the reference measurement
        float rh= measureHumidity(); //get humidity
        float temp = measureTemp();  //get temperature
        reference=convert(rh, temp);  //get reference value
        output="Reference measurement: " + String(reference) + " grams";  //update output string
      }
      if (state==2){ //take first measurement
        lastMeasure=millis();  //update lastMeasure to now
        float rh= measureHumidity();  //get humidity
        float temp = measureTemp();  //get temperature
        float grams = convert(rh, temp);  //get measurement
        measurements.push_back(grams);  //save measurement
        output="<br>0 minutes: "+ String(grams) + " grams";  //update output string
      }
    }
  }

  if (state == 0) { //if ready state
    digitalWrite(ledPin, LOW);  //led is off
  }
  if (state == 1) {  //if calibrated
    unsigned long currentMillis = millis();  //see current time
    if (currentMillis - lastLEDUpdate >= interval) {  //if enough time has passed, toggle the led voltage
      lastLEDUpdate = currentMillis; //update this variable as led value is about to change

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }
  }
  if (state == 2) {  //if measuring
    digitalWrite(ledPin, HIGH);  //led is on
    
    unsigned long currentMillis = millis(); //see how much total time has passed
    if(currentMillis-lastMeasure>60000){ //if it's been a minute since the last measurement, measure again
      lastMeasure=currentMillis;  //last measurement is now
      float rh= measureHumidity(); //get humidity
      float temp = measureTemp();  //get temperature
      float grams = convert(rh, temp);  //get measurement
      measurements.push_back(grams);  //store measurement
      minutesElapsed++;  //track that a minute has passed
      output=output + "<br>" + minutesElapsed + " minutes: " + grams + " grams";  //update output string
      
    }
  }
  if (state == 3) {  //reset state to 0
    state = 0;
  }

  lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState
  clientService(); // update web server
}

void printWiFiStatus() {
  IPAddress ip = WiFi.localIP(); //get the ip address

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

//updates the web server
void clientService() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character


          if (currentLine.length() == 0) { //if the current line is blank
            // send a http response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
    
            String finalOut=""; //processed string to print
            if (state == 0) {
              finalOut="Press button to calibrate <br>"+output;
            }
            if (state == 1) {
              finalOut="Press button to measure <br>" +output;
            }
            if (state == 2) {
              finalOut="Press button to complete measurement <br>" + output;
            }
            client.println(finalOut);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}
//use i2c interface to measure humidity, returning relative humidity (i.e. 90, as in 90%, note this is not in decimal form)
float measureHumidity(){
  float retval=0;
  Wire.beginTransmission(0x40);  //address the slave
  Wire.write(0xE5);  //write the command to the slave
  Wire.endTransmission();  //actually does the writing to the slave
  //read from the slave
  Wire.requestFrom(0x40,2);
  unsigned short raw=0;
  if(Wire.available()==2){  //sensor returns two bytes, the first is the high byte, the next the low byte
    byte h=Wire.read();
    byte l=Wire.read();
    raw=word(h,l);
  }
  retval=((125*raw)/65536)-6; //calculation as described in the documentation
  return retval;
}
//returns temperature in degrees C
float measureTemp(){
  float retval=0;
  Wire.beginTransmission(0x40);  //address the slave
  Wire.write(0xE0);  //this command actually just pulls a temperature reading from a previous humidity reading
  Wire.endTransmission();  //actually does the writing to the slave
  //read from the slave
  Wire.requestFrom(0x40,2);
  unsigned short raw=0;
  if(Wire.available()==2){  //sensor returns two bytes, the first is the high byte, the next the low byte
    byte h=Wire.read();
    byte l=Wire.read();
    raw=word(h,l);
  }
  retval=((175.72*raw)/65536)-46.85; //calculation as described in the documentation
  return retval;
}

//converts a relative humidity measurement to grams of water vapor assuming a 1 gallon measuring cavity
// takes temp as degrees Celsius
float convert(float rh, float temp){ 
  return 18.016*(((rh/100)*(1.24*temp-7.4)*3.785)/((temp+273)*62.363));
}
