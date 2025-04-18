#define SDA_PIN 35
#define SCL_PIN 33

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Arduino.h>
#include <Wire.h>

void requestEvent();

void setup() {
    /////////////////////SERIAL///////////////////////
    Serial.begin(115200);
    ///////////////////////I2C///////////////////////
    Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
    Wire.begin(0x0C); // Specify I2C address
    Wire.onRequest(requestEvent); // Register request event handler
    
    ///////////////////////PINS///////////////////////
    pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin 15 as an output
    

    //Flash LED
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
    delay(1000); // wait for a second
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage HIGH


    //////////////////////WIFI///////////////////////
    // WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.

    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    // wm.resetSettings();

    // Automatically connect using saved credentials ,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP" "password"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("Successfully connected to WiFi");
        //flash LED
        digitalWrite(LED_BUILTIN, HIGH); 
        delay(500); 
        digitalWrite(LED_BUILTIN, LOW);
        delay(500); 
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
    }

}

void loop() {
    //wait to receive  message on I2c, this device is the slave
    while(Wire.available()) { // slave may send less than requested
        char c = Wire.read(); // receive a byte as character
        Serial.print ("I2C received: ");
        Serial.print(c); // print the character

        if (c == '1') {
            //briefly flash led
            digitalWrite(LED_BUILTIN, LOW); 
            delay(100); 
            digitalWrite(LED_BUILTIN, HIGH);
            
            Serial.println("Command 1 received, sending IP address");
            //Send Wifi IP address to master
            String ipAddress = WiFi.localIP().toString(); // Get the IP address as a string
            for (int i = 0; i < ipAddress.length(); i++) {
                Wire.write(ipAddress[i]); // Send each character of the IP address
            }
            Wire.write('\n'); // Send a newline character to indicate the end of the message

        } else if (c == '2') {
            Serial.println("Command 2 received");
            // Add your code for command 2 here
        } else if (c == '3') {
            Serial.println("Command 3 received");
            // Add your code for command 3 here
        } else if (c == '4') {
            Serial.println("Command 4 received");
            // Add your code for command 4 here
        } else if (c == '5') {
            Serial.println("Command 5 received");
            // Add your code for command 5 here
        } else {
            Serial.println("Unknown command");
        }
    }

}

void requestEvent() {
    Wire.write("Hello voidloop"); // respond with message of  14 bytes
    // as expected by master
  }