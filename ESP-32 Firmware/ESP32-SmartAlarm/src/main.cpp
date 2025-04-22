#define SDA_PIN 37
#define SCL_PIN 39
#define I2C_ADDRESS 0x5C // I2C address of the slave device

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Arduino.h>
#include <Wire.h>

void requestEvent();
void receiveEvent(int howMany);

void setup() {
    /////////////////////SERIAL///////////////////////
    Serial.begin(115200);
    delay(1000); // Give time for serial monitor to open
    ///////////////////////I2C///////////////////////
   
    Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN); // Specify I2C address
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent); // Register request event handler
    
    ///////////////////////PINS///////////////////////
    pinMode(LED_BUILTIN, OUTPUT); // initialize LED digital pin 15 as an output
    

    // Flash LED to indicate boot
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);

    //////////////////////WIFI///////////////////////
    WiFiManager wm;
    bool res = wm.autoConnect("AutoConnectAP", "password");

    if (!res) {
        Serial.println("Failed to connect to WiFi");
        // ESP.restart();
    } else {
        Serial.println("Connected to WiFi");
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void loop() {
    delay(10);
    //blink to signal activity every 2 seconds
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 2000) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        lastBlink = millis();
    }
}

// When STM32 sends data to this ESP32
void receiveEvent(int howMany) {
    Serial.print("Receiving ");
    Serial.print(howMany);
    Serial.println(" byte(s)...");

    while (Wire.available()) {
        char c = Wire.read();
        Serial.print("Received char: ");
        Serial.println(c);

        // Flash LED as feedback
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);

        if (c == '1') {
            Serial.println("Command 1 received: Ready to send IP on request.");
            // Store command for use in requestEvent (optional: add flag/state logic)
        } else if (c == '2') {
            Serial.println("Command 2 received: [You can handle it here]");
        } else {
            Serial.println("Unknown command received.");
        }
    }
}

// When STM32 requests data from this ESP32
void requestEvent() {
    String ipAddress = WiFi.localIP().toString();
    Serial.println("Master requested data. Sending IP: " + ipAddress);

    for (int i = 0; i < ipAddress.length(); i++) {
        Wire.write(ipAddress[i]);
    }
    Wire.write('\n'); // End of transmission

    // Flash LED rapidly to indicate data sent
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
    }
}