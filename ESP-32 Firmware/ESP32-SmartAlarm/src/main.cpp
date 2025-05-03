#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Arduino.h>
#include <driver/i2c.h>

#define SDA_PIN 37
#define SCL_PIN 39
#define I2C_ADDRESS 0x5C // I2C address of the slave device

void setup()
{
    /////////////////////SERIAL///////////////////////
    Serial.begin(115200);
    delay(1000); // Give time for serial monitor to open

    ///////////////////////I2C///////////////////////
    i2c_config_t conf = {};
    conf.sda_io_num = (gpio_num_t)SDA_PIN;
    conf.scl_io_num = (gpio_num_t)SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.mode = I2C_MODE_SLAVE;
    conf.slave.addr_10bit_en = 0;
    conf.slave.slave_addr = I2C_ADDRESS;

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE, 128, 128, 0);

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

    if (!res)
    {
        Serial.println("Failed to connect to WiFi");
        // ESP.restart();
    }
    else
    {
        Serial.println("Connected to WiFi");
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void loop()
{
    static uint8_t data[128];
    static size_t data_len = 0;
    static unsigned long lastBlink = 0;

    // Handle incoming data
    data_len = i2c_slave_read_buffer(I2C_NUM_0, data, sizeof(data), 10 / portTICK_PERIOD_MS);
    if (data_len > 0)
    {
        Serial.print("Received ");
        Serial.print(data_len);
        Serial.println(" byte(s):");

        for (size_t i = 0; i < data_len; i++)
        {
            Serial.print("Received char: ");
            Serial.println((char)data[i]);

            // Flash LED as feedback
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);

            if (data[i] == '1')
            {
                Serial.println("Command 1 received: Ready to send IP on request.");
            }
            else if (data[i] == '2')
            {
                Serial.println("Command 2 received: [You can handle it here]");
            }
            else
            {
                Serial.println("Unknown command received.");
            }
        }
    }

    // Handle master request
    if (data_len == 0)
    {
        String ipAddress = WiFi.localIP().toString();
        size_t ip_len = ipAddress.length();
        for (size_t i = 0; i < ip_len; i++)
        {
            data[i] = ipAddress[i];
        }
        data[ip_len] = '\n'; // End of transmission

        i2c_slave_write_buffer(I2C_NUM_0, data, ip_len + 1, 10 / portTICK_PERIOD_MS);

        Serial.println("Master requested data. Sent IP: " + ipAddress);
    }

    // Blink LED every 100ms (0.05s on, 0.05s off)
    if (millis() - lastBlink > 100)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED state
        lastBlink = millis();
     }
}