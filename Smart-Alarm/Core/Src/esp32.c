#include "esp32.h"
#include "display.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

char device_ip[32] = {0};
bool wifi_connected = false;

void esp32getIP(void) {
    // Attempt to retrieve the IP address from the ESP32
    for (int attempt = 1; attempt <= MAX_CONNECTION_ATTEMPTS; attempt++) {
        uint8_t send_byte = '1'; // Command to request IP
        HAL_StatusTypeDef tx_status = HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR << 1), &send_byte, 1, HAL_MAX_DELAY);

        // Check if the I2C transmission was successful
        if (tx_status != HAL_OK) {
            printf("[ERROR] I2C Transmit failed on attempt %d\n", attempt);
            continue;
        }

        HAL_Delay(10); // Short delay before receiving data

        uint8_t rx_raw[DATA_SIZE] = {0}; // Buffer to store received data
        HAL_StatusTypeDef rx_status = HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDR << 1), rx_raw, DATA_SIZE, HAL_MAX_DELAY);

        // Check if the I2C reception was successful
        if (rx_status == HAL_OK) {
            char ip_str[DATA_SIZE + 1] = {0}; // Buffer for the IP string (null-terminated)

            // Convert raw data to a printable string
            for (int i = 0; i < DATA_SIZE; i++) {
                if (rx_raw[i] == '\n' || rx_raw[i] == '\r') {
                    break; // Stop at newline or carriage return
                }
                if (isprint(rx_raw[i])) {
                    ip_str[i] = rx_raw[i];
                } else {
                    ip_str[i] = 0; // Null-terminate on invalid character
                    break;
                }
            }
            ip_str[DATA_SIZE] = '\0'; // Ensure null termination

            // Validate the received IP address
            if (is_valid_ip(ip_str)) {
                strncpy(device_ip, ip_str, sizeof(device_ip)); // Save the valid IP
                device_ip[sizeof(device_ip) - 1] = '\0'; // Ensure null termination

                printf("[OK] Received valid IP: %s\n", device_ip);
                Display_IPAddress(device_ip); // Update the display with the IP
                HAL_Delay(100);
                wifi_connected = true; // Mark Wi-Fi as connected
                break;
            } else {
                printf("[WARN] Invalid IP received: %s\n", ip_str);
            }
        } else {
            printf("[ERROR] No IP received on attempt %d\n", attempt);
        }

        // If no valid IP is received, display an error
        strcpy(device_ip, "ERROR");
        Display_IPAddress(device_ip);
        HAL_Delay(200);
    }
}

bool is_valid_ip(const char *ip) {
    int num, dots = 0;
    char *ptr;
    char ip_copy[32];
    strncpy(ip_copy, ip, sizeof(ip_copy));
    ip_copy[sizeof(ip_copy) - 1] = '\0';

    ptr = strtok(ip_copy, ".");
    if (ptr == NULL)
        return false;

    while (ptr) {
        if (!isdigit(*ptr))
            return false;

        num = atoi(ptr);
        if (num < 0 || num > 255)
            return false;

        ptr = strtok(NULL, ".");
        if (ptr != NULL)
            dots++;
    }

    return dots == 3;
}
