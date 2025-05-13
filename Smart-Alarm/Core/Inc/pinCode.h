#ifndef PINCODE_H
#define PINCODE_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define MAX_USERS 10  // Maximum number of users that can be registered

typedef struct {
    uint8_t uid[5];  // RFID tag UID (4 bytes + 1 checksum byte)
    char pin[PINCODE_LENGTH + 1];  // PIN associated with the user
} User;

typedef struct {
    User users[MAX_USERS];  // Array of registered users
    uint8_t user_count;     // Number of registered users
    char entered_pincode[PINCODE_LENGTH + 1];
    uint8_t pincode_position;
    uint8_t pin_attempts;
    uint32_t lockout_start;
    bool system_locked;
    bool system_armed;
    bool message_sent;
} PincodeState;

void Pincode_Init(PincodeState *state);
void Pincode_Check(PincodeState *state);
bool Register_NewUser(PincodeState *state, uint8_t *uid, const char *pin);

#endif // PINCODE_H
