#include "pinCode.h"
#include "config.h"
#include "string.h"
#include "stdio.h"
#include "flashMemory.h"

void Pincode_Init(PincodeState *state) {
    memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
    state->pincode_position = 0;
    state->pin_attempts = 0;
    state->lockout_start = 0;
    state->system_locked = false;
    state->system_armed = false;
    state->message_sent = false;
    state->user_count = 0; // Initialize user count to 0
}

void Pincode_Check(PincodeState *state) {
	uint32_t stored_pincode = ReadFromFlash(address1);
	char stored_pincode_str[PINCODE_LENGTH + 1];
	snprintf(stored_pincode_str, sizeof(stored_pincode_str), "%04lu",
			(unsigned long) stored_pincode);

	if (state->system_armed) {
		// System has been deactivated
		state->message_sent = false; // Reset the variable when the system is turned off
	}

	if (strncmp(state->entered_pincode, stored_pincode_str, PINCODE_LENGTH)
			== 0) {
		state->pin_attempts = 0;
		state->system_armed = !state->system_armed;
		Display_AccessGranted();
		Display_SystemStatus(state->system_armed);
	} else {
		state->pin_attempts++;
		if (state->pin_attempts >= MAX_PIN_ATTEMPTS) {
			state->system_locked = true;
			state->lockout_start = HAL_GetTick();
			Display_LockedScreen(LOCKOUT_TIME_MS / 1000);
		} else {
			Display_AccessDenied();
			state->pincode_position = 0;
			memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
			Display_EnterPincode(state->pincode_position,
					state->entered_pincode);
		}
	}
}

bool Register_NewUser(PincodeState *state, uint8_t *uid, const char *pin) {
    if (state->user_count >= MAX_USERS) {
        printf("User limit reached. Cannot register new user.\n");
        return false;
    }

    memcpy(state->users[state->user_count].uid, uid, 5);
    strncpy(state->users[state->user_count].pin, pin, PINCODE_LENGTH);
    state->users[state->user_count].pin[PINCODE_LENGTH] = '\0';
    state->user_count++;
    printf("New user registered successfully.\n");
    return true;
}
