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
}

void Pincode_Check(PincodeState *state) {
	uint32_t stored_pincode = ReadFromFlash(address1);
	char stored_pincode_str[PINCODE_LENGTH + 1];
	snprintf(stored_pincode_str, sizeof(stored_pincode_str), "%04lu",
			(unsigned long) stored_pincode);

	// Check if the entered pincode matches the stored pincode
	if (strncmp(state->entered_pincode, stored_pincode_str, PINCODE_LENGTH) == 0) {
		state->pin_attempts = 0;
		state->system_armed = !state->system_armed;

		// Always reset entered pincode and position when the system state changes
		memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
		state->pincode_position = 0;

		Display_AccessGranted();
		Display_SystemStatus(state->system_armed);

		// Reset message flag if the system is disarmed
		if (!state->system_armed) {
			state->message_sent = false;
		}
	} else {
		state->pin_attempts++;

		// Check if maximum attempts are reached
		if (state->pin_attempts >= MAX_PIN_ATTEMPTS) {
			state->system_locked = true;
			state->lockout_start = HAL_GetTick();

			// Reset entered pincode and position when the system is locked
			memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
			state->pincode_position = 0;

			Display_LockedScreen(LOCKOUT_TIME_MS / 1000);
		} else {
			Display_AccessDenied();

			// Always reset entered pincode and position after wrong attempt
			memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
			state->pincode_position = 0;

			Display_EnterPincode(state);
		}
	}
}

