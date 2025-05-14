#include "pinCode.h"
#include "config.h"
#include "string.h"
#include "stdio.h"
#include "flashMemory.h"

/**
 * @brief Initialize the pincode system state.
 *
 * Resets the pincode buffer, counters, and system status.
 * This function must be called before using other pincode functionalities.
 *
 * @param state Pointer to the PincodeState structure.
 */
void Pincode_Init(PincodeState *state) {
	memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
	state->pincode_position = 0;
	state->pin_attempts = 0;
	state->lockout_start = 0;
	state->system_locked = false;
	state->system_armed = false;
	state->message_sent = false;
}

/**
 * @brief Check the entered pincode against the stored pincode.
 *
 * Validates the entered pincode. If correct, toggles the system armed state.
 * If incorrect, increments the attempt counter and handles lockout if necessary.
 *
 * @param state Pointer to the PincodeState structure.
 */
void Pincode_Check(PincodeState *state) {
	uint32_t stored_pincode = ReadFromFlash(address1);
	char stored_pincode_str[PINCODE_LENGTH + 1];

	// Convert stored pincode to string with zero-padding
	snprintf(stored_pincode_str, sizeof(stored_pincode_str), "%04lu",
			(unsigned long) stored_pincode);

	// Check if entered pincode matches the stored one
	if (strncmp(state->entered_pincode, stored_pincode_str, PINCODE_LENGTH)
			== 0) {
		state->pin_attempts = 0;
		state->system_armed = !state->system_armed;

		// Clear entered pincode and reset position after success
		memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
		state->pincode_position = 0;

		Display_AccessGranted();
		Display_SystemStatus(state->system_armed);

		// Reset message flag if system is disarmed
		if (!state->system_armed) {
			state->message_sent = false;
		}
	} else {
		state->pin_attempts++;

		// Check if max attempts exceeded to lock the system
		if (state->pin_attempts >= MAX_PIN_ATTEMPTS) {
			state->system_locked = true;
			state->lockout_start = HAL_GetTick();

			// Clear entered pincode and reset position on lockout
			memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
			state->pincode_position = 0;

			Display_LockedScreen(LOCKOUT_TIME_MS / 1000);
		} else {
			Display_AccessDenied();

			// Clear entered pincode and reset position for next attempt
			memset(state->entered_pincode, 0, sizeof(state->entered_pincode));
			state->pincode_position = 0;

			Display_EnterPincode(state);
		}
	}
}
