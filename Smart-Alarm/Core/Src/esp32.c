#include <stdbool.h>  // For bool, true, false
#include <ctype.h>    // For isdigit()
#include <string.h>   // For strlen(), etc.
#include <stdlib.h>   // For atoi()
/*
 * Function: is_valid_ip
 * ----------------------
 * Validates if the given string is a valid IPv4 address.
 *
 * Rules:
 * - Must have exactly 4 numbers separated by dots.
 * - Each number must be between 0 and 255.
 * - Leading zeros are not allowed (e.g., "192.168.01.1" is invalid).
 *
 * Parameters:
 *  ip: pointer to a null-terminated string representing the IP address.
 *
 * Returns:
 *  true if the IP is valid, false otherwise.
 */
bool is_valid_ip(const char *ip) {
    if (ip == NULL)
        return 0;

    int dots = 0;
    const char *ptr = ip;

    while (*ptr) {
        // Each block must start with a digit
        if (!isdigit(*ptr))
            return 0;

        int num = 0;
        int digit_count = 0;

        // Check for leading zero
        if (*ptr == '0' && isdigit(*(ptr + 1)))
            return 0;

        while (*ptr && isdigit(*ptr)) {
            num = num * 10 + (*ptr - '0');
            ptr++;
            digit_count++;

            // Each block should have at most 3 digits
            if (digit_count > 3)
                return false;
        }

        // Check numeric range (0-255)
        if (num < 0 || num > 255)
            return false;

        if (*ptr == '.') {
            dots++;
            ptr++;

            // Should not have more than 3 dots
            if (dots > 3)
                return false;

            // Next character must be a digit
            if (!isdigit(*ptr))
                return false;
        } else if (*ptr != '\0') {
            // Invalid character detected
            return false;
        }
    }

    // Must contain exactly 3 dots
    return dots == 3;
}
