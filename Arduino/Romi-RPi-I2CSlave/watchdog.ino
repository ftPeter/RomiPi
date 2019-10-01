/* watchdog
 * 
 * watchdog timer to detect for improved reliability
 * 
 * resets the 32u4 processor if the reset function 
 * is not called every 100 ms
 * 
 * current reset at the same time the PID functionality is invoked
 * 
 * NOTE: In the Romi-RPi design, this watchdog has nothing to
 *       do with the deadman's switch. 
 * 
 * July 12, 2019
 */

#include <Adafruit_SleepyDog.h>

#define WATCHDOG_TIMEOUT_MS 50

void watchdog_init() {
  Watchdog.enable(WATCHDOG_TIMEOUT_MS);
}

void watchdog_reset() {
  Watchdog.reset();
}
