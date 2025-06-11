import pigpio
import time

# Initialize the pigpio library
pi = pigpio.pi()

# Check if pigpio is connected
if not pi.connected:
    exit()

# Set the GPIO pin for PWM output (e.g., GPIO 18)
pwm_pin = 18

# Set the PWM frequency (e.g., 1000 Hz)
pwm_frequency = 50
pi.set_PWM_frequency(pwm_pin, pwm_frequency)

# Set the PWM range (e.g., 1000, for values from 0 to 1000)
pwm_range = 1000
pi.set_PWM_range(pwm_pin, pwm_range)

# Loop to change the duty cycle
try:
    while True:
        # Gradually increase the duty cycle
        for duty_cycle in range(0, pwm_range + 1, 10):
            pi.set_PWM_dutycycle(pwm_pin, duty_cycle)
            time.sleep(0.01)

        # Gradually decrease the duty cycle
        for duty_cycle in range(pwm_range, -1, -10):
            pi.set_PWM_dutycycle(pwm_pin, duty_cycle)
            time.sleep(0.01)

except KeyboardInterrupt:
    # Clean up and stop PWM on Ctrl+C
    pi.set_PWM_dutycycle(pwm_pin, 0)
    pi.stop()

