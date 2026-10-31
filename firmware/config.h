/* Régulation de la vitesse d'un véhicule (obstacle-based speed control) — configuration
 * Target: ATmega328P @ 16 MHz (Arduino Uno / Nano board)            */
#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 16000000UL

/* Speed law (distance in cm, duty in 0..255) */
#define D_STOP      15   /* below: stop                               */
#define D_SLOW      25   /* STOP..SLOW: creep speed                   */
#define D_FULL      80   /* above: full speed, linear between SLOW and FULL */
#define DUTY_CREEP  90   /* minimum duty that still moves the robot   */
#define DUTY_MAX    230  /* ~90 % to keep a margin for the L298N      */

/* Acceleration limit: max duty change per control period.
 * Braking is allowed to be much stronger than accelerating.          */
#define RAMP_UP     12
#define RAMP_DOWN   60

#define PERIOD_MS   60   /* control period (>= 60 ms for HC-SR04)     */
#define ECHO_TIMEOUT_MS 30

#define UART_BAUD   9600

/* Pin map (Arduino names in brackets)
 *   HC-SR04 TRIG  PB1 [D9]       HC-SR04 ECHO PB0 [D8] = ICP1
 *   ENA (PWM left)  PD6 [D6] = OC0A   ENB (PWM right) PD5 [D5] = OC0B
 *   IN1 PD2 [D2]  IN2 PD3 [D3]  IN3 PD4 [D4]  IN4 PD7 [D7]
 *   Zone LEDs: green PC0 [A0], yellow PC1 [A1], red PC2 [A2]
 *   Buzzer PB2 [D10]                                                 */

#endif
