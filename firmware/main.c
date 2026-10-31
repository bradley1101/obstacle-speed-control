/*
 * Régulation de la vitesse d'un véhicule — ATmega328P, register-level C
 *
 * An HC-SR04 measures the distance to the obstacle in front of a 2-motor
 * robot. Timer1 Input Capture (ICP1) measures the echo pulse with 0.5 us
 * resolution, without blocking. Timer0 generates the PWM for the L298N.
 *
 * Speed law:
 *   d <  D_STOP            -> stop, red LED, buzzer
 *   D_STOP <= d < D_SLOW   -> creep speed, yellow LED
 *   D_SLOW <= d < D_FULL   -> linear from DUTY_CREEP to DUTY_MAX, green LED
 *   d >= D_FULL / no echo  -> full speed, green LED
 * A median-of-3 filter rejects isolated wrong readings and a ramp limits
 * acceleration (braking is faster than acceleration).
 */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

/* ------------------------------------------------------------------ UART */
static int uart_putchar(char c, FILE *stream)
{
    (void)stream;
    if (c == '\n') uart_putchar('\r', stream);
    while (!(UCSR0A & _BV(UDRE0))) { }
    UDR0 = c;
    return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void)
{
    const uint16_t ubrr = (F_CPU / 16 / UART_BAUD) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);   /* 8N1 */
    stdout = &uart_out;
}

/* ------------------------------------------------- HC-SR04 / Timer1 ICP */
static volatile uint16_t echo_start;
static volatile uint16_t echo_ticks;       /* 1 tick = 0.5 us */
static volatile uint8_t  echo_ready;

ISR(TIMER1_CAPT_vect)
{
    if (TCCR1B & _BV(ICES1)) {             /* rising edge: pulse starts */
        echo_start = ICR1;
        TCCR1B &= ~_BV(ICES1);             /* next: falling edge        */
    } else {                               /* falling edge: pulse ends  */
        echo_ticks = ICR1 - echo_start;    /* unsigned: wrap-around OK  */
        TCCR1B |= _BV(ICES1);
        echo_ready = 1;
    }
    TIFR1 = _BV(ICF1);                     /* edge change may set ICF1  */
}

static void sonar_init(void)
{
    DDRB |= _BV(PB1);                      /* TRIG output               */
    DDRB &= ~_BV(PB0);                     /* ECHO = ICP1 input         */
    TCCR1A = 0;                            /* normal mode               */
    TCCR1B = _BV(ICNC1) | _BV(ICES1) | _BV(CS11);  /* noise canceler, rising, clk/8 */
    TIMSK1 = _BV(ICIE1);
}

/* Returns distance in cm, or 0xFFFF when no echo (nothing in range). */
static uint16_t sonar_read_cm(void)
{
    echo_ready = 0;
    TCCR1B |= _BV(ICES1);
    PORTB |= _BV(PB1);
    _delay_us(10);
    PORTB &= ~_BV(PB1);

    for (uint8_t ms = 0; ms < ECHO_TIMEOUT_MS; ms++) {
        if (echo_ready) break;
        _delay_ms(1);
    }
    if (!echo_ready) return 0xFFFF;

    uint16_t us;
    cli();
    us = echo_ticks / 2;
    sei();
    return us / 58;                        /* sound: 58 us per cm (round trip) */
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { b = a; }
    return b;
}

/* -------------------------------------------------- Motors / Timer0 PWM */
static void motors_init(void)
{
    DDRD |= _BV(PD6) | _BV(PD5) | _BV(PD2) | _BV(PD3) | _BV(PD4) | _BV(PD7);
    /* both motors forward: IN1=1 IN2=0, IN3=1 IN4=0 */
    PORTD = (PORTD & ~(_BV(PD3) | _BV(PD7))) | _BV(PD2) | _BV(PD4);
    /* Fast PWM 8-bit, non-inverting on OC0A and OC0B, clk/64 -> 976 Hz */
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS01) | _BV(CS00);
    OCR0A = 0;
    OCR0B = 0;
}

static void motors_set(uint8_t duty)
{
    OCR0A = duty;
    OCR0B = duty;
}

/* ------------------------------------------------------- Indicators */
enum zone { ZONE_STOP, ZONE_SLOW, ZONE_GO };

static void indicators_init(void)
{
    DDRC |= _BV(PC0) | _BV(PC1) | _BV(PC2);
    DDRB |= _BV(PB2);
}

static void indicators_set(enum zone z)
{
    PORTC &= ~(_BV(PC0) | _BV(PC1) | _BV(PC2));
    PORTC |= (z == ZONE_GO) ? _BV(PC0) : (z == ZONE_SLOW) ? _BV(PC1) : _BV(PC2);
    if (z == ZONE_STOP) PORTB |= _BV(PB2); else PORTB &= ~_BV(PB2);
}

/* -------------------------------------------------------- Speed law */
static uint8_t target_duty(uint16_t d, enum zone *z)
{
    if (d < D_STOP) { *z = ZONE_STOP; return 0; }
    if (d < D_SLOW) { *z = ZONE_SLOW; return DUTY_CREEP; }
    *z = ZONE_GO;
    if (d >= D_FULL) return DUTY_MAX;
    return (uint8_t)(DUTY_CREEP +
           (uint32_t)(d - D_SLOW) * (DUTY_MAX - DUTY_CREEP) / (D_FULL - D_SLOW));
}

static uint8_t ramp(uint8_t current, uint8_t target)
{
    if (target > current)
        return (target - current > RAMP_UP) ? current + RAMP_UP : target;
    if (current - target > RAMP_DOWN || target == 0)
        return (target == 0) ? 0 : current - RAMP_DOWN;   /* stop is immediate */
    return target;
}

/* ------------------------------------------------------------- main */
int main(void)
{
    uart_init();
    sonar_init();
    motors_init();
    indicators_init();
    sei();

    printf("Obstacle speed control ready\n");

    /* seed the median filter with real readings, so that an obstacle
       present at power-up is seen immediately */
    uint16_t h0 = sonar_read_cm();
    _delay_ms(PERIOD_MS);
    uint16_t h1 = sonar_read_cm();
    uint8_t duty = 0;

    for (;;) {
        uint16_t raw = sonar_read_cm();
        uint16_t d = median3(raw, h0, h1);
        h1 = h0;
        h0 = raw;

        enum zone z;
        uint8_t target = target_duty(d, &z);
        duty = ramp(duty, target);
        motors_set(duty);
        indicators_set(z);

        if (d == 0xFFFF) printf("d=---- cm  duty=%3u\n", duty);
        else             printf("d=%4u cm  duty=%3u\n", d, duty);

        _delay_ms(PERIOD_MS - ECHO_TIMEOUT_MS);
    }
}
