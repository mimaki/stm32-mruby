/*
** stm32driver.h - stm32 device driver for mruby
**
** Copyright(c) FUKUOKA CSK CORPORATION
** Copyright(c) Manycolors Inc
*/

#ifndef ENZIDRIVER_H
#define ENZIDRIVER_H

#include <inttypes.h>

#define PWM_DUTY 256
#define PIN_MAX         19
#define DIGITAL_PIN_MIN 0
#define DIGITAL_PIN_MAX 13
#define ANALOG_PIN_MIN  14
#define ANALOG_PIN_MAX  19

#define PORT_UNKNOWN 0
#define PORT_A 1
#define PORT_B 2
#define PORT_C 3
#define PORT_D 4
#define PORT_E 5
#define PORT_F 6
#define PORT_G 7

typedef enum {
  PIN_UNDEFINED = 0,
  PIN_DIGITAL_OUT = 1,
  PIN_DIGITAL_IN = 2,
  PIN_ANALOG_OUT = 3,
  PIN_ANALOG_IN = 4,
  PIN_USED = 0xfe,
  PIN_UNKNOWN = 0xff,
} pinStausNum;

typedef struct {
  uint16_t arduino_pin_num;
  uint16_t port_num;
  uint16_t pin_num;
  pinStausNum pin_status;
} pinStatus;

void Delay(uint32_t);
uint32_t Millis(void);
void analogWrite(uint16_t, uint16_t);
uint16_t analogRead(uint16_t);
void digitalWrite(uint16_t, uint16_t);
uint16_t digitalRead(uint16_t);
void LEDon(void);
void LEDoff(void);

void initUSB(void);
void setup_pin(uint16_t arduino_pin_num);
void xprintf(const char*, ...);

#endif /* ENZIDRIVER_H */
