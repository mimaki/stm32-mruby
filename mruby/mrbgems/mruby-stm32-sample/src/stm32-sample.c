/*
** smt32-sample.c - stm32-sample Library
**
** Copyright(c) Hiroshi Mimaki
*/

#include "mruby.h"
#include "mruby/variable.h"
#include "mruby/class.h"
#include "mruby/string.h"
#include "mruby/hash.h"
#include <stdio.h>
#include <string.h>

/* external functions */
void LEDon(void);
void LEDoff(void);
void analogWrite(uint16_t, uint16_t);
void Delay(uint32_t);

/*
 *  call-seq:
 *     led.on => nil
 *     LED.on => nil
 *
 *  LED turn on.
 */
static mrb_value
mrb_led_on(mrb_state *mrb, mrb_value self)
{
  LEDon();
  return mrb_nil_value();
}

/*
 *  call-seq:
 *     led.off => nil
 *     LED.off => nil
 *
 *  LED turn off.
 */
static mrb_value
mrb_led_off(mrb_state *mrb, mrb_value self)
{
  LEDoff();
  return mrb_nil_value();
}

/*
 *  call-seq:
 *     PWM.new(pin) => PWM
 *
 *  Constructs a PWM.
 */
static mrb_value
mrb_pwm_initialize(mrb_state *mrb, mrb_value self)
{
  mrb_int pin;

  mrb_get_args(mrb, "i", &pin);

  mrb_iv_set(mrb, self, mrb_intern(mrb, "pin"), mrb_fixnum_value(pin));

  return self;
}

/*
 *  call-seq:
 *     pwm.write(v) => Fixnum
 *
 *  Write PWM data.
 */
static mrb_value
mrb_pwm_write(mrb_state *mrb, mrb_value self)
{
  mrb_int duty;
  mrb_value pin;

  mrb_get_args(mrb, "i", &duty);
  pin = mrb_iv_get(mrb, self, mrb_intern(mrb, "pin"));

  analogWrite((uint16_t)mrb_fixnum(pin), (uint16_t)duty);

  return mrb_fixnum_value(duty);
}

/*
 *  call-seq:
 *     delay(ms) => nil
 *
 *  delay while ms milliseconds.
 */
static mrb_value
mrb_delay(mrb_state *mrb, mrb_value self)
{
  mrb_int ms;

  mrb_get_args(mrb, "i", &ms);

  Delay(ms);

  return mrb_nil_value();
}

void
mrb_stm32_sample_gem_init(mrb_state *mrb)
{
  struct RClass *led, *pwm;

  /* LED class */
  led = mrb_define_class(mrb, "LED", mrb->object_class);

  /* LED methods */
  mrb_define_method(mrb, led, "on",  mrb_led_on,  ARGS_NONE());
  mrb_define_method(mrb, led, "off", mrb_led_off, ARGS_NONE());
  mrb_define_class_method(mrb, led, "on", mrb_led_on, ARGS_NONE());
  mrb_define_class_method(mrb, led, "off", mrb_led_off, ARGS_NONE());

  /* PWM class */
  pwm = mrb_define_class(mrb, "PWM", mrb->object_class);

  /* PWM methods */
  mrb_define_method(mrb, pwm, "initialize", mrb_pwm_initialize, ARGS_REQ(1));
  mrb_define_method(mrb, pwm, "write",      mrb_pwm_write,      ARGS_REQ(1));

  /* Object methods */
  mrb_define_method(mrb, mrb->object_class, "delay", mrb_delay, ARGS_REQ(1));
}

void
mrb_stm32_sample_gem_final(mrb_state *mrb)
{
}
