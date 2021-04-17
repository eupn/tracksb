//! PWM status LED

use embedded_hal::PwmPin;
use stm32wb_hal::{
    pwm::{Pwm, PwmExt1, C1},
    rcc::Rcc,
    stm32::TIM2,
    time::U32Ext,
};

use crate::bsp::StatusLedPin;

pub struct StatusLed {
    pwm_pin: Pwm<TIM2, C1>,
    max_duty: u32,
    duty: u32,
    up: bool,
}

impl StatusLed {
    pub fn new(tim2: TIM2, pin: StatusLedPin, rcc: &mut Rcc) -> Self {
        let mut pwm_pin = tim2.pwm(pin, 1.khz(), rcc);
        let max_duty = pwm_pin.get_max_duty();
        pwm_pin.set_duty(0);
        pwm_pin.enable();

        StatusLed {
            pwm_pin,
            max_duty,
            duty: 0,
            up: true,
        }
    }

    pub fn tick_animation(&mut self) {
        let blink_speed = self.max_duty / 16;

        if self.duty == 0 {
            self.duty = 0;
            self.up = true;
        } else if self.duty == self.max_duty {
            self.duty = self.max_duty;
            self.up = false;
        }

        if self.up {
            self.duty = self.duty.saturating_add(blink_speed);
        } else {
            self.duty = self.duty.saturating_sub(blink_speed);
        }

        self.pwm_pin.set_duty(self.duty);
    }

    pub fn turn_off(&mut self) {
        self.pwm_pin.set_duty(self.max_duty);
    }

    pub fn set_duty_percentage(&mut self, percent: u32) {
        let frac_duty = self.max_duty as f32 * (percent as f32 / 100f32);
        let inv_duty = self.max_duty - frac_duty as u32;
        self.pwm_pin.set_duty(inv_duty);
    }
}
