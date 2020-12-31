//! RGB LED control routines.

use embedded_hal::digital::v2::OutputPin;

use core::convert::Infallible;

// TODO: try PWM for RGB pins

pub struct RgbLed<
    R: OutputPin<Error = Infallible>,
    G: OutputPin<Error = Infallible>,
    B: OutputPin<Error = Infallible>,
> {
    red_led: R,
    red_led_toggle_state: bool,
    green_led: G,
    green_led_toggle_state: bool,
    blue_led: B,
    blue_led_toggle_state: bool,
}

#[derive(Copy, Clone)]
pub enum LedColor {
    Red,
    Green,
    Blue,
}

impl<
        R: OutputPin<Error = Infallible>,
        G: OutputPin<Error = Infallible>,
        B: OutputPin<Error = Infallible>,
    > RgbLed<R, G, B>
{
    pub fn new(red_led: R, green_led: G, blue_led: B) -> Self {
        Self {
            red_led,
            red_led_toggle_state: false,
            green_led,
            green_led_toggle_state: false,
            blue_led,
            blue_led_toggle_state: false,
        }
    }

    fn led_and_state(
        &mut self,
        led: LedColor,
    ) -> (&mut dyn OutputPin<Error = Infallible>, &mut bool) {
        match led {
            LedColor::Red => (
                &mut self.red_led as &mut dyn OutputPin<Error = Infallible>,
                &mut self.red_led_toggle_state,
            ),
            LedColor::Green => (
                &mut self.green_led as &mut dyn OutputPin<Error = Infallible>,
                &mut self.green_led_toggle_state,
            ),
            LedColor::Blue => (
                &mut self.blue_led as &mut dyn OutputPin<Error = Infallible>,
                &mut self.blue_led_toggle_state,
            ),
        }
    }

    pub fn set(&mut self, led: LedColor, on: bool) {
        let (led, state) = self.led_and_state(led);

        if on {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }

        *state = on;
    }

    pub fn toggle(&mut self, led: LedColor) {
        let (_, state) = self.led_and_state(led);
        *state = !*state;

        let new_state = *state;
        self.set(led, new_state);
    }
}
