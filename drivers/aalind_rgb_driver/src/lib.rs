use embedded_hal::digital::v2::OutputPin;

/*
    Develop an enum for all the possible cases.
    This was useful in the RoboCup software tutorial
    so I'm going to assume it is useful in this case.
*/
enum Colors {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    PURPLE,
    CYAN,
}

pub struct RGB_LED<OutputPin>
{
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
}

impl<OutputPin> RGB_LED<OutputPin>
where
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
{
    pub fn initializePins(red_pin: OutputPin, green_pin: OutputPin, blue_pin: OutputPin) -> RGB_LED<OutputPin> {
        RGB_LED {
            red_pin,
            green_pin,
            blue_pin,
        }
    }

    pub fn setColor(&mut self, c: Colors) {
        match c {
            colors::RED => {
                self.P1.set_high()?;
                self.P2.set_low()?;
                self.P3.set_low()?;
            }
            colors::BLUE => {
                self.P1.set_low()?;
                self.P2.set_low()?;
                self.P3.set_high()?;
            }
            colors::GREEN => {
                self.P1.set_low()?;
                self.P2.set_high()?;
                self.P3.set_low()?;
            }
            colors::YELLOW => {
                self.P1.set_high()?;
                self.P2.set_high()?;
                self.P3.set_low()?;
            }
            colors::PURPLE => {
                self.P1.set_high()?;
                self.P2.set_low()?;
                self.P3.set_high()?;
            }
            colors::PURPLE => {
                self.P1.set_low()?;
                self.P2.set_high()?;
                self.P3.set_high()?;
            }
        }
        Ok(()) //saw this in an example, seems to verify that things are "ok" (not very descriptive i know)
    }

    pub fn turnOff(&mut self) {
        self.r.setLow()?;
        self.g.setLow()?;
        self.b.setLow()?;
    }
}

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
