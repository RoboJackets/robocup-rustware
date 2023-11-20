use embedded_hal::adc::{Channel, OneShot};
use teensy4_bsp::hal::adc::Adc;

const MAX_SAFE_VOLTAGE_READ: f32 = 2.692;
const MIN_SAFE_VOLTAGE_READ: f32 = 1.923;

const VOLTAGE_READ_RANGE: f32 = MAX_SAFE_VOLTAGE_READ - MIN_SAFE_VOLTAGE_READ;

const ADC_CR1_AWDCH_POS: u8 = 0;

const ADC_CR1_AWDCH_2: u32 = 0x04 << ADC_CR1_AWDCH_POS;

const ADC_MAX: f32 = 4096.0;

const ADC_RATIO: f32 = 3.3 / ADC_MAX;

struct BatteryAdc<const T: u8> {
    adc: Adc<T>
}

struct AdcChannel<const T: u8>;

impl<const T: u8> Channel<Adc<T>> for AdcChannel<T> {
    type ID = u32;

    fn channel() -> u32 {
        ADC_CR1_AWDCH_2
    }
}

impl<const T: u8> OneShot<Adc<T>, f32, AdcChannel<T>> for BatteryAdc<T> {
    type Error = BatteryError;

    fn read(&mut self, _channel: &mut AdcChannel<T>) -> nb::Result<f32, Self::Error> {
        let reading = self.adc.read_blocking_channel(AdcChannel::<T>::channel()); 
        Ok(reading as f32 * ADC_RATIO)
    }
}

enum BatteryError {
    ReadError,
}

struct Battery<const T: u8> {
    last_percentage: f32,
    raw_reading: u8,
    adc: BatteryAdc<T>,
}

impl<const T: u8> Battery<T> {
    pub fn new(adc: BatteryAdc<T>) -> Self {
        Self {
            last_percentage: 0.0,
            raw_reading: 0,
            adc,
        }
    }

    pub fn update(&mut self) -> Result<(), BatteryError> {
        let mut pin = AdcChannel {};
        let result = self
            .adc
            .read(&mut pin)
            .map_err(|_e| BatteryError::ReadError)?;
        self.last_percentage = (result - MIN_SAFE_VOLTAGE_READ) / VOLTAGE_READ_RANGE;
        self.raw_reading = (self.last_percentage * 255.0) as u8;
        Ok(())
    }

    pub fn get_percentage(&self) -> f32 {
        self.last_percentage
    }

    pub fn is_critical(&self) -> bool {
        return self.last_percentage < 0.05 || self.last_percentage > 1.0;
    }
}

struct BatteryModule {}
