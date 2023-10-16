pub trait ConfigurableSpi {
    fn set_frequency(&mut self, frequency: u32);
}