#![no_std]
#![no_main]

extern crate alloc;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use core::{cell::RefCell, default::Default, mem::MaybeUninit};
use embassy_executor::Spawner;
use embassy_stm32::{adc::Adc, bind_interrupts, gpio::{Input, Level, Output, Speed}, usart::{self, Uart}};
use embassy_time::{Duration, Timer, Delay};
use embassy_stm32::{i2c, spi};
use embassy_sync::{blocking_mutex::{NoopMutex, raw::NoopRawMutex}, pubsub::PubSubChannel};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

use rf24::radio::RF24;
use kicker_controller::Kicker;
use control_v2::{gpio::{decode_robot_id, decode_team, self}, graphics, kicker, motor, radio::{self, control_command::ControlMessage}, control, battery};
use common::{motor::{MotorCommand, MotorMoveResponse}, dribbler::DribblerCommand};

// Allocator
use embedded_alloc::LlffHeap;
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
const HEAP_SIZE: usize = 1024 * 8;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

/// The channel for publishing ControlMessages received from the radio
pub static COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, ControlMessage, 4, 2, 1>> = StaticCell::new();
/// The channel for publishing kicker states
pub static KICKER_STATE_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, kicker_controller::KickerState, 4, 2, 1>> = StaticCell::new();
/// The channel for publishing imu data
pub static IMU_DATA_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, control_v2::imu::ImuData, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 1 commands
pub static MOTOR1_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 1 statuses
pub static MOTOR1_STATUS_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 2 commands
pub static MOTOR2_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 2 statuses
pub static MOTOR2_STATUS_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 3 commands
pub static MOTOR3_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 3 statuses
pub static MOTOR3_STATUS_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 4 commands
pub static MOTOR4_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing motor 4 statuses
pub static MOTOR4_STATUS_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>> = StaticCell::new();
/// The channel for publishing dribbler commands
pub static DRIBBLER_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, DribblerCommand, 4, 1, 1>> = StaticCell::new();
/// The channel for turning off the robot
pub static POWER_OFF_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, (), 4, 2, 2>> = StaticCell::new();
/// The channel for publishing the current battery voltage
pub static BATTERY_VOLTAGE_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, f32, 4, 2, 1>> = StaticCell::new();

// Gpio Interrupts
bind_interrupts!(pub struct GpioIrqs {
    EXTI0 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI0>;
    EXTI1 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI1>;
    EXTI15_10 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        #[allow(static_mut_refs)]
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());

    // Enable the motor board
    let mut kill_n = Output::new(p.PB2, Level::High, Speed::Low);
    kill_n.set_high();
    let mut motor_en = Output::new(p.PC1, Level::High, Speed::Low);
    motor_en.set_high();
    let power_off_irq = embassy_stm32::exti::ExtiInput::new(p.PB1, p.EXTI1, embassy_stm32::gpio::Pull::None, GpioIrqs);

    // Initialize the dip switches
    let s1 = Input::new(p.PE5, embassy_stm32::gpio::Pull::None);
    let s2 = Input::new(p.PE4, embassy_stm32::gpio::Pull::None);
    let s3 = Input::new(p.PE3, embassy_stm32::gpio::Pull::None);
    let s4 = Input::new(p.PE2, embassy_stm32::gpio::Pull::None);

    let robot_id = decode_robot_id(s1.is_high(), s2.is_high(), s3.is_high());
    let team = decode_team(s4.is_high());

    // Initialize the ADC
    let adc = Adc::new(p.ADC1);
    let battery_pin = p.PB0;

    // Initialize the display
    let display_i2c = i2c::I2c::new(p.I2C2, p.PB10, p.PB11, graphics::DisplayIrqs, p.DMA1_CH2, p.DMA1_CH3, i2c::Config::default());
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(display_i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0
    ).into_buffered_graphics_mode();
    if let Err(_err) = display.init() {
        loop {
            error!("Error Initializing Display");
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    // Initialize the motor uarts
    let motor1_uart = Uart::new(p.UART7, p.PB3, p.PB4, motor::Motor1Irqs, p.DMA2_CH0, p.DMA2_CH1, usart::Config::default()).unwrap();
    let motor2_uart = Uart::new(p.USART1, p.PB7, p.PB6, motor::Motor2Irqs, p.DMA2_CH2, p.DMA2_CH3, usart::Config::default()).unwrap();
    let motor3_uart = Uart::new(p.USART3, p.PD9, p.PD8, motor::Motor3Irqs, p.DMA2_CH4, p.DMA2_CH5, usart::Config::default()).unwrap();
    let motor4_uart = Uart::new(p.USART6, p.PC7, p.PC6, motor::Motor4Irqs, p.DMA2_CH6, p.DMA2_CH7, usart::Config::default()).unwrap();
    let dribbler_uart = Uart::new(p.UART4, p.PA1, p.PA0, motor::DribblerIrqs, p.DMA1_CH6, p.DMA1_CH7, usart::Config::default()).unwrap();

    // Initialize the kicker controller
    let kicker_spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi::Config::default());
    let kicker_spi = kicker::KICKER_SPI.init(NoopMutex::new(RefCell::new(kicker_spi)));
    let kicker_csn = Output::new(p.PC4, Level::High, Speed::High);
    let kicker_spi = SpiDevice::new(kicker_spi, kicker_csn);
    let kicker_reset = Output::new(p.PC5, Level::High, Speed::Low);
    let kicker: Kicker<Output<'_>, SpiDevice<'_, embassy_sync::blocking_mutex::raw::NoopRawMutex, spi::Spi<'static, embassy_stm32::mode::Blocking, spi::mode::Master>, Output<'_>>> = Kicker::new(kicker_spi, kicker_reset);

    // Initialize the Radio
    let radio_spi = spi::Spi::new(p.SPI4, p.PE12, p.PE14, p.PE13, p.DMA1_CH4, p.DMA1_CH5, spi::Config::default());
    let radio_spi = radio::RADIO_SPI.init(NoopMutex::new(RefCell::new(radio_spi)));
    let csn = Output::new(p.PE11, Level::High, Speed::High);
    let ce = Output::new(p.PE9, Level::Low, Speed::Low);
    let radio_spi = SpiDevice::new(radio_spi, csn);
    let mut rf_radio  = RF24::new(ce, radio_spi, Delay);
    radio::init_radio(&mut rf_radio, &mut display, robot_id, team);
    let radio_irq = embassy_stm32::exti::ExtiInput::new(p.PE10, p.EXTI10, embassy_stm32::gpio::Pull::None, GpioIrqs);

    // Create communication channels
    let command_channel = COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, ControlMessage, 4, 2, 1>::new());
    let kicker_channel = KICKER_STATE_CHANNEL.init(PubSubChannel::<NoopRawMutex, kicker_controller::KickerState, 4, 2, 1>::new());
    let power_off_channel = POWER_OFF_CHANNEL.init(PubSubChannel::<NoopRawMutex, (), 4, 2, 2>::new());
    let battery_voltage_channel = BATTERY_VOLTAGE_CHANNEL.init(PubSubChannel::<NoopRawMutex, f32, 4, 2, 1>::new());

    let motor_one_command_channel = MOTOR1_COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorCommand, 4, 1, 1>::new());
    let motor_one_status_channel = MOTOR1_STATUS_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorMoveResponse, 4, 1, 1>::new());
    let motor_two_command_channel = MOTOR2_COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorCommand, 4, 1, 1>::new());
    let motor_two_status_channel = MOTOR2_STATUS_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorMoveResponse, 4, 1, 1>::new());
    let motor_three_command_channel = MOTOR3_COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorCommand, 4, 1, 1>::new());
    let motor_three_status_channel = MOTOR3_STATUS_CHANNEL.init (PubSubChannel::<NoopRawMutex, MotorMoveResponse, 4, 1, 1>::new());
    let motor_four_command_channel = MOTOR4_COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorCommand, 4, 1, 1>::new());
    let motor_four_status_channel = MOTOR4_STATUS_CHANNEL.init(PubSubChannel::<NoopRawMutex, MotorMoveResponse, 4, 1, 1>::new());
    let dribbler_command_channel = DRIBBLER_COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, DribblerCommand, 4, 1, 1>::new());

    // Spawn the power switch task
    spawner.spawn(
        gpio::power_switch(
            power_off_irq,
            power_off_channel.publisher().unwrap(),
            kill_n
        )
    ).unwrap();

    // Spawn the Radio Receive Task
    spawner.spawn(
        radio::receive_radio_data(
            radio_irq,
            rf_radio,
            robot_id,
            team,
            command_channel.publisher().unwrap(),
            kicker_channel.subscriber().unwrap(),
            battery_voltage_channel.subscriber().unwrap()
        )
    ).unwrap();

    // Spawn the Motor Controller Tasks
    spawner.spawn(
        motor::command_motor_1(
            motor1_uart,
            motor_one_command_channel.subscriber().unwrap(),
            motor_one_status_channel.publisher().unwrap()
        )
    ).unwrap();
    spawner.spawn(
        motor::command_motor_2(
            motor2_uart,
            motor_two_command_channel.subscriber().unwrap(),
            motor_two_status_channel.publisher().unwrap()
        )
    ).unwrap();
    spawner.spawn(
        motor::command_motor_3(
            motor3_uart,
            motor_three_command_channel.subscriber().unwrap(),
            motor_three_status_channel.publisher().unwrap()
        )
    ).unwrap();
    spawner.spawn(
        motor::command_motor_4(
            motor4_uart,
            motor_four_command_channel.subscriber().unwrap(),
            motor_four_status_channel.publisher().unwrap()
        )
    ).unwrap();
    spawner.spawn(
        motor::command_dribbler(
            dribbler_uart,
            dribbler_command_channel.subscriber().unwrap()
        )
    ).unwrap();

    // Spawn the Kicker Control Task
    spawner.spawn(
        kicker::service_kicker(
            kicker,
            command_channel.subscriber().unwrap(),
            kicker_channel.publisher().unwrap(),
            power_off_channel.subscriber().unwrap()
        )
    ).unwrap();

    // Spawn the Control Task
    spawner.spawn(
        control::control_task(
            command_channel.subscriber().unwrap(),
            [motor_one_command_channel.publisher().unwrap(), motor_two_command_channel.publisher().unwrap(), motor_three_command_channel.publisher().unwrap(), motor_four_command_channel.publisher().unwrap()],
            power_off_channel.subscriber().unwrap()
        )
    ).unwrap();
    
    // Spawn the battery monitoring task
    spawner.spawn(
        battery::monitor_battery_voltage(
            adc,
            battery_pin,
            battery_voltage_channel.publisher().unwrap(),
            power_off_channel.publisher().unwrap()
        )
    ).unwrap();

    // Spawn the display task
    spawner.spawn(
        graphics::display_robot_status(
            display,
            kicker_channel.subscriber().unwrap(),
            battery_voltage_channel.subscriber().unwrap()
        )
    ).unwrap();
}
