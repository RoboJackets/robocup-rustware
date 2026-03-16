#![no_std]
#![no_main]

extern crate alloc;

use {defmt_rtt as _, panic_probe as _};

use core::{default::Default, mem::MaybeUninit};
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, gpio::Input};

use control_v2::{
    gpio::{decode_robot_id, decode_team},
    utils,
};

// Allocator
use embedded_alloc::LlffHeap;
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
const HEAP_SIZE: usize = 1024 * 8;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

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

    // Initialize the dip switches
    let s1 = Input::new(p.PE5, embassy_stm32::gpio::Pull::None);
    let s2 = Input::new(p.PE4, embassy_stm32::gpio::Pull::None);
    let s3 = Input::new(p.PE3, embassy_stm32::gpio::Pull::None);
    let s4 = Input::new(p.PE2, embassy_stm32::gpio::Pull::None);

    let robot_id = decode_robot_id(s1.is_high(), s2.is_high(), s3.is_high());
    let team = decode_team(s4.is_high());

    // spawn switches test
    spawner
        .spawn(utils::test_dip_switches(robot_id, team))
        .unwrap();
}
