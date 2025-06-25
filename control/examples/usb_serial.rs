//! Demonstrates a bare-bones "parroting" application using serial over USB.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device =teensy4_bsp, peripherals =true)]
mod app {
    use bsp::board;
    use hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};

    use imxrt_hal as hal;
    use teensy4_bsp as bsp;

    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::{SerialPort, UsbError};

    /// Change me if you want to play with a full-speed USB device.
    const SPEED: Speed = Speed::High;
    /// Matches whatever is in imxrt-log.
    const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);
    /// The USB GPT timer we use to (infrequently) send mouse updates.
    const GPT_INSTANCE: imxrt_usbd::gpt::Instance = imxrt_usbd::gpt::Instance::Gpt0;
    /// How frequently should we push mouse updates to the host?
    const MOUSE_UPDATE_INTERVAL_MS: u32 = 200;

    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<1024> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    type Bus = BusAdapter;

    #[local]
    struct Local {
        class: SerialPort<'static, Bus>,
        device: UsbDevice<'static, Bus>,
        led: board::Led,
    }

    #[shared]
    struct Shared {}

    #[init(local = [bus: Option<UsbBusAllocator<Bus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb: usbd,
            pins,
            mut gpio2,
            ..
        } = board::t41(ctx.device);

        let led = board::led(&mut gpio2, pins.p13);

        let bus = BusAdapter::with_speed(usbd, &EP_MEMORY, &EP_STATE, SPEED);
        bus.set_interrupts(true);
        bus.gpt_mut(GPT_INSTANCE, |gpt| {
            gpt.stop();
            gpt.clear_elapsed();
            gpt.set_interrupt_enabled(true);
            gpt.set_mode(imxrt_usbd::gpt::Mode::Repeat);
            gpt.set_load(MOUSE_UPDATE_INTERVAL_MS * 1000);
            gpt.reset();
            gpt.run();
        });

        let bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));
        // Note that "4" correlates to a 1ms polling interval. Since this is a high speed
        // device, bInterval is computed differently.
        let class = SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, VID_PID)
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .build();

        (Shared {}, Local { class, device, led })
    }

    #[task(binds = USB_OTG1, local = [device, class, led, configured: bool = false], priority = 2)]
    fn usb1(ctx: usb1::Context) {
        let usb1::LocalResources {
            class,
            device,
            led,
            configured,
            ..
        } = ctx.local;

        device.poll(&mut [class]);

        if device.state() == UsbDeviceState::Configured {
            if !*configured {
                device.bus().configure();
            }
            *configured = true;
        } else {
            *configured = false;
        }

        if *configured {
            let elapsed = device.bus().gpt_mut(GPT_INSTANCE, |gpt| {
                let elapsed = gpt.is_elapsed();
                while gpt.is_elapsed() {
                    gpt.clear_elapsed();
                }
                elapsed
            });

            if elapsed {
                let mut buf = [0u8; 64];
                let mut to_write = 0;

                match class.read(&mut buf[..]) {
                    Ok(count) => {
                        // count bytes were read to &buf[..count]
                        led.toggle();
                        to_write = count;
                    }
                    Err(UsbError::WouldBlock) => {} // No data received
                    Err(_err) => {}                 // An error occurred
                };

                match class.write(&buf[0 .. to_write]) {
                    Ok(_count) => {
                        // count bytes were written
                    }
                    Err(UsbError::WouldBlock) => {} // No data could be written (buffers full)
                    Err(_err) => {}                 // An error occurred
                };
            }
        }
    }
}
