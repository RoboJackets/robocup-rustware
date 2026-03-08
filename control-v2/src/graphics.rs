//!
//! Useful functionality for displaying graphics related to the robot
//! 

use embedded_graphics::{
    mono_font::{ascii::{FONT_6X10, FONT_5X8}, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Alignment, Text},
};
use embassy_stm32::{i2c::{self, I2c}, peripherals, bind_interrupts, mode};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::{Subscriber, WaitResult}};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};

/// The number of times to refresh the graphics display (in Hz)
pub const DISPLAY_REFRESH_RATE_HZ: u64 = 5;

bind_interrupts!(pub struct DisplayIrqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

/// Renders a standardized error screen to the display
pub fn draw_error_screen<D>(display: &mut D, error_msg: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    display.clear(BinaryColor::Off)?;

    // 1. Draw an "Error" header bar
    let header_area = Rectangle::new(Point::zero(), Size::new(128, 15));
    header_area
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(display)?;

    let header_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
    Text::with_alignment(
        " SYSTEM ERROR ",
        header_area.center(),
        header_style,
        Alignment::Center,
    )
    .draw(display)?;

    // 2. Draw the dynamic error text
    // We use a smaller font/standard style for the message body
    let body_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    
    Text::new(
        error_msg,
        Point::new(4, 30), // Offset slightly from the header and left edge
        body_style,
    )
    .draw(display)?;

    Ok(())
}

/// Render the full robot status screen onto an SSD1306 display.
pub fn render_display<DI, SIZE>(
    display: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    kicker_voltage: f32,
    kicker_healthy: bool,
    ball_sensed: bool,
    battery_voltage: f32,
) -> Result<(), display_interface::DisplayError>
where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    display.clear_buffer();

    let header_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let value_style  = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let small_style  = MonoTextStyle::new(&FONT_5X8,  BinaryColor::On);

    // ── Section headers ──────────────────────────────────────────────────────
    Text::with_baseline("KICKER", Point::new(0, 0), header_style, Baseline::Top)
        .draw(display)
        .ok();

    Text::with_baseline("BATT", Point::new(92, 0), header_style, Baseline::Top)
        .draw(display)
        .ok();

    // Vertical divider between kicker and battery columns
    Line::new(Point::new(83, 0), Point::new(83, 52))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)
        .ok();

    // ── Kicker Voltage ───────────────────────────────────────────────────────
    let mut kv_buf = [0u8; 16];
    let kv_str = format_kicker_voltage(kicker_voltage, &mut kv_buf);

    Text::with_baseline("V:", Point::new(0, 13), value_style, Baseline::Top)
        .draw(display)
        .ok();
    Text::with_baseline(kv_str, Point::new(14, 13), value_style, Baseline::Top)
        .draw(display)
        .ok();

    // ── Ball Sensed ──────────────────────────────────────────────────────────
    let ball_str = if ball_sensed { "Ball: YES" } else { "Ball: NO " };
    let ball_color = if ball_sensed { BinaryColor::On } else { BinaryColor::Off };
    // Draw a small filled circle indicator next to the label
    let indicator_style = PrimitiveStyle::with_fill(ball_color);
    let outline_style   = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    // Circle at x=75, y=28 (right side of kicker column)
    use embedded_graphics::primitives::Circle;
    if ball_sensed {
        Circle::new(Point::new(67, 27), 7)
            .into_styled(indicator_style)
            .draw(display)
            .ok();
    } else {
        Circle::new(Point::new(67, 27), 7)
            .into_styled(outline_style)
            .draw(display)
            .ok();
    }

    Text::with_baseline(ball_str, Point::new(0, 27), small_style, Baseline::Top)
        .draw(display)
        .ok();

    // ── Kicker Health ────────────────────────────────────────────────────────
    let health_str = if kicker_healthy {
        "Hlth: OK   "
    } else {
        "Hlth: FAULT"
    };

    // If faulted, draw an inverted rectangle behind the text for emphasis
    if !kicker_healthy {
        Rectangle::new(Point::new(0, 39), Size::new(82, 12))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)
            .ok();
        let inv_style = MonoTextStyle::new(&FONT_5X8, BinaryColor::Off);
        Text::with_baseline(health_str, Point::new(0, 40), inv_style, Baseline::Top)
            .draw(display)
            .ok();
    } else {
        Text::with_baseline(health_str, Point::new(0, 40), small_style, Baseline::Top)
            .draw(display)
            .ok();
    }

    // ── Battery Voltage ──────────────────────────────────────────────────────
    let mut bv_buf = [0u8; 10];
    let bv_str = format_battery_voltage(battery_voltage, &mut bv_buf);

    Text::with_baseline(bv_str, Point::new(86, 13), small_style, Baseline::Top)
        .draw(display)
        .ok();

    // Battery bar indicator (height proportional to charge, 2.8V–4.2V per cell,
    // assuming 4S LiPo: 11.2V empty, 16.8V full)
    draw_battery_bar(display, battery_voltage, Point::new(86, 24), Size::new(38, 8));

    // ── Horizontal divider ───────────────────────────────────────────────────
    Line::new(Point::new(0, 53), Point::new(127, 53))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)
        .ok();

    // ── Bottom status bar ────────────────────────────────────────────────────
    let status = if !kicker_healthy {
        "!! KICKER FAULT !!"
    } else if ball_sensed {
        "  BALL READY       "
    } else {
        "  ROBOT READY      "
    };
    Text::with_baseline(status, Point::new(0, 55), small_style, Baseline::Top)
        .draw(display)
        .ok();

    Ok(())
}

/// Draw a simple battery bar gauge.
/// Assumes 4S LiPo: ~11.2V (0%) to 16.8V (100%).
fn draw_battery_bar<DI, SIZE>(
    display: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    voltage: f32,
    origin: Point,
    size: Size,
) where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    const V_MIN: f32 = 2.8;
    const V_MAX: f32 = 3.7;

    let pct = ((voltage - V_MIN) / (V_MAX - V_MIN)).clamp(0.0, 1.0);
    let fill_w = (pct * (size.width as f32 - 2.0)) as u32;

    // Outer border
    Rectangle::new(origin, size)
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)
        .ok();

    // Inner fill
    if fill_w > 0 {
        Rectangle::new(
            Point::new(origin.x + 1, origin.y + 1),
            Size::new(fill_w, size.height - 2),
        )
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(display)
        .ok();
    }
}

/// Format kicker voltage as "NNN.NV" into a pre-allocated buffer.
/// Returns a &str slice into buf. Uses no_std-friendly integer arithmetic.
fn format_kicker_voltage<'a>(volts: f32, buf: &'a mut [u8; 16]) -> &'a str {
    let v = volts.clamp(0.0, 999.9);
    let int_part = v as u32;
    let dec_part = ((v - int_part as f32) * 10.0) as u32;

    // Write "NNN.NV\0"
    let mut pos = 0usize;
    write_u32_3digit(buf, &mut pos, int_part);
    buf[pos] = b'.'; pos += 1;
    buf[pos] = b'0' + dec_part as u8; pos += 1;
    buf[pos] = b'V';
    core::str::from_utf8(&buf[..=pos]).unwrap_or("???V")
}

/// Format battery voltage as "NN.NNV" into a pre-allocated buffer.
fn format_battery_voltage<'a>(volts: f32, buf: &'a mut [u8; 10]) -> &'a str {
    let v = volts.clamp(0.0, 99.99);
    let int_part = v as u32;
    let dec_part = ((v - int_part as f32) * 100.0) as u32;

    let mut pos = 0usize;
    buf[pos] = b'0' + (int_part / 10) as u8; pos += 1;
    buf[pos] = b'0' + (int_part % 10) as u8; pos += 1;
    buf[pos] = b'.'; pos += 1;
    buf[pos] = b'0' + (dec_part / 10) as u8; pos += 1;
    buf[pos] = b'0' + (dec_part % 10) as u8; pos += 1;
    buf[pos] = b'V';
    core::str::from_utf8(&buf[..=pos]).unwrap_or("??.??V")
}

fn write_u32_3digit(buf: &mut [u8], pos: &mut usize, val: u32) {
    buf[*pos] = b'0' + (val / 100 % 10) as u8; *pos += 1;
    buf[*pos] = b'0' + (val /  10 % 10) as u8; *pos += 1;
    buf[*pos] = b'0' + (val       % 10) as u8; *pos += 1;
}

#[embassy_executor::task]
pub async fn display_robot_status(
    mut display: Ssd1306<I2CInterface<I2c<'static, mode::Async, i2c::Master>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
    mut kicker_status_subscriber: Subscriber<'static, NoopRawMutex, kicker_controller::KickerState, 4, 2, 1>,
    mut battery_voltage_subscriber: Subscriber<'static, NoopRawMutex, f32, 4, 2, 1>,
) {
    let mut kicker_state = kicker_controller::KickerState {
        current_voltage: 0,
        ball_sensed: false,
        healthy: false
    };
    let mut battery_voltage = 3.3;
    loop {
        // Check for a new kicker status message
        if let Some(WaitResult::Message(state)) = kicker_status_subscriber.try_next_message() {
            kicker_state = state;
        }
        // Check for a new battery voltage message
        if let Some(WaitResult::Message(voltage)) = battery_voltage_subscriber.try_next_message() {
            battery_voltage = voltage;
        }
        
        // Clear the display and render the current status
        let _ = render_display(&mut display, kicker_state.current_voltage as f32, kicker_state.healthy, kicker_state.ball_sensed, battery_voltage)
            .and_then(|_| display.flush());
        
        embassy_time::Timer::after(embassy_time::Duration::from_hz(DISPLAY_REFRESH_RATE_HZ)).await;
    }
}