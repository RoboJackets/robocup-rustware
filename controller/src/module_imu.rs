extern crate alloc;

use core::cmp;

use robojackets_robocup_control::{Delay2, RFRadio, SharedSPI, CHANNEL};
use rtic_monotonics::{systick::Systick, Monotonic};
use rtic_nrf24l01::config::power_amplifier::PowerAmplifier;

use robojackets_robocup_rtp::{
    control_message::{Mode, ShootMode, TriggerMode},
    imu_test_message::{ImuTestMessage, IMU_MESSAGE_SIZE},
    ControlMessage, ControlMessageBuilder, RobotStatusMessage, Team, BASE_STATION_ADDRESSES,
    CONTROL_MESSAGE_SIZE, ROBOT_RADIO_ADDRESSES, ROBOT_STATUS_SIZE,
};

use crate::types::{InputStateUpdate, NextModule};
use crate::{
    types::{Button, Display},
    util::get_successful_ack_count,
};
use crate::{
    types::{ControllerModule, RadioState},
    util::render_status_title,
};

use crate::util::{encode_btn_state, render_text};

use ncomm_utils::packing::Packable;

struct InputState {
    btn_last: u8,
    btn_press_timeout: u32,

    first_read_flag: bool,
}

#[derive(PartialEq, Clone, Copy, Debug)]
enum BenchmarkStatus {
    Idle = 0,
    Initialize = 1,
    Running = 2,
    Result = 3,
    ErrorInitializing = 4,
}

struct BenchmarkFrame {
    gyro_z: f32,
    accel_x: f32,
    accel_y: f32,
}

struct BenchmarkState {
    benchmark_active: bool,
    start_timestamp: u32,
    last_timestamp: u32,
    state: BenchmarkStatus,
    got_first_frame: bool,
    frame: BenchmarkFrame,
    init_attempts: u16,
}

pub struct IMUTestMod {
    input_state: InputState,
    radio_state: RadioState,
    benchmark_state: BenchmarkState,

    return_menu_flag: bool,
    config_radio_flag: bool,
}

impl IMUTestMod {
    pub fn new() -> Self {
        Self {
            input_state: InputState {
                btn_last: 0,
                btn_press_timeout: 0,
                first_read_flag: true,
            },
            radio_state: RadioState {
                team: 0,
                robot_id: 0,
                conn_acks_results: [false; 100],
                conn_acks_attempts: 0,
            },
            benchmark_state: BenchmarkState {
                benchmark_active: false,
                start_timestamp: 0,
                last_timestamp: 0,
                state: BenchmarkStatus::Idle,
                frame: BenchmarkFrame {
                    gyro_z: 0.0,
                    accel_x: 0.0,
                    accel_y: 0.0,
                },
                got_first_frame: false,
                init_attempts: 0,
            },
            return_menu_flag: false,
            config_radio_flag: true,
        }
    }
    fn btn_rising(&mut self, old_state: u8, new_state: u8, button: Button) -> bool {
        let mask = 1 << button as u8;
        //This is ~500 ms
        if (old_state & mask) == 0 && (new_state & mask) != 0 {
            self.input_state.btn_press_timeout = Systick::now().ticks() + 500;
            return true;
        } else {
            return false;
        }
    }

    fn update_buttons(&mut self, new_state: u8) {
        let old_state = self.input_state.btn_last;
        self.input_state.btn_last = new_state;

        if self.btn_rising(old_state, new_state, Button::Left) {
            self.return_menu_flag = true;
        }

        if self.btn_rising(old_state, new_state, Button::Right) {
            match self.benchmark_state.state {
                BenchmarkStatus::Idle => {
                    self.benchmark_state.state = BenchmarkStatus::Initialize;
                    self.benchmark_state.benchmark_active = true;
                    self.benchmark_state.start_timestamp = Systick::now().ticks();
                    self.benchmark_state.last_timestamp = Systick::now().ticks();
                    self.benchmark_state.got_first_frame = false;
                    self.benchmark_state.init_attempts = 0;
                }
                BenchmarkStatus::ErrorInitializing => {
                    self.benchmark_state.state = BenchmarkStatus::Initialize;
                    self.benchmark_state.benchmark_active = true;
                    self.benchmark_state.start_timestamp = Systick::now().ticks();
                    self.benchmark_state.last_timestamp = Systick::now().ticks();
                    self.benchmark_state.got_first_frame = false;
                    self.benchmark_state.init_attempts = 0;
                }
                BenchmarkStatus::Result => {
                    self.benchmark_state.state = BenchmarkStatus::Initialize;
                    self.benchmark_state.got_first_frame = false;
                }
                _ => {}
            }
        }
    }

    fn render_idle_screen(&self, display: Display) {
        render_status_title(display, "IMU Test");
        render_text(display, "Press Left to return", 0, 14, false);
        render_text(display, "Press Right to start", 0, 24, false);
    }
    fn render_init_screen(&self, display: Display) {
        render_status_title(display, "IMU Test");
        render_text(display, "Starting test...", 0, 14, false);
        let num_attempts = alloc::fmt::format(format_args!(
            "Attempts: {}",
            self.benchmark_state.init_attempts
        ));
        render_text(display, &num_attempts, 0, 24, false);
    }
    fn render_running_screen(&self, display: Display) {
        render_status_title(display, "IMU Test");
        if self.benchmark_state.got_first_frame {
            render_text(display, "Gyro Z: ", 0, 14, false);
            render_text(display, "Accel X: ", 0, 24, false);
            render_text(display, "Accel Y: ", 0, 34, false);

            let gyro_z =
                alloc::fmt::format(format_args!("{:.4}", self.benchmark_state.frame.gyro_z));
            let accel_x =
                alloc::fmt::format(format_args!("{:.4}", self.benchmark_state.frame.accel_x));
            let accel_y =
                alloc::fmt::format(format_args!("{:.4}", self.benchmark_state.frame.accel_y));
            render_text(display, &gyro_z, 50, 14, false);
            render_text(display, &accel_x, 50, 24, false);
            render_text(display, &accel_y, 50, 34, false);
        } else {
            render_text(display, "Waiting for data...", 0, 14, false);
        }
    }
    fn render_result_screen(&self, display: Display) {
        render_status_title(display, "IMU Test");
        render_text(display, "Test complete", 0, 14, false);
        render_text(display, "Press Left to return", 0, 24, false);
        render_text(display, "Press Right to reset", 0, 34, false);
    }
    fn render_error_screen(&self, display: Display) {
        render_status_title(display, "IMU Test");
        render_text(display, "Error initializing", 0, 14, false);
        render_text(display, "Press Left to return", 0, 24, false);
        render_text(display, "Press Right to retry", 0, 34, false);
    }

    fn enable_radio_listen(
        &mut self,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        delay: &mut Delay2,
    ) {
        radio.set_payload_size(IMU_MESSAGE_SIZE as u8, spi, delay);
        radio.start_listening(spi, delay);
    }

    fn disable_radio_listen(
        &mut self,
        radio: &mut RFRadio,
        spi: &mut SharedSPI,
        delay: &mut Delay2,
    ) {
        radio.set_payload_size(CONTROL_MESSAGE_SIZE as u8, spi, delay);
        radio.stop_listening(spi, delay);
    }
    fn radio_configure(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        radio.set_pa_level(PowerAmplifier::PALow, spi, delay);
        radio.set_channel(CHANNEL, spi, delay);
        radio.open_writing_pipe(
            ROBOT_RADIO_ADDRESSES[self.radio_state.team as usize]
                [self.radio_state.robot_id as usize],
            spi,
            delay,
        );
        radio.open_reading_pipe(1, BASE_STATION_ADDRESSES[0], spi, delay);
        self.enable_radio_listen(radio, spi, delay);
    }

    fn radio_poll(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        if radio.packet_ready(spi, delay) {
            let mut data = [0u8; IMU_MESSAGE_SIZE];
            radio.read(&mut data, spi, delay);

            match ImuTestMessage::unpack(&data[..]) {
                Ok(data) => {
                    log::info!("Data Received!");
                    log::info!("Gyro Z: {}", data.gyro_z);
                    log::info!("Accel X: {}", data.accel_x);
                    log::info!("Accel Y: {}", data.accel_y);

                    self.benchmark_state.frame.gyro_z = data.gyro_z;
                    self.benchmark_state.frame.accel_x = data.accel_x;
                    self.benchmark_state.frame.accel_y = data.accel_y;
                    self.benchmark_state.got_first_frame = true;
                    self.benchmark_state.last_timestamp = Systick::now().ticks();
                }
                Err(err) => {
                    log::info!("Unable to Unpack Data: {:?}", err)
                }
            }
        }
    }
}

impl ControllerModule for IMUTestMod {
    fn update_display(&self, display: Display) {
        match self.benchmark_state.state {
            BenchmarkStatus::Idle => self.render_idle_screen(display),
            BenchmarkStatus::Initialize => self.render_init_screen(display),
            BenchmarkStatus::Running => self.render_running_screen(display),
            BenchmarkStatus::Result => self.render_result_screen(display),
            BenchmarkStatus::ErrorInitializing => self.render_error_screen(display),
        }
    }

    fn update_inputs(&mut self, inputs: InputStateUpdate) {
        //only update buttons if all are present
        if inputs.btn_left.is_some()
            && inputs.btn_right.is_some()
            && inputs.btn_up.is_some()
            && inputs.btn_down.is_some()
            && inputs.btn_a.is_some()
            && inputs.btn_b.is_some()
        {
            let new_state = encode_btn_state(
                inputs.btn_left.unwrap(),
                inputs.btn_right.unwrap(),
                inputs.btn_up.unwrap(),
                inputs.btn_down.unwrap(),
                inputs.btn_a.unwrap(),
                inputs.btn_b.unwrap(),
            );

            //we don't want edge triggers on the first read
            if self.input_state.first_read_flag {
                self.input_state.first_read_flag = false;
                self.input_state.btn_last = new_state;
            }

            self.update_buttons(new_state);
        }
    }

    fn radio_update(&mut self, radio: &mut RFRadio, spi: &mut SharedSPI, delay: &mut Delay2) {
        if self.config_radio_flag {
            self.radio_configure(radio, spi, delay);
            self.disable_radio_listen(radio, spi, delay);
            self.config_radio_flag = false;
        }

        match self.benchmark_state.state {
            BenchmarkStatus::Initialize => {
                //try to send a start message
                let msg = ControlMessageBuilder::new().mode(Mode::ImuTest).build();
                let mut buf = [0u8; CONTROL_MESSAGE_SIZE];
                msg.pack(&mut buf).unwrap();

                let report = radio.write(&buf, spi, delay);
                radio.flush_tx(spi, delay);

                if report {
                    //we start listening for the IMU data
                    self.enable_radio_listen(radio, spi, delay);
                    self.benchmark_state.state = BenchmarkStatus::Running;
                    self.benchmark_state.start_timestamp = Systick::now().ticks();
                    self.benchmark_state.last_timestamp = Systick::now().ticks();
                    self.benchmark_state.got_first_frame = false;
                } else {
                    self.benchmark_state.init_attempts += 1;
                    if self.benchmark_state.init_attempts > 100 {
                        log::info!("Initialization failed after 100 attempts");
                        self.benchmark_state.state = BenchmarkStatus::ErrorInitializing;
                        self.disable_radio_listen(radio, spi, delay);
                    }
                }
            }
            BenchmarkStatus::Running => {
                //poll
                self.radio_poll(radio, spi, delay);

                //check if we have a timeout
                if Systick::now().ticks() - self.benchmark_state.last_timestamp > 1000 {
                    self.benchmark_state.state = BenchmarkStatus::Result;
                    self.disable_radio_listen(radio, spi, delay);
                }
            }
            _ => {}
        }
    }

    fn update_settings(&mut self, settings: &mut RadioState) {
        //this module only reads the settings
        self.radio_state.team = settings.team;
        self.radio_state.robot_id = settings.robot_id;
        self.radio_state.conn_acks_results = settings.conn_acks_results;
        self.radio_state.conn_acks_attempts = settings.conn_acks_attempts;
    }

    fn next_module(&mut self) -> NextModule {
        if self.return_menu_flag {
            self.return_menu_flag = false;
            return NextModule::Menu;
        }

        NextModule::None
    }

    fn reset(&mut self) {
        self.return_menu_flag = false;
        self.benchmark_state.benchmark_active = false;
        self.benchmark_state.state = BenchmarkStatus::Idle;
        self.input_state.first_read_flag = true;
        self.config_radio_flag = true;
    }
}
