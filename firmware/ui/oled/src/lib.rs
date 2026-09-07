#![no_std]
#![forbid(unsafe_code)]

use rip_status_view::{HealthState, StatusPage, StatusView};

pub const OLED_WIDTH: usize = 128;
pub const OLED_PAGE_COUNT: usize = 8;
pub const OLED_TEXT_ROWS: usize = 8;
pub const OLED_TEXT_COLUMNS: usize = 21;
pub const OLED_BUFFER_SIZE: usize = OLED_WIDTH * OLED_PAGE_COUNT;

const CMD_LOWER_COLUMN: u8 = 0x00;
const CMD_HIGHER_COLUMN: u8 = 0x10;
const CMD_MEMORY_MODE: u8 = 0x20;
const CMD_FADE_BLINKING: u8 = 0x23;
const CMD_DEACTIVATE_SCROLL: u8 = 0x2e;
const CMD_START_LINE: u8 = 0x40;
const CMD_CONTRAST: u8 = 0x81;
const CMD_CHARGE_PUMP: u8 = 0x8d;
const CMD_SEGMENT_REMAP: u8 = 0xa1;
const CMD_RESUME_RAM: u8 = 0xa4;
const CMD_NORMAL_DISPLAY: u8 = 0xa6;
const CMD_MULTIPLEX: u8 = 0xa8;
const CMD_INTERNAL_IREF: u8 = 0xad;
const CMD_DISPLAY_OFF: u8 = 0xae;
const CMD_DISPLAY_ON: u8 = 0xaf;
const CMD_PAGE_BASE: u8 = 0xb0;
const CMD_COM_SCAN_DEC: u8 = 0xc8;
const CMD_DISPLAY_OFFSET: u8 = 0xd3;
const CMD_DISPLAY_CLOCK: u8 = 0xd5;
const CMD_ZOOM: u8 = 0xd6;
const CMD_PRECHARGE: u8 = 0xd9;
const CMD_COM_PINS: u8 = 0xda;
const CMD_VCOM_DESELECT: u8 = 0xdb;

const MEMORY_MODE_PAGE: u8 = 0x02;
const COM_PINS_ALTERNATIVE: u8 = 0x12;
const CLOCK_DEFAULT: u8 = 0x80;
const PRECHARGE_DEFAULT: u8 = 0xf1;
const IREF_INTERNAL_19UA: u8 = 0x10;
const VCOM_0P77: u8 = 0x20;
const CHARGE_PUMP_ON: u8 = 0x14;

pub trait OledIo {
    fn reset(&mut self);
    fn write_command(&mut self, command: u8);
    fn write_data(&mut self, data: &[u8]);
}

pub struct Ssd1315<I> {
    io: I,
    buffer: [u8; OLED_BUFFER_SIZE],
    contrast: u8,
    dirty_pages: u8,
    flush_page: u8,
    flush_column: u8,
    flush_active: bool,
    initialized: bool,
}

impl<I> Ssd1315<I>
where
    I: OledIo,
{
    pub const fn new(io: I, contrast: u8) -> Self {
        Self {
            io,
            buffer: [0; OLED_BUFFER_SIZE],
            contrast,
            dirty_pages: 0,
            flush_page: 0,
            flush_column: 0,
            flush_active: false,
            initialized: false,
        }
    }

    pub fn init(&mut self) {
        self.io.reset();
        self.command(CMD_DISPLAY_OFF);
        self.command_pair(CMD_MEMORY_MODE, MEMORY_MODE_PAGE);
        self.command(CMD_START_LINE);
        self.command_pair(CMD_FADE_BLINKING, 0x00);
        self.command(CMD_DEACTIVATE_SCROLL);
        self.command_pair(CMD_ZOOM, 0x00);
        self.command_pair(CMD_CONTRAST, self.contrast);
        self.command(CMD_SEGMENT_REMAP);
        self.command(CMD_COM_SCAN_DEC);
        self.command(CMD_NORMAL_DISPLAY);
        self.command_pair(CMD_MULTIPLEX, 0x3f);
        self.command_pair(CMD_DISPLAY_OFFSET, 0x00);
        self.command_pair(CMD_DISPLAY_CLOCK, CLOCK_DEFAULT);
        self.command_pair(CMD_PRECHARGE, PRECHARGE_DEFAULT);
        self.command_pair(CMD_INTERNAL_IREF, IREF_INTERNAL_19UA);
        self.command_pair(CMD_COM_PINS, COM_PINS_ALTERNATIVE);
        self.command_pair(CMD_VCOM_DESELECT, VCOM_0P77);
        self.command_pair(CMD_CHARGE_PUMP, CHARGE_PUMP_ON);
        self.command(CMD_RESUME_RAM);
        self.command(CMD_DISPLAY_ON);
        self.buffer.fill(0);
        self.dirty_pages = 0xff;
        self.flush_active = false;
        self.initialized = true;
    }

    pub fn set_contrast(&mut self, contrast: u8) {
        self.contrast = contrast;
        if self.initialized {
            self.command_pair(CMD_CONTRAST, contrast);
        }
    }

    pub const fn contrast(&self) -> u8 {
        self.contrast
    }

    pub const fn is_idle(&self) -> bool {
        self.dirty_pages == 0 && !self.flush_active
    }

    pub fn write_frame(&mut self, frame: &OledTextFrame) {
        for row in 0..OLED_TEXT_ROWS {
            self.write_line(row, frame.row(row));
        }
    }

    /// Service at most `max_data_bytes` of display RAM traffic.
    ///
    /// This method deliberately exposes a bounded background slice so the
    /// write-only software-SPI transport cannot turn one UI refresh into an
    /// unbounded full-frame burst in the control workload.
    pub fn service(&mut self, max_data_bytes: usize) {
        if !self.initialized || max_data_bytes == 0 {
            return;
        }
        if !self.flush_active {
            for page in 0..OLED_PAGE_COUNT {
                if self.dirty_pages & (1_u8 << page) != 0 {
                    self.flush_page = page as u8;
                    self.flush_column = 0;
                    self.flush_active = true;
                    self.command(CMD_PAGE_BASE | self.flush_page);
                    self.command(CMD_LOWER_COLUMN);
                    self.command(CMD_HIGHER_COLUMN);
                    break;
                }
            }
        }
        if !self.flush_active {
            return;
        }

        let remaining = OLED_WIDTH - usize::from(self.flush_column);
        let transfer = remaining.min(max_data_bytes);
        let base = usize::from(self.flush_page) * OLED_WIDTH + usize::from(self.flush_column);
        self.io.write_data(&self.buffer[base..base + transfer]);
        self.flush_column = (usize::from(self.flush_column) + transfer) as u8;
        if usize::from(self.flush_column) >= OLED_WIDTH {
            self.dirty_pages &= !(1_u8 << self.flush_page);
            self.flush_active = false;
            self.flush_column = 0;
        }
    }

    pub fn into_inner(self) -> I {
        self.io
    }

    fn write_line(&mut self, row: usize, text: &[u8; OLED_TEXT_COLUMNS]) {
        let mut row_buffer = [0_u8; OLED_WIDTH];
        for (character_index, &character) in text.iter().enumerate() {
            let glyph = glyph_for_character(character);
            let offset = character_index * 6;
            row_buffer[offset..offset + 5].copy_from_slice(&glyph);
        }
        let base = row * OLED_WIDTH;
        if self.buffer[base..base + OLED_WIDTH] != row_buffer {
            self.buffer[base..base + OLED_WIDTH].copy_from_slice(&row_buffer);
            self.dirty_pages |= 1_u8 << row;
        }
    }

    fn command(&mut self, command: u8) {
        self.io.write_command(command);
    }

    fn command_pair(&mut self, command: u8, value: u8) {
        self.command(command);
        self.command(value);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OledTextFrame {
    rows: [[u8; OLED_TEXT_COLUMNS]; OLED_TEXT_ROWS],
}

impl OledTextFrame {
    pub const fn blank() -> Self {
        Self {
            rows: [[b' '; OLED_TEXT_COLUMNS]; OLED_TEXT_ROWS],
        }
    }

    pub const fn row(&self, index: usize) -> &[u8; OLED_TEXT_COLUMNS] {
        &self.rows[index]
    }
}

pub struct OledRenderer;

impl OledRenderer {
    pub fn render(view: StatusView, page: StatusPage) -> OledTextFrame {
        let mut frame = OledTextFrame::blank();
        match page {
            StatusPage::Status => render_status(&mut frame, view),
            StatusPage::Sensor => render_sensor(&mut frame, view),
            StatusPage::Safety => render_safety(&mut frame, view),
            StatusPage::Control => render_control(&mut frame, view),
            StatusPage::Maintenance => render_maintenance(&mut frame, view),
        }
        frame
    }
}

fn render_status(frame: &mut OledTextFrame, view: StatusView) {
    write_bytes(&mut frame.rows[0], 0, b"STATUS 1/5");
    write_label_value(&mut frame.rows[1], b"CYCLE ", cycle_name(view.cycle));
    write_label_value(
        &mut frame.rows[2],
        b"REGIME ",
        regime_name(view.control_regime),
    );
    write_label_value(&mut frame.rows[3], b"TIMING ", health_name(view.timing));
    write_label_value(&mut frame.rows[4], b"WATCH ", health_name(view.watchdog));
    write_label_value(&mut frame.rows[5], b"AUTH ", yes_no(view.authorized));
    write_label_value(
        &mut frame.rows[6],
        b"TELEM ",
        on_off(view.telemetry_enabled),
    );
    write_bytes(&mut frame.rows[7], 0, b"M:NEXT X:BACK");
}

fn render_sensor(frame: &mut OledTextFrame, view: StatusView) {
    write_bytes(&mut frame.rows[0], 0, b"SENSOR 2/5");
    write_signed(&mut frame.rows[1], b"TH mrad ", view.theta_mrad);
    write_signed(&mut frame.rows[2], b"TD mr/s ", view.theta_dot_mrad_s);
    write_signed(&mut frame.rows[3], b"PH mrad ", view.phi_mrad);
    write_signed(&mut frame.rows[4], b"PD mr/s ", view.phi_dot_mrad_s);
    write_unsigned(&mut frame.rows[5], b"ADC ", u32::from(view.pendulum_adc));
    write_signed(&mut frame.rows[6], b"ENC ", view.arm_encoder_count);
    write_bytes(&mut frame.rows[7], 0, b"M:NEXT X:BACK");
}

fn render_safety(frame: &mut OledTextFrame, view: StatusView) {
    write_bytes(&mut frame.rows[0], 0, b"SAFETY 3/5");
    write_label_value(&mut frame.rows[1], b"TIMING ", health_name(view.timing));
    write_label_value(&mut frame.rows[2], b"WATCH ", health_name(view.watchdog));
    write_hex32(&mut frame.rows[3], b"QUAL 0x", view.qualification_reasons);
    write_hex32(&mut frame.rows[4], b"AUTH 0x", view.authority_reasons);
    write_unsigned(&mut frame.rows[5], b"MISS ", view.inferred_missed_ticks);
    write_unsigned(&mut frame.rows[6], b"OVERRUN ", view.deadline_overruns);
    write_bytes(&mut frame.rows[7], 0, b"M:NEXT X:BACK");
}

fn render_control(frame: &mut OledTextFrame, view: StatusView) {
    write_bytes(&mut frame.rows[0], 0, b"CONTROL 4/5");
    write_label_value(
        &mut frame.rows[1],
        b"REGIME ",
        regime_name(view.control_regime),
    );
    write_signed(&mut frame.rows[2], b"DEM uNm ", view.demand_torque_unm);
    write_signed(&mut frame.rows[3], b"CMD ppm ", view.bounded_command_ppm);
    write_signed(&mut frame.rows[4], b"PRED uNm ", view.predicted_torque_unm);
    write_label_value(&mut frame.rows[5], b"SAT ", yes_no(view.actuator_saturated));
    write_label_value(&mut frame.rows[6], b"AUTH ", yes_no(view.authorized));
    write_bytes(&mut frame.rows[7], 0, b"M:NEXT X:BACK");
}

fn render_maintenance(frame: &mut OledTextFrame, view: StatusView) {
    write_bytes(&mut frame.rows[0], 0, b"MAINT 5/5");
    write_label_value(
        &mut frame.rows[1],
        b"MOTOR SINK ",
        if view.motor_sink_bound {
            b"BOUND"
        } else {
            b"UNBOUND"
        },
    );
    write_label_value(
        &mut frame.rows[2],
        b"D2 OUTPUT ",
        if view.motor_sink_bound {
            b"AUTH"
        } else {
            b"SAFE OFF"
        },
    );
    write_label_value(
        &mut frame.rows[3],
        b"TELEM ",
        on_off(view.telemetry_enabled),
    );
    write_bytes(&mut frame.rows[4], 0, b"USER:TELEM TOGGLE");
    write_bytes(&mut frame.rows[5], 0, b"+/-:CONTRAST");
    write_bytes(&mut frame.rows[6], 0, b"SWD ACTIVE");
    write_bytes(&mut frame.rows[7], 0, b"M:NEXT X:BACK");
}

fn cycle_name(cycle: u32) -> &'static [u8] {
    match cycle {
        1 => b"PRIMED",
        2 => b"REJECT",
        3 => b"COMPUTED",
        4 => b"ERROR",
        _ => b"IDLE",
    }
}

fn regime_name(regime: u32) -> &'static [u8] {
    match regime {
        1 => b"CAPTURE",
        2 => b"BALANCE",
        _ => b"SWING",
    }
}

fn health_name(health: HealthState) -> &'static [u8] {
    match health {
        HealthState::Unknown => b"UNKNOWN",
        HealthState::Ok => b"OK",
        HealthState::Late => b"LATE",
        HealthState::Fault => b"FAULT",
    }
}

fn yes_no(value: bool) -> &'static [u8] {
    if value {
        b"YES"
    } else {
        b"NO"
    }
}

fn on_off(value: bool) -> &'static [u8] {
    if value {
        b"ON"
    } else {
        b"OFF"
    }
}

fn write_label_value(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: &[u8]) {
    write_bytes(row, 0, label);
    write_bytes(row, label.len(), value);
}

fn write_signed(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: i32) {
    write_bytes(row, 0, label);
    let mut cursor = label.len();
    if cursor < row.len() {
        row[cursor] = if value < 0 { b'-' } else { b'+' };
        cursor += 1;
    }
    write_u32_decimal(row, cursor, value.unsigned_abs());
}

fn write_unsigned(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: u32) {
    write_bytes(row, 0, label);
    write_u32_decimal(row, label.len(), value);
}

fn write_hex32(row: &mut [u8; OLED_TEXT_COLUMNS], label: &[u8], value: u32) {
    write_bytes(row, 0, label);
    for (offset, shift) in (0..8).rev().enumerate() {
        let cursor = label.len() + offset;
        if cursor >= row.len() {
            break;
        }
        let nibble = ((value >> (shift * 4)) & 0x0f) as u8;
        row[cursor] = if nibble < 10 {
            b'0' + nibble
        } else {
            b'A' + nibble - 10
        };
    }
}

fn write_u32_decimal(row: &mut [u8; OLED_TEXT_COLUMNS], start: usize, mut value: u32) {
    let mut digits = [0_u8; 10];
    let mut count = 0;
    loop {
        digits[count] = (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 || count == digits.len() {
            break;
        }
    }
    for index in (0..count).rev() {
        let position = start + count - 1 - index;
        if position >= row.len() {
            break;
        }
        row[position] = b'0' + digits[index];
    }
}

fn write_bytes(row: &mut [u8; OLED_TEXT_COLUMNS], start: usize, bytes: &[u8]) {
    for (offset, &byte) in bytes.iter().enumerate() {
        let index = start + offset;
        if index >= row.len() {
            break;
        }
        row[index] = byte;
    }
}

fn glyph_for_character(mut character: u8) -> [u8; 5] {
    if character >= b'a' && character <= b'z' {
        character = character - b'a' + b'A';
    }
    if character >= b'0' && character <= b'9' {
        return DIGIT_FONT[(character - b'0') as usize];
    }
    if character >= b'A' && character <= b'Z' {
        return UPPER_FONT[(character - b'A') as usize];
    }
    match character {
        b'+' => [0x08, 0x08, 0x3e, 0x08, 0x08],
        b'-' => [0x08, 0x08, 0x08, 0x08, 0x08],
        b'.' => [0x00, 0x60, 0x60, 0x00, 0x00],
        b'/' => [0x20, 0x10, 0x08, 0x04, 0x02],
        b':' => [0x00, 0x36, 0x36, 0x00, 0x00],
        b'%' => [0x63, 0x13, 0x08, 0x64, 0x63],
        b'=' => [0x14, 0x14, 0x14, 0x14, 0x14],
        b'_' => [0x40, 0x40, 0x40, 0x40, 0x40],
        b' ' => [0; 5],
        _ => [0x02, 0x01, 0x51, 0x09, 0x06],
    }
}

const DIGIT_FONT: [[u8; 5]; 10] = [
    [0x3e, 0x51, 0x49, 0x45, 0x3e],
    [0x00, 0x42, 0x7f, 0x40, 0x00],
    [0x42, 0x61, 0x51, 0x49, 0x46],
    [0x21, 0x41, 0x45, 0x4b, 0x31],
    [0x18, 0x14, 0x12, 0x7f, 0x10],
    [0x27, 0x45, 0x45, 0x45, 0x39],
    [0x3c, 0x4a, 0x49, 0x49, 0x30],
    [0x01, 0x71, 0x09, 0x05, 0x03],
    [0x36, 0x49, 0x49, 0x49, 0x36],
    [0x06, 0x49, 0x49, 0x29, 0x1e],
];

const UPPER_FONT: [[u8; 5]; 26] = [
    [0x7e, 0x11, 0x11, 0x11, 0x7e],
    [0x7f, 0x49, 0x49, 0x49, 0x36],
    [0x3e, 0x41, 0x41, 0x41, 0x22],
    [0x7f, 0x41, 0x41, 0x22, 0x1c],
    [0x7f, 0x49, 0x49, 0x49, 0x41],
    [0x7f, 0x09, 0x09, 0x09, 0x01],
    [0x3e, 0x41, 0x49, 0x49, 0x7a],
    [0x7f, 0x08, 0x08, 0x08, 0x7f],
    [0x00, 0x41, 0x7f, 0x41, 0x00],
    [0x20, 0x40, 0x41, 0x3f, 0x01],
    [0x7f, 0x08, 0x14, 0x22, 0x41],
    [0x7f, 0x40, 0x40, 0x40, 0x40],
    [0x7f, 0x02, 0x0c, 0x02, 0x7f],
    [0x7f, 0x04, 0x08, 0x10, 0x7f],
    [0x3e, 0x41, 0x41, 0x41, 0x3e],
    [0x7f, 0x09, 0x09, 0x09, 0x06],
    [0x3e, 0x41, 0x51, 0x21, 0x5e],
    [0x7f, 0x09, 0x19, 0x29, 0x46],
    [0x46, 0x49, 0x49, 0x49, 0x31],
    [0x01, 0x01, 0x7f, 0x01, 0x01],
    [0x3f, 0x40, 0x40, 0x40, 0x3f],
    [0x1f, 0x20, 0x40, 0x20, 0x1f],
    [0x3f, 0x40, 0x38, 0x40, 0x3f],
    [0x63, 0x14, 0x08, 0x14, 0x63],
    [0x07, 0x08, 0x70, 0x08, 0x07],
    [0x61, 0x51, 0x49, 0x45, 0x43],
];

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct MockIo {
        commands: Vec<u8>,
        data_bytes: usize,
        resets: u32,
    }

    impl OledIo for MockIo {
        fn reset(&mut self) {
            self.resets += 1;
        }

        fn write_command(&mut self, command: u8) {
            self.commands.push(command);
        }

        fn write_data(&mut self, data: &[u8]) {
            self.data_bytes += data.len();
        }
    }

    fn view() -> StatusView {
        StatusView {
            cycle: 3,
            control_regime: 2,
            timing: HealthState::Ok,
            watchdog: HealthState::Ok,
            authorized: false,
            actuator_saturated: false,
            telemetry_enabled: true,
            motor_sink_bound: false,
            theta_mrad: -45,
            phi_mrad: 123,
            ..StatusView::default()
        }
    }

    #[test]
    fn status_frame_contains_runtime_state() {
        let frame = OledRenderer::render(view(), StatusPage::Status);
        assert!(frame.row(0).starts_with(b"STATUS 1/5"));
        assert!(frame.row(2).starts_with(b"REGIME BALANCE"));
        assert!(frame.row(6).starts_with(b"TELEM ON"));
    }

    #[test]
    fn service_limits_each_background_transfer() {
        let mut display = Ssd1315::new(MockIo::default(), 0x7f);
        display.init();
        display.service(8);
        let io = display.into_inner();
        assert_eq!(io.resets, 1);
        assert_eq!(io.data_bytes, 8);
    }
}
