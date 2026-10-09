//! OV5640 control over SCCB (I2C, 7-bit address 0x3C, 16-bit registers).
#![no_std]

use embedded_hal::{delay::DelayNs, i2c::I2c};

#[rustfmt::skip]
pub mod regs;

pub mod mode;

pub const ADDR: u8 = 0x3C;
pub const CHIP_ID: u16 = 0x5640;
const REG_CHIP_ID_H: u16 = 0x300A;

/// One step of a register list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Write(u16, u8),
    DelayMs(u16),
    /// Read `reg`, replace the bits in `mask` with `value`, write it back.
    Modify(u16, u8, u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    WrongChip(u16),
}

pub struct Ov5640<I> {
    i2c: I,
}

impl<I: I2c> Ov5640<I> {
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    pub fn release(self) -> I {
        self.i2c
    }

    pub fn write(&mut self, reg: u16, val: u8) -> Result<(), Error<I::Error>> {
        let [h, l] = reg.to_be_bytes();
        self.i2c.write(ADDR, &[h, l, val]).map_err(Error::Bus)
    }

    pub fn read(&mut self, reg: u16) -> Result<u8, Error<I::Error>> {
        let mut v = [0u8];
        self.i2c
            .write_read(ADDR, &reg.to_be_bytes(), &mut v)
            .map_err(Error::Bus)?;
        Ok(v[0])
    }

    /// Reads 0x300A/0x300B and checks it is an OV5640.
    pub fn check_id(&mut self) -> Result<u16, Error<I::Error>> {
        let id = u16::from_be_bytes([self.read(REG_CHIP_ID_H)?, self.read(REG_CHIP_ID_H + 1)?]);
        if id == CHIP_ID {
            Ok(id)
        } else {
            Err(Error::WrongChip(id))
        }
    }

    pub fn apply(&mut self, list: &[Op], delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        for op in list {
            match *op {
                Op::Write(r, v) => self.write(r, v)?,
                Op::DelayMs(ms) => delay.delay_ms(ms.into()),
                Op::Modify(r, mask, v) => {
                    let old = self.read(r)?;
                    self.write(r, (old & !mask) | (v & mask))?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;
    use std::vec::Vec;

    #[derive(Default)]
    struct RecordingDelay {
        ms: Vec<u32>,
    }

    impl DelayNs for RecordingDelay {
        fn delay_ns(&mut self, _ns: u32) {}

        fn delay_ms(&mut self, ms: u32) {
            self.ms.push(ms);
        }
    }

    #[test]
    fn check_id_accepts_ov5640() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x30, 0x0A], vec![0x56]),
            T::write_read(ADDR, vec![0x30, 0x0B], vec![0x40]),
        ]);
        assert_eq!(Ov5640::new(bus.clone()).check_id(), Ok(0x5640));
        bus.done();
    }

    #[test]
    fn check_id_rejects_ov2640() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x30, 0x0A], vec![0x26]),
            T::write_read(ADDR, vec![0x30, 0x0B], vec![0x42]),
        ]);
        assert_eq!(
            Ov5640::new(bus.clone()).check_id(),
            Err(Error::WrongChip(0x2642))
        );
        bus.done();
    }

    #[test]
    fn apply_writes_big_endian_register_and_skips_delays_on_bus() {
        let mut bus = Mock::new(&[
            T::write(ADDR, vec![0x30, 0x08, 0x82]),
            T::write(ADDR, vec![0x31, 0x03, 0x13]),
        ]);
        let list = [
            Op::Write(0x3008, 0x82),
            Op::DelayMs(10),
            Op::Write(0x3103, 0x13),
        ];
        let mut delay = RecordingDelay::default();
        Ov5640::new(bus.clone())
            .apply(&list, &mut delay)
            .expect("mock bus");
        assert_eq!(delay.ms, [10]);
        bus.done();
    }

    #[test]
    fn modify_reads_then_writes_merged_value() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x50, 0x01], vec![0xA3]),
            T::write(ADDR, vec![0x50, 0x01, 0x83]),
        ]);
        let mut delay = RecordingDelay::default();
        Ov5640::new(bus.clone())
            .apply(&[Op::Modify(0x5001, 0x20, 0x00)], &mut delay)
            .expect("mock bus");
        bus.done();
    }

    #[test]
    fn modify_only_touches_masked_bits() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x50, 0x01], vec![0x83]),
            T::write(ADDR, vec![0x50, 0x01, 0xA3]),
        ]);
        let mut delay = RecordingDelay::default();
        Ov5640::new(bus.clone())
            .apply(&[Op::Modify(0x5001, 0x20, 0xFF)], &mut delay)
            .expect("mock bus");
        bus.done();
    }

    #[test]
    fn generated_default_table_starts_with_a_reset_and_is_nonempty() {
        assert!(regs::SENSOR_DEFAULT_REGS.len() > 50);
        assert_eq!(regs::SENSOR_DEFAULT_REGS[0], Op::Write(0x3008, 0x82));
    }
}
