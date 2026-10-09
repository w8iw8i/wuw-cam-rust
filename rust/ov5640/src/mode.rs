//! JPEG output mode, ported from esp32-camera `set_framesize`,
//! `set_image_options`, `set_pll` and `set_quality` (JPEG only, no flip or mirror).

use crate::Op::{self, Write};
use heapless::Vec;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Aspect {
    R4x3,
    R3x2,
    R16x10,
    R5x3,
    R16x9,
    R21x9,
    R5x4,
    R1x1,
    R9x16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameSize {
    pub w: u16,
    pub h: u16,
    pub aspect: Aspect,
}

pub const QVGA: FrameSize = FrameSize {
    w: 320,
    h: 240,
    aspect: Aspect::R4x3,
};
pub const SVGA: FrameSize = FrameSize {
    w: 800,
    h: 600,
    aspect: Aspect::R4x3,
};
pub const XGA: FrameSize = FrameSize {
    w: 1024,
    h: 768,
    aspect: Aspect::R4x3,
};
pub const HD: FrameSize = FrameSize {
    w: 1280,
    h: 720,
    aspect: Aspect::R16x9,
};

/// Number of ops `jpeg_mode` returns.
pub const JPEG_MODE_LEN: usize = 37;
const CAP: usize = 48;
const _: () = assert!(JPEG_MODE_LEN <= CAP);

/// One row of `ratio_table` in vendor/ov5640_settings.h.
struct Ratio {
    max_w: u16,
    max_h: u16,
    start_x: u16,
    start_y: u16,
    end_x: u16,
    end_y: u16,
    offset_x: u16,
    offset_y: u16,
    total_x: u16,
    total_y: u16,
}

const fn r(v: [u16; 10]) -> Ratio {
    Ratio {
        max_w: v[0],
        max_h: v[1],
        start_x: v[2],
        start_y: v[3],
        end_x: v[4],
        end_y: v[5],
        offset_x: v[6],
        offset_y: v[7],
        total_x: v[8],
        total_y: v[9],
    }
}

fn ratio(a: Aspect) -> Ratio {
    #[rustfmt::skip]
    let row = match a {
        //                   mw,   mh,  sx,  sy,   ex,   ey, ox, oy,   tx,   ty
        Aspect::R4x3   => [2560, 1920,   0,   0, 2623, 1951, 32, 16, 2844, 1968],
        Aspect::R3x2   => [2560, 1704,   0, 110, 2623, 1843, 32, 16, 2844, 1752],
        Aspect::R16x10 => [2560, 1600,   0, 160, 2623, 1791, 32, 16, 2844, 1648],
        Aspect::R5x3   => [2560, 1536,   0, 192, 2623, 1759, 32, 16, 2844, 1584],
        Aspect::R16x9  => [2560, 1440,   0, 240, 2623, 1711, 32, 16, 2844, 1488],
        Aspect::R21x9  => [2560, 1080,   0, 420, 2623, 1531, 32, 16, 2844, 1128],
        Aspect::R5x4   => [2400, 1920,  80,   0, 2543, 1951, 32, 16, 2684, 1968],
        Aspect::R1x1   => [1920, 1920, 320,   0, 2543, 1951, 32, 16, 2684, 1968],
        Aspect::R9x16  => [1088, 1920, 736,   0, 1887, 1951, 32, 16, 1884, 1968],
    };
    r(row)
}

fn area(s: &FrameSize) -> u32 {
    u32::from(s.w) * u32::from(s.h)
}

/// Register steps that switch a reset + SENSOR_FMT_JPEG sensor to JPEG at `size`.
pub fn jpeg_mode(size: &FrameSize, xclk_hz: u32, quality: u8) -> Vec<Op, CAP> {
    let mut v: Vec<Op, CAP> = Vec::new();
    let mut put = |op: Op| v.push(op).unwrap();

    let (w, h) = (size.w, size.h);
    let s = ratio(size.aspect);
    let binning = w <= s.max_w / 2 && h <= s.max_h / 2;
    let scale = !((w == s.max_w && h == s.max_h) || (w == s.max_w / 2 && h == s.max_h / 2));

    // set_framesize: window, output size, totals, offsets.
    let (tx, ty, ox, oy) = if !binning {
        (s.total_x, s.total_y, s.offset_x, s.offset_y)
    } else if w > 920 {
        (
            s.total_x - 200,
            s.total_y / 2,
            s.offset_x / 2,
            s.offset_y / 2,
        )
    } else {
        (2060, s.total_y / 2, s.offset_x / 2, s.offset_y / 2)
    };
    let pairs = [
        (0x3800, s.start_x, s.start_y),
        (0x3804, s.end_x, s.end_y),
        (0x3808, w, h),
        (0x380C, tx, ty),
        (0x3810, ox, oy),
    ];
    for (reg, x, y) in pairs {
        let [xh, xl] = x.to_be_bytes();
        let [yh, yl] = y.to_be_bytes();
        put(Write(reg, xh));
        put(Write(reg + 1, xl));
        put(Write(reg + 2, yh));
        put(Write(reg + 3, yl));
    }
    put(Op::Modify(0x5001, 0x20, if scale { 0x20 } else { 0 }));

    // set_image_options: compression on, no flip or mirror.
    if binning {
        put(Write(0x3820, 0x01));
        put(Write(0x3821, 0x21));
        put(Write(0x4514, 0xAA));
        put(Write(0x4520, 0x0B));
        put(Write(0x3814, 0x31));
        put(Write(0x3815, 0x31));
    } else {
        put(Write(0x3820, 0x40));
        put(Write(0x3821, 0x20));
        put(Write(0x4514, 0x88));
        put(Write(0x4520, 0x10));
        put(Write(0x3814, 0x11));
        put(Write(0x3815, 0x11));
    }

    // set_pll(false, sys_mul, 4, 2, false, 2, true, 4).
    let a = area(size);
    let sys_mul: u8 = if a < area(&QVGA) || xclk_hz == 16_000_000 {
        160
    } else if a < area(&XGA) {
        180
    } else {
        200
    };
    let (sys_div, pre_div, pclk_root_div, pclk_div) = (4u8, 2u8, 2u8, 4u8);
    put(Write(0x3039, 0x00));
    put(Write(0x3034, 0x1A));
    put(Write(0x3035, 0x01 | ((sys_div & 0x0F) << 4)));
    put(Write(0x3036, sys_mul));
    put(Write(0x3037, pre_div & 0x0F));
    put(Write(0x3108, ((pclk_root_div & 0x03) << 4) | 0x06));
    put(Write(0x3824, pclk_div & 0x1F));
    put(Write(0x460C, 0x22));
    put(Write(0x3103, 0x13));

    // set_quality.
    put(Write(0x4407, quality & 0x3F));
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Op::{Modify as M, Write as W};

    #[test]
    fn svga_24mhz_q12() {
        #[rustfmt::skip]
        let want = [
            W(0x3800, 0x00), W(0x3801, 0x00), W(0x3802, 0x00), W(0x3803, 0x00),
            W(0x3804, 0x0A), W(0x3805, 0x3F), W(0x3806, 0x07), W(0x3807, 0x9F),
            W(0x3808, 0x03), W(0x3809, 0x20), W(0x380A, 0x02), W(0x380B, 0x58),
            W(0x380C, 0x08), W(0x380D, 0x0C), W(0x380E, 0x03), W(0x380F, 0xD8),
            W(0x3810, 0x00), W(0x3811, 0x10), W(0x3812, 0x00), W(0x3813, 0x08),
            M(0x5001, 0x20, 0x20),
            W(0x3820, 0x01), W(0x3821, 0x21), W(0x4514, 0xAA), W(0x4520, 0x0B),
            W(0x3814, 0x31), W(0x3815, 0x31),
            W(0x3039, 0x00), W(0x3034, 0x1A), W(0x3035, 0x41), W(0x3036, 0xB4),
            W(0x3037, 0x02), W(0x3108, 0x26), W(0x3824, 0x04), W(0x460C, 0x22),
            W(0x3103, 0x13),
            W(0x4407, 0x0C),
        ];
        assert_eq!(jpeg_mode(&SVGA, 24_000_000, 12).as_slice(), &want);
    }

    // HD, 16x9 row: max 2560x1440, so 1280x720 is exactly half.
    // binning (1280 <= 1280, 720 <= 720), no scale (exact half) -> 0x5001 bit 5 off.
    // start 0,240 -> 0x00F0; end 2623,1711 -> 0x0A3F,0x06AF; out 0x0500,0x02D0.
    // w > 920: total_x 2844-200 = 2644 = 0x0A54, total_y 1488/2 = 744 = 0x02E8.
    // offsets 32/2 = 16, 16/2 = 8. HD is past XGA -> sys_mul 200 = 0xC8.
    #[test]
    fn hd_24mhz_q12() {
        #[rustfmt::skip]
        let want = [
            W(0x3800, 0x00), W(0x3801, 0x00), W(0x3802, 0x00), W(0x3803, 0xF0),
            W(0x3804, 0x0A), W(0x3805, 0x3F), W(0x3806, 0x06), W(0x3807, 0xAF),
            W(0x3808, 0x05), W(0x3809, 0x00), W(0x380A, 0x02), W(0x380B, 0xD0),
            W(0x380C, 0x0A), W(0x380D, 0x54), W(0x380E, 0x02), W(0x380F, 0xE8),
            W(0x3810, 0x00), W(0x3811, 0x10), W(0x3812, 0x00), W(0x3813, 0x08),
            M(0x5001, 0x20, 0x00),
            W(0x3820, 0x01), W(0x3821, 0x21), W(0x4514, 0xAA), W(0x4520, 0x0B),
            W(0x3814, 0x31), W(0x3815, 0x31),
            W(0x3039, 0x00), W(0x3034, 0x1A), W(0x3035, 0x41), W(0x3036, 0xC8),
            W(0x3037, 0x02), W(0x3108, 0x26), W(0x3824, 0x04), W(0x460C, 0x22),
            W(0x3103, 0x13),
            W(0x4407, 0x0C),
        ];
        assert_eq!(jpeg_mode(&HD, 24_000_000, 12).as_slice(), &want);
    }

    fn sys_mul(size: &FrameSize, xclk_hz: u32) -> Op {
        jpeg_mode(size, xclk_hz, 12)[30]
    }

    #[test]
    fn sys_mul_by_size_and_xclk() {
        let small = FrameSize {
            w: 240,
            h: 240,
            aspect: Aspect::R1x1,
        };
        assert_eq!(sys_mul(&small, 24_000_000), W(0x3036, 0xA0));
        assert_eq!(sys_mul(&QVGA, 24_000_000), W(0x3036, 0xB4));
        assert_eq!(sys_mul(&XGA, 24_000_000), W(0x3036, 0xC8));
        assert_eq!(sys_mul(&HD, 16_000_000), W(0x3036, 0xA0));
    }

    #[test]
    fn full_size_skips_binning_and_scale() {
        let full = FrameSize {
            w: 2560,
            h: 1920,
            aspect: Aspect::R4x3,
        };
        let ops = jpeg_mode(&full, 24_000_000, 10);
        assert_eq!(
            ops[12..20],
            [
                W(0x380C, 0x0B),
                W(0x380D, 0x1C),
                W(0x380E, 0x07),
                W(0x380F, 0xB0),
                W(0x3810, 0x00),
                W(0x3811, 0x20),
                W(0x3812, 0x00),
                W(0x3813, 0x10),
            ]
        );
        assert_eq!(ops[20], M(0x5001, 0x20, 0x00));
        assert_eq!(ops[21], W(0x3820, 0x40));
        assert_eq!(ops[22], W(0x3821, 0x20));
    }

    #[test]
    fn length_matches_const_and_quality_is_masked() {
        let ops = jpeg_mode(&SVGA, 24_000_000, 0xFF);
        assert_eq!(ops.len(), JPEG_MODE_LEN);
        assert_eq!(ops[JPEG_MODE_LEN - 1], W(0x4407, 0x3F));
    }
}
