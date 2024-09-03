#![no_std]
#![feature(wrapping_next_power_of_two)]
#![feature(type_alias_impl_trait)]
#![feature(atomic_from_mut)]
#![feature(array_chunks)]
#![feature(inline_const)]
#![feature(slice_first_last_chunk)]
#![feature(const_mut_refs)]
#![no_main]

use panic_probe as _;
use rtt_target::{rprint, rprintln, rtt_init_print};

use hal::gpio::GpioExt;
use hal::prelude::*;
use stm32f4xx_hal as hal;

use seq_macro::seq;

use embedded_graphics::{draw_target::*, pixelcolor::Rgb888, prelude::*, primitives::*};

#[rtic::app(device = hal::pac, peripherals=true)]
mod app {

    use core::{
        fmt::{write, Formatter},
        iter::{Cycle, Flatten, Skip, StepBy},
        slice::{ChunksExact, Iter},
        sync::atomic::Ordering::*,
        sync::atomic::{AtomicPtr, AtomicU32, AtomicU8, AtomicUsize},
    };

    use embedded_graphics::{
        mono_font::{
            ascii::FONT_7X14_BOLD, iso_8859_1::FONT_8X13_ITALIC, iso_8859_10::FONT_4X6,
            MonoTextStyle,
        },
        pixelcolor::raw::RawU4,
        text::Text,
    };
    use hal::{
        gpio::PinState,
        pac::{
            GPIOC, GPIOD, GPIOE, TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5,
            TIM6, TIM7, TIM8, TIM9,
        },
        timer::Channel::*,
        timer::{Channel1, CounterUs, DelayUs, Polarity, PwmChannel},
        timer::{Channel2, PwmHz},
        timer::{Channel3, Event},
        timer::{Channel4, CounterHz},
    };
    use owo_colors::colors::*;
    use owo_colors::OwoColorize;
    use rtt_target::rdbg;

    use super::*;

    // const fn image_load() -> [[u8; 16 * 16 * 16 * 3]; 2] {
    //     let test = [[0xFF; 16 * 16 * 16 * 4 * (16 - 1)];
    //     // test[0].copy_from_slice(include_bytes!(r"..\image.bmp").into());
    //     test
    // }

    #[shared]
    struct Shared {
        graphics: Graphics,
    }

    static FBPOOL: [[AtomicU32; 4]; 16 * 8 * 2] =
        [const { [const { AtomicU32::new(0) }; 4] }; 16 * 8 * 2];
    static FRAME_OFFSET: AtomicUsize = const { AtomicUsize::new(0) };
    static CURRENT_BUF_OFFSET: AtomicUsize = const { AtomicUsize::new(0) };
    static DRAWING_BUF_OFFSET: AtomicUsize = const { AtomicUsize::new(16) };
    #[derive(Debug)]
    pub struct Graphics {
        layer: u8,
        current_gen: u32,
    }

    #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
    pub enum RgbBinary {
        Black = 0,
        Blue,
        Green,
        Cyan,
        Red,
        Magenta,
        Yellow,
        White,
    }

    impl PixelColor for RgbBinary {
        type Raw = ();
    }

    impl Graphics {
        pub fn flush(&self) {
            while FRAME_OFFSET.load(Relaxed) != 0 {}
            DRAWING_BUF_OFFSET.store(
                CURRENT_BUF_OFFSET.swap(DRAWING_BUF_OFFSET.load(Relaxed), Relaxed),
                Relaxed,
            );
        }
    }

    impl OriginDimensions for Graphics {
        fn size(&self) -> Size {
            Size::new(16, 16)
        }
    }

    impl DrawTarget for Graphics {
        type Color = RgbBinary;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(coord, color) in pixels.into_iter() {
                let (x, y) = coord.into();
                let (x, y) = (15 & x as u32, 15 & y as u8);
                if let Some([r, g, b, _]) = FBPOOL
                    .chunks_exact(8)
                    .skip(DRAWING_BUF_OFFSET.load(Relaxed))
                    .take(16)
                    .nth(self.layer as usize)
                    .unwrap_or_default()
                    .iter()
                    .nth(y as usize)
                {
                    let fill = 0b1 << (x + if y > 7 { 16 } else { 0 });
                    let color = color as u32;
                    if color & 0b100 > 0 {
                        r.fetch_or(fill, Relaxed);
                    };
                    if color & 0b010 > 0 {
                        g.fetch_or(fill, Relaxed);
                    };
                    if color & 0b001 > 0 {
                        b.fetch_or(fill, Relaxed);
                    };
                }
            }
            Ok(())
        }

        //TODO : Remove if in for loops
        fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
            let (x, y) = area.top_left.into();
            let (x, y) = (x as u32, y as u32);
            let (w, h) = area.size.into();

            let fill = (0xFFFFFFFF << x) ^ (0xFFFFFFFF >> (x + y));
            let buf = FBPOOL
                .chunks_exact(8)
                .skip(DRAWING_BUF_OFFSET.load(Relaxed))
                .take(16)
                .nth(self.layer as usize)
                .unwrap_or_default()
                .iter()
                .cycle()
                .take(16)
                .enumerate()
                .take(h as usize)
                .map(|(i, x)| (if i > 7 { fill << 16 } else { fill }, x));
            let color = color as u32;
            for (fill, [R, G, B, Gen]) in buf {
                if color & 0b100 > 0 {
                    R.fetch_or(fill, Relaxed);
                };
                if color & 0b010 > 0 {
                    G.fetch_or(fill, Relaxed);
                };
                if color & 0b001 > 0 {
                    B.fetch_or(fill, Relaxed);
                };
            }

            Ok(())
        }
    }

    impl core::fmt::Display for Graphics {
        fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
            for (r, g, b) in FBPOOL
                .chunks_exact(8)
                .skip(DRAWING_BUF_OFFSET.load(Relaxed))
                .take(16)
                .nth(self.layer as usize)
                .unwrap_or_default()
                .iter()
                .cycle()
                .take(16)
                .enumerate()
                .map(|(i, [r, g, b, _])| {
                    let offset = if i > 7 { 16 } else { 0 };
                    (
                        r.load(Relaxed) >> offset,
                        g.load(Relaxed) >> offset,
                        b.load(Relaxed) >> offset,
                    )
                })
            {
                for c in (0..16).map(|x| ((r >> x) & 0b1, (g >> x) & 0b1, (b >> x) & 0b1)) {
                    write!(
                        f,
                        "{}",
                        "• ".color(owo_colors::Rgb(
                            255 * c.0 as u8,
                            255 * c.1 as u8,
                            255 * c.2 as u8
                        ))
                    )
                    .ok();
                }
                write!(f, "\n").ok();
            }

            Ok(())
        }
    }

    #[local]
    struct Local {
        timer7: CounterHz<TIM7>,
        timer6: DelayUs<TIM6>,
    }

    /// STM32 Init code
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!(NoBlockSkip, 65535);

        rprintln!("STM32 LED CUBE");

        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(168.MHz()).freeze();

        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        seq!(N in 0..=11{
            _ = gpioc.pc~N.into_push_pull_output_in_state(PinState::Low).set_speed(hal::gpio::Speed::VeryHigh);
        });
        seq!(N in 0..16{
            _ = gpiod.pd~N.into_push_pull_output_in_state(PinState::Low).set_speed(hal::gpio::Speed::VeryHigh);
        });
        seq!(N in 0..16{
            _ = gpioe.pe~N.into_push_pull_output_in_state(PinState::Low).set_speed(hal::gpio::Speed::VeryHigh);
        });

        let mut t7 = dp.TIM7.counter_hz(&clocks);

        // t7.start((1).Hz()).expect("Unable to start frame clock");

        // t7.listen(Event::Update);

        // rtic::pend(hal::pac::Interrupt::TIM7);

        let mut graphics = Graphics {
            layer: 0,
            current_gen: 0,
            // buf: fbpoo,
            // buf2: unsafe { &mut FBPOOL[1] },
            // layer: 0,
        };

        // Circle::new(Point::new(5, 5), 1)
        //     .into_styled(PrimitiveStyle::with_stroke(RgbBinary::Red, 1))
        //     .draw(&mut graphics)
        //     .unwrap();

        let style = MonoTextStyle::new(&FONT_4X6, RgbBinary::Red);
        for i in 0..16 {
            graphics.layer = i;
            Text::new("HeHe", Point::new(0, 6), style)
                .draw(&mut graphics)
                .unwrap();
        }

        // Pixel(Point::new(0, 0), Rgb888::WHITE)
        //     .draw(&mut graphics)
        //     .unwrap();
        // graphics.draw_iter([])

        // Rectangle::new(Point::new(0, 0), Size::new(16, 16))
        //     .into_styled(PrimitiveStyle::with_fill(Rgb888::WHITE))
        //     .draw(&mut graphics)
        //     .ok();

        for i in 0..16 {
            graphics.layer = i;
            rprintln!("Layer: {}\n{}", i, graphics);
        }

        // graphics.flush();

        let timer6 = dp.TIM6.delay_us(&clocks);

        // draw::spawn().ok();
        (Shared { graphics }, Local { timer6, timer7: t7 })
    }

    #[idle(shared=[&graphics])]
    fn idle(_: idle::Context) -> ! {
        // ctx.shared.buf.lock(|buf| {
        //     buf.iter_mut().enumerate().for_each(|(i, w)| *w = i as u8);
        // });

        loop {
            // rprintln!("hehhe");
            // rprintln!("Working");
            // cortex_m::asm::wfi();
        }
    }

    // #[task(shared=[&graphics])]
    // async fn draw(_: draw::Context) {
    //     // _ = ctx.shared.graphics;
    // }

    #[inline]
    fn update_frame(buffer: &[[AtomicU32; 4]], layer: u8) {
        let (gpioc, gpiod, gpioe) = unsafe {
            (
                GPIOC::ptr().as_ref().unwrap_unchecked(),
                GPIOD::ptr().as_ref().unwrap_unchecked(),
                GPIOE::ptr().as_ref().unwrap_unchecked(),
            )
        };
        for (i, (r, g, b)) in buffer
            .iter()
            .take(8)
            .map(|[r, g, b, _]| (r.load(Relaxed), g.load(Relaxed), b.load(Relaxed)))
            .enumerate()
        {
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111111 << 16) | i as u32 | 0b001000) });
            gpiod.odr.write(|w| unsafe { w.bits(r) });
            gpioe.odr.write(|w| unsafe { w.bits(r >> 16) });
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111000 << 16) | 0b010000) });
            gpiod.odr.write(|w| unsafe { w.bits(g) });
            gpioe.odr.write(|w| unsafe { w.bits(g >> 16) });
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111000 << 16) | 0b100000) });
            gpiod.odr.write(|w| unsafe { w.bits(b) });
            gpioe.odr.write(|w| unsafe { w.bits(b >> 16) });
        }
        gpioc
            .odr
            .write(|w| unsafe { w.bits((((layer as u32) << 6) & 0b1111000000) as u32) });
    }

    #[task(binds = TIM7, shared=[], local=[timer6, timer7])]
    fn frame_update_subroutine(ctx: frame_update_subroutine::Context) {
        // rprintln!("Update");
        // let timer6 = ctx.local.timer6;
        // timer6.start(2.micros()).unwrap();

        let frame_offset = FRAME_OFFSET.fetch_add(32, Relaxed);
        let layer = (frame_offset >> 5) & 0b1111;

        update_frame(
            FBPOOL
                .chunks_exact(8)
                .skip(DRAWING_BUF_OFFSET.load(Relaxed))
                .take(16)
                .nth(layer)
                .unwrap_or_default(),
            layer as u8,
        );
        // let frame_offset = graphics.buf.load(Relaxed) + frame_offset;
        // let fb = buf[frame_offset..(frame_offset + 16)].iter();
        // let frame_offset = frame_offset + 16 * 8 * 4;
        // let mut fb = fb.chain(buf[frame_offset..(frame_offset + 16)].iter());

        // seq!(N in 0..32{
        //     ctx.local.pwm_channels.N.set_duty(fb.next().map_or(0, |v| v.load(Relaxed)) as u16);
        // });

        // let duration = timer6.now().duration_since_epoch();
        // timer6.cancel().unwrap();

        // rprintln!("Timer elasped {}", duration);

        ctx.local.timer7.clear_interrupt(Event::Update);
    }
}
