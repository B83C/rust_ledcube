#![no_std]
#![feature(wrapping_next_power_of_two)]
#![feature(type_alias_impl_trait)]
#![feature(atomic_from_mut)]
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
        sync::atomic::{AtomicPtr, AtomicU32, AtomicU8, AtomicUsize},
    };

    use embedded_graphics::{
        mono_font::{iso_8859_10::FONT_4X6, MonoTextStyle},
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
    use rtt_target::rdbg;

    use super::*;

    // const fn image_load() -> [[u8; 16 * 16 * 16 * 3]; 2] {
    //     let test = [[0xFF; 16 * 16 * 16 * 4 * (16 - 1)];
    //     // test[0].copy_from_slice(include_bytes!(r"..\image.bmp").into());
    //     test
    // }

    #[shared]
    struct Shared {
        // #[lock_free]
        graphics: Graphics,
        // #[lock_free]
        // fbu: Cycle<Flatten<StepBy<ChunksExact<'static, u32>>>>,
        // #[lock_free]
        // fbd: Cycle<Flatten<StepBy<Skip<ChunksExact<'static, u32>>>>>,
    }

    #[derive(Debug)]
    pub struct Graphics {
        fbpool: &'static mut [AtomicU32],
        frame_offset: AtomicUsize,
        buf: AtomicUsize,
        buf2: AtomicUsize,
        layer: AtomicU8,
    }

    impl Graphics {
        pub fn flush(&mut self) {
            while self
                .frame_offset
                .load(core::sync::atomic::Ordering::Relaxed)
                != 0
            {}
            core::mem::swap(&mut self.buf, &mut self.buf2);
        }
    }

    impl OriginDimensions for Graphics {
        fn size(&self) -> Size {
            Size::new(16, 16)
        }
    }

    impl DrawTarget for Graphics {
        type Color = Rgb888;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(coord, color) in pixels.into_iter() {
                // let (x, y) = coord.into();
                // // let (x, y) = rdbg!(coord.into());
                // let layer = self.layer.load(core::sync::atomic::Ordering::Relaxed) as usize;
                // let buf = self.fbpool.as_mut();
                // let off = self.buf2.load(core::sync::atomic::Ordering::Relaxed);
                // let pos =
                //     (layer * 1024) + ((x as usize) & (16 - 1)) + ((y as usize) & (16 - 1)) * 64;
                // buf[off + pos + 0] = color.r().into();
                // buf[off + pos + 16] = color.g().into();
                // buf[off + pos + 32] = color.b().into();
                // self.buf2[(layer * 1024)
                //     + ((x as usize) & (16 - 1))
                //     + (((y as usize) & (16 - 1)) * 64)
                //     + 0] = color.r();
                // self.buf2[(layer * 1024)
                //     + ((x as usize) & (16 - 1))
                //     + (((y as usize) & (16 - 1)) * 64)
                //     + 16] = color.g();
                // self.buf2[(layer * 1024)
                //     + ((x as usize) & (16 - 1))
                //     + (((y as usize) & (16 - 1)) * 64)
                //     + 32] = color.b();
            }
            Ok(())
        }

        // fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        //     let (x, y) = area.top_left.into();
        //     let (w, h) = area.size.into();

        //     Ok(())
        // }
    }

    impl core::fmt::Display for Graphics {
        fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
            self.fbpool
                .chunks(16)
                .skip(16 * 16 * 4)
                .step_by(4)
                .take(16)
                .for_each(|x| {
                    x.iter().for_each(|b| {
                        if b.load(core::sync::atomic::Ordering::Relaxed) > 0 {
                            write!(f, "• ").ok();
                        } else {
                            write!(f, "◦ ").ok();
                        }
                    });
                    write!(f, "\n").ok();
                });
            Ok(())
        }
    }

    #[local]
    struct Local {
        en: u8,
        timer7: CounterHz<TIM7>,
        timer6: DelayUs<TIM6>,
    }

    /// STM32 Init code
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!(NoBlockSkip, 4096);

        rprintln!("STM32 LED CUBE");

        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(168.MHz()).freeze();

        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        //256 ARR
        // APB1 operates at 168/2 MHz whereas APB2 operates at 168 MHz
        // let hertz_apb1 = (clocks.hclk().raw() >> 8).Hz();
        // let hertz_apb2 = (clocks.hclk().raw() >> 9).Hz();
        // let t1 = dp
        //     .TIM1
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpioa.pa8.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpioa.pa9.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(
        //                 gpioa
        //                     .pa10
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //             Channel4::new(
        //                 gpioe
        //                     .pe14
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //         ),
        //         hertz_apb2,
        //         &clocks,
        //     )
        //     .split();
        // let t2 = dp
        //     .TIM2
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpioa.pa5.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpiob.pb3.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(
        //                 gpiob
        //                     .pb10
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //             Channel4::new(
        //                 gpiob
        //                     .pb11
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t3 = dp
        //     .TIM3
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpiob.pb4.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpiob.pb5.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(gpiob.pb0.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel4::new(gpiob.pb1.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();

        // let t4 = dp
        //     .TIM4
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpiob.pb6.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpiob.pb7.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(
        //                 gpiod
        //                     .pd14
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //             Channel4::new(
        //                 gpiod
        //                     .pd15
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();

        // let t5 = dp
        //     .TIM5
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpioa.pa0.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpioa.pa1.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(gpioa.pa2.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel4::new(gpioa.pa3.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();

        // let t8 = dp
        //     .TIM8
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpioc.pc6.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpioc.pc7.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel3::new(gpioc.pc8.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel4::new(gpioc.pc9.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         ),
        //         hertz_apb2,
        //         &clocks,
        //     )
        //     .split();
        // let t9 = dp
        //     .TIM9
        //     .pwm_hz(
        //         (
        //             Channel1::new(gpioe.pe5.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //             Channel2::new(gpioe.pe6.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t10 = dp
        //     .TIM10
        //     .pwm_hz(
        //         Channel1::new(gpiob.pb8.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t11 = dp
        //     .TIM11
        //     .pwm_hz(
        //         Channel1::new(gpiob.pb9.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t12 = dp
        //     .TIM12
        //     .pwm_hz(
        //         (
        //             Channel1::new(
        //                 gpiob
        //                     .pb14
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //             Channel2::new(
        //                 gpiob
        //                     .pb15
        //                     .into_alternate()
        //                     .speed(hal::gpio::Speed::VeryHigh),
        //             ),
        //         ),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t13 = dp
        //     .TIM13
        //     .pwm_hz(
        //         Channel1::new(gpioa.pa6.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();
        // let t14 = dp
        //     .TIM14
        //     .pwm_hz(
        //         Channel1::new(gpioa.pa7.into_alternate().speed(hal::gpio::Speed::VeryHigh)),
        //         hertz_apb1,
        //         &clocks,
        //     )
        //     .split();

        // let mut channels = (
        //     t1.0, t1.1, t1.2, t1.3, t2.0, t2.1, t2.2, t2.3, t3.0, t3.1, t3.2, t3.3, t4.0, t4.1,
        //     t4.2, t4.3, t5.0, t5.1, t5.2, t5.3, t8.0, t8.1, t8.2, t8.3, t9.0, t9.1, t10, t11,
        //     t12.0, t12.1, t13, t14,
        // );

        // seq!(N in 0..32{
        //     channels.N.enable();
        // });

        // // tim13.set_polarity(C1, hal::timer::Polarity::ActiveLow);
        // // tim13.enable(C1);

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

        t7.start((1).Hz()).expect("Unable to start frame clock");

        t7.listen(Event::Update);

        // rtic::pend(hal::pac::Interrupt::TIM7);

        static mut BUF: [u32; 16 * 8 * 4 * 2] = [0xFFFFFFFFu32; 16 * 8 * 4 * 2];
        let test = AtomicU32::from_mut_slice(unsafe { &mut BUF });
        let mut graphics = Graphics {
            frame_offset: AtomicUsize::new(0),
            buf: AtomicUsize::new(0),
            buf2: AtomicUsize::new(16 * 8 * 4),
            layer: AtomicU8::new(0),
            fbpool: test,
            // buf: fbpoo,
            // buf2: unsafe { &mut FBPOOL[1] },
            // layer: 0,
        };

        // Circle::new(Point::new(5, 5), 2)
        //     .into_styled(PrimitiveStyle::with_stroke(Rgb888::WHITE, 1))
        //     .draw(&mut graphics)
        //     .unwrap();

        let style = MonoTextStyle::new(&FONT_4X6, Rgb888::WHITE);

        Text::new("HeHe", Point::new(0, 6), style)
            .draw(&mut graphics)
            .unwrap();

        // Pixel(Point::new(0, 0), Rgb888::WHITE)
        //     .draw(&mut graphics)
        //     .unwrap();
        // graphics.draw_iter([])

        // Rectangle::new(Point::new(0, 0), Size::new(16, 16))
        //     .into_styled(PrimitiveStyle::with_fill(Rgb888::WHITE))
        //     .draw(&mut graphics)
        //     .ok();

        rprintln!("{}", graphics);

        graphics.flush();

        let timer6 = dp.TIM6.delay_us(&clocks);

        // draw::spawn().ok();
        (
            Shared { graphics },
            Local {
                timer6,
                en: 0b100,
                timer7: t7,
            },
        )
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

    #[task(binds = TIM7, shared=[&graphics], local=[timer6, timer7, en])]
    fn frame_update(ctx: frame_update::Context) {
        // rprintln!("Update");
        // let timer6 = ctx.local.timer6;
        // timer6.start(2.micros()).unwrap();
        let graphics = ctx.shared.graphics;

        let frame_offset = ctx
            .shared
            .graphics
            .frame_offset
            .load(core::sync::atomic::Ordering::Relaxed);
        ctx.shared.graphics.frame_offset.store(
            (frame_offset + 32) & ((16 * 8 * 4) - 1),
            core::sync::atomic::Ordering::Relaxed,
        );

        let buf = &graphics.fbpool;
        let frame_offset = graphics.buf.load(core::sync::atomic::Ordering::Relaxed) + frame_offset;

        // unsafe {
        //     (*GPIOC::ptr())
        //         .odr
        //         .write(|w| w.bits((0b10000000000) as u32));
        // }
        let (gpioc, gpiod, gpioe) = unsafe {
            (
                GPIOC::ptr().as_ref().unwrap_unchecked(),
                GPIOD::ptr().as_ref().unwrap_unchecked(),
                GPIOE::ptr().as_ref().unwrap_unchecked(),
            )
        };
        gpioc.bsrr.write(|w| unsafe { w.bits(0b10000000000) });
        gpiod.odr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        gpioe.odr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        for (i, d) in (0..8).enumerate().map(|(i, x)| (i, x << 2)) {
            rprintln!("Port : {:#06b}", i);
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111111 << 16) | i as u32 | 0b001000) });
            ctx.local.timer6.delay(100.millis());
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111000 << 16) | 0b010000) });
            ctx.local.timer6.delay(100.millis());
            gpioc
                .bsrr
                .write(|w| unsafe { w.bits((0b111000 << 16) | 0b100000) });
            ctx.local.timer6.delay(100.millis());
        }
        // for (i, d) in (0..8).enumerate().map(|(i, x)| (i, x << 2)) {
        //     rprintln!("Port : {:#06b}", i);
        //     let r = buf[frame_offset + d].load(core::sync::atomic::Ordering::Relaxed);
        //     let g = buf[frame_offset + 1 + d].load(core::sync::atomic::Ordering::Relaxed);
        //     let b = buf[frame_offset + 2 + d].load(core::sync::atomic::Ordering::Relaxed);
        //     gpioc
        //         .bsrr
        //         .write(|w| unsafe { w.bits((0b111111 << 16) | i as u32 | 0b001000) });
        //     gpiod.odr.write(|w| unsafe { w.bits(r & (65535)) });
        //     gpioe.odr.write(|w| unsafe { w.bits(r >> 16) });
        //     ctx.local.timer6.delay(1.secs());
        //     gpioc
        //         .bsrr
        //         .write(|w| unsafe { w.bits((0b111000 << 16) | 0b010000) });
        //     gpiod.odr.write(|w| unsafe { w.bits(g & (65535)) });
        //     gpioe.odr.write(|w| unsafe { w.bits(g >> 16) });
        //     ctx.local.timer6.delay(1.secs());
        //     gpioc
        //         .bsrr
        //         .write(|w| unsafe { w.bits((0b111000 << 16) | 0b100000) });
        //     gpiod.odr.write(|w| unsafe { w.bits(b & (65535)) });
        //     gpioe.odr.write(|w| unsafe { w.bits(b >> 16) });
        //     ctx.local.timer6.delay(1.secs());
        // }
        gpioc
            .odr
            .write(|w| unsafe { w.bits(((frame_offset << 1) & 0b1111000000) as u32) });
        // let frame_offset = graphics.buf.load(core::sync::atomic::Ordering::Relaxed) + frame_offset;
        // let fb = buf[frame_offset..(frame_offset + 16)].iter();
        // let frame_offset = frame_offset + 16 * 8 * 4;
        // let mut fb = fb.chain(buf[frame_offset..(frame_offset + 16)].iter());

        // seq!(N in 0..32{
        //     ctx.local.pwm_channels.N.set_duty(fb.next().map_or(0, |v| v.load(core::sync::atomic::Ordering::Relaxed)) as u16);
        // });

        // let duration = timer6.now().duration_since_epoch();
        // timer6.cancel().unwrap();

        // rprintln!("Timer elasped {}", duration);

        ctx.local.timer7.clear_interrupt(Event::Update);
    }
}
