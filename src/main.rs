#![no_std]
#![feature(wrapping_next_power_of_two)]
#![feature(type_alias_impl_trait)]
#![no_main]

use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};

use hal::gpio::GpioExt;
use hal::prelude::*;
use stm32f4xx_hal as hal;

use seq_macro::seq;

use embedded_graphics::{draw_target::*, pixelcolor::Rgb888, prelude::*, primitives::*};

#[rtic::app(device = hal::pac, peripherals=true)]
mod app {

    use core::{
        fmt::Formatter,
        iter::{Cycle, Flatten, Skip, StepBy},
        slice::{ChunksExact, Iter},
    };

    use hal::{
        gpio::PinState,
        pac::{
            GPIOD, TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM7, TIM8,
            TIM9,
        },
        timer::Channel::*,
        timer::{Channel1, Polarity, PwmChannel},
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

    static mut FBPOOL: [[u8; 16 * 16 * 16 * 4]; 2] = [[0; 16 * 16 * 16 * 4]; 2];

    #[shared]
    struct Shared {
        // #[lock_free]
        graphics: Graphics,
        // #[lock_free]
        // fbu: Cycle<Flatten<StepBy<ChunksExact<'static, u32>>>>,
        // #[lock_free]
        // fbd: Cycle<Flatten<StepBy<Skip<ChunksExact<'static, u32>>>>>,
    }

    pub struct Graphics {
        frame_offset: usize,
        buf: &'static mut [u8],
        buf2: &'static mut [u8],
        layer: u8,
    }

    impl Graphics {
        pub fn flush(&mut self) {
            while self.frame_offset != 0 {}
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
                let (x, y) = coord.into();
                self.buf2[((self.layer as usize) * 1024)
                    + ((x as usize) & (16 - 1))
                    + (((y as usize) & (16 - 1)) * 64)
                    + 0] = color.r();
                self.buf2[((self.layer as usize) * 1024)
                    + ((x as usize) & (16 - 1))
                    + (((y as usize) & (16 - 1)) * 64)
                    + 16] = color.g();
                self.buf2[((self.layer as usize) * 1024)
                    + ((x as usize) & (16 - 1))
                    + (((y as usize) & (16 - 1)) * 64)
                    + 32] = color.b();
            }
            Ok(())
        }

        fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
            let (x, y) = area.top_left.into();
            let (w, h) = area.size.into();

            Ok(())
        }
    }

    impl core::fmt::Display for Graphics {
        fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
            Ok(())
        }
    }

    #[local]
    struct Local {
        en: u8,
        port: u8,
        // ctrl_gpios: [ErasedPin],
        pwm_channels: (
            PwmChannel<TIM1, 0, false>,
            PwmChannel<TIM1, 1, false>,
            PwmChannel<TIM1, 2, false>,
            PwmChannel<TIM1, 3, false>,
            PwmChannel<TIM2, 0, false>,
            PwmChannel<TIM2, 1, false>,
            PwmChannel<TIM2, 2, false>,
            PwmChannel<TIM2, 3, false>,
            PwmChannel<TIM3, 0, false>,
            PwmChannel<TIM3, 1, false>,
            PwmChannel<TIM3, 2, false>,
            PwmChannel<TIM3, 3, false>,
            PwmChannel<TIM4, 0, false>,
            PwmChannel<TIM4, 1, false>,
            PwmChannel<TIM4, 2, false>,
            PwmChannel<TIM4, 3, false>,
            PwmChannel<TIM5, 0, false>,
            PwmChannel<TIM5, 1, false>,
            PwmChannel<TIM5, 2, false>,
            PwmChannel<TIM5, 3, false>,
            PwmChannel<TIM8, 0, false>,
            PwmChannel<TIM8, 1, false>,
            PwmChannel<TIM8, 2, false>,
            PwmChannel<TIM8, 3, false>,
            PwmChannel<TIM9, 0, false>,
            PwmChannel<TIM9, 1, false>,
            PwmChannel<TIM10, 0, false>,
            PwmChannel<TIM11, 0, false>,
            PwmChannel<TIM12, 0, false>,
            PwmChannel<TIM12, 1, false>,
            PwmChannel<TIM13, 0, false>,
            PwmChannel<TIM14, 0, false>,
        ),
        timer7: CounterHz<TIM7>,
    }

    /// STM32 Init code
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        rprintln!("STM32 LED CUBE");

        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(168.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        //256 ARR
        // APB1 operates at 168/2 MHz whereas APB2 operates at 168 MHz
        let hertz_apb1 = (clocks.hclk().raw() >> 9).Hz();
        let hertz_apb2 = (clocks.hclk().raw() >> 8).Hz();

        let t1 = dp
            .TIM1
            .pwm_hz(
                (
                    Channel1::new(gpioa.pa8.into_alternate()),
                    Channel2::new(gpioa.pa9.into_alternate()),
                    Channel3::new(gpioa.pa10.into_alternate()),
                    Channel4::new(gpioe.pe14.into_alternate()),
                ),
                hertz_apb2,
                &clocks,
            )
            .split();
        let t2 = dp
            .TIM2
            .pwm_hz(
                (
                    Channel1::new(gpioa.pa5.into_alternate()),
                    Channel2::new(gpiob.pb3.into_alternate()),
                    Channel3::new(gpiob.pb10.into_alternate()),
                    Channel4::new(gpiob.pb11.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t3 = dp
            .TIM3
            .pwm_hz(
                (
                    Channel1::new(gpiob.pb4.into_alternate()),
                    Channel2::new(gpiob.pb5.into_alternate()),
                    Channel3::new(gpiob.pb0.into_alternate()),
                    Channel4::new(gpiob.pb1.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();

        let t4 = dp
            .TIM4
            .pwm_hz(
                (
                    Channel1::new(gpiob.pb6.into_alternate()),
                    Channel2::new(gpiob.pb7.into_alternate()),
                    Channel3::new(gpiod.pd14.into_alternate()),
                    Channel4::new(gpiod.pd15.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();

        let t5 = dp
            .TIM5
            .pwm_hz(
                (
                    Channel1::new(gpioa.pa0.into_alternate()),
                    Channel2::new(gpioa.pa1.into_alternate()),
                    Channel3::new(gpioa.pa2.into_alternate()),
                    Channel4::new(gpioa.pa3.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();

        let t8 = dp
            .TIM8
            .pwm_hz(
                (
                    Channel1::new(gpioc.pc6.into_alternate()),
                    Channel2::new(gpioc.pc7.into_alternate()),
                    Channel3::new(gpioc.pc8.into_alternate()),
                    Channel4::new(gpioc.pc9.into_alternate()),
                ),
                hertz_apb2,
                &clocks,
            )
            .split();
        let t9 = dp
            .TIM9
            .pwm_hz(
                (
                    Channel1::new(gpioe.pe5.into_alternate()),
                    Channel2::new(gpioe.pe6.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t10 = dp
            .TIM10
            .pwm_hz(
                Channel1::new(gpiob.pb8.into_alternate()),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t11 = dp
            .TIM11
            .pwm_hz(
                Channel1::new(gpiob.pb9.into_alternate()),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t12 = dp
            .TIM12
            .pwm_hz(
                (
                    Channel1::new(gpiob.pb14.into_alternate()),
                    Channel2::new(gpiob.pb15.into_alternate()),
                ),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t13 = dp
            .TIM13
            .pwm_hz(
                Channel1::new(gpioa.pa6.into_alternate()),
                hertz_apb1,
                &clocks,
            )
            .split();
        let t14 = dp
            .TIM14
            .pwm_hz(
                Channel1::new(gpioa.pa7.into_alternate()),
                hertz_apb1,
                &clocks,
            )
            .split();

        let mut channels = (
            t1.0, t1.1, t1.2, t1.3, t2.0, t2.1, t2.2, t2.3, t3.0, t3.1, t3.2, t3.3, t4.0, t4.1,
            t4.2, t4.3, t5.0, t5.1, t5.2, t5.3, t8.0, t8.1, t8.2, t8.3, t9.0, t9.1, t10, t11,
            t12.0, t12.1, t13, t14,
        );

        seq!(N in 0..32{
            channels.N.enable();
        });

        // // tim13.set_polarity(C1, hal::timer::Polarity::ActiveLow);
        // // tim13.enable(C1);

        seq!(N in 3..13{
            gpiod.pd~N.into_push_pull_output_in_state(PinState::Low);
        });

        let mut t7 = dp.TIM7.counter_hz(&clocks);

        t7.start((16 * 8 * 3 * 2).Hz())
            .expect("Unable to start frame clock");

        t7.listen(Event::Update);

        // rtic::pend(hal::pac::Interrupt::TIM7);

        let mut graphics = Graphics {
            frame_offset: 0,
            buf: unsafe { &mut FBPOOL[0] },
            buf2: unsafe { &mut FBPOOL[1] },
            layer: 0,
        };

        Circle::new(Point::new(7, 7), 4)
            .into_styled(PrimitiveStyle::with_stroke(Rgb888::WHITE, 1))
            .draw(&mut graphics)
            .ok();

        draw::spawn().ok();
        (
            Shared { graphics },
            Local {
                pwm_channels: channels,
                en: 0,
                port: 0b00000000,
                timer7: t7,
            },
        )
    }

    // #[idle]
    // fn idle(_: idle::Context) -> ! {
    //     // ctx.shared.buf.lock(|buf| {
    //     //     buf.iter_mut().enumerate().for_each(|(i, w)| *w = i as u8);
    //     // });

    //     loop {
    //         // rprintln!("hehhe");
    //         // rprintln!("Working");
    //         cortex_m::asm::wfi();
    //     }
    // }

    #[task(shared=[graphics])]
    async fn draw(_: draw::Context) {
        // _ = ctx.shared.graphics;
    }

    #[task(binds = TIM7, shared=[graphics], local=[pwm_channels, timer7, port, en], priority=1)]
    fn frame_update(ctx: frame_update::Context) {
        // rprintln!("Update");
        let graphics = ctx.shared.graphics.lock(|l| *l);

        let temp_en = *ctx.local.en;
        let en = ((temp_en << 1) | (temp_en >> 2)) & 0b1110000;
        *ctx.local.en = en;
        let frame_offset = graphics.frame_offset;
        let frame_offset = frame_offset + (frame_offset & (16 * 8 * 4));
        graphics.frame_offset = frame_offset + ((en & 64) as usize);
        let frame_offset = (frame_offset + (en & 0b110000) as usize) & (16384 - 1);
        // rprintln!("{} {:#08b} {}", frame_offset, port, en);

        seq!(N in 0..32{
            ctx.local.pwm_channels.N.set_duty(0);
        });

        unsafe {
            (*GPIOD::ptr()).odr.modify(|_, w| {
                w.bits((((en >> 1) as usize) | (frame_offset & 0b1111111000000)) as u32)
            });
        }

        let buf = &graphics.buf;
        let fb = buf[frame_offset..(frame_offset + 16)].iter();
        let frame_offset = frame_offset + 16 * 8 * 4;
        let mut fb = fb.chain(buf[frame_offset..(frame_offset + 16)].iter());

        seq!(N in 0..32{
            ctx.local.pwm_channels.N.set_duty(*fb.next().unwrap_or(&0) as u16);
        });

        ctx.local.timer7.clear_interrupt(Event::Update);
    }
}
