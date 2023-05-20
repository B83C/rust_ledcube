#![no_std]
#![feature(wrapping_next_power_of_two)]
#![no_main]

use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};

use hal::gpio::GpioExt;
use hal::prelude::*;
use stm32f4xx_hal as hal;

use seq_macro::seq;

#[rtic::app(device = hal::pac, peripherals=true)]
mod app {

    use core::{
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
        timer::Channel1,
        timer::{Channel2, PwmHz},
        timer::{Channel3, Event},
        timer::{Channel4, CounterHz},
    };
    use rtt_target::rdbg;

    use super::*;

    static fbpool: [[u32; 16 * 16 * 16]; 2] = [[0xFFFFFFu32; 16 * 16 * 16]; 2];

    #[shared]
    struct Shared {
        #[lock_free]
        frame_offset: usize,
        #[lock_free]
        buf: &'static [u32],
        // #[lock_free]
        // fbu: Cycle<Flatten<StepBy<ChunksExact<'static, u32>>>>,
        // #[lock_free]
        // fbd: Cycle<Flatten<StepBy<Skip<ChunksExact<'static, u32>>>>>,
    }

    #[local]
    struct Local {
        en: u8,
        port: u8,
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
        let hertz = (168000000 >> 8).Hz();

        _ = dp.TIM1.pwm_hz(
            (
                Channel1::new(gpioa.pa8.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpioa.pa9.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpioa.pa10.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpioe.pe14.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );
        _ = dp.TIM2.pwm_hz(
            (
                Channel1::new(gpioa.pa5.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpiob.pb3.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpiob.pb10.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpiob.pb11.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );
        _ = dp.TIM3.pwm_hz(
            (
                Channel1::new(gpiob.pb4.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpiob.pb5.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpiob.pb0.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpiob.pb1.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );

        _ = dp.TIM4.pwm_hz(
            (
                Channel1::new(gpiob.pb6.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpiob.pb7.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpiod.pd14.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpiod.pd15.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );

        _ = dp.TIM5.pwm_hz(
            (
                Channel1::new(gpioa.pa0.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpioa.pa1.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpioa.pa2.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpioa.pa3.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );

        _ = dp.TIM8.pwm_hz(
            (
                Channel1::new(gpioc.pc6.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpioc.pc7.into_push_pull_output_in_state(PinState::Low)),
                Channel3::new(gpioc.pc8.into_push_pull_output_in_state(PinState::Low)),
                Channel4::new(gpioc.pc9.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );
        _ = dp.TIM9.pwm_hz(
            (
                Channel1::new(gpioe.pe5.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpioe.pe6.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );
        _ = dp.TIM10.pwm_hz(
            Channel1::new(gpiob.pb8.into_push_pull_output_in_state(PinState::Low)),
            hertz,
            &clocks,
        );
        _ = dp.TIM11.pwm_hz(
            Channel1::new(gpiob.pb9.into_push_pull_output_in_state(PinState::Low)),
            hertz,
            &clocks,
        );
        _ = dp.TIM12.pwm_hz(
            (
                Channel1::new(gpiob.pb14.into_push_pull_output_in_state(PinState::Low)),
                Channel2::new(gpiob.pb15.into_push_pull_output_in_state(PinState::Low)),
            ),
            hertz,
            &clocks,
        );
        _ = dp.TIM13.pwm_hz(
            Channel1::new(gpioa.pa6.into_push_pull_output_in_state(PinState::Low)),
            hertz,
            &clocks,
        );
        _ = dp.TIM14.pwm_hz(
            Channel1::new(gpioa.pa7.into_push_pull_output_in_state(PinState::Low)),
            hertz,
            &clocks,
        );

        seq!(N in 0..10{
        _ = gpiod.pd~N.into_push_pull_output();
        });

        let mut tim7 = dp.TIM7.counter_hz(&clocks);

        tim7.start((16 * 8 * 3 * 120).Hz())
            .expect("Unable to start frame clock");

        tim7.listen(Event::Update);

        // rtic::pend(hal::pac::Interrupt::TIM7);

        (
            Shared {
                frame_offset: 0,
                buf: &fbpool[0],
            },
            Local {
                en: 0,
                port: 0b00000000,
                timer7: tim7,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // rprintln!("hehhe");
            // rprintln!("Working");
            // cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM7, shared=[frame_offset, &buf], local=[timer7, port, en])]
    fn frame_update(ctx: frame_update::Context) {
        // rprintln!("Update");

        let en = *ctx.local.en;
        let port = *ctx.local.port;
        let frame_offset = *ctx.shared.frame_offset;

        // rprintln!("{} {:#08b} {}", frame_offset, port, en);
        let buf = ctx.shared.buf;

        let timers = (
            TIM1::ptr(),
            TIM2::ptr(),
            TIM3::ptr(),
            TIM4::ptr(),
            TIM5::ptr(),
            TIM8::ptr(),
            TIM9::ptr(),
            TIM10::ptr(),
            TIM11::ptr(),
            TIM12::ptr(),
            TIM13::ptr(),
            TIM14::ptr(),
        );

        unsafe {
            seq!(N in 0..12 {
                (*timers.N).ccr.iter().for_each(|x| x.reset());
            });

            (*GPIOD::ptr())
                .odr
                .modify(|_, w| w.bits(((0b10000000 << (en >> 3)) | port & 0b1111111) as u32));

            let buf = *buf;
            let mut fb = buf[frame_offset..(frame_offset + 16)].iter();

            seq!(N in 0..4 {
                (*timers.N).ccr.iter().for_each(|x| {
                    let temp = (*fb.next().unwrap_unchecked() >> en) as u8;
                    // rdbg!(&temp);
                    x.write(|w| w.ccr().bits(temp.into()))
                });
            });

            let frame_offset = frame_offset + 128;
            let mut fb = buf[frame_offset..(frame_offset + 16)].iter();

            seq!(N in 4..12 {
                (*timers.N).ccr.iter().for_each(|x| {
                    x.write(|w| w.ccr().bits(((*fb.next().unwrap_unchecked() >> en) as u8).into()))
                });
            });
        }

        *ctx.local.en = en + 8;
        if en == 16 {
            *ctx.local.en -= 24;
            *ctx.local.port += 1;
            let frame_offset = frame_offset + 16;
            *ctx.shared.frame_offset = (16 * 16 * 16 - 1) & (frame_offset + (frame_offset & 128));
        }

        ctx.local.timer7.clear_interrupt(Event::Update);
    }
}
