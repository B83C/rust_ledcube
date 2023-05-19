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
        fbu: Cycle<Flatten<StepBy<ChunksExact<'static, u32>>>>,
        #[lock_free]
        fbd: Cycle<Flatten<StepBy<Skip<ChunksExact<'static, u32>>>>>,
    }

    #[local]
    struct Local {
        en: u8,
        port: u8,
        timer7: CounterHz<TIM7>,
        // timers: (
        //     PwmHz<
        //         TIM1,
        //         (
        //             Channel1<TIM1, false>,
        //             Channel2<TIM1, false>,
        //             Channel3<TIM1, false>,
        //             Channel4<TIM1, false>,
        //         ),
        //     >,
        //     PwmHz<
        //         TIM2,
        //         (
        //             Channel1<TIM2, false>,
        //             Channel2<TIM2, false>,
        //             Channel3<TIM2, false>,
        //             Channel4<TIM2, false>,
        //         ),
        //     >,
        //     PwmHz<
        //         TIM3,
        //         (
        //             Channel1<TIM3, false>,
        //             Channel2<TIM3, false>,
        //             Channel3<TIM3, false>,
        //             Channel4<TIM3, false>,
        //         ),
        //     >,
        //     PwmHz<
        //         TIM4,
        //         (
        //             Channel1<TIM4, false>,
        //             Channel2<TIM4, false>,
        //             Channel3<TIM4, false>,
        //             Channel4<TIM4, false>,
        //         ),
        //     >,
        //     PwmHz<
        //         TIM5,
        //         (
        //             Channel1<TIM5, false>,
        //             Channel2<TIM5, false>,
        //             Channel3<TIM5, false>,
        //             Channel4<TIM5, false>,
        //         ),
        //     >,
        //     PwmHz<
        //         TIM8,
        //         (
        //             Channel1<TIM8, false>,
        //             Channel2<TIM8, false>,
        //             Channel3<TIM8, false>,
        //             Channel4<TIM8, false>,
        //         ),
        //     >,
        //     PwmHz<TIM9, (Channel1<TIM9, false>, Channel2<TIM9, false>)>,
        //     PwmHz<TIM10, Channel1<TIM10, false>>,
        //     PwmHz<TIM11, Channel1<TIM11, false>>,
        //     PwmHz<TIM12, (Channel1<TIM12, false>, Channel2<TIM12, false>)>,
        //     PwmHz<TIM13, Channel1<TIM13, false>>,
        //     PwmHz<TIM14, Channel1<TIM14, false>>,
        // ),
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

        _ = gpiod.pd0.into_push_pull_output();
        _ = gpiod.pd1.into_push_pull_output();
        _ = gpiod.pd2.into_push_pull_output();
        _ = gpiod.pd3.into_push_pull_output();
        _ = gpiod.pd4.into_push_pull_output();
        _ = gpiod.pd5.into_push_pull_output();

        let mut tim7 = dp.TIM7.counter_hz(&clocks);

        tim7.start((16 * 8 * 3 * 120).Hz())
            .expect("Unable to start frame clock");

        tim7.listen(Event::Update);

        // rtic::pend(hal::pac::Interrupt::TIM7);

        // tim1.enable(hal::timer::Channel::C1);

        (
            Shared {
                fbu: fbpool[0].chunks_exact(128).step_by(2).flatten().cycle(),
                fbd: fbpool[0]
                    .chunks_exact(128)
                    .skip(1)
                    .step_by(2)
                    .flatten()
                    .cycle(),
            },
            Local {
                en: 0,
                port: 0,
                timer7: tim7,
                // timers: (
                //     tim1, tim2, tim3, tim4, tim5, tim8, tim9, tim10, tim11, tim12, tim13, tim14,
                // ),
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

    #[task(binds = TIM7, shared=[fbu, fbd], local=[timer7, port, en])]
    fn frame_update(ctx: frame_update::Context) {
        // rprintln!("Update");
        // let (t1, t2, t3, t4, t5, t8, t9, t10, t11, t12, t13, t14) = ctx.local.timers;
        let en = ctx.local.en;
        let port = ctx.local.port;
        let fbu = ctx.shared.fbu;
        let fbd = ctx.shared.fbd;

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
                .modify(|_, w| w.bits(((0b1000 << *en) | *port & 0b111) as u32));

            seq!(N in 0..4 {
                (*timers.N).ccr.iter().for_each(|x| {
                    let temp = ((*fbu.next().unwrap_unchecked() >> *en) as u8);
                    rdbg!(&temp);
                    x.write(|w| w.ccr().bits(temp.into()))
                });
            });

            // let mut chunk = fb.skip(7).next().unwrap_unchecked().iter().take(16);

            seq!(N in 4..12 {
                (*timers.N).ccr.iter().for_each(|x| {
                    x.write(|w| w.ccr().bits(((*fbd.next().unwrap_unchecked() >> *en) as u8).into()))
                });
            });
        }

        rdbg!(*en);
        *en = if *en < 2 { *en + 1 } else { 0 };
        *port += 1;

        // iter1.
        // unsafe {
        //     let iter1 = (*TIM1::ptr()).ccr.iter();
        //     let iter2 = (*TIM2::ptr()).ccr.iter();
        //     let iter3 = (*TIM3::ptr()).ccr.iter();
        //     let iter4 = (*TIM4::ptr()).ccr.iter();

        // let test = itertools::chain!(iter1, iter2);
        // iter1.chain(iter2).enumerate().for_each(|(i, x)| {
        //     x.write(|w| w.ccr().bits(((fb[off + 0 + i] >> en) as u8).into()))
        // });
        // unsafe {
        //     let en = *en;
        //     let off = *off as usize;
        //     (*TIM1::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 0 + i] >> en) as u8).into()))
        //     });
        //     (*TIM2::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 4 + i] >> en) as u8).into()))
        //     });
        //     (*TIM3::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 8 + i] >> en) as u8).into()))
        //     });
        //     (*TIM4::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });

        //     let off = off + 128;
        //     (*TIM5::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 0 + i] >> en) as u8).into()))
        //     });
        //     (*TIM8::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 4 + i] >> en) as u8).into()))
        //     });
        //     (*TIM9::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 8 + i] >> en) as u8).into()))
        //     });
        //     (*TIM10::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });
        //     (*TIM11::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });
        //     (*TIM12::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });
        //     (*TIM13::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });
        //     (*TIM12::ptr()).ccr.iter().enumerate().for_each(|(i, x)| {
        //         x.write(|w| w.ccr().bits(((fb[off + 12 + i] >> en) as u8).into()))
        //     });
        // }

        // {
        //     let en = *en;
        //     let port = *port;
        //     let offset = *off;
        //     unsafe {
        //         (*GPIOD::ptr())
        //             .odr
        //             .modify(|_, w| w.bits(((en << 3) | port & 0b111) as u32));
        //     }
        //     t1.set_duty(C1, ((fb[offset + 0] >> en) as u8) as u16);
        //     t1.set_duty(C2, ((fb[offset + 1] >> en) as u8) as u16);
        //     t1.set_duty(C3, ((fb[offset + 2] >> en) as u8) as u16);
        //     t1.set_duty(C4, ((fb[offset + 3] >> en) as u8) as u16);
        //     t2.set_duty(C1, ((fb[offset + 4] >> en) as u8) as u16);
        //     t2.set_duty(C2, ((fb[offset + 5] >> en) as u8) as u16);
        //     t2.set_duty(C3, ((fb[offset + 6] >> en) as u8) as u16);
        //     t2.set_duty(C4, ((fb[offset + 7] >> en) as u8) as u16);
        //     t3.set_duty(C1, ((fb[offset + 8] >> en) as u8) as u16);
        //     t3.set_duty(C2, ((fb[offset + 9] >> en) as u8) as u16);
        //     t3.set_duty(C3, ((fb[offset + 10] >> en) as u8) as u16);
        //     t3.set_duty(C4, ((fb[offset + 11] >> en) as u8) as u16);
        //     t4.set_duty(C1, ((fb[offset + 12] >> en) as u8) as u16);
        //     t4.set_duty(C2, ((fb[offset + 13] >> en) as u8) as u16);
        //     t4.set_duty(C3, ((fb[offset + 14] >> en) as u8) as u16);
        //     t4.set_duty(C4, ((fb[offset + 15] >> en) as u8) as u16);

        //     let offset = offset + 128;

        //     t5.set_duty(C1, ((fb[offset + 0] >> en) as u8) as u16);
        //     t5.set_duty(C2, ((fb[offset + 1] >> en) as u8) as u16);
        //     t5.set_duty(C3, ((fb[offset + 2] >> en) as u8) as u16);
        //     t5.set_duty(C4, ((fb[offset + 3] >> en) as u8) as u16);
        //     t8.set_duty(C1, ((fb[offset + 4] >> en) as u8) as u16);
        //     t8.set_duty(C2, ((fb[offset + 5] >> en) as u8) as u16);
        //     t8.set_duty(C3, ((fb[offset + 6] >> en) as u8) as u16);
        //     t8.set_duty(C4, ((fb[offset + 7] >> en) as u8) as u16);
        //     t9.set_duty(C1, ((fb[offset + 8] >> en) as u8) as u16);
        //     t9.set_duty(C2, ((fb[offset + 9] >> en) as u8) as u16);
        //     t10.set_duty(C1, ((fb[offset + 10] >> en) as u8) as u16);
        //     t11.set_duty(C1, ((fb[offset + 11] >> en) as u8) as u16);
        //     t12.set_duty(C1, ((fb[offset + 12] >> en) as u8) as u16);
        //     t12.set_duty(C2, ((fb[offset + 13] >> en) as u8) as u16);
        //     t13.set_duty(C1, ((fb[offset + 14] >> en) as u8) as u16);
        //     t14.set_duty(C1, ((fb[offset + 15] >> en) as u8) as u16);
        // }

        // *off += 128;

        ctx.local.timer7.clear_interrupt(Event::Update);
    }
}
