#![no_std]
#![no_main]

use cortex_m::asm;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};

use hal::gpio::{self, GpioExt};
use hal::prelude::*;
use stm32f4xx_hal as hal;

#[rtic::app(device = hal::pac, peripherals=true)]
mod app {

    use hal::{
        gpio::gpioa,
        pac::{GPIOA, TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM8, TIM9},
        timer::Channel::*,
        timer::Channel1,
        timer::Channel3,
        timer::Channel4,
        timer::{Channel2, PwmHz},
    };

    use super::*;

    #[shared]
    struct Shared {
        #[lock_free]
        framebuffer: &'static [u32],
    }

    #[local]
    struct Local {
        timers: (
            PwmHz<
                TIM1,
                (
                    Channel1<TIM1, false>,
                    Channel2<TIM1, false>,
                    Channel3<TIM1, false>,
                    Channel4<TIM1, false>,
                ),
            >,
            PwmHz<
                TIM2,
                (
                    Channel1<TIM2, false>,
                    Channel2<TIM2, false>,
                    Channel3<TIM2, false>,
                    Channel4<TIM2, false>,
                ),
            >,
            PwmHz<
                TIM3,
                (
                    Channel1<TIM3, false>,
                    Channel2<TIM3, false>,
                    Channel3<TIM3, false>,
                    Channel4<TIM3, false>,
                ),
            >,
            PwmHz<
                TIM4,
                (
                    Channel1<TIM4, false>,
                    Channel2<TIM4, false>,
                    Channel3<TIM4, false>,
                    Channel4<TIM4, false>,
                ),
            >,
            PwmHz<
                TIM5,
                (
                    Channel1<TIM5, false>,
                    Channel2<TIM5, false>,
                    Channel3<TIM5, false>,
                    Channel4<TIM5, false>,
                ),
            >,
            PwmHz<
                TIM8,
                (
                    Channel1<TIM8, false>,
                    Channel2<TIM8, false>,
                    Channel3<TIM8, false>,
                    Channel4<TIM8, false>,
                ),
            >,
            PwmHz<TIM9, (Channel1<TIM9, false>, Channel2<TIM9, false>)>,
            PwmHz<TIM10, (Channel1<TIM10, false>)>,
            PwmHz<TIM11, (Channel1<TIM11, false>)>,
            PwmHz<TIM12, (Channel1<TIM12, false>, Channel2<TIM12, false>)>,
            PwmHz<TIM13, (Channel1<TIM13, false>)>,
            PwmHz<TIM14, (Channel1<TIM14, false>)>,
        ),
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        rprintln!("STM32 LED CUBE");

        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(168.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        let tim1 = dp.TIM1.pwm_hz(
            (
                Channel1::new(gpioa.pa8),
                Channel2::new(gpioa.pa9),
                Channel3::new(gpioa.pa10),
                Channel4::new(gpioe.pe14),
            ),
            1.MHz(),
            &clocks,
        );
        let mut tim2 = dp.TIM2.pwm_hz(
            (
                Channel1::new(gpioa.pa5),
                Channel2::new(gpiob.pb3.into_alternate()),
                Channel3::new(gpiob.pb10),
                Channel4::new(gpiob.pb11),
            ),
            1.MHz(),
            &clocks,
        );
        let mut tim3 = dp.TIM3.pwm_hz(
            (
                Channel1::new(gpiob.pb4.into_alternate()),
                Channel2::new(gpiob.pb5),
                Channel3::new(gpiob.pb0),
                Channel4::new(gpiob.pb1),
            ),
            1.MHz(),
            &clocks,
        );

        let mut tim4 = dp.TIM4.pwm_hz(
            (
                Channel1::new(gpiob.pb6),
                Channel2::new(gpiob.pb7),
                Channel3::new(gpiod.pd14),
                Channel4::new(gpiod.pd15),
            ),
            1.MHz(),
            &clocks,
        );

        let mut tim5 = dp.TIM5.pwm_hz(
            (
                Channel1::new(gpioa.pa0),
                Channel2::new(gpioa.pa1),
                Channel3::new(gpioa.pa2),
                Channel4::new(gpioa.pa3),
            ),
            1.MHz(),
            &clocks,
        );

        let mut tim8 = dp.TIM8.pwm_hz(
            (
                Channel1::new(gpioc.pc6),
                Channel2::new(gpioc.pc7),
                Channel3::new(gpioc.pc8),
                Channel4::new(gpioc.pc9),
            ),
            1.MHz(),
            &clocks,
        );
        let mut tim9 = dp.TIM9.pwm_hz(
            (Channel1::new(gpioe.pe5), Channel2::new(gpioe.pe6)),
            1.MHz(),
            &clocks,
        );
        let mut tim10 = dp.TIM10.pwm_hz(Channel1::new(gpiob.pb8), 1.MHz(), &clocks);
        let mut tim11 = dp.TIM11.pwm_hz(Channel1::new(gpiob.pb9), 1.MHz(), &clocks);
        let mut tim12 = dp.TIM12.pwm_hz(
            (Channel1::new(gpiob.pb14), Channel2::new(gpiob.pb15)),
            1.MHz(),
            &clocks,
        );
        let mut tim13 = dp.TIM13.pwm_hz(Channel1::new(gpioa.pa6), 1.MHz(), &clocks);
        let mut tim14 = dp.TIM14.pwm_hz(Channel1::new(gpioa.pa7), 1.MHz(), &clocks);

        // tim1.enable(hal::timer::Channel::C1);

        (
            Shared {
                framebuffer: &[0; 16 * 16 * 16],
            },
            Local {
                timers: (
                    tim1, tim2, tim3, tim4, tim5, tim8, tim9, tim10, tim11, tim12, tim13, tim14,
                ),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM7, local=[timers])]
    fn frame_update(ctx: frame_update::Context) {
        let (t1, t2, t3, t4, t5, t8, t9, t10, t11, t12, t13, t14) = ctx.local.timers;
        t1.set_duty(C1, 0);
        t1.set_duty(C2, 0);
        t1.set_duty(C3, 0);
        t1.set_duty(C4, 0);
        t2.set_duty(C1, 0);
        t2.set_duty(C2, 0);
        t2.set_duty(C3, 0);
        t2.set_duty(C4, 0);
        t3.set_duty(C1, 0);
        t3.set_duty(C2, 0);
        t3.set_duty(C3, 0);
        t3.set_duty(C4, 0);
        t4.set_duty(C1, 0);
        t4.set_duty(C2, 0);
        t4.set_duty(C3, 0);
        t4.set_duty(C4, 0);
        t5.set_duty(C1, 0);
        t5.set_duty(C2, 0);
        t5.set_duty(C3, 0);
        t5.set_duty(C4, 0);
        t8.set_duty(C1, 0);
        t8.set_duty(C2, 0);
        t8.set_duty(C3, 0);
        t8.set_duty(C4, 0);
        t9.set_duty(C1, 0);
        t9.set_duty(C2, 0);
        t10.set_duty(C1, 0);
        t11.set_duty(C1, 0);
        t12.set_duty(C1, 0);
        t12.set_duty(C2, 0);
        t13.set_duty(C1, 0);
        t14.set_duty(C1, 0);
    }
}
