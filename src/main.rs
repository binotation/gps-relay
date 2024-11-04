#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use heapless::spsc::Queue;
use nrf24l01_commands::{commands, commands::Command, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{
    interrupt, Interrupt, Peripherals, DMA1, EXTI, GPIOA, RTC, SPI1, TIM2, USART2,
};

const USART2_TDR: u32 = 0x4000_4428;
const SPI1_DR: u32 = 0x4001_300C;
const TX_ADDR: u64 = 0xA2891FFF6A;

// Commands
const NOP: [u8; 1] = commands::Nop::bytes();
const W_RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(110)).bytes();
const R_RF_CH: [u8; 2] = commands::RRegister::<registers::RfCh>::bytes();
const W_RF_SETUP: [u8; 2] =
    commands::WRegister(registers::RfSetup::new().with_rf_dr(false)).bytes();
const R_RF_SETUP: [u8; 2] = commands::RRegister::<registers::RfSetup>::bytes();
const W_TX_ADDR: [u8; 6] =
    commands::WRegister(registers::TxAddr::<5>::new().with_tx_addr(TX_ADDR)).bytes();
const R_TX_ADDR: [u8; 6] = commands::RRegister::<registers::TxAddr<5>>::bytes();
const W_RX_ADDR_P0: [u8; 6] =
    commands::WRegister(registers::RxAddrP0::<5>::new().with_rx_addr_p0(TX_ADDR)).bytes();
const R_RX_ADDR_P0: [u8; 6] = commands::RRegister::<registers::RxAddrP0<5>>::bytes();
const W_CONFIG: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_pwr_up(true)
        .with_mask_rx_dr(true),
)
.bytes();
const R_CONFIG: [u8; 2] = commands::RRegister::<registers::Config>::bytes();
const RESET_TX_DS: [u8; 2] = commands::WRegister(registers::Status::new().with_tx_ds(true)).bytes();
const RESET_MAX_RT: [u8; 2] =
    commands::WRegister(registers::Status::new().with_max_rt(true)).bytes();

static W_TX_PAYLOAD: [u8; 33] = [
    commands::WTxPayload::<32>::WORD,
    b't',
    b'h',
    b'e',
    b' ',
    b'l',
    b'a',
    b'z',
    b'y',
    b' ',
    b'f',
    b'o',
    b'x',
    b' ',
    b'j',
    b'u',
    b'm',
    b'p',
    b'e',
    b'd',
    b' ',
    b'o',
    b'v',
    b'e',
    b'r',
    b' ',
    b't',
    b'h',
    b'e',
    b' ',
    b'b',
    b'r',
    b'o',
];
// const W_TX_PAYLOAD: [u8; 33] = commands::WTxPayload(PAYLOAD).bytes();

struct SyncPeripheral<P>(UnsafeCell<Option<P>>);

impl<P> SyncPeripheral<P> {
    const fn new() -> Self {
        SyncPeripheral(UnsafeCell::new(None))
    }

    fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    const fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncPeripheral<GPIOA> {}
unsafe impl Sync for SyncPeripheral<USART2> {}
unsafe impl Sync for SyncPeripheral<SPI1> {}
unsafe impl Sync for SyncPeripheral<DMA1> {}
unsafe impl Sync for SyncPeripheral<TIM2> {}
unsafe impl Sync for SyncPeripheral<RTC> {}
unsafe impl Sync for SyncPeripheral<EXTI> {}
unsafe impl Sync for SyncPeripheral<CorePeripherals> {}

static GPIOA_PERIPHERAL: SyncPeripheral<GPIOA> = SyncPeripheral::new();
static USART2_PERIPHERAL: SyncPeripheral<USART2> = SyncPeripheral::new();
static SPI1_PERIPHERAL: SyncPeripheral<SPI1> = SyncPeripheral::new();
static DMA1_PERIPHERAL: SyncPeripheral<DMA1> = SyncPeripheral::new();
static TIM2_PERIPHERAL: SyncPeripheral<TIM2> = SyncPeripheral::new();
static RTC_PERIPHERAL: SyncPeripheral<RTC> = SyncPeripheral::new();
static EXTI_PERIPHERAL: SyncPeripheral<EXTI> = SyncPeripheral::new();
static CORE_PERIPHERALS: SyncPeripheral<CorePeripherals> = SyncPeripheral::new();

struct SyncQueue<T, const N: usize>(UnsafeCell<Queue<T, N>>);

impl<T, const N: usize> SyncQueue<T, N> {
    const fn new() -> Self {
        Self(UnsafeCell::new(Queue::new()))
    }

    const fn get(&self) -> &mut Queue<T, N> {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncQueue<&[u8], 16> {}

struct SyncBuffer<const N: usize>(UnsafeCell<[u8; N]>);

impl<const N: usize> SyncBuffer<N> {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; N]))
    }

    const fn get(&self) -> &mut [u8; N] {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl<const N: usize> Sync for SyncBuffer<N> {}

static INIT_COMMANDS: SyncQueue<&[u8], 16> = SyncQueue::new();
static SPI1_RX_BUFFER: SyncBuffer<33> = SyncBuffer::new();

// struct DualBuffer {
//     buffer: [[u8; 33]; 2],
//     writing_to: usize,
// }

// struct SyncDualBuffer(UnsafeCell<DualBuffer>);

// impl SyncDualBuffer {
//     const fn new() -> Self {
//         Self (UnsafeCell::new(
//             DualBuffer {
//                 buffer: [[0; 33]; 2],
//                 writing_to: 0,
//             }
//         ))
//     }

//     const fn get(&self) -> &mut DualBuffer {
//         unsafe { &mut *self.0.get() }
//     }
// }

// unsafe impl Sync for SyncDualBuffer {}

#[inline]
fn send_command(command: &[u8], dma1: &mut DMA1, spi1: &mut SPI1) {
    // Write memory address
    dma1.ch3()
        .mar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    let transfer_size = command.len() as u32;
    // Set DMA transfer size for SPI1 RX, TX, USART2 TX
    dma1.ch2()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dma1.ch3()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dma1.ch7()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    // Enable DMA for SPI1 RX, TX
    dma1.ch2().cr().modify(|_, w| w.en().set_bit());
    dma1.ch3().cr().modify(|_, w| w.en().set_bit());
    // Enable SPI1
    spi1.cr1().modify(|_, w| w.spe().enabled());
}

#[inline]
fn pulse_ce(gpioa: &mut GPIOA, tim2: &mut TIM2) {
    gpioa.bsrr().write(|w| w.bs0().set_bit());
    // Enable counter, one-pulse mode
    tim2.cr1().write(|w| w.opm().enabled().cen().enabled());
}

#[interrupt]
fn USART2() {
    let gpioa = GPIOA_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let tim2 = TIM2_PERIPHERAL.get();
    let dma1 = DMA1_PERIPHERAL.get();

    // Read incoming bytes from USART2 and queue onto tx buffer
    if usart2.isr().read().rxne().bit_is_set() {
        // Read data, this clears RXNE
        let received_byte = usart2.rdr().read().rdr().bits();

        match received_byte {
            97 => {
                // a
                // NOP
                send_command(&NOP, dma1, spi1);
            }
            98 => {
                // b
                // Write payload
                send_command(&W_TX_PAYLOAD, dma1, spi1);
            }
            99 => {
                // c
                pulse_ce(gpioa, tim2);
            }
            100 => {
                // d
                // Reset TX_DS flag
                send_command(&RESET_TX_DS, dma1, spi1);
            }
            101 => {
                // e
                // Reset MAX_RT flag
                send_command(&RESET_MAX_RT, dma1, spi1);
            }
            _ => (),
        }
    }
    if usart2.isr().read().ore().bit_is_set() {
        usart2.icr().write(|w| w.orecf().set_bit());
    }
}

/// USART2 TX DMA stream
#[interrupt]
fn DMA1_CH7() {
    let dma1 = DMA1_PERIPHERAL.get();
    let init_commands = INIT_COMMANDS.get();
    let spi1 = SPI1_PERIPHERAL.get();

    if dma1.isr().read().tcif7().bit_is_set() {
        dma1.ch7().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif7().set_bit());

        // Send initialization commands
        if let Some(command) = init_commands.dequeue() {
            send_command(command, dma1, spi1);
        }
    }
}

/// SPI1 RX DMA stream
#[interrupt]
fn DMA1_CH2() {
    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let tim2 = TIM2_PERIPHERAL.get();
    let init_commands = INIT_COMMANDS.get();

    if dma1.isr().read().tcif2().bit_is_set() {
        dma1.ch2().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif2().set_bit());

        // Disable SPI1
        spi1.cr1().modify(|_, w| w.spe().clear_bit());

        if dma1.ch3().mar().read().ma() == W_TX_PAYLOAD.as_ptr() as u32 {
            pulse_ce(gpioa, tim2);
        }

        // Send initialization commands
        if let Some(command) = init_commands.dequeue() {
            send_command(command, dma1, spi1);
        }
    }
}

/// SPI1 TX DMA stream
#[interrupt]
fn DMA1_CH3() {
    let dma1 = DMA1_PERIPHERAL.get();
    if dma1.isr().read().tcif3().bit_is_set() {
        dma1.ch3().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

#[interrupt]
fn TIM2() {
    let tim2 = TIM2_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let cp = CORE_PERIPHERALS.get();

    if tim2.sr().read().uif().bit_is_set() {
        gpioa.bsrr().write(|w| w.br0().set_bit());
        tim2.sr().write(|w| w.uif().clear_bit());
        unsafe {
            cp.SCB.scr.write(0b110);
        }
    }
}

#[interrupt]
fn RTC_WKUP() {
    let rtc = RTC_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let exti = EXTI_PERIPHERAL.get();
    let cp = CORE_PERIPHERALS.get();
    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();

    if rtc.isr().read().wutf().bit_is_set() {
        rtc.isr().modify(|_, w| w.wutf().clear_bit());
        exti.pr1().write(|w| w.pr20().clear_bit_by_one());
        unsafe {
            cp.SCB.scr.write(0b100);
        }

        if gpioa.odr().read().odr8().bit_is_set() {
            gpioa.bsrr().write(|w| w.br8().set_bit());
        } else {
            gpioa.bsrr().write(|w| w.bs8().set_bit());
        }
        send_command(&W_TX_PAYLOAD, dma1, spi1);
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    dp.RCC
        .cr()
        .write(|w| w.msirange().range200k().msirgsel().set_bit());

    // Enable peripheral clocks: DMA1, GPIOA, USART2, TIM2, SPI1
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| {
        w.usart2en()
            .enabled()
            .tim2en()
            .set_bit()
            .pwren()
            .set_bit()
            .rtcapben()
            .set_bit()
    });
    dp.RCC.apb2enr().write(|w| w.spi1en().set_bit());

    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    // GPIO: A1 (IRQ), A0 (CE)
    dp.GPIOA.moder().write(|w| {
        w.moder0()
            .output()
            .moder1()
            .input()
            .moder2()
            .alternate()
            .moder3()
            .alternate()
            .moder4()
            .alternate()
            .moder5()
            .alternate()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
            .moder8()
            .output()
    });
    dp.GPIOA
        .otyper()
        .write(|w| w.ot0().push_pull().ot8().push_pull());
    // NSS, IRQ are active low
    dp.GPIOA
        .pupdr()
        .write(|w| w.pupdr1().pull_up().pupdr4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeedr2()
            .low_speed()
            .ospeedr3()
            .low_speed()
            .ospeedr4()
            .low_speed()
            .ospeedr5()
            .medium_speed()
            .ospeedr6()
            .medium_speed()
            .ospeedr7()
            .medium_speed()
    });
    dp.GPIOA.afrl().write(|w| {
        w.afrl2()
            .af7()
            .afrl3()
            .af7()
            .afrl4()
            .af5()
            .afrl5()
            .af5()
            .afrl6()
            .af5()
            .afrl7()
            .af5()
    });

    // DMA channel selection
    dp.DMA1
        .cselr()
        .write(|w| w.c2s().map1().c3s().map1().c6s().map2().c7s().map2());

    // DMA channel 7 USART2 TX
    dp.DMA1
        .ch7()
        .par()
        .write(|w| unsafe { w.pa().bits(USART2_TDR) });
    dp.DMA1
        .ch7()
        .mar()
        .write(|w| unsafe { w.ma().bits(SPI1_RX_BUFFER.get() as *const [u8; 33] as u32) });
    dp.DMA1
        .ch7()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit().dir().set_bit());

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(21) }); // 200khz / 9600 approx. 21

    // USART2: enable DMA
    dp.USART2
        .cr3()
        .write(|w| w.dmar().set_bit().dmat().set_bit());

    // SPI1: Set FIFO reception threshold to 1/4, data frame size to 8 bits, enable slave select output,
    // enable RXNE interupt, enable DMA
    dp.SPI1.cr2().write(|w| unsafe {
        w.frxth()
            .set_bit()
            .ds()
            .bits(7)
            .ssoe()
            .enabled()
            .rxneie()
            .set_bit()
            .txdmaen()
            .set_bit()
            .rxdmaen()
            .set_bit()
    });
    // SPI1: set SPI master
    dp.SPI1.cr1().write(|w| w.mstr().set_bit());

    // DMA channel 2 SPI1 RX
    dp.DMA1
        .ch2()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });
    dp.DMA1
        .ch2()
        .mar()
        .write(|w| unsafe { w.ma().bits(SPI1_RX_BUFFER.get() as *const [u8; 33] as u32) });
    dp.DMA1
        .ch2()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit());

    // DMA channel 3 SPI1 TX
    dp.DMA1
        .ch3()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });
    dp.DMA1
        .ch3()
        .cr()
        .write(|w| w.minc().set_bit().dir().set_bit().tcie().set_bit());

    // Enable USART, transmitter, receiver and RXNE interrupt
    // dp.USART2.cr1().write(|w| {
    //     w.re()
    //         .set_bit()
    //         .te()
    //         .set_bit()
    //         .ue()
    //         .set_bit()
    //         .rxneie()
    //         .set_bit()
    // });

    // Set 11us interval
    dp.TIM2.arr().write(|w| unsafe { w.arr().bits(3) }); // 11us * 200khz approx. 3

    // Enable TIM2 update interrupt
    dp.TIM2.dier().write(|w| w.uie().set_bit());

    // Set SleepDeep bit
    unsafe { cp.SCB.scr.write(0b100) };
    // Set Stop 2 low-power mode, remove write protection from BDCR
    dp.PWR
        .cr1()
        .write(|w| unsafe { w.lpms().bits(0b010).dbp().set_bit() });

    // Configure RTC, set 5s periodic wake up
    dp.RCC.csr().write(|w| w.lsion().set_bit());
    while dp.RCC.csr().read().lsirdy().bit_is_clear() {}

    dp.RCC.bdcr().write(|w| w.rtcsel().lsi().rtcen().set_bit());
    // Remove write protection from RTC registers
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0xCA) });
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0x53) });

    // Enter init mode to set prescaler values
    dp.RTC.isr().write(|w| w.init().set_bit());
    while dp.RTC.isr().read().initf().bit_is_clear() {}
    dp.RTC
        .prer()
        .write(|w| unsafe { w.prediv_a().bits(127).prediv_s().bits(249) });
    dp.RTC.isr().write(|w| w.init().clear_bit());

    // Turn off wake-up timer
    dp.RTC.cr().write(|w| w.wute().clear_bit());
    while dp.RTC.isr().read().wutwf().bit_is_clear() {}

    // Write wake-up timer registers
    dp.RTC.wutr().write(|w| unsafe { w.wut().bits(4) });
    dp.EXTI.rtsr1().write(|w| w.tr20().set_bit());
    dp.EXTI.imr1().write(|w| w.mr20().set_bit());
    dp.RTC.cr().write(|w| {
        unsafe { w.wucksel().bits(0b100) }
            .wutie()
            .set_bit()
            .wute()
            .set_bit()
    });

    // Initialization commands
    let init_commands = INIT_COMMANDS.get();

    let _ = init_commands.enqueue(&W_RF_SETUP);
    let _ = init_commands.enqueue(&W_TX_ADDR);
    let _ = init_commands.enqueue(&W_RX_ADDR_P0);
    let _ = init_commands.enqueue(&W_CONFIG);
    let _ = init_commands.enqueue(&RESET_MAX_RT);

    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH6);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH7);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2)
    }
    GPIOA_PERIPHERAL.set(dp.GPIOA);
    SPI1_PERIPHERAL.set(dp.SPI1);
    USART2_PERIPHERAL.set(dp.USART2);
    DMA1_PERIPHERAL.set(dp.DMA1);
    TIM2_PERIPHERAL.set(dp.TIM2);
    RTC_PERIPHERAL.set(dp.RTC);
    EXTI_PERIPHERAL.set(dp.EXTI);
    CORE_PERIPHERALS.set(cp);

    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();

    send_command(&W_RF_CH, dma1, spi1);

    #[allow(clippy::empty_loop)]
    loop {
        // asm::wfi();
    }
}
