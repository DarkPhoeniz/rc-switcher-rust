#![no_std]
#![no_main]
#![no_mangle]

extern crate panic_halt;

mod dma_serial;
use dma_serial::*;

use stm32f1xx_hal::{
    serial::Serial,
    serial::Config,
    serial::Parity,
    serial::StopBits,
    gpio::Output,
    gpio::PushPull,
    gpio::gpiob,
    gpio::GpioExt,
    time::U32Ext,
    flash::FlashExt,
    rcc::RccExt,
    afio::AfioExt,
    dma::DmaExt,
};
use embedded_hal::digital::v2::OutputPin;
use rtfm::{
    app,
    Instant,
};
use cortex_m::singleton;

static CLK_SPEED: u32 = 64_000_000;

macro_rules! msToCycles {
    ($ms: expr) => {
        (CLK_SPEED / 1000 * $ms).cycles()
    };
}

#[app(device = stm32f1::stm32f103)]
const APP: () = {

    static mut LED: gpiob::PB13<Output<PushPull>> = ();
    static mut LED_STATE: bool = false;
    static mut TX_SERIAL: dma_serial1::Serial = ();
    static mut RC_SERIAL: dma_serial3::Serial = ();
    static mut OUT_SERIAL: dma_serial2::Serial = ();
    static mut TX_BUFFER: Option<&'static mut [dma_serial1::Buffer; 2]> = ();
    static mut RC_BUFFER: Option<&'static mut [dma_serial3::Buffer; 2]> = ();
    static mut OUT_BUFFER: Option<&'static mut dma_serial2::Buffer> = ();

    #[init(schedule = [read_tx, read_rc])]
    fn init() -> init::LateResources {

        let device: stm32f1::stm32f103::Peripherals = device;
        
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        
        // Freeze the configuration of all the clocks in the system and store
        // the frozen frequencies in `clocks`
        let clocks = rcc.cfgr.sysclk(CLK_SPEED.hz()).pclk1(32.mhz()).freeze(&mut flash.acr);
        
        // Acquire the GPIOC peripheral
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        
        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
        // in order to configure the port. For pins 0-7, crl should be passed instead.
        let mut led = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        led.set_high().unwrap();
        
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        
        let pin_tx1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let pin_rx1 = gpioa.pa10;

        let pin_tx2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let pin_rx2 = gpioa.pa3;

        let pin_tx3 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let pin_rx3 = gpiob.pb11;

        let serial1 = Serial::usart1(
            device.USART1,
            (pin_tx1, pin_rx1),
            &mut afio.mapr,
            Config{
                baudrate: 100_000.bps(),
                parity: Parity::ParityEven,
                stopbits: StopBits::STOP2,
            },
            clocks,
            &mut rcc.apb2,
        );
        let serial2 = Serial::usart2(
            device.USART2,
            (pin_tx2, pin_rx2),
            &mut afio.mapr,
            Config{
                baudrate: 100_000.bps(),
                parity: Parity::ParityEven,
                stopbits: StopBits::STOP2,
            },
            clocks,
            &mut rcc.apb1,
        );
        let serial3 = Serial::usart3(
            device.USART3,
            (pin_tx3, pin_rx3),
            &mut afio.mapr,
            Config{
                baudrate: 100_000.bps(),
                parity: Parity::ParityEven,
                stopbits: StopBits::STOP2,
            },
            clocks,
            &mut rcc.apb1,
        );
        
        let channels = device.DMA1.split(&mut rcc.ahb);

        let dma_serial1 = dma_serial1::Serial::new(serial1, channels.4, channels.5);
        let dma_serial2 = dma_serial2::Serial::new(serial2, channels.7, channels.6);
        let dma_serial3 = dma_serial3::Serial::new(serial3, channels.2, channels.3);
        
        let dma1_rx_buf = singleton!(:[dma_serial1::Buffer; 2] = [dma_serial1::Buffer::new(), dma_serial1::Buffer::new()]).unwrap();
        let dma2_tx_buf = singleton!(:dma_serial2::Buffer = dma_serial2::Buffer::new()).unwrap();
        let dma3_rx_buf = singleton!(:[dma_serial3::Buffer; 2] = [dma_serial3::Buffer::new(), dma_serial3::Buffer::new()]).unwrap();

        let now = Instant::now();
        schedule.read_rc(now).unwrap();
        schedule.read_tx(now).unwrap();

        init::LateResources{
            LED: led,
            TX_SERIAL: dma_serial1,
            RC_SERIAL: dma_serial3,
            OUT_SERIAL: dma_serial2,
            TX_BUFFER: Some(dma1_rx_buf),
            RC_BUFFER: Some(dma3_rx_buf),
            OUT_BUFFER: Some(dma2_tx_buf),
        }
    }

    #[idle/*(resources = [LED])*/]
    fn idle() -> ! {
        //resources.LED.set_low().unwrap();
        loop {
            
        }
    }

    #[task(resources = [TX_SERIAL, TX_BUFFER], schedule = [read_tx])]
    fn read_tx() -> () {
        let now = Instant::now();
        let mut serial = resources.TX_SERIAL;

        if serial.is_idle() {
            if let Some(rx_buffer) = serial.get_read_buffer() {
                if !rx_buffer[0].parity_error {
                    // Parse and send
                }
                *resources.TX_BUFFER = Some(rx_buffer);
            }

            if let Some(rx_buffer) = resources.TX_BUFFER.take() {
                serial.read(rx_buffer);
            }
        }

        schedule.read_tx(now + msToCycles!(5)).unwrap();
    }

    #[task(resources = [RC_SERIAL, RC_BUFFER], schedule = [read_rc])]
    fn read_rc() -> () {
        let now = Instant::now();
        let mut serial = resources.RC_SERIAL;

        if serial.is_idle() {
            if let Some(rx_buffer) = serial.get_read_buffer() {
                if !rx_buffer[0].parity_error {
                    // Parse and send
                }
                *resources.RC_BUFFER = Some(rx_buffer);
            }

            if let Some(rx_buffer) = resources.RC_BUFFER.take() {
                serial.read(rx_buffer);
            }
        }

        schedule.read_rc(now + msToCycles!(5)).unwrap();
    }

    #[task(resources = [OUT_SERIAL, OUT_BUFFER, LED, LED_STATE], schedule = [send_out])]
    fn send_out() -> () {
        let now = Instant::now();
        let mut serial = resources.OUT_SERIAL;

        if serial.is_idle() {
            if let Some(tx_buffer) = serial.get_write_buffer() {
                *resources.OUT_BUFFER = Some(tx_buffer);
            }

            if let Some(tx_buffer) = resources.OUT_BUFFER.take() {
                for i in 0..tx_buffer.buf.len() {
                    tx_buffer.buf[i] = b'f'; 
                }
                serial.write(tx_buffer);
            }
            if *resources.LED_STATE {
                resources.LED.set_high().unwrap();
            }
            else {
                resources.LED.set_low().unwrap();
            }
            *resources.LED_STATE = !*resources.LED_STATE;
        }

        schedule.send_out(now + msToCycles!(500)).unwrap();
    }

    #[interrupt(resources = [TX_SERIAL])]
    fn USART1() -> () {
        resources.TX_SERIAL.handle_interrupt();
    }

    #[interrupt(resources = [OUT_SERIAL])]
    fn USART2() -> () {
        resources.OUT_SERIAL.handle_interrupt();
    }

    #[interrupt(resources = [RC_SERIAL])]
    fn USART3() -> () {
        resources.RC_SERIAL.handle_interrupt();
    }

    extern "C" {
        fn ADC1_2();
    }
};