macro_rules! DMASerial {
    (create $NAME:ident using $USARTX:ident with channel $TXCHANNEL:ident, $RXCHANNEL:ident and buffer size $SIZE:expr; read dma length with $NDTR:stmt) => {

        pub mod $NAME {

            use stm32f1xx_hal::{
                serial::Tx,
                serial::Rx,
                serial::Pins,
                dma::Transfer,
                dma::R,
                dma::CircBuffer,
                dma::CircReadDma,
                dma::WriteDma,
                dma::RxDma,
                dma::TxDma,
                dma::Half,
                dma::dma1::$TXCHANNEL,
                dma::dma1::$RXCHANNEL,
            };
            use stm32f1::stm32f103::$USARTX;
            use core::convert::AsMut;
            use core::convert::AsRef;

            enum State {
                Reading(CircBuffer<Buffer, RxDma<Rx<$USARTX>, $RXCHANNEL>>),
                Writing(Transfer<R, &'static Buffer, TxDma<Tx<$USARTX>, $TXCHANNEL>>),
            }

            pub struct Buffer {
                pub buf: [u8; $SIZE],
                pub content_length: usize,
                pub parity_error: bool,
            }
            impl Buffer {
                pub fn new() -> Self {
                    Self{buf: [0; $SIZE], content_length: 0, parity_error: false}
                }
            }
            impl AsMut<[u8]> for Buffer {
                fn as_mut(&mut self) -> &mut [u8] {
                    &mut self.buf
                }   
            }
            impl AsRef<[u8]> for Buffer {
                fn as_ref(&self) -> &[u8] {
                    &self.buf
                }
            }

            pub enum BufferType {
                Write(&'static Buffer),
                Read(&'static mut [Buffer; 2]),
            }
            
            pub struct Serial {
                tx_dma: Option<TxDma<Tx<$USARTX>, $TXCHANNEL>>,
                rx_dma: Option<RxDma<Rx<$USARTX>, $RXCHANNEL>>,
                state: Option<State>,
                buf: Option<BufferType>,
                should_stop: bool,
            }

            impl Serial {
                pub fn new<PINS: Pins<$USARTX>>(serial: stm32f1xx_hal::serial::Serial<$USARTX, PINS>, tx_channel: $TXCHANNEL, rx_channel: $RXCHANNEL) -> Self {
                    let (tx, rx) = serial.split();
                    let tx_dma = tx.with_dma(tx_channel);
                    let rx_dma = rx.with_dma(rx_channel);
                    
                    // Enable idle line interrupt
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) };

                    Self {
                        tx_dma: Some(tx_dma),
                        rx_dma: Some(rx_dma),
                        state: None,
                        buf: None,
                        should_stop: false,
                    }
                }

                pub fn read(&mut self, buf: &'static mut [Buffer; 2]) -> Option<&'static mut [Buffer; 2]> {
                    if !self.is_idle(){
                        return Some(buf)
                    }
                    if let Some(rx) = self.rx_dma.take() {
                        self.state = Some(State::Reading(rx.circ_read(buf)));
                        self.should_stop = false;
                        return None
                    }
                    Some(buf)
                }

                pub fn write(&mut self, buf: &'static Buffer) -> Option<&'static Buffer> {
                    if !self.is_idle(){
                        return Some(buf)
                    }
                    if let Some(tx) = self.tx_dma.take() {
                        self.state = Some(State::Writing(tx.write(buf)));
                        return None
                    }
                    Some(buf)
                }

                pub fn stop(&mut self) -> () {
                    self.should_stop = true;
                }

                pub fn currently_read(&self) -> usize {
                    return (2*$SIZE) - {$NDTR} as usize
                }

                fn check_state(&mut self) -> () {
                    if match &mut self.state {
                        Some(State::Reading(t)) => t.readable_half().unwrap() == Half::First || self.should_stop,
                        Some(State::Writing(t)) => t.is_done(),
                        None => false
                    } {
                        if let Some(st) = self.state.take() {
                            match st {
                                State::Reading(t) => {
                                    let (mut b, rx) = t.stop();
                                    b[0].content_length = self.currently_read();
                                    let sr = unsafe { (*$USARTX::ptr()).sr.read() };
                                    //TODO: check other errors than parity
                                    b[0].parity_error = sr.pe().bit_is_set();
                                    self.rx_dma = Some(rx);
                                    self.buf = Some(BufferType::Read(b));
                                },
                                State::Writing(t) => {
                                    let (b, tx) = t.wait();
                                    self.tx_dma = Some(tx);
                                    self.buf = Some(BufferType::Write(b));
                                },
                            };
                        }
                    }
                }

                pub fn get_read_buffer(&mut self) -> Option<&'static mut [Buffer; 2]> {
                    if let Some(BufferType::Read(_)) = &self.buf {
                        if let Some(BufferType::Read(buf)) = self.buf.take() {
                            return Some(buf)
                        }
                    }
                    None
                }

                pub fn get_write_buffer(&mut self) -> Option<&'static mut Buffer> {
                    if let Some(BufferType::Write(_)) = &self.buf {
                        if let Some(BufferType::Write(buf)) = self.buf.take() {
                            #[allow(mutable_transmutes)]
                            return unsafe { Some(core::mem::transmute::<&'static Buffer, &'static mut Buffer>(buf)) }
                        }
                    }
                    None
                }

                pub fn is_reading(&mut self) -> bool {
                    self.check_state();
                    match self.state { Some(State::Reading(_)) => true, _ => false}
                }

                pub fn is_writing(&mut self) -> bool {
                    self.check_state();
                    match self.state { Some(State::Writing(_)) => true, _ => false}
                }

                pub fn is_idle(&mut self) -> bool {
                    self.check_state();
                    self.state.is_none()
                }

                pub fn handle_interrupt(&mut self) -> () {
                    if unsafe{(*$USARTX::ptr()).sr.read().idle().bit_is_set()} {
                        if self.currently_read() > 0 {
                            self.stop();
                        }
                    }
                }
            }
        }
    };
}

DMASerial!(create dma_serial1 using USART1 with channel C4, C5 and buffer size 27; read dma length with unsafe{(*stm32f1::stm32f103::DMA1::ptr()).ch5.ndtr.read().bits()});
DMASerial!(create dma_serial2 using USART2 with channel C7, C6 and buffer size 27; read dma length with unsafe{(*stm32f1::stm32f103::DMA1::ptr()).ch6.ndtr.read().bits()});
DMASerial!(create dma_serial3 using USART3 with channel C2, C3 and buffer size 27; read dma length with unsafe{(*stm32f1::stm32f103::DMA1::ptr()).ch3.ndtr.read().bits()});