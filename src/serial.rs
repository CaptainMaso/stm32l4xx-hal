//! Serial module
//!
//! This module support both polling and interrupt based accesses to the serial peripherals.

use as_slice::AsMutSlice;
use core::fmt;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::DerefMut;
use core::ptr;
use core::sync::atomic::{self, Ordering};
use generic_array::ArrayLength;
use stable_deref_trait::StableDeref;

use crate::hal::serial::{self, Write};

use crate::stm32::{USART1, USART2};

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::stm32::USART3;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::stm32::UART4;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::stm32::UART5;

use crate::gpio::gpioa::{PA0, PA1, PA10, PA11, PA12, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB3, PB4, PB6, PB7};
use crate::gpio::gpiod::{PD3, PD4, PD5, PD6};
use crate::gpio::{Alternate, Floating, Input, AF7};

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpioa::PA6;

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpiob::{PB1, PB10, PB11, PB13, PB14};

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpiod::{PD11, PD12, PD2};

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpioc::{PC10, PC11, PC4, PC5};

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::gpio::gpioa::PA15;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::gpio::gpiob::PB5;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::gpio::gpioc::PC12;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::gpio::gpiod::{PD8, PD9};

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::gpio::AF8;

use crate::dma::{dma1, CircBuffer, DMAFrame, FrameReader, FrameSender};
use crate::rcc::{Clocks, APB1R1, APB2};
use crate::time::{Bps, U32Ext};

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::dma::dma2;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// The line has gone idle
    Idle,
    /// Character match
    CharacterMatch,
    /// Receiver timeout
    ReceiverTimeout,
}

/// Serial error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

/// USART parity settings
pub enum Parity {
    /// No parity
    ParityNone,
    /// Even parity
    ParityEven,
    /// Odd parity
    ParityOdd,
}

/// USART stopbits settings
pub enum StopBits {
    /// 1 stop bit
    STOP1,
    /// 0.5 stop bits
    STOP0P5,
    /// 2 stop bits
    STOP2,
    // 1.5 stop bits
    STOP1P5,
}

/// USART oversampling settings
pub enum Oversampling {
    /// Oversample 8 times (allows for faster data rates)
    Over8,
    /// Oversample 16 times (higher stability)
    Over16,
}

/// USART Configuration structure
pub struct Config {
    baudrate: Bps,
    parity: Parity,
    stopbits: StopBits,
    oversampling: Oversampling,
    character_match: Option<u8>,
    receiver_timeout: Option<u32>,
}

impl Config {
    /// Set the baudrate to a specific value
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    /// Set parity to none
    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    /// Set parity to even
    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    /// Set parity to odd
    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    /// Set the number of stopbits
    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    /// Set the oversampling size
    pub fn oversampling(mut self, oversampling: Oversampling) -> Self {
        self.oversampling = oversampling;
        self
    }

    /// Set the character match character
    pub fn character_match(mut self, character_match: u8) -> Self {
        self.character_match = Some(character_match);
        self
    }

    /// Set the receiver timeout, the value is the number of bit durations
    ///
    /// Note that it only takes 24 bits, using more than this will cause a panic.
    pub fn receiver_timeout(mut self, receiver_timeout: u32) -> Self {
        assert!(receiver_timeout < 1 << 24);
        self.receiver_timeout = Some(receiver_timeout);
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            oversampling: Oversampling::Over16,
            character_match: None,
            receiver_timeout: None,
        }
    }
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $APB:ident,
            $usartXen:ident,
            $usartXrst:ident,
            $pclkX:ident,
            tx: ($dmacst:ident, $tx_chan:path),
            rx: ($dmacsr:ident, $rx_chan:path)
        ),
    )+) => {
        $(
            impl<PINS> Serial<$USARTX, PINS> {
                /// Configures the serial interface and creates the interface
                /// struct.
                ///
                /// `Config` is a config struct that configures baud rate, stop bits and parity.
                ///
                /// `Clocks` passes information about the current frequencies of
                /// the clocks.  The existence of the struct ensures that the
                /// clock settings are fixed.
                ///
                /// The `serial` struct takes ownership over the `USARTX` device
                /// registers and the specified `PINS`
                ///
                /// `MAPR` and `APBX` are register handles which are passed for
                /// configuration. (`MAPR` is used to map the USART to the
                /// corresponding pins. `APBX` is used to reset the USART.)
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    config: Config,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // enable or reset $USARTX
                    apb.enr().modify(|_, w| w.$usartXen().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().clear_bit());

                    // Reset other registers to disable advanced USART features
                    usart.cr1.reset();
                    usart.cr2.reset();
                    usart.cr3.reset();

                    // Configure baud rate
                    match config.oversampling {
                        Oversampling::Over8 => {
                            let uartdiv = 2 * clocks.$pclkX().0 / config.baudrate.0;
                            assert!(uartdiv >= 16, "impossible baud rate");

                            let lower = (uartdiv & 0xf) >> 1;
                            let brr = (uartdiv & !0xf) | lower;

                            usart.cr1.modify(|_, w| w.over8().set_bit());
                            usart.brr.write(|w| unsafe { w.bits(brr) });
                        }
                        Oversampling::Over16 => {
                            let brr = clocks.$pclkX().0 / config.baudrate.0;
                            assert!(brr >= 16, "impossible baud rate");

                            usart.brr.write(|w| unsafe { w.bits(brr) });
                        }
                    }

                    if let Some(val) = config.receiver_timeout {
                        usart.rtor.modify(|_, w| w.rto().bits(val));
                    }

                    // enable DMA transfers
                    usart.cr3.modify(|_, w| w.dmat().set_bit().dmar().set_bit());

                    // Configure hardware flow control (CTS/RTS or RS485 Driver Enable)
                    if PINS::FLOWCTL {
                        usart.cr3.modify(|_, w| w.rtse().set_bit().ctse().set_bit());
                    } else if PINS::DEM {
                        usart.cr3.modify(|_, w| w.dem().set_bit());

                        // Pre/post driver enable set conservative to the max time
                        usart.cr1.modify(|_, w| w.deat().bits(0b1111).dedt().bits(0b1111));
                    } else {
                        usart.cr3.modify(|_, w| w.rtse().clear_bit().ctse().clear_bit());
                    }

                    // Enable One bit sampling method
                    usart.cr3.modify(|_, w| w.onebit().set_bit());

                    // Configure parity and word length
                    // Unlike most uart devices, the "word length" of this usart device refers to
                    // the size of the data plus the parity bit. I.e. "word length"=8, parity=even
                    // results in 7 bits of data. Therefore, in order to get 8 bits and one parity
                    // bit, we need to set the "word" length to 9 when using parity bits.
                    let (word_length, parity_control_enable, parity) = match config.parity {
                        Parity::ParityNone => (false, false, false),
                        Parity::ParityEven => (true, true, false),
                        Parity::ParityOdd => (true, true, true),
                    };
                    usart.cr1.modify(|_r, w| {
                        w
                            .m0().bit(word_length)
                            .ps().bit(parity)
                            .pce().bit(parity_control_enable)
                    });

                    // Configure stop bits
                    let stop_bits = match config.stopbits {
                        StopBits::STOP1 => 0b00,
                        StopBits::STOP0P5 => 0b01,
                        StopBits::STOP2 => 0b10,
                        StopBits::STOP1P5 => 0b11,
                    };
                    usart.cr2.modify(|_r, w| {
                        w.stop().bits(stop_bits);

                        // Setup character match (if requested)
                        if let Some(c) = config.character_match {
                            w.add().bits(c);
                        }

                        if config.receiver_timeout.is_some() {
                            w.rtoen().set_bit();
                        }

                        w
                    });


                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .cr1
                        .modify(|_, w| w.ue().set_bit().re().set_bit().te().set_bit());

                    Serial { usart, pins }
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().set_bit())
                        },
                        Event::CharacterMatch => {
                            self.usart.cr1.modify(|_, w| w.cmie().set_bit())
                        },
                        Event::ReceiverTimeout => {
                            self.usart.cr1.modify(|_, w| w.rtoie().set_bit())
                        },
                    }
                }

                /// Stops listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().clear_bit())
                        },
                        Event::CharacterMatch => {
                            self.usart.cr1.modify(|_, w| w.cmie().clear_bit())
                        },
                        Event::ReceiverTimeout => {
                            self.usart.cr1.modify(|_, w| w.rtoie().clear_bit())
                        },
                    }
                }

                /// Splits the `Serial` abstraction into a transmitter and a receiver half
                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }

                /// Frees the USART peripheral
                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }
            }

            impl<PINS> serial::Read<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    // NOTE(unsafe): Only used for atomic writes, to clear error flags.
                    let icr = unsafe { &(*$USARTX::ptr()).icr };

                    Err(if isr.pe().bit_is_set() {
                        icr.write(|w| w.pecf().clear());
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        icr.write(|w| w.fecf().clear());
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        icr.write(|w| w.ncf().clear());
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        icr.write(|w| w.orecf().clear());
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> serial::Write<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.write(byte)
                }
            }

            impl serial::Write<u8> for Tx<$USARTX> {
                // NOTE(Void) See section "29.7 USART interrupts"; the only possible errors during
                // transmission are: clear to send (which is disabled in this case) errors and
                // framing errors (which only occur in SmartCard mode); neither of these apply to
                // our hardware configuration
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl Rx<$USARTX> {
                pub fn circ_read<B, H>(
                    &self,
                    mut chan: $rx_chan,
                    mut buffer: B,
                ) -> CircBuffer<B, $rx_chan>
                where
                    B: StableDeref<Target = [H; 2]> + DerefMut + 'static,
                    H: AsMutSlice<Element = u8>
                {
                    let buf = buffer[0].as_mut_slice();
                    chan.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).rdr as *const _ as u32 }, false);
                    chan.set_memory_address(buf.as_ptr() as u32, true);
                    chan.set_transfer_length((buf.len() * 2) as u16);

                    // Tell DMA to request from serial
                    chan.cselr().modify(|_, w| {
                        w.$dmacsr().bits(0b0010) // TODO: Fix this, not valid for DMA2
                    });

                    chan.ccr().modify(|_, w| unsafe {
                        w.mem2mem()
                            .clear_bit()
                            // 00: Low, 01: Medium, 10: High, 11: Very high
                            .pl()
                            .bits(0b01)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .msize()
                            .bits(0b00)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .psize()
                            .bits(0b00)
                            .circ()
                            .set_bit()
                            .dir()
                            .clear_bit()
                    });

                    // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                    // the next statement, which starts the DMA transfer
                    atomic::compiler_fence(Ordering::Release);

                    chan.start();

                    CircBuffer::new(buffer, chan)
                }

                /// Create a frame reader that can either react on the Character match interrupt or
                /// Transfer Complete from the DMA.
                pub fn frame_read<BUFFER, N>(
                    &self,
                    mut channel: $rx_chan,
                    buffer: BUFFER,
                ) -> FrameReader<BUFFER, $rx_chan, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        N: ArrayLength<MaybeUninit<u8>>,
                {
                    let usart = unsafe{ &(*$USARTX::ptr()) };

                    // Setup DMA transfer
                    let buf = &*buffer;
                    channel.set_peripheral_address(&usart.rdr as *const _ as u32, false);
                    channel.set_memory_address(unsafe { buf.buffer_address_for_dma() } as u32, true);
                    channel.set_transfer_length(buf.max_len() as u16);

                    // Tell DMA to request from serial
                    channel.cselr().modify(|_, w| {
                        w.$dmacsr().bits(0b0010) // TODO: Fix this, not valid for DMA2
                    });

                    channel.ccr().modify(|_, w| unsafe {
                        w.mem2mem()
                            .clear_bit()
                            // 00: Low, 01: Medium, 10: High, 11: Very high
                            .pl()
                            .bits(0b01)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .msize()
                            .bits(0b00)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .psize()
                            .bits(0b00)
                            // Peripheral -> Mem
                            .dir()
                            .clear_bit()
                    });

                    // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                    // the next statement, which starts the DMA transfer
                    atomic::compiler_fence(Ordering::Release);

                    channel.start();

                    FrameReader::new(buffer, channel, usart.cr2.read().add().bits())
                }

                /// Checks to see if the USART peripheral has detected an idle line and clears
                /// the flag
                pub fn is_idle(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*$USARTX::ptr()).icr };

                    if isr.idle().bit_is_set() {
                        if clear {
                            icr.write(|w| w.idlecf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }

                /// Checks to see if the USART peripheral has detected an character match and
                /// clears the flag
                pub fn is_character_match(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*$USARTX::ptr()).icr };

                    if isr.cmf().bit_is_set() {
                        if clear {
                            icr.write(|w| w.cmcf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }

                /// Checks to see if the USART peripheral has detected an receiver timeout and
                /// clears the flag
                pub fn is_receiver_timeout(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*$USARTX::ptr()).icr };

                    if isr.rtof().bit_is_set() {
                        if clear {
                            icr.write(|w| w.rtocf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }
            }

            impl Tx<$USARTX> {
                /// Creates a new DMA frame sender
                pub fn frame_sender<BUFFER, N>(
                    &self,
                    mut channel: $tx_chan,
                ) -> FrameSender<BUFFER, $tx_chan, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        N: ArrayLength<MaybeUninit<u8>>,
                {
                    let usart = unsafe{ &(*$USARTX::ptr()) };

                    // Setup DMA
                    channel.set_peripheral_address(&usart.tdr as *const _ as u32, false);

                    // Tell DMA to request from serial
                    channel.cselr().modify(|_, w| {
                        w.$dmacst().bits(0b0010) // TODO: Fix this, not valid for DMA2
                    });

                    channel.ccr().modify(|_, w| unsafe {
                        w.mem2mem()
                            .clear_bit()
                            // 00: Low, 01: Medium, 10: High, 11: Very high
                            .pl()
                            .bits(0b01)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .msize()
                            .bits(0b00)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .psize()
                            .bits(0b00)
                            // Mem -> Peripheral
                            .dir()
                            .set_bit()
                    });

                    FrameSender::new(channel)
                }
            }
        )+
    }
}

hal! {
    USART1: (usart1, APB2, usart1en, usart1rst, pclk2, tx: (c4s, dma1::C4), rx: (c5s, dma1::C5)),
    USART2: (usart2, APB1R1, usart2en, usart2rst, pclk1, tx: (c7s, dma1::C7), rx: (c6s, dma1::C6)),
}

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
hal! {
    USART3: (usart3, APB1R1, usart3en, usart3rst, pclk1, tx: (c2s, dma1::C2), rx: (c3s, dma1::C3)),
}

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
hal! {
    UART4: (uart4, APB1R1, uart4en, uart4rst, pclk1, tx: (c3s, dma2::C3), rx: (c5s, dma2::C5)),
}

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
hal! {
    UART5: (uart5, APB1R1, uart5en, uart5rst, pclk1, tx: (c1s, dma2::C1), rx: (c2s, dma2::C2)),
}

impl<USART, PINS> fmt::Write for Serial<USART, PINS>
where
    Serial<USART, PINS>: crate::hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .iter()
            .map(|c| nb::block!(self.write(*c)))
            .last();
        Ok(())
    }
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: crate::hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .iter()
            .map(|c| nb::block!(self.write(*c)))
            .last();
        Ok(())
    }
}

/// Marks pins as being as being TX pins for the given USART instance
pub trait TxPin<Instance>: private::SealedTx {}

/// Marks pins as being as being RX pins for the given USART instance
pub trait RxPin<Instance>: private::SealedRx {}

/// Marks pins as being as being RTS pins for the given USART instance
pub trait RtsDePin<Instance>: private::SealedRtsDe {}

/// Marks pins as being as being CTS pins for the given USART instance
pub trait CtsPin<Instance>: private::SealedCts {}

macro_rules! impl_pin_traits {
    (
        $(
            $instance:ident: {
                $(
                    $af:ident: {
                        TX: $($tx:ident),*;
                        RX: $($rx:ident),*;
                        RTS_DE: $($rts_de:ident),*;
                        CTS: $($cts:ident),*;
                    }
                )*
            }
        )*
    ) => {
        $(
            $(
                $(
                    impl private::SealedTx for
                        $tx<Alternate<$af, Input<Floating>>> {}
                    impl TxPin<$instance> for
                        $tx<Alternate<$af, Input<Floating>>> {}
                )*

                $(
                    impl private::SealedRx for
                        $rx<Alternate<$af, Input<Floating>>> {}
                    impl RxPin<$instance> for
                        $rx<Alternate<$af, Input<Floating>>> {}
                )*

                $(
                    impl private::SealedRtsDe for
                        $rts_de<Alternate<$af, Input<Floating>>> {}
                    impl RtsDePin<$instance> for
                        $rts_de<Alternate<$af, Input<Floating>>> {}
                )*

                $(
                    impl private::SealedCts for
                        $cts<Alternate<$af, Input<Floating>>> {}
                    impl CtsPin<$instance> for
                        $cts<Alternate<$af, Input<Floating>>> {}
                )*
            )*
        )*
    };
}

impl_pin_traits! {
    USART1: {
        AF7: {
            TX: PA9, PB6;
            RX: PA10, PB7;
            RTS_DE: PA12, PB3;
            CTS: PA11, PB4;
        }
    }
    USART2: {
        AF7: {
            TX: PA2, PD5;
            RX: PA3, PD6;
            RTS_DE: PA1, PD4;
            CTS: PA0, PD3;
        }
    }
}

#[cfg(any(
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
impl_pin_traits! {
    USART3: {
        AF7: {
            TX: PB10, PC4, PC10, PD8;
            RX: PB11, PC5, PC11, PD9;
            RTS_DE: PB1, PB14, PD2, PD12;
            CTS: PA6, PB13, PD11;
        }
    }
}

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
impl_pin_traits! {
    UART4: {
        AF8: {
            TX: PA0, PC10;
            RX: PA1, PC11;
            RTS_DE: PA15;
            CTS: PB7;
        }
    }
    UART5: {
        AF8: {
            TX: PC12;
            RX: PD2;
            RTS_DE: PB4;
            CTS: PB5;
        }
    }
}

/// Pins trait for detecting hardware flow control or RS485 mode.
pub trait Pins<USART> {
    const FLOWCTL: bool;
    const DEM: bool;
}

// No flow control, just Rx+Tx
impl<Instance, Tx, Rx> Pins<Instance> for (Tx, Rx)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
{
    const FLOWCTL: bool = false;
    const DEM: bool = false;
}

// Hardware flow control, Rx+Tx+Rts+Cts
impl<Instance, Tx, Rx, Rts, Cts> Pins<Instance> for (Tx, Rx, Rts, Cts)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
    Rts: RtsDePin<Instance>,
    Cts: CtsPin<Instance>,
{
    const FLOWCTL: bool = true;
    const DEM: bool = false;
}

// DEM for RS485 mode
impl<Instance, Tx, Rx, De> Pins<Instance> for (Tx, Rx, De)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
    De: RtsDePin<Instance>,
{
    const FLOWCTL: bool = false;
    const DEM: bool = true;
}

/// Contains supertraits used to restrict which traits users can implement
mod private {
    pub trait SealedTx {}
    pub trait SealedRx {}
    pub trait SealedRtsDe {}
    pub trait SealedCts {}
}
