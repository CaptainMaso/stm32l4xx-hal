use stm32l4::stm32l4x1::{SYSCFG, EXTI, Interrupt};

use crate::rcc::{APB2, Enable};

use super::*;

/// Pin type with dynamic mode and erased Port and Pin
///
/// WARNING: This type does not adhere to the standard
/// safety limitations that other GPIO pins do, namely
/// that it does not require mutable access to a GPIO
/// Register to edit the types. In normal cases, this 
/// usually will not cause issues as the register writes
/// are atomic, but just in case this type will disable
/// interrupts during the conversion between modes.
pub struct FlexPin {
    // Bits 0-3: Pin, Bits 4-7: Port
    pin_port: u8,
    /// Current pin mode
    pub(crate) mode: Dynamic,
}

impl fmt::Debug for FlexPin {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_fmt(format_args!(
            "P{}{}<{:?}>",
            self.port_id(),
            self.pin_id(),
            self.mode
        ))
    }
}

impl FlexPin {
    pub(super) const fn new(port : u8, pin : u8, mode: Dynamic) -> Self {
        Self {
            pin_port: port << 4 | pin,
            mode
        }
    }

    #[inline(always)]
    pub fn pin_id(&self) -> u8 {
        self.pin_port & 0x0f
    }
    #[inline(always)]
    pub fn port_id(&self) -> u8 {
        self.pin_port >> 4
    }

    #[inline]
    unsafe fn block(&self) -> &crate::pac::gpioa::RegisterBlock {
        // This function uses pointer arithmetic instead of branching to be more efficient

        // The logic relies on the following assumptions:
        // - GPIOA register is available on all chips
        // - all gpio register blocks have the same layout
        // - consecutive gpio register blocks have the same offset between them, namely 0x0400
        // - ErasedPin::new was called with a valid port

        // FIXME could be calculated after const_raw_ptr_to_usize_cast stabilization #51910
        const GPIO_REGISTER_OFFSET: usize = 0x0400;

        let offset = GPIO_REGISTER_OFFSET * self.port_id() as usize;
        let block_ptr =
            (crate::pac::GPIOA::ptr() as usize + offset) as *const crate::pac::gpioa::RegisterBlock;

        &*block_ptr
    }

    /// Set the internal pull-up and pull-down resistor
    fn set_internal_resistor(&mut self, resistor: Pull) {
        let offset = 2 * self.pin_id();
        let value = resistor as u32;
        unsafe {
            self.block()
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }

    /// Puts `self` into mode `M`.
    ///
    /// This violates the type state constraints from `MODE`, so callers must
    /// ensure they use this properly.
    #[inline(always)]
    pub(super) fn set_mode<MODE : PinMode>(&mut self) {
        let n = self.pin_id();
        let offset = 2 * n;
        unsafe {
            if MODE::OTYPER != self.mode.otyper() {
                if let Some(otyper) = MODE::OTYPER {
                    self.block()
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(0b1 << n) | (otyper << n)));
                }
            }

            if MODE::MODER != self.mode.moder() {
                self.block()
                    .moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (MODE::MODER << offset)));
            }
        }
    }

    #[inline(always)]
    /// Converts pin into specified mode
    pub fn change_mode(&mut self, mode : Dynamic) {
        unsafe { cortex_m::interrupt::CriticalSection::new(); }

        match mode {
            Dynamic::InputFloating => {
                self.set_mode::<Input>();
                self.set_internal_resistor(Pull::None);
            },
            Dynamic::InputPullUp => {
                self.set_mode::<Input>();
                self.set_internal_resistor(Pull::Up);
            },
            Dynamic::InputPullDown => {
                self.set_mode::<Input>();
                self.set_internal_resistor(Pull::Down);
            },
            Dynamic::OutputPushPull => {
                self.set_internal_resistor(Pull::None);
                self.set_mode::<Output<PushPull>>();
            },
            Dynamic::OutputOpenDrain => {
                self.set_internal_resistor(Pull::None);
                self.set_mode::<Output<OpenDrain>>();
            },
        }

        self.mode = mode;
    }

    pub fn mode(&self) -> Dynamic {
        self.mode
    }

    pub fn is_input(&self) -> bool {
        self.mode.is_input()
    }

    pub fn is_output(&self) -> bool {
        self.mode.is_output()
    }

    pub fn is_interruptable(&self) -> bool {
        self.mode.is_interruptable()
    }

    /// Drives the pin high
    pub fn set_high(&mut self) -> Result<(), PinModeError> {
        if self.mode.is_output() {
            self._set_high();
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }

    /// Drives the pin low
    pub fn set_low(&mut self) -> Result<(), PinModeError> {
        if self.mode.is_output() {
            self._set_low();
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }

    /// Is the input pin high?
    pub fn is_high(&self) -> Result<bool, PinModeError> {
        self.is_low().map(|b| !b)
    }

    /// Is the input pin low?
    pub fn is_low(&self) -> Result<bool, PinModeError> {
        if self.mode.is_input() {
            Ok(self._is_low())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }

    /// Is the output pin set low?
    pub fn is_set_high(&self) -> Result<bool, PinModeError> {
        self.is_set_low().map(|b| !b)
    }

    /// Is the output pin set low?
    pub fn is_set_low(&self) -> Result<bool, PinModeError> {
        if self.mode.is_output() {
            Ok(self._is_set_low())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }

    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { self.block().bsrr.write(|w| w.bits(1 << self.pin_id())) }
    }
    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { self.block().bsrr.write(|w| w.bits(1 << (16 + self.pin_id()))) }
    }
    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { self.block().odr.read().bits() & (1 << self.pin_id()) == 0 }
    }
    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { self.block().idr.read().bits() & (1 << self.pin_id()) == 0 }
    }
}

impl ExtiPin for FlexPin {
    type Result<T> = Result<T,PinModeError>;

    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG, apb2: &mut APB2) -> Result<(),PinModeError> {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }

        // SYSCFG clock must be enabled in order to do register writes
        SYSCFG::enable(apb2);

        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                syscfg.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                syscfg.exticr2.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                syscfg.exticr3.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                syscfg.exticr4.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }

        Ok(())
    }

    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) -> Result<(),PinModeError>  {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }

        let i = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }

        Ok(())
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) -> Result<(),PinModeError>  {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
        Ok(())
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) -> Result<(),PinModeError>  {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
        Ok(())
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self) -> Result<(),PinModeError>  {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }
        unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << self.pin_id())) };
        Ok(())
    }

    /// Reads the interrupt pending bit for this pin
    #[inline(always)]
    fn check_interrupt(&self) -> Result<bool,PinModeError>  {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }
        Ok(unsafe { ((*EXTI::ptr()).pr1.read().bits() & (1 << self.pin_id())) != 0 })
    }

    /// Get the `Interrupt` for this pin.
    fn interrupt(&self) -> Result<Interrupt,PinModeError> {
        if !self.mode.is_interruptable() {
            return Err(PinModeError::IncorrectMode);
        }

        let res = match self.pin_id() {
            0 => Interrupt::EXTI0,
            1 => Interrupt::EXTI1,
            2 => Interrupt::EXTI2,
            3 => Interrupt::EXTI3,
            4 => Interrupt::EXTI4,
            5..=9 => Interrupt::EXTI9_5,
            10..=15 => Interrupt::EXTI15_10,
            _ => unreachable!(),
        };
        Ok(res)
    }
}
