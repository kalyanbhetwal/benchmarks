#![allow(unsafe_code, unused, non_upper_case_globals)]
use core::mem;
use core::ptr;
use cortex_m::asm::{nop, self};
use panic_halt as _;

use cortex_m_semihosting::{debug, hprintln};
use stm32f3xx_hal_v2::pac::FLASH;

const UNLOCK_KEY1: u32 = 0x4567_0123;
const UNLOCK_KEY2: u32 = 0xCDEF_89AB;

pub fn unlock(flash: &mut FLASH) ->bool{

    if flash.cr.read().lock().bit_is_clear(){
        return true;
    }

    flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEY1) });
    flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEY2) });

    if flash.cr.read().lock().bit_is_clear() {
        return true;
    } else {
        return false;
    }
}

pub fn wait_ready(flash: &FLASH) {
    while flash.sr.read().bsy().bit() {}
}

pub fn clear_error_flags(flash: &FLASH) {
    if flash.sr.read().wrprterr().bit_is_set() {
        flash.sr.modify(|_, w| w.wrprterr().set_bit());
    }
    if flash.sr.read().pgerr().bit_is_set() {
        flash.sr.modify(|_, w| w.pgerr().set_bit());
    }
}

pub fn erase_page(flash: &mut FLASH, page: u32){

    // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
    // status register (FLASH_SR).
    if flash.sr.read().bsy().bit_is_set() {
        hprintln!("Flash is busy.");
     }

    // 2. Check and clear all error programming flags due to a previous programming. If not,
     // PGSERR is set.
    clear_error_flags(&flash);

    // 3. Set the PER bit and select the page you wish to erase (PNB). For dual bank variants:
     //  - with the associated bank(BKER) in the Flash control register (FLASH_CR).

     flash.cr.modify(|_, w| w.per().set_bit());

     // Program the FLASH_CR register
     // regs.ar.modify(|_, w| w.far().bits(page as u8));
     flash.ar.write(|w| unsafe { w.bits(page as u32) });


     // 4. Set the STRT bit in the FLASH_CR register.
     flash.cr.modify(|_, w| w.strt().set_bit());

    // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
      
    while flash.sr.read().bsy().bit_is_set() {}

    // 6. lock the flash
    while flash.sr.read().bsy().bit_is_set() {}
    flash.cr.modify(|_, w| w.lock().set_bit());

}

pub fn write_to_flash(flash: &mut FLASH, addr: u32, data: u32) {
        unlock(flash);

        // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
        if flash.sr.read().bsy().bit_is_set() {
            hprintln!("Flash is busy.");
        }
         
        clear_error_flags(&flash);
       // 2. Set the PG bit in the Flash control register (FLASH_CR).
       flash.cr.modify(|_, w| w.pg().set_bit());


        // 3. Perform the data write (half-word) at the desired address.
        unsafe{
                ptr::write_volatile(addr as *mut u16, data as u16);
                ptr::write_volatile((addr as usize + 2) as * mut u16, (data.wrapping_shr(16)) as u16);
        }

        // 4. Wait for the BSY bit to be cleared in the FLASH_SR register.

        while flash.sr.read().bsy().bit_is_set() {}
        // 5. lock the flash
        flash.cr.modify(|_, w| w.lock().set_bit());
        

        // 6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
        // operation has succeed), and clear it by software.
        if flash.sr.read().eop().bit_is_set() {
            flash.sr.modify(|_, w| w.eop().set_bit()); // Clear
        }

         // 6. Clear the PG bit in the FLASH_CR register if there no more programming request
        // anymore.
        flash.cr.modify(|_, w| w.pg().clear_bit());

}