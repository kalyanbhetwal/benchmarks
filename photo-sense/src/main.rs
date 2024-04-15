#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;

#[macro_use]
mod intermittent;
use intermittent::*;


fn gpioUp() -> (){}
fn gpioDown() -> (){}
fn gpioTwiddle() -> (){}
fn delay(time:u32) -> (){}
fn checkpoint() -> (){}
fn restore_vol() -> (){}
fn msp_sleep(time:u16) -> (){}


//GUIDE: IO op annotations as pub static function pointer
#[no_mangle]
pub static IO_NAME : fn(count:u16) -> u16 =  read_light;

//simulated sensor function to make the app not crash 
// when using JIT checkpointing on intermittent power
fn read_light (count:u16) -> u16
{
    //spend a little time, because orig photo resistor was expensive
    let mut val = count;
    for _i in 0..50{
	val+=1;
    }
    //typical room level
    return 1400 + (val % 17)
}
 
//#[no_mangle]
//pub static IO_NAME : unsafe extern "C" fn() -> u16 =  read_light;


//GUIDE: and now write the app in rust normally
const WINDOW_SIZE:u16 = 3;

fn  average_light(count:u16) -> u16 {
	//Inferred region for Consistent set 1 starts here
	// annotation is yet to come, at line 78.
	let mut light:u16 = unsafe{read_light(count)};
	for _i in 1..WINDOW_SIZE {
            light += unsafe{read_light(count + _i)};
	    
	    //unsafe{msp_sleep(10);}
	}
	//Inferred region ends in the loop exit block
	light /= WINDOW_SIZE;
	return light;
}

#[entry]
fn main() -> ! {
    loop {
        for i in 0..100 {
            //PROTECT_BEGIN();
            //unsafe { printf(b"start\r\n\0".as_ptr());}
            //PROTECT_END();
            
                //PROTECT_BEGIN();
            let light = average_light(i);
            Consistent(light, 1);
            //PROTECT_END();
            //PROTECT_BEGIN();
            
            unsafe{
            //start_atomic();
            //printf(b"end %l\r\n\0".as_ptr(), light as u32);
            //end_atomic();
            }
            
            //PROTECT_END();
        }
        unsafe{ start_atomic();}
        unsafe{gpioTwiddle();}
            unsafe{gpioTwiddle();}
        unsafe{end_atomic();}
    }
}
