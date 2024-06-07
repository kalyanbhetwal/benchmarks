#![allow(unsafe_code,unused, non_upper_case_globals,non_snake_case, non_camel_case_types )]
#![no_main]
#![no_std]
use core::mem;
use core::ptr;
use cortex_m::asm::nop;
use panic_halt as _;


use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use cortex_m::interrupt;
use cortex_m_semihosting::hprintln;
use stm32f3xx_hal_v2::{pac::Peripherals, pac::Interrupt};
use volatile::Volatile;
use cortex_m::peripheral::NVIC;

#[macro_use]
mod intermittent;
use intermittent::*;

fn gpioUp() -> (){}
fn gpioDown() -> (){}
fn gpioTwiddle() -> (){}
fn gpioOneTwiddle() -> (){}
fn delay(time:u32) -> (){}

//GUIDE: IO op annotations as pub static function pointer
#[no_mangle]
pub static IO_NAME :  /*unsafe extern "C"*/ fn() -> u16 =  adcConfig;
#[no_mangle]
pub static IO_NAME2 : /*unsafe extern "C"*/   fn(u16) -> u16 =  adcSample;
#[no_mangle]
pub static IO_NAME3 : /*unsafe extern "C"*/   fn() -> u16 =  tempConfig;
#[no_mangle]
pub static IO_NAME4 : /*unsafe extern "C"*/  fn(u16) -> f32 =  tempDegC;

fn adcConfig () -> u16
{
    //basically no-op without a sensor, but make it take a few cycles
    let mut reg = 0;
    for _i in 0..40 {
	reg+=1;
    }
    return 0;
}

fn tempConfig () -> u16
{
    //basically no-op without a sensor, but make it take a few cycles
    let mut reg = 0;
    for _i in 0..40 {
	reg+=1;
    }
    return 0;
}

fn adcSample (count:u16) -> u16
{
    //use printf as it is expensive, like sensor
    unsafe{
    //output_guard_start();
    //printf(b"adc\r\n\0".as_ptr());
    hprintln!("adc").unwrap();
	//output_guard_end();
    }
    return 1000 + (count % 17)
}

fn tempDegC (count:u16) -> f32
{
    //use printf as it is expensive, like sensor
    unsafe{
    //output_guard_start();
    //printf(b"temp\r\n\0".as_ptr());
    hprintln!("temp\r\n\0").unwrap();
	//output_guard_end();
    }
    return 4.7 + count as f32;
}

// //GUIDE:This is neessary! It causes everything to be visible to C 
// #[no_mangle]
// pub extern "C" fn _entry() {
//     app();
// }

//GUIDE: and now write the app in rust normally

pub const SAMPLE_SIZE:usize  = 5;

struct Tuple {
    m:u16,
    t:f32,
}


fn calcAvg(moisture:&[u16], temperature:&[f32]) -> Tuple
{
    let mut avg:Tuple = Tuple{m:0, t:0.0};
    for i in 0..SAMPLE_SIZE
    {
        avg.m += moisture[i];
        avg.t = avg.t + temperature[i];
    }
    avg.m = avg.m/(SAMPLE_SIZE as u16);
    avg.t = avg.t/(SAMPLE_SIZE as f32);
    return avg;
}

fn storeData(m:u16, t:f32, moisture:&mut [u16], temperature:&mut [f32]) -> ()
{
    for i in 1..SAMPLE_SIZE
    {
        moisture[i] = moisture[i-1];
        temperature[i] = temperature[i-1];
    }
    moisture[0] = m;
    temperature[0] = t;
}

fn compute(avg:&Tuple) -> ()
{
    //ledConfig();
    if avg.t > 10.0 && avg.t < 22.0
    {
        unsafe{gpioOneTwiddle()};
    }
    else if avg.t >= 22.0
    {
        unsafe{gpioOneTwiddle()};
        unsafe{gpioOneTwiddle()};
    }
    else
    {
        unsafe{gpioOneTwiddle()};
        unsafe{gpioOneTwiddle()};
        unsafe{gpioOneTwiddle()};
    }

}

fn sendData(data:&Tuple) -> ()
{
    //unsafe{ delay(30000)};
    //mimic the delay... function doesn't compile properly with clang  
    let mut i:u16 = 0;
    while i < 3000 {
	i+=1;
    }
}


#[entry]
#[no_mangle]
fn main() -> ! {

    //delete_all_pg();
    //delete_pg(0x0803_0000 as u32); 
    // Get the peripheral access
    let dp  = Peripherals::take().unwrap();
    
     //enable HSI
    dp.RCC.cr.write(|w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().bit_is_clear() {}
 
     //configure PLL
     // Step 1: Disable the PLL by setting PLLON to 0
     dp.RCC.cr.modify(|_r, w| w.pllon().clear_bit());
 
     // Step 2: Wait until PLLRDY is cleared
     while dp.RCC.cr.read().pllrdy().bit_is_set() {}
 
     // Step 3: Change the desired parameter
     // For example, modify PLL multiplier (PLLMUL)
 
     dp.RCC.cfgr.modify(|_, w| w.pllsrc().hsi_div_prediv());
 
     // Set PLL Prediv to /1
     dp.RCC.cfgr2.modify(|_, w| w.prediv().div1());
 
     // Set PLL MUL to x9
     dp.RCC.cfgr.modify(|_, w| w.pllmul().mul9());
 
     // Step 4: Enable the PLL again by setting PLLON to 1
    // dp.RCC.cr.modify(|_r, w| w.pllon().set_bit());
 
     dp.RCC.cr.modify(|_, w| w.pllon().on());
 
     while dp.RCC.cr.read().pllrdy().bit_is_clear(){}
 
        // Configure prescalar values for HCLK, PCLK1, and PCLK2
        dp.RCC.cfgr.modify(|_, w| {
         w.hpre().div1() // HCLK prescaler: no division
         .ppre1().div2() // PCLK1 prescaler: divide by 2
         .ppre2().div1() // PCLK2 prescaler: no division
     });
 
 
     // Enable FLASH Prefetch Buffer and set Flash Latency (required for high speed)
     // was crashing just because this was missing
     dp.FLASH.acr
         .modify(|_, w| w.prftbe().enabled().latency().ws1());
 
      // Select PLL as system clock source
      dp.RCC.cfgr.modify(|_, w| w.sw().pll());
 
      while dp.RCC.cfgr.read().sw().bits() != 0b10 {}
 
       // Wait for system clock to stabilize
       while dp.RCC.cfgr.read().sws().bits() != 0b10 {}
 
      dp.RCC.ahbenr.modify(|_, w| w.iopden().set_bit());
      dp.RCC.ahbenr.modify(|_, w| w.iopeen().set_bit());
      dp.RCC.ahbenr.modify(|_, w| w.iopfen().set_bit());
      dp.RCC.ahbenr.modify(|_, w| w.iopgen().set_bit());
      dp.RCC.ahbenr.modify(|_, w| w.iophen().set_bit());  
      dp.RCC.ahbenr.modify(|_, w| w.sramen().set_bit());  
      dp.RCC.ahbenr.modify(|_, w| w.flitfen().set_bit());  
      dp.RCC.ahbenr.modify(|_, w| w.fmcen().set_bit());  
 
 
      dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
      dp.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
 
    let mut gpiod = dp.GPIOD;
    let mut gpioe = dp.GPIOE;
    let mut gpiof = dp.GPIOF;
    let mut gpiog = dp.GPIOG;
    let mut gpioh = dp.GPIOH;

     // ph.ph0.into_af12(&mut ph.moder, &mut ph.afrl); //FMC_A0
    // ph.ph1.into_af12(&mut ph.moder, &mut ph.afrl); //FMC_A1
    // pf.pf2.into_af12(&mut pf.moder, &mut pf.afrl); //FMC_A2
    // pf.pf3.into_af12(&mut pf.moder, &mut pf.afrl); //FMC_A3
    // pf.pf4.into_af12(&mut pf.moder, &mut pf.afrl); //FMC_A4
    // pf.pf5.into_af12(&mut pf.moder, &mut pf.afrl); //FMC_A5

    gpioh.moder.modify(|_, w| {w.moder0().alternate()});
    gpioh.afrl.modify(|_, w| {  w.afrl0().af12()});
    gpioh.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());


    gpioh.moder.modify(|_, w| {w.moder1().alternate()});
    gpioh.afrl.modify(|_, w| {  w.afrl1().af12()});
    gpioh.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder2().alternate()});
    gpiof.afrl.modify(|_, w| {  w.afrl2().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder3().alternate()});
    gpiof.afrl.modify(|_, w| {  w.afrl3().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());

    gpiof.moder.modify(|_, w| {w.moder4().alternate()});
    gpiof.afrl.modify(|_, w| {  w.afrl4().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder5().alternate()});
    gpiof.afrl.modify(|_, w| {  w.afrl5().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());

        
        // pf.pf12.into_af12(&mut pf.moder, &mut pf.afrh); //FMC_A6
        // pf.pf13.into_af12(&mut pf.moder, &mut pf.afrh); //FMC_A7
        // pf.pf14.into_af12(&mut pf.moder, &mut pf.afrh); //FMC_A8
        // pf.pf15.into_af12(&mut pf.moder, &mut pf.afrh); //FMC_A9

    gpiof.moder.modify(|_, w| {w.moder12().alternate()});
    gpiof.afrh.modify(|_, w| {  w.afrh12().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr12().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder13().alternate()});
    gpiof.afrh.modify(|_, w| {  w.afrh13().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr13().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder14().alternate()});
    gpiof.afrh.modify(|_, w| {  w.afrh14().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());


    gpiof.moder.modify(|_, w| {w.moder15().alternate()});
    gpiof.afrh.modify(|_, w| {  w.afrh15().af12()});
    gpiof.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());

  // pg.pg0.into_af12(&mut pg.moder, &mut pg.afrl); //FMC_A10
    // pg.pg1.into_af12(&mut pg.moder, &mut pg.afrl); //FMC_A11
    // pg.pg2.into_af12(&mut pg.moder, &mut pg.afrl); //FMC_A12
    // pg.pg3.into_af12(&mut pg.moder, &mut pg.afrl); //FMC_A13
    // pg.pg4.into_af12(&mut pg.moder, &mut pg.afrl); //FMC_A14

    gpiog.moder.modify(|_, w| {w.moder0().alternate()});
    gpiog.afrl.modify(|_, w| {  w.afrl0().af12()});
    gpiog.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());

    
    gpiog.moder.modify(|_, w| {w.moder1().alternate()});
    gpiog.afrl.modify(|_, w| {  w.afrl1().af12()});
    gpiog.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());

    
    gpiog.moder.modify(|_, w| {w.moder2().alternate()});
    gpiog.afrl.modify(|_, w| {  w.afrl2().af12()});
    gpiog.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());

    
    gpiog.moder.modify(|_, w| {w.moder3().alternate()});
    gpiog.afrl.modify(|_, w| {  w.afrl3().af12()});
    gpiog.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());

    
    gpiog.moder.modify(|_, w| {w.moder4().alternate()});
    gpiog.afrl.modify(|_, w| {  w.afrl4().af12()});
    gpiog.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());

    gpiod.moder.modify(|_, w| {w.moder14().alternate()});
    gpiod.afrh.modify(|_, w| {  w.afrh14().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());

    gpiod.moder.modify(|_, w| {w.moder15().alternate()});
    gpiod.afrh.modify(|_, w| {  w.afrh15().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());

    gpiod.moder.modify(|_, w| {w.moder0().alternate()});
    gpiod.afrl.modify(|_, w| {  w.afrl0().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());


    gpiod.moder.modify(|_, w| {w.moder1().alternate()});
    gpiod.afrl.modify(|_, w| {  w.afrl1().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());

    gpioe.moder.modify(|_, w| {w.moder7().alternate()});
    gpioe.afrl.modify(|_, w| {  w.afrl7().af12()});
    gpioe.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());

    gpioe.moder.modify(|_, w| {w.moder8().alternate()});
    gpioe.afrh.modify(|_, w| {  w.afrh8().af12()});
    gpioe.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());

    gpioe.moder.modify(|_, w| {w.moder9().alternate()});
    gpioe.afrh.modify(|_, w| {  w.afrh9().af12()});
    gpioe.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());


    gpioe.moder.modify(|_, w| {w.moder10().alternate()});
    gpioe.afrh.modify(|_, w| {  w.afrh10().af12()});
    gpioe.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());


    gpiod.moder.modify(|_, w| {w.moder7().alternate()});
    gpiod.afrl.modify(|_, w| {  w.afrl7().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());


    gpiod.moder.modify(|_, w| {w.moder4().alternate()});
    gpiod.afrl.modify(|_, w| {  w.afrl4().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());


    gpiod.moder.modify(|_, w| {w.moder5().alternate()});
    gpiod.afrl.modify(|_, w| {  w.afrl5().af12()});
    gpiod.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
 
 
   // Configure FMC for SRAM memory(in our case F-RAM)
     unsafe{
         dp.FMC.bcr1.modify(|_, w| {
         w.mbken().set_bit(); // Enable FRAM bank 1
         w.mtyp().bits(0b00); // FRAM memory type
         w.mwid().bits(0b00); // 8-bit width
         w.bursten().clear_bit(); //disable brust access mode
         w.wren().clear_bit(); // wrap disable
         w.muxen().clear_bit(); // Non-multiplexed
         w.extmod().clear_bit(); // extended mode
         w.asyncwait().clear_bit(); //disable async wait
         w
      });
 
      /*
         Timing.AddressSetupTime = 1;
         Timing.AddressHoldTime = 1;
         Timing.DataSetupTime = 5;
         Timing.BusTurnAroundDuration = 0;
         Timing.CLKDivision = 0;
         Timing.DataLatency = 0;
         Timing.AccessMode = FMC_ACCESS_MODE_A;
    */
      dp.FMC.btr1.modify(|_,w|  {
        // Set address setup time to 1 cycle
         w.addset().bits(0x1);
         // Set data setup time to 5 cycle
         w.datast().bits(0x5);
         // address hold time
         w.addhld().bits(0x1);
         // bus turn around
         w.busturn().bits(0x0);
         // clock division
         w.clkdiv().bits(0x0);
         //data latency
         w.datlat().bits(0x0);
         //access mode
         w.accmod().bits(0x0);
 
         w
     });
 }

      //old globs
      let senseCount1:&'static mut u8 = big_nv!(SENSE1:u8 = 0);
      let senseCount2:&'static mut u8 = big_nv!(SENSE2:u8 = 0);
      let computeCount:&'static mut u8 = big_nv!(COMPUTEC:u8 = 0);
      let sendCount:&'static mut u8 = big_nv!(SENDC:u8 = 0);
  
      let moisture:&'static mut[u16;SAMPLE_SIZE] = big_nv!(MOISTURE:[u16;SAMPLE_SIZE] = [0,0,0,0,0]);
      let temperature:&'static mut[f32;SAMPLE_SIZE] = big_nv!(TEMPERATURE:[f32;SAMPLE_SIZE] = [0.0,0.0,0.0,0.0,0.0]);
      let moist:&'static mut u16 = big_nv!(MOIST:u16 = 0);
      let temp:&'static mut f32 = big_nv!(TEMP:f32 = 0.0);
      let moistTempAvg:&'static mut Tuple = big_nv!(AVG:Tuple = Tuple{m:0, t:0.0});
      //end old
  
   loop {
     //platformInit();
        //if(P8IN & 0x02)
        for _i in 0..40
        {
            // checkpoint
        //Inferred region for Consistent set 1 starts here
	    let init =  unsafe {adcConfig()};
	    Consistent(init,1);
	    *moist = unsafe {adcSample(_i)};
        *senseCount1+=1;

        let init2 = unsafe{tempConfig()};
	    Consistent(init2, 1);
	    *temp = unsafe{tempDegC(_i)};
        //inferred region ends here, even though 
        //there is another declaration in the set yet to come, on line 183
        *senseCount2+=1;

        // checkpoint
	    storeData(*moist, *temp, moisture, temperature);

        // checkpoint
	    let moistTempAvgLocal = calcAvg(moisture, temperature);
	    Consistent(&moistTempAvgLocal, 1);
        //note that this declaration does not need to be in the region, 
        //since no lines of code from 174 to here sample inputs
	    
	    moistTempAvg.m = moistTempAvgLocal.m;
            moistTempAvg.t = moistTempAvgLocal.t;
            /*unsafe {
		output_guard_start();
                printf(b"Moisture: %l Moist avg.:%l Temp: %l MoistTempAvg: %l overflow %l %l %l \n\r\0".as_ptr(),
                       *moist as u32, moistTempAvg.m as u32, *temp as f64, moistTempAvg.t as f64, 0, 0, 0);
		output_guard_end();
            }*/
            compute(moistTempAvg);
            *computeCount+=1;

            // checkpoint
	    
            sendData(moistTempAvg);
            *sendCount+=1;

            // checkpoint
	    //delay(5);
        }
	unsafe{
	    /*output_guard_start();
	    unsafe{printf(b"clear buff\r\n\0".as_ptr());}
            unsafe {
		printf(b"Counts: SenseOne:%l SenseTwo: %l Compute: %l Send: %l\n\r\0".as_ptr(),
                       *senseCount1 as u32, *senseCount2 as u32, *computeCount as u32, *sendCount as u32);
            }
	    output_guard_end();*/
	    unsafe{gpioTwiddle()};
        unsafe{gpioTwiddle()};
	}
	*senseCount1 = 0;
	*senseCount2 = 0;
	*computeCount = 0;
	*sendCount = 0;
        //while 1==1{};
    }
}
