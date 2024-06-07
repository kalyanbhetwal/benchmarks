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

//fn sqrt16(val:u32) -> u16;
pub fn gpioTwiddle() -> (){}

pub static IO_NAME:fn(u16) -> kPa = readPressure;
pub static IO_NAME2:fn(u16) -> u8 = readAccel;
pub static IO_NAME3:fn(u16, &u16) -> u16 = readTemp;

const PRESSURE_WINDOW_SIZE:usize = 5;

// Times to loop in one experiment
const LOOP_IDX:u16 = 264;

const ABS_PRESS_COEF:u16 = 2;

const BURST_THRESHOLD:u16 = 25;
//based on Nissan Rogue numbers, ideal is 33 psi/228~ kpa
const LOW_PRESSURE_WARNING:u16 = 200;
//sea level pressure, rounded down to int
const ATMOS:u16 = 101;


type pressureReading = u16;
type kPa = u16;
type pressureWindow = [pressureReading; PRESSURE_WINDOW_SIZE as usize];

struct History {
    meanPress: kPa,
    recentPressure: pressureWindow,
    lastPressure:pressureReading,
    lastDiff: kPa
}

//accel Types, needed to get if the vehicle is stationary or moving
const ACCEL_WINDOW_SIZE:usize = 3;
struct threeAxis{
    x:u8,
    y:u8,
    z:u8,
}
type accelReading = threeAxis;
type accelWindow = [accelReading; ACCEL_WINDOW_SIZE as usize];
const SAMPLE_NOISE_FLOOR:u8 = 10; // TODO: made up value


/* Globals */

/*simulated input input Functions*/
//pressure is always between atomspheric and 450
#[no_mangle]
fn readPressure(input:u16) -> kPa 
{
    let res = if input % 450 > ATMOS {input % 450} else {input % 450 + ATMOS};
    return res;
}

//simulated input function
#[no_mangle]
fn readAccel(input:u16) -> u8 
{
    return (input % 85) as u8;
}

//simulated input  function
#[no_mangle]
fn readTemp(sensor:u16,input:&u16) -> u16 
{
    match sensor {
        1 => /*Tire Temp*/ (32+(*input % 68) + (*input % 200))%255, 
        _ => /*Ambient Temp*/ 32+(*input % 68), 
    } 
}


/*Sampling and Adjusting Functions*/
/*get sample and adjust to relative pressure*/
fn relativePressure(seed:&u16) -> pressureReading
{
    let sample = readPressure(seed*17 - seed%17);
    return sample;
}

//based on the equation on page 12 here: 
//https://crashstats.nhtsa.dot.gov/Api/Public/ViewPublication/811681
fn coldPressure(input:pressureReading, seed:& mut u16) -> pressureReading
{
    let tempTire = readTemp(1, seed);
    let tempAmbient = readTemp(2, seed); 
    *seed+=1;
    let result = input - ((tempTire - tempAmbient)/10);
    
    result
}

/*Based on Activity Recognition*/
fn accelSample(seed:&u16) -> threeAxis
{
    //Inferred region for consistent set five starts here
    let xs = readAccel(seed*8);
    Consistent(xs,5);
    let ys = readAccel(seed*8*8);
    Consistent(ys,5);
    let zs = readAccel(seed*8*8*8);
    Consistent(zs,5);
    //Inferred region for consistent set five ends here
    let result: threeAxis = threeAxis{x:xs, y:ys, z:zs};
    return result;
}

fn acquireWindow(seed:& mut u16) -> accelWindow
{
    //accelReading sample;
    let mut window:accelWindow= [accelReading{x:0,y:0, z:0},
                 accelReading{x:0,y:0, z:0},accelReading{x:0,y:0, z:0}];
   for i in 0..ACCEL_WINDOW_SIZE {
        let mut sample: accelReading = accelSample(seed);
        sample.x = if sample.x > SAMPLE_NOISE_FLOOR {sample.x} else {0};
        sample.y = if sample.y > SAMPLE_NOISE_FLOOR {sample.y} else {0};
        sample.z = if sample.z > SAMPLE_NOISE_FLOOR {sample.z} else {0};
        window[i] = sample;
        //only increase it sometimes, so we get a mixture of stat. and mov.
        if *seed%5 == 0 {
            *seed+=1;
        }

    }
    return window;
}

fn isMoving(aWin:&accelWindow) -> bool
{
    //check if the readings are the same, with some margin of error
    let mut x = aWin[0].x;
    let mut y = aWin[0].y;
    let mut z = aWin[0].z;
    let mut likelyMoving = 0;
    let mut likelyStopped = 0;
    for i in 1..ACCEL_WINDOW_SIZE {
        if x <= aWin[i].x + SAMPLE_NOISE_FLOOR {
            likelyStopped +=1
        } else {
            likelyMoving+=1;
        }
        x = aWin[i].x;

        if y <= aWin[i].y + SAMPLE_NOISE_FLOOR {
            likelyStopped +=1
        } else {
            likelyMoving+=1;
        }
        y = aWin[i].y;

        if z <= aWin[i].z + SAMPLE_NOISE_FLOOR {
            likelyStopped +=1
        } else {
            likelyMoving+=1;
        }
        z = aWin[i].z;
    }

    if likelyMoving > likelyStopped {
        true
    } else {
        false
    }
}

//Now pressure functions
fn updateHistorical(data:&mut History, newReading:kPa)
{
    let mut mean = 0;
    data.lastDiff = if data.lastPressure > newReading 
    { data.lastPressure - newReading} else {0};
    data.lastPressure = newReading;

    //shift the recent readings window
    for i in 1..PRESSURE_WINDOW_SIZE {
        data.recentPressure[i] = data.recentPressure[i - 1];
        mean += data.recentPressure[i - 1];
    }
    data.recentPressure[0] = newReading;
    mean += newReading;
    mean /= PRESSURE_WINDOW_SIZE as u16;
    data.meanPress = mean;
}

fn sendData(data:&str) -> ()
{
    //unsafe{ delay(30000)};
    //manually guard output
    //unsafe{output_guard_start();}
    //unsafe{printf(data.as_bytes().as_ptr());}
    unsafe{hprintln!("{:?}",data.as_bytes().as_ptr());}
    //unsafe{output_guard_end();}
    //println!("{}",data);
}

fn end_of_benchmark(urgent:&u16, medium:&u16) -> ()
{
    unsafe {
   // output_guard_start();
    //manually guard output
    // printf(b"Urgent: %l Medium: %l\n\0".as_ptr(),
    // *urgent as u32, *medium as u32);
    hprintln!("Urgent: {} Medium: {}",
    *urgent as u32, *medium as u32);
    //output_guard_end();
    }
    //println!("This is the end of the Tire benchmark {}  {}\n\0", *urgent, *medium);
}

fn initRecord(rec:&mut History, seed:& mut u16){
    let mut mean = 0;
    

    //shift the recent readings window
    for i in 0..PRESSURE_WINDOW_SIZE {
        let reading = relativePressure(seed);
        let adjusted = coldPressure(reading, seed);
        rec.recentPressure[i] = adjusted;
        mean += adjusted;
    }
    rec.lastDiff = if rec.recentPressure[1] > rec.recentPressure[0] 
    {rec.recentPressure[1] - rec.recentPressure[0]} else {0};
    rec.lastPressure = rec.recentPressure[0];
    mean /= PRESSURE_WINDOW_SIZE as u16;
    rec.meanPress = mean;

}

#[entry]
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
   
    let warningCount: &'static mut u16 = big_nv!(COUNT_NV: u16 = 0);
    let mediumWarningCount: &'static mut u16 = big_nv!(MWC_NV: u16 = 0);
    let urgentWarningCount: &'static mut u16 = big_nv!(UWC_NV: u16 = 0);
    let record: &'static mut History =
	big_nv!(HIST_NV:History = History{meanPress:0, recentPressure:[0;PRESSURE_WINDOW_SIZE],
        lastPressure:0, lastDiff:0});
    //nv!(model, model_t);
     let nv_seed: &'static mut u16 = big_nv!(SEED_NV: u16 = 1);
    
    //init();
    
    //count = 1;
    loop {
	*urgentWarningCount = 0;
	*mediumWarningCount = 0;
	*warningCount = 0;
	*nv_seed = 0;

        initRecord(record, nv_seed);
        
        //arbitrary idx for experiment length
        for i in 0..LOOP_IDX {
            //first get the accel data
            
            let window:accelWindow  = acquireWindow(nv_seed);
            if isMoving(&window) == false {
                //only sample every third time if the vehicle is stopped
                if i%3 != 0 {
                    continue;
                }

            } 
            //Inferred region for Fresh var 'reading' starts here
            let reading = relativePressure(nv_seed);
            Fresh(reading);
            let adjusted = coldPressure(reading, nv_seed);
            //Inferred region for var 'reading' ends here

            updateHistorical(record, adjusted);
            //possible burst tire, because of a sudden large decrease
            if record.lastDiff > BURST_THRESHOLD {
            //get a series of fresh and consistent readings, to check that 
            //it's a burst tire, not just, eg. a cold day
                //Outer-most inferred region for the fresh and consistent annotations
                // at lines 299 and 301 begins here
                let reading0 = relativePressure(nv_seed);

		        let reading1 = relativePressure(nv_seed);

		        let sumDiff = if reading0 < record.lastDiff 
                {record.lastDiff - reading0} else {0} + 
                if reading1 < reading0 {reading0 - reading1} else {0};

                let avgDiff = sumDiff/2;
                FreshConsistent(avgDiff,1);
                let currMotion:accelWindow = acquireWindow(nv_seed); 
                FreshConsistent(&currMotion,1);
                //car is moving and the pressure is actively decreasing
                if isMoving(&currMotion)  {
                    if avgDiff > 0 {
                    sendData("urgent_burst_tire!\r\n\0");
                    *urgentWarningCount +=1;
                    }
                    
                } 
                //Outermost Inferred region for the Fresh and consistent set 1 ends here
                //looking at the LLVM-IR, there will also be two inner inferred region, 
                //which were generated by inferrence for the freshness constraints

            }

            //otherwise, just check for low pressure
            else if record.meanPress < LOW_PRESSURE_WARNING {
                //send a warning 
                *warningCount += 1;
                if *warningCount > 1000 {
                    // if it's already been warned a lot, send a more urgent warning
                    sendData("urgent_low_pressure\r\n\0");
                    *urgentWarningCount +=1;
                } else {
                    sendData("medium_low_pressure\r\n\0");
                    *mediumWarningCount +=1;
                }
            }

            //otoh, if the diff is 0 (pressure has increased)
            //reinit the window, as the tire has gotten refilled
            //and the old data isn't valid anymore
            if record.lastDiff == 0 {
                initRecord(record, nv_seed);
                *warningCount = 0;
            } 

            
        }
        //end of benchmark
        end_of_benchmark(urgentWarningCount, mediumWarningCount);
        unsafe{
		gpioTwiddle();
        gpioTwiddle();
        }
	    
	}
}
