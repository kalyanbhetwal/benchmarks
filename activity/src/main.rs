#![allow(unsafe_code,unused, non_upper_case_globals,non_snake_case, non_camel_case_types )]
#![no_main]
#![no_std]
use core::mem;
use core::ptr;
use cortex_m::asm::nop;
use panic_halt as _;

extern crate libm;

use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f3xx_hal_v2::{pac::Peripherals, pac::Interrupt};

use volatile::Volatile;

use cortex_m::peripheral::NVIC;
use stm32f3xx_hal_v2::interrupt;

mod checkpoint;
use checkpoint::{checkpoint, restore, delete_pg, delete_all_pg};


#[macro_use]
mod intermittent;

//use num_traits::Float;

// use num_traits::real::Real;

// Number of samples to discard before recording training set
const NUM_WARMUP_SAMPLES:u16 = 3;

const ACCEL_WINDOW_SIZE:usize = 3;
const  MODEL_SIZE:usize = 16;
const SAMPLE_NOISE_FLOOR:u8 = 10; // TODO: made up value

// Number of classifications to complete in one experiment
const SAMPLES_TO_COLLECT:u16 = 128;

/*
#define TASK_CHECKPOINT(...)

#include "ftest_util.h"
void mspconsole_init();
 */

struct threeAxis_t_8{
    x:u8,
    y:u8,
    z:u8,
}

type accelReading = threeAxis_t_8;
#[allow(non_camel_case_types)]
type accelWindow = [accelReading; ACCEL_WINDOW_SIZE as usize];

struct features_t {
    meanmag: u16,
    stddevmag: u16,
}


enum class_t {
    CLASS_STATIONARY,
    CLASS_MOVING,
}

struct model_t {
    stationary: [features_t; MODEL_SIZE as usize],
    moving: [features_t; MODEL_SIZE as usize],
}

enum run_mode_t {
    MODE_IDLE = 3,
    MODE_TRAIN_STATIONARY = 2,
    MODE_TRAIN_MOVING = 1,
    MODE_RECOGNIZE = 0, // default
}

struct stats_t {
    totalCount:u16,
    movingCount:u16,
    stationaryCount: u16,
}

/* Globals */
#[no_mangle]
pub static IO_NAME:fn(u16) ->u8 = readSensor;
//simulated input function
#[no_mangle]
fn readSensor(input:u16) -> u8 
{
    return (input % 85) as u8;
}
/*Will need more paras to account for no globals*/
fn accel_sample(nv_seed:& mut u16) -> threeAxis_t_8
{ 
    let mut seed:u16 = *nv_seed;
    let xs = readSensor(seed*17);
    let tmp:u32 = (seed as u32 * 17 * 17 *17 )% 85;

    let ys = tmp as u8;//readSensor(seed*17*17);
    let tmp:u32 = (seed as u32 * 17 * 17 *17)%85;
    //hprintln!("the tmp val for zs {}", tmp);

    let zs =  tmp as u8; //readSensor(seed*17*17*17 as u16);
    let result: threeAxis_t_8 = threeAxis_t_8{x:xs, y:ys, z:zs};
    
    seed = seed +1;
    *nv_seed = seed;
    return result;
}


//#define accel_sample ACCEL_singleSample

fn acquire_window(window:&mut accelWindow, seed:& mut u16) -> ()
{
    //accelReading sample;
    let mut samplesInWindow:usize = 0;

    //TASK_CHECKPOINT();

    while samplesInWindow < ACCEL_WINDOW_SIZE {
        let sample: accelReading = accel_sample(seed);
        //LOG("acquire: sample %u %u %u\r\n", sample.x, sample.y, sample.z);

        window[samplesInWindow] = sample;
	samplesInWindow += 1;
    }
}

fn transform(window:&mut accelWindow) -> ()
{

    //LOG("transform\r\n");

    for i in 0..ACCEL_WINDOW_SIZE {
        let sample:&mut accelReading = &mut window[i];

        if sample.x < SAMPLE_NOISE_FLOOR ||
            sample.y < SAMPLE_NOISE_FLOOR ||
            sample.z < SAMPLE_NOISE_FLOOR {

            //LOG("transform: sample %u %u %u\r\n",
            //    sample->x, sample->y, sample->z);

            (*sample).x = if sample.x > SAMPLE_NOISE_FLOOR {sample.x} else {0};
            (*sample).y = if sample.y > SAMPLE_NOISE_FLOOR {sample.y} else {0};
            (*sample).z = if sample.z > SAMPLE_NOISE_FLOOR {sample.z} else {0};
        }
    }
}

fn featurize(features:&mut features_t, aWin:&accelWindow) -> ()
{
    //TASK_CHECKPOINT();

    let mut mean = accelReading {x:0,y:0,z:0};
    let mut stddev = accelReading {x:0,y:0,z:0};
    
    
    for i in 0..ACCEL_WINDOW_SIZE {
        mean.x += aWin[i].x;  // x
        mean.y += aWin[i].y;  // y
        mean.z += aWin[i].z;  // z
    }
    
    mean.x >>= 2;
    mean.y >>= 2;
    mean.z >>= 2;

    for i in 0..ACCEL_WINDOW_SIZE {
        stddev.x += if aWin[i].x > mean.x {aWin[i].x - mean.x} else
        {mean.x - aWin[i].x};  // x
        stddev.y += if aWin[i].y > mean.y { aWin[i].y - mean.y } else {
            mean.y - aWin[i].y};  // y
        stddev.z += if aWin[i].z > mean.z { aWin[i].z - mean.z } else
            { mean.z - aWin[i].z};  // z
    }
    
    stddev.x >>= 2;
    stddev.y >>= 2;
    stddev.z >>= 2;

    let meanmag:u32 = mean.x as u32*mean.x as u32 + mean.y as u32 *mean.y as u32+
	mean.z as u32*mean.z as u32 ;
    let stddevmag:u32 = stddev.x as u32*stddev.x as u32 + stddev.y as u32*stddev.y as u32
	+ stddev.z as u32 *stddev.z as u32;

    features.meanmag   =  libm::sqrtf(meanmag as f32) as u16; //1;//unsafe{sqrt16(meanmag)};
    features.stddevmag =   libm::sqrtf(stddevmag as f32) as u16; //2;//unsafe{sqrt16(stddevmag)};

    //LOG("featurize: mean %u sd %u\r\n", features->meanmag, features->stddevmag);
}
fn classify(features:&features_t, model:&model_t) -> class_t
{
    let mut move_less_error:i16 = 0;
    let mut stat_less_error:i16 = 0;
    let mut model_features:features_t = features_t{meanmag:0, stddevmag:0};

    //TASK_CHECKPOINT();

    for i in 0..MODEL_SIZE {
        model_features.meanmag = model.stationary[i].meanmag;
	model_features.stddevmag = model.stationary[i].stddevmag;

        let stat_mean_err:i32 = if model_features.meanmag > features.meanmag
        { (model_features.meanmag  - features.meanmag) as i32} else
        { (features.meanmag - model_features.meanmag) as i32};

        let stat_sd_err:i32 = if model_features.stddevmag > features.stddevmag
        {(model_features.stddevmag - features.stddevmag) as i32}
        else {(features.stddevmag - model_features.stddevmag) as i32};

        model_features.meanmag = model.moving[i].meanmag;
	model_features.stddevmag = model.moving[i].stddevmag;

        let move_mean_err:i32 = if model_features.meanmag > features.meanmag
        {(model_features.meanmag - features.meanmag) as i32}
        else  {(features.meanmag - model_features.meanmag) as i32};

        let move_sd_err:i32 = if model_features.stddevmag > features.stddevmag
        {(model_features.stddevmag - features.stddevmag) as i32} else
        {(features.stddevmag - model_features.stddevmag) as i32};

        if move_mean_err < stat_mean_err {
            move_less_error+=1;
        } else {
            stat_less_error+=1;
        }

        if move_sd_err < stat_sd_err {
            move_less_error+=1;
        } else {
            stat_less_error+=1;
        }
    }

    let class:class_t = if move_less_error > stat_less_error
    {class_t::CLASS_MOVING} else {class_t::CLASS_STATIONARY };
    //LOG("classify: class %u\r\n", class);

    return class;
}

fn record_stats(stats:&mut stats_t, class:class_t) -> ()
{
    //TASK_CHECKPOINT();

    /* stats->totalCount, stats->movingCount, and stats->stationaryCount have an
     * nv-internal consistency requirement.  This code should be atomic. */

    stats.totalCount+=1;

    match class {
        class_t::CLASS_MOVING => stats.movingCount+=1,
            
        class_t::CLASS_STATIONARY => stats.stationaryCount+=1,
    }

/*    unsafe{printf(b"stats: s %u\0".as_ptr(),
		  stats.stationaryCount as u32);
    printf(b" m %u\0".as_ptr(),
	   stats.movingCount as u32);
    printf(b" t %u\r\n\0".as_ptr(),
		  stats.totalCount as u32);}*/
}

fn print_stats(stats:&stats_t) -> ()
{
    let resultStationaryPct = stats.stationaryCount * 100 / stats.totalCount;
    let resultMovingPct = stats.movingCount * 100 / stats.totalCount;

    let sum = stats.stationaryCount + stats.movingCount;

    unsafe {
	//if not u32, rust throws errors. 
    //manually guard output
	// output_guard_start();

    hprintln!("{}", stats.stationaryCount);
	// printf(b"stats: s %l (%lu%%) m %l (%l%%) sum/tot %l/%l: %c\r\n\0".as_ptr(),
	//        stats.stationaryCount as u32, resultStationaryPct as u32,
    //        stats.movingCount as u32, resultMovingPct as u32,
    //        stats.totalCount as u32, sum as u32,
    //            if sum == stats.totalCount && sum == SAMPLES_TO_COLLECT { 'V'} else {'X'});
	// output_guard_end();
	
    }
}


fn warmup_sensor(seed:&mut u16) -> ()
{
    let mut discardedSamplesCount:u16 = 0;
    let mut _sample:accelReading;

    //TASK_CHECKPOINT();

    //LOG("warmup\r\n");

    while discardedSamplesCount < NUM_WARMUP_SAMPLES {
	_sample = accel_sample(seed);
	discardedSamplesCount += 1;
    }
}

fn train(classModel:&mut [features_t; MODEL_SIZE], seed:& mut u16) -> ()
{
    let mut sampleWindow:accelWindow= [accelReading{x:0,y:0, z:0},
				 accelReading{x:0,y:0, z:0},accelReading{x:0,y:0, z:0}];
    let mut features:features_t = features_t{meanmag:0, stddevmag:0};
    warmup_sensor(seed);

    for i in 0..MODEL_SIZE {
        acquire_window(&mut sampleWindow, seed);
        transform(&mut sampleWindow);
        featurize(&mut features, &sampleWindow);

        //TASK_CHECKPOINT();

        classModel[i].meanmag = features.meanmag;
	classModel[i].stddevmag = features.stddevmag;
    }

//    unsafe {
//	printf(b"train: done: mn %l sd %l\r\n\0".as_ptr(), features.meanmag as u32, features.stddevmag as u32);
    //}
    
}

fn recognize(model:&model_t, seed:&mut u16) -> ()
{
    let mut stats:stats_t = stats_t{totalCount:0,movingCount:0,stationaryCount:0};
    let mut sampleWindow:accelWindow = [accelReading{x:0,y:0, z:0},
				 accelReading{x:0,y:0, z:0},accelReading{x:0,y:0, z:0}];
    let mut features:features_t = features_t{meanmag:0, stddevmag:0};
    let mut class:class_t;
    
    for _i in 0..SAMPLES_TO_COLLECT {
        acquire_window(&mut sampleWindow, seed);
        transform(&mut sampleWindow);
        featurize(&mut features, &sampleWindow);
        class = classify(&features, model);
	    record_stats(&mut stats, class);
    }

    print_stats(&stats);
}
fn end_of_benchmark() -> (){}

fn count_error(count:&u16) -> ()
{
    unsafe {
	//printf(b"An error occured during count, count = %d\n\0".as_ptr(), *count as u32);
    }
}

fn select_mode(prev_pin_state:&mut u8, count: &mut u16) -> u8 
{
    let mut pin_state:u8 = run_mode_t::MODE_IDLE as u8;

    //TASK_CHECKPOINT();

    *count = *count + 1;

    /* The InK order
     *  rounds:
     *      1,2 = MODE_TRAIN_MOVING
     *      3,4 = MODE_TRAIN_STATIONARY
     *      5,6 = MODE_RECOGNIZE
     *      7   = END OF BENCHMARK
     */
    match *count {
        1|2 => pin_state = run_mode_t::MODE_TRAIN_MOVING as u8,
        3|4 => pin_state = run_mode_t::MODE_TRAIN_STATIONARY as u8,
        5|6  => pin_state = run_mode_t::MODE_RECOGNIZE as u8,
        7 => end_of_benchmark(),
        _ => {
            pin_state = run_mode_t::MODE_IDLE as u8;
            count_error(count)
	},
    }

    //loop benchmark
    if *count == 7 {
	return run_mode_t::MODE_IDLE as u8;
    }

    //pin_state = GPIO(PORT_AUX, IN) & (BIT(PIN_AUX_1) | BIT(PIN_AUX_2));

    // Don't re-launch training after finishing training
    // Vito: could have done this while assigning pin_state. But keep is the same as the original
    if (pin_state == run_mode_t::MODE_TRAIN_STATIONARY as u8  ||
        pin_state == run_mode_t::MODE_TRAIN_MOVING as u8) &&
        pin_state == *prev_pin_state  {
        pin_state = run_mode_t::MODE_IDLE as u8;
    } else {
        *prev_pin_state = pin_state;
    }

    //LOG("selectMode: pins %04x\r\n", pin_state);

    return pin_state;
}

#[entry]
fn main() -> ! {
    //delete_all_pg();
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
    unsafe{
    let dp = Peripherals::steal(); //take().unwrap();

    // Enable the clock for GPIOA and SYSCFG
    dp.RCC.ahbenr.modify(|_, w| w.iopaen().set_bit());
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // Configure PA0 as input
    dp.GPIOA.moder.modify(|_, w| w.moder0().input());
    dp.GPIOA.pupdr.modify(|_, w| w.pupdr0().pull_up());

    dp.SYSCFG.exticr1.modify(|_, w| w.exti0().pa0());

    // Configure EXTI0 for falling edge trigger and enable it
    dp.EXTI.imr1.modify(|_, w| w.mr0().set_bit());
    dp.EXTI.ftsr1.modify(|_, w| w.tr0().set_bit());
    }
    // Enable EXTI0 interrupt in the NVIC
    unsafe { NVIC::unmask(Interrupt::EXTI0) };

    // Enable interrupts globally
    // unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0) };

    restore();

    let mut prev_pin_state:u8 = run_mode_t::MODE_IDLE as u8;
    let count: &'static mut u16 = big_nv!(COUNT_NV: u16 = 1);
    let model: &'static mut model_t =
	big_nv!(MODEL_NV:model_t = model_t{
	    moving:[features_t{meanmag:0,stddevmag:0}, features_t{meanmag:0,stddevmag:0}, features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0}],
	    stationary:[features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
		    features_t{meanmag:0,stddevmag:0}]
			       });
        
    let _v_seed: &'static mut u16 = big_nv!(SEED_NV: u16 = 1);

    
    loop {
        let mut localSeed = *_v_seed;
        let mode:u8 = select_mode(&mut prev_pin_state, count);
        if mode == 2 {
            train(&mut model.stationary, &mut localSeed);
        } else if mode == 1 {
            
            train(&mut model.moving, &mut localSeed);
        } else if mode == 0 {
            recognize(&model, &mut localSeed);
        
        } else if mode == 3 && *count == 7{
            //idle, for restarting the bench
            localSeed = 1;
            *count = 1;
            prev_pin_state = run_mode_t::MODE_IDLE as u8;
            unsafe{
            // gpioTwiddle();
            // gpioTwiddle();
            }
        }
        *_v_seed = localSeed;
    }
}

// Interrupt handler for EXTI0
#[interrupt]
fn EXTI0() {
    // Clear the interrupt pending bit
    // let lr: u32;
    // unsafe {
    //     asm!(
    //         "mov {}, lr",
    //         out(reg) lr
    //     );
    // }
    // hprintln!("LR value: {:#010x}", lr).unwrap();

    unsafe{
        let peripherals = Peripherals::steal();
        peripherals.EXTI.pr1.modify(|_, w| w.pr0().set_bit());
    }
   hprintln!("Interrupt happened").unwrap();
    checkpoint();
   //hprintln!("Checkpoint taken").unwrap();
    // let a = 10 + 2;
    // let b = a + 10;
    //reset_mcu();
    // Your interrupt handling code here
}

#[no_mangle]
fn reset_mcu() -> ! {
    // Perform a software reset
    hprintln!("reset mcu").unwrap();
    SCB::sys_reset();
}
#[no_mangle]
fn delay_nop(count: u32) {
    for _ in 0..count {
        nop();
    }
}