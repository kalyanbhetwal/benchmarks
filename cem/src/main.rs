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

//GUIDE: IO op annotations as pub static function pointer
#[no_mangle]
pub static IO_NAME : fn(Letter) -> Sample =  acquire_sample;


const LOOP_IDX:u16 = 5;

const NIL:usize =  0; // like NULL, but for indexes, not real pointers

const DICT_SIZE:u16 = 512;
const BLOCK_SIZE:u16 = 64;

const NUM_LETTERS_IN_SAMPLE:u16 =2;
const LETTER_MASK:u16 = 0x00FF;
const LETTER_SIZE_BITS:usize = 8;
const NUM_LETTERS:u16 =  LETTER_MASK + 1;
type Index = usize;
type Letter = u16;
type Sample = u16;

// NOTE: can't use pointers, since need to ChSync, etc
#[derive(Clone, Copy)]
struct Node {
	letter:Letter, // 'letter' of the alphabet
	sibling:Index, // this node is a member of the parent's children list
	child:Index   // link-list of children
}
type Dict = [Node;DICT_SIZE as usize];
/*struct Dict {
	nodes:[Node;DICT_SIZE as usize],
	node_count:u16
} */

type Log = [Index;BLOCK_SIZE as usize];
/*struct Log {
	data:[Index;BLOCK_SIZE as usize],
	count:u16,
	sample_count:u16,
} */
//__attribute__((always_inline))
fn print_log(log:&Log, log_count:&u16, log_sample_count:&u16)
{
    unsafe{
	//output_guard_start();
    hprintln!("rate: samples/block: {} {}", *log_sample_count as u32, *log_count as u32);
	//printf(b"rate: samples/block: %l/%l\r\n\0".as_ptr(), *log_sample_count as u32, *log_count as u32);
	//printf(b"atomic depth: %l\r\n\0".as_ptr(), atomic_depth as u32);
	//output_guard_end();
	//	BLOCK_PRINTF("compressed block:\r\n");
	//output_guard_start();
	for i in 0..*log_count {
	    //
        hprintln!("{}", log[i as usize] as u32);
	   // printf(b"%04x \0".as_ptr(), log[i as usize] as u32);
	    //		if (i > 0 && ((i + 1) & (8 - 1)) == 0){
	    //		}
	    //		BLOCK_PRINTF("\r\n");
	    //	}
	    //	if ((log->count & (8 - 1)) != 0){
	}
	//printf(b"\r\n\0".as_ptr());
	//output_guard_end();
    }
    
    if *log_sample_count != 353 {
	unsafe{
	//output_guard_start();
	//unsafe{printf(b"print log exit tripped!\r\n\0".as_ptr())}
    hprintln!("print log exit tripped! {}").unwrap();
	//exit(0);
	//output_guard_end();
	}
    }
    
}


fn acquire_sample(prev_sample:Letter) -> Sample
{
	//letter_t sample = rand() & 0x0F;
	let sample:Letter = (prev_sample as Sample + 1) & 0x03;
	return sample;
}

fn init_dict(dict:&mut Dict, node_count:&mut u16) -> ()
{
	
	//LOG("init dict\r\n");
    for l in 0..NUM_LETTERS {
	dict[l as usize].letter = l;
	dict[l as usize].sibling = 0;
	dict[l as usize].child = 0;
	//dict.node_count = dict.node_count + 1;
//	unsafe{printf(b"init node l %l s %l c %l count %l\r\n\0".as_ptr(), dict.nodes[l as usize].letter as u32,
//		      dict.nodes[l as usize].sibling as u32, dict.nodes[l as usize].child as u32, dict.node_count as u32);}
		
    }
    *node_count = 256;

}

fn find_child(letter:Letter, parent:Index, dict:&Dict) -> Index
{
    //atomic_start();
    let parent_node:&Node = &dict[parent];
    let mut ret:usize = NIL;
	//LOG("find child: l %u p %u c %u\r\n", letter, parent, parent_node->child);

	if parent_node.child == NIL {
		//LOG("find child: not found (no children)\r\n");
		return NIL;
	}

	let mut sibling:Index = parent_node.child;
	while sibling != NIL { //bound: temp

		let sibling_node:&Node = &dict[sibling];

		//LOG("find child: l %u, s %u l %u s %u\r\n", letter,
		//		sibling, sibling_node->letter, sibling_node->sibling);

		if sibling_node.letter == letter { // found
			//LOG("find child: found %u\r\n", sibling);
		    
		    ret = sibling;
		    break;
		} else {
			sibling = sibling_node.sibling;
		}
	}

	//LOG("find child: not found (no match)\r\n");
    //atomic_end();
    return ret;
}

fn add_node(letter:Letter, parent:Index, dict:&mut Dict, node_count:&mut u16) -> ()
{
    if *node_count == DICT_SIZE {
	
	unsafe{
        //printf(b"add node: table full\r\n\0".as_ptr());
        hprintln!("add node: table full");
    }
	return;
	
    }
    //Make a local var for the new node
    let mut node:Node = Node{letter:letter, sibling:NIL, child:NIL};
    
    let node_index:Index = *node_count as usize;
    let child:Index = dict[parent].child;
    *node_count = *node_count + 1;
    //unsafe {printf(b"adding node: i %l l %u, p: %l pc %l\r\n\0".as_ptr(),node_index as u32, letter as u32, parent as u32 as u32, child);}
    if child != 0 {
	//LOG("add node: is sibling\r\n");
	// Find the last sibling in list
	let mut sibling:Index = child;
	//let mut sibling_node:&Node = &dict[sibling];
	let mut sibling_next:Index = sibling;
	while sibling_next != NIL { //temp bound for test
	    // unsafe{printf(b"add node: sibling %u, l %u s %u\r\n\0".as_ptr(),
	    //		  sibling as u32, letter as u32, sibling_node.sibling as u32);}
	    // unsafe {gpioTwiddle();}
	    sibling = sibling_next;
	    sibling_next = dict[sibling].sibling;
	}

	// Link-in the new node
	//unsafe{printf(b"add node: last sibling %u\r\n\0".as_ptr(), sibling as u32)};
	dict[sibling].sibling = node_index;
    } else {
	//  unsafe{printf(b"add node: is only child\r\n\0".as_ptr())};
	dict[parent].child = node_index;
    }
    dict[node_index].letter = node.letter;
    dict[node_index].sibling = node.sibling;
    dict[node_index].child = node.child;

}

fn append_compressed(parent:Index, log:&mut Log, log_count:&mut u16) -> ()
{
    //LOG("append comp: p %u cnt %u\r\n", parent, log->count);
    log[*log_count as usize] = parent;
    *log_count= *log_count + 1;
 }

#[entry]
#[no_mangle]
fn main() -> ! {
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

    let mut log: &'static mut Log = big_nv!(LOGNV:Log = [0;BLOCK_SIZE as usize]);
    let mut log_count: &'static mut u16 = big_nv!(LOG_COUNTNV:u16 = 0);
    let mut log_sample_count: &'static mut u16 = big_nv!(LOG_SAMPLECOUNTNV:u16 = 0);
    

    let mut dict: &'static mut Dict = big_nv!(DICTNV:Dict = [Node{letter:0, sibling:0, child:0};DICT_SIZE as usize]);

    let mut node_count: &'static mut u16 = big_nv!(NODE_COUNTNV:u16 = 0);
    loop {
        for _cnt in 0..LOOP_IDX {
	    
            init_dict(&mut dict, &mut node_count);
            //dict.node_count = 255;
            // Initialize the pointer into the dictionary to one of the root nodes
            // Assume all streams start with a fixed prefix ('0'), to avoid having
            // to letterize this out-of-band sample.
            let mut letter:Letter = 0;
    
            
            let mut letter_idx:u16 = 0;
                let mut parent:Index = 0;
                //let mut child:Index = 0;
                let mut sample:Sample = 0;
                let mut prev_sample:Sample = 0;
    
            *log_sample_count = 1; // count the initial sample (see above)
            *log_count = 0; // init compressed counter
            while 1 == 1 { //Note: this loop breaks -- it's not infinite
            let mut  child:Index = letter as Index; // relies on initialization of dict
            //LOG("compress: parent %u\r\n", child); // naming is odd due to loop
            
            if letter_idx == 0 {
                //region for Fresh var 'sample' starts here'
                let sample = acquire_sample(prev_sample);
                Fresh(sample);
                prev_sample = sample;
                //region for Fresh sample ends here
            }
            //LOG("letter index: %u\r\n", letter_idx);
            letter_idx+=1;
            if letter_idx == NUM_LETTERS_IN_SAMPLE {
                letter_idx = 0;
            }
            //do {
            let mut letter_idx_tmp:u16 = if letter_idx == 0 { NUM_LETTERS_IN_SAMPLE } else { letter_idx - 1}; 
            
            let mut letter_shift:u16 = LETTER_SIZE_BITS as u16 * letter_idx_tmp;
            letter = if letter_shift == 16 {0} else {sample & 0xFF};
            //LOG("letterize: sample %x letter %x (%u)\r\n",
            //		sample, letter, letter);
            
            *log_sample_count+=1;
            //unsafe{printf(b"atomic depth: %l\r\n\0".as_ptr(), atomic_depth as u32);}
            parent = child;
            child = find_child(letter, parent, dict);
        
            //LOG("child: %u\r\n", child);
                    //}
            while child != NIL {
                    
                letter_idx_tmp = if letter_idx == 0 { NUM_LETTERS_IN_SAMPLE } else { letter_idx - 1}; 
                
                        letter_shift = LETTER_SIZE_BITS as u16 * letter_idx_tmp;
                letter = if letter_shift == 16 {0} else {sample & 0xFF};
                //LOG("letterize: sample %x letter %x (%u)\r\n",
                        //		sample, letter, letter);
                
                        *log_sample_count+=1;
                //unsafe{printf(b"atomic depth: %l\r\n\0".as_ptr(), atomic_depth as u32);}
                parent = child;
                        child = find_child(letter, parent, dict);
            }
            
            
            append_compressed(parent,log, log_count);
            add_node(letter, parent, dict, node_count);
            if *log_count == BLOCK_SIZE {
            
                print_log(&log, log_count, log_sample_count);
               // while(1==1){
        //		print_log(&log);
        //	    }
                *log_count = 0;
                *log_sample_count = 0;
                break;
            }
            
            }
        }
        //atomic_start();
        // unsafe {gpioTwiddle();}
        // unsafe {gpioTwiddle();}

    }
}
