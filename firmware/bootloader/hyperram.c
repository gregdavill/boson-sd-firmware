#include <stdio.h>
#include <stdint.h>

#include "hyperram.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/git.h>



#ifdef CSR_HYPERRAM_BASE

/* Prototypes */

void set_io_delay(int);
void set_clk_delay(int);
static int basic_memtest(void);


void set_io_delay(int cnt){
	hyperram_io_loadn_write(0);
	hyperram_io_loadn_write(1);
	hyperram_io_direction_write(0);

	/* 25ps of delay per tap.
	   Each rising edge adds to the io delay */
	for(int i = 0; i < cnt; i++){ 
		hyperram_io_move_write(1);
		hyperram_io_move_write(0);
	}
}

void set_clk_delay(int cnt){
	hyperram_clk_loadn_write(0);
	hyperram_clk_loadn_write(1);
	hyperram_clk_direction_write(0);

	/* 25ps of delay per tap.
	   Each rising edge adds to the io delay */
	for(int i = 0; i < cnt; i++){ 
		hyperram_clk_move_write(1);
		hyperram_clk_move_write(0);
	}
}



/* 
	Test memory location by writing a value and attempting read-back.
	Try twice to avoid situation where memory is read-only and set from a previous test.
*/
static int basic_memtest(void){

	*((volatile uint32_t*)HYPERRAM_BASE) = 0xFF55AACD;
	if(*((volatile uint32_t*)HYPERRAM_BASE) != 0xFF55AACD)
		return 0;
//

	flush_cpu_dcache();
flush_cpu_icache();
	*((volatile uint32_t*)HYPERRAM_BASE) = 0xA3112233;
	flush_cpu_dcache();
	if(*((volatile uint32_t*)HYPERRAM_BASE) != 0xA3112233)
		return 0;
	
	return 1;
}


void hyperram_init(void){
	int window = 0;
	int clk_del = 0;
	int io_del = 0;

	while(clk_del < 128){
		set_clk_delay(clk_del >> 2);
		set_io_delay(io_del);
		int i = 0;
		for(i = 0; i < 64; i++){

			int pass = basic_memtest();

			// Shift our PLL
			crg_phase_sel_write(0);
			crg_phase_dir_write(0);
			crg_phase_step_write(0);
			crg_phase_step_write(1);

			if(pass == 1){
				window++;
			}
			else if(pass != 1){
				if(window >= 6){
					break;
				}else {
					window = 0;
				}
			}

		}
		if(window >= 5){
			for(i = 0; i < window/2; i++){
				// Shift our PLL up
				crg_phase_sel_write(0);
				crg_phase_dir_write(1);
				crg_phase_step_write(0);
				crg_phase_step_write(1);
			}

			log_printf("Hyperram: PLL phase tuned: window=%u", window);


			return;
		}
		window = 0;
		clk_del = (clk_del + 1);

		crg_slip_hr2x90_write(clk_del & 1 ? 1 : 0);
		crg_slip_hr2x_write(clk_del & 2 ? 1 : 0);

		crg_slip_hr2x90_write(0);
		crg_slip_hr2x_write(0);
	}

	log_printf("Hyperram: Error: RAM Init failed, restarting");
	busy_wait(100);
	
	while(1){
		reset_out_write(1);
	}
	
}

#else


void hyperram_init(){
	return;
}

#endif
