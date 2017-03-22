/*
 * circ_memory.h
 *
 *  Created on: 17.04.2014
 *      Author: a_cherepanov
 */

#ifndef WIND_MEMORY_H_
#define WIND_MEMORY_H_

#include <stdint.h>

typedef struct circ_buf {
	uint32_t start;
	uint32_t end;
} circ_buf;


extern uint32_t read_circ_memory(circ_buf *wnd, char *buf, uint32_t adr, int len);
extern uint32_t write_circ_memory(circ_buf *wnd, char *buf, uint32_t adr, int len);
extern uint32_t shift_in_circ_buf(circ_buf *wnd, uint32_t adr, int delta);
extern uint32_t check_cln_circ_buf (circ_buf *wnd, uint32_t addr);

#endif /* WIND_MEMORY_H_ */
