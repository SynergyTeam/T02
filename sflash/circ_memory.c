/*
 * circ_memory.c
 *
 *  Created on: 17.04.2014
 *      Author: a_cherepanov
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driverlib/debug.h"

#include "ext_memory.h"
#include "circ_memory.h"

/*------------------------------------------------------------------------------
 * ������ ����� ������ � ������ ������ ���������� ������
 * � ������ Adr � ����� *data �������� len ����
 * ��������� ����� ������ ��������� �� ��������� ������ ������
 */
uint32_t read_circ_memory(circ_buf *wnd, char *buf, uint32_t adr, int len) {
	uint16_t rlen;
	ASSERT(len > 0 && len < 0x1000);
	do {
		if ((adr + len) > wnd->end) {
			rlen = (adr + len) - (wnd->end + 1);
			rlen = len - rlen;
		} else
			rlen = len;
		flash_read((uint8_t*)buf, adr, rlen);
		buf += rlen;
		adr += rlen;
		if ((len -= rlen) > 0)
			adr = wnd->start;
	} while (len > 0);
	return adr;
}

/*------------------------------------------------------------------------------
 * ������ ����� ������ � ������ ������ ���������� ������
 * � ������ Adr �� ������ *data �������� len ����
 * ��������� ����� ������ ��������� �� ���������� ������ ������
 */
uint32_t write_circ_memory(circ_buf *wnd, char *buf, uint32_t adr, int len) {
	uint16_t rlen;
	ASSERT(len > 0 && len < 0x1000);
	do {
		if ((adr + len) > wnd->end) {
			rlen = (adr + len) - (wnd->end + 1);
			rlen = len - rlen;
		} else
			rlen = len;
		flash_write((uint8_t*)buf, adr, rlen);
		buf += rlen;
		adr += rlen;
		if ((len -= rlen) > 0)
			adr = wnd->start;
	} while (len > 0);
	return adr;
}

/*------------------------------------------------------------------------------
 * ���������� ��������� ������ ���������� ������ wnd � ������ �������� delta
 */
uint32_t shift_in_circ_buf(circ_buf *wnd, uint32_t adr, int delta) {
	adr += delta;
	if(adr > wnd->end) {
		adr = wnd->start + (adr - (wnd->end + 1));
	}
	else {
		if(adr < wnd->start)
			adr = wnd->start;
	}
	return adr;
}

/*------------------------------------------------------------------------------
 * �������� ������ �� ������� ������� � ������ addr � �� ����� ����� (max 4��)
 * ���� ���� �� ����� ���������� ������ ����� (������ FLASH_BLOCK_SIZE), ��� ���������� 0
 */
uint32_t check_cln_circ_buf (circ_buf *wnd, uint32_t addr) {
	uint32_t len, blen, cnt;
	uint8_t *mBuf;

	if(!(addr & FLASH_BLOCK_MASK))
		return (0);
	len = (addr & ~FLASH_BLOCK_MASK) + FLASH_BLOCK_SIZE;
	len = (len - 1) - addr;
    blen = 0x200;
    mBuf = malloc(blen);
	while (len > 0) {
		if(len < blen) blen = len;
		flash_read(mBuf, addr, blen);
		for(cnt = 0; cnt < blen; ++cnt)
			if(mBuf[cnt] != 0xFF) {
				len -= cnt;
                free(mBuf);
				return (FLASH_BLOCK_MASK - len);
			}
		addr += blen;
		len -= blen;
	}
    free(mBuf);
	return (0);
}
