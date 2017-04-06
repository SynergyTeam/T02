#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "utils/data_opr.h"
#include "utils/ustdlib.h"

//-------------------------------------------------------------------------------
// поиск N-го (Num) байта (Byte) в массиве d1 размером Len
// возвращает указатель на следующий байт если байт найден
// 0x0000 - нет данных
char *SearchByte(char *d1, char byte, int32_t len, char num) {
	if (num > 0) {
		while (len > 0) {
			if (byte == *d1++) {
				if (!--num) return d1;
			}
			--len;
		}
	}
	return (0);
}
/*------------------------------------------------------------------------------
 * ѕоиск последовательности длинной l2 из буфера d2 в буфере d1 размером l1
 * возвращает 0 если данных нет, или указатель на начало данных
 */
uint8_t *SearchData(uint8_t *d1, int32_t l1, uint8_t *d2, uint16_t l2) {
	while (l1 > 0) {
		if (*d1 == *d2) {
			uint16_t cl2 = l2;
			uint8_t *cd2 = d2;
			while ((l1 > 0) && (cl2 > 0)) {
				if (*d1 != *cd2) break;
				++d1;
				--l1;
				++cd2;
				--cl2;
			}
			if (!cl2) return (d1 - l2);
		} else {
			++d1;
			--l1;
		}
	}
	return (0);
}

/*----------------------------------------------------------------------------*/
/* поиск строки s2 в буфере s1, размером l1 */
/* возвращает 0 если данных нет, или указатель на начало строки */
char *SearchStr(char *s1, int32_t l1, char *s2) {
	if(l1 <= 0) return 0;
	return ((char*)SearchData((uint8_t*)s1, l1, (uint8_t*)s2, ustrlen(s2)));
}

//------------------------------------------------------------------------------
//преобразование текстового числа в BCD
//bp - счетчик цифр до точки, ap - счетчик цифр после точки
//source - источник, dest - приемник
//в случае ошибки приемник не обновл€етс€
void StrToBCD(char* dest, char bp, char ap, char* source) {
	if (source) { //поле найдено
        uint8_t len, tlen, dig[10], dg;
		tlen = len = bp + ap;
		if (ap) ap = 1;
		for (dg = 0; dg < 10 && dg < len;) {									//цикл преобразовани€
			char byte = *source++;
			if (byte == '.') {
				if (ap) {
					--ap;
					len -= bp;
					bp = 0;
					continue;
				}
			}
			byte = byte ^ '0';
			if (byte > 9) {
				byte = 0;
				--source;
			}
			dig[dg++] = byte;													//код
			if (bp) --bp;
		}
		if (tlen & 1) ++tlen;
		tlen >>= 1;
		dest += tlen;
		for (bp = 0, ap = 0; tlen > 0;) {										//преобразование
			if (len > 0) bp |= (dig[--len] << 4);
			if (ap++ & 1) {
				*--dest = bp;
				bp = 0;
				--tlen;
			} else bp = (bp >> 4) & 0x0F;
		}
	}
}

//------------------------------------------------------------------------------
//перевод BCD значени€ в ASCII строку
char *BCDToStr(char *Index, char Value) {
    *Index++ = 0x30 + (Value >> 4);
    *Index++ = 0x30 + (Value & 0x0F);
    return Index;
}

//------------------------------------------------------------------------------
//ѕреобразование Int числа в BCD. ѕреобразование начинаетс€ с младшего разр€да Vlm!
//buf - начало буфера, aChr размер буфера (байт)
void IntToBCD(char *buf, char aChr, int Vlm) {
	if(Vlm < 0) Vlm = -Vlm;
	while (aChr > 0) {
		--aChr;
		buf[(uint8_t)aChr] = Vlm % 10;
		Vlm = Vlm / 10;
		buf[(uint8_t)aChr] |= (Vlm % 10) << 4;
		Vlm = Vlm / 10;
	}
}

//------------------------------------------------------------------------------
//перевод BCD значени€ в целое
int BCDToInt(char *str, char Len) {
	int Tmp, Value = 0x00;
	while (Len--) {
		Tmp = Value = Value << 1;
		Value = (Value << 2) + Tmp;
		Value += (*str >> 4);
		Tmp = Value = Value << 1;
		Value = (Value << 2) + Tmp;
		Value += (*str++ & 0x0F);
	}
	return Value;
}

//------------------------------------------------------------------------------
//рассчет контрольной суммы по алгоритму CRC8
uint8_t CRCx08(uint8_t *data, int32_t len) {
    uint8_t bt, byte, crc = 0x00;
    while (len > 0) {
        byte = *data++;
        --len;
        for (bt = 0; bt < 8; bt++) {
            if ((byte ^ crc) & 0x01) crc = ((crc ^ 0x18) >> 1) | 0x80;
            else crc >>= 1;
            byte >>= 1;
        }
    }
    return crc;
}

//------------------------------------------------------------------------------
//вычисление контрольной суммы по полиному 1021
uint16_t CRCx1021(uint8_t *data, int32_t len) {
    return (block1021(0xFFFF, (uint8_t*)data, len));
}

/*------------------------------------------------------------------------------
 * ѕодсчет контрольной суммы блока данных
 * CRC - начальное значение, data - указатель на буфер, len - размер
 */
uint16_t block1021(uint16_t prvCRC, uint8_t *data, int32_t len) {
    while (len > 0) {
        uint8_t bt, Byte = *data++;
        for (bt = 0; bt < 8; bt++) {
            if (((Byte << (bt + 8)) ^ prvCRC) & 0x8000) prvCRC = (prvCRC << 1) ^ 0x1021;
            else prvCRC <<= 1;
        }
        --len;
    }
    return prvCRC;
}
/*------------------------------------------------------------------------------
 * ¬ычисление контрольной суммы дл€ системы измерени€ давлени€ в шинах
 * PRESSURE PRO
 */
uint8_t CRC_PRESS(uint8_t *data, int32_t len) {
    uint8_t result = 0;
    uint8_t i;
    for (i=0; i < len; i++) {
        result += *data++;
    }
    result = (~(result & 0xff) + 1) & 0xFF;
    return result;
}
/*------------------------------------------------------------------------------
 * CRC16 for MODBUS RTU protocol
*/
uint16_t CRC_MODBUS(char *data, int32_t len) {
    uint16_t result,tresult;
    uint8_t i,j;
    result = 0xFFFF;

    for (i = 0; i < len; i++) {
        result = result ^ data[i];
        for (j = 0; j < 8; j++) {
            tresult = result;
            result = result >> 1;
            if (tresult & 0x0001) {
                result = result ^ 0xA001;
            }
        }
    }
    return result;
}
