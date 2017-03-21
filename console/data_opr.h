#ifndef __DATA_OP_H__
#define __DATA_OP_H__

//определения
#define EOS                         0x00

//функции
extern char *SearchByte(char *Index, char Byte, int32_t MaxLen, char FieldNum);
extern uint8_t *SearchData(uint8_t *d1, int32_t l1, uint8_t *d2, uint16_t l2);
extern char *SearchStr(char *s1, int32_t l1, char *s2);

extern void StrToBCD(char* dest, char bp, char ap, char* source);
extern char *BCDToStr(char *Index, char Value);
extern void IntToBCD(char *buf, char aChr, int Vlm);
extern int BCDToInt(char *Index, char Len);

#endif
