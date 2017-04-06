#include "Gost28147.h"

void Gost28147_encode(uint8_t* blockin, uint8_t* blockout, uint32_t* Key, uint8_t** SubTable)
{
	// Задаем ключ шифрования и таблицу замен

	unsigned long n1 = 0, n2 = 0, SUM232 = 0; // накопители N1, N2, и сумматор

	n1 = *((unsigned long *)&blockin[0]);
	n2 = *((unsigned long *)&blockin[4]);

	// 32 цикла простой замены
	// ключ считываем в требуемом ГОСТом порядке
	int c = 0;
	for (int k = 0; k<32; k++)
	{
		if (k == 24)	
			c = 7;

		// суммируем в сумматоре СМ1
		SUM232 = Key[c] + n1;

		// заменяем по таблице замен
		uint8_t first_uint8_t = 0, second_uint8_t = 0, zam_symbol = 0;
		int n = 7;
		for (int q = 3; q >= 0; q--)
		{
			zam_symbol = *((uint8_t *)&SUM232 + q);
			first_uint8_t = (zam_symbol & 0xF0) >> 4;
			second_uint8_t = (zam_symbol & 0x0F);
			first_uint8_t = SubTable[n][first_uint8_t];
			n--;
			second_uint8_t = SubTable[n][second_uint8_t];
			n--;
			zam_symbol = (first_uint8_t << 4) | second_uint8_t;
			*((uint8_t *)&SUM232 + q) = zam_symbol;
		}

		SUM232 = (SUM232 << 11) | (SUM232 >> 21); // циклический сдвиг на 11
		SUM232 = n2^SUM232; // складываем в сумматоре СМ2

		if (k<31)
		{
			n2 = n1;
			n1 = SUM232;
		}

		if (k<24)
		{
			c++;
			if (c>7) c = 0;
		}
		else
		{
			c--;
			if (c<0) c = 7;
		}
	}
	n2 = SUM232;

	// записываем результат
	int ind = 0;
	for (int q = 0; q <= 3; q++)
		blockout[ind++] = *((uint8_t *)&n1 + q);

	for (int q = 0; q <= 3; q++)
		blockout[ind++] = *((uint8_t *)&n2 + q);
}

void Gost28147_decode(uint8_t* blockin, uint8_t* blockout, uint32_t* Key, uint8_t** SubTable)
{
	///< Задаем ключ шифрования и таблицу замен

	unsigned long n1 = 0, n2 = 0, SUM232 = 0; // накопители N1, N2, и сумматор

	n1 = *((unsigned long *)&blockin[0]);
	n2 = *((unsigned long *)&blockin[4]);

	// 32 цикла простой замены
	// ключ считываем в требуемом ГОСТом порядке
	int c = 0;
	for (int k = 0; k<32; k++)
	{
		if (k == 8) c = 7;

		// суммируем в сумматоре СМ1
		SUM232 = Key[c] + n1;

		// заменяем по таблице замен
		uint8_t first_uint8_t = 0, second_uint8_t = 0, zam_symbol = 0;
		int n = 7;
		for (int q = 3; q >= 0; q--)
		{
			zam_symbol = *((uint8_t *)&SUM232 + q);
			first_uint8_t = (zam_symbol & 0xF0) >> 4;
			second_uint8_t = (zam_symbol & 0x0F);
			first_uint8_t = SubTable[n][first_uint8_t];
			n--;
			second_uint8_t = SubTable[n][second_uint8_t];
			n--;
			zam_symbol = (first_uint8_t << 4) | second_uint8_t;
			*((uint8_t *)&SUM232 + q) = zam_symbol;
		}

		SUM232 = (SUM232 << 11) | (SUM232 >> 21); // циклический сдвиг на 11
		SUM232 = n2^SUM232; // складываем в сумматоре СМ2

		if (k<31)
		{
			n2 = n1;
			n1 = SUM232;
		}

		if (k<8)
		{
			c++;
			if (c>7) c = 0;
		}
		else
		{
			c--;
			if (c<0) c = 7;
		}
	}
	n2 = SUM232;

	// записываем результат
	int ind = 0;
	for (int q = 0; q <= 3; q++)
		blockout[ind++] = *((uint8_t *)&n1 + q);

	for (int q = 0; q <= 3; q++)
		blockout[ind++] = *((uint8_t *)&n2 + q);
}