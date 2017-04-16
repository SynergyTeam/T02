#include <stdlib.h>
#include <stdio.h>
#include "Gost28147.h"

long filesize(FILE *stream);

using namespace std;

int main(int argc, char** argv)
{
	if (argc != 4)
	{
		printf("Usage:\n\tgost-28147-89.exe <mode> <path to source file> <path to target file>\n\tmode = 1 - encoding\n\tmode = 2 - decoding\nInput any key to continue...");
		getchar();
		return EXIT_FAILURE;
	}

	uint32_t* Key = (uint32_t*)malloc(8 * sizeof(uint32_t));
	Key[0] = 0x0123;
	Key[1] = 0x4567;
	Key[2] = 0x89AB;
	Key[3] = 0xCDEF;
	Key[4] = 0x0123;
	Key[5] = 0x4567;
	Key[6] = 0x89AB;
	Key[7] = 0xCDEF;

	uint8_t** SubTable;
	SubTable = (uint8_t**)malloc(8 * sizeof(uint8_t*));
	for (int i = 0; i < 8; i++)
	{
		SubTable[i] = (uint8_t*)malloc(16 * sizeof(uint8_t));
		for (int j = 0; j < 16; j++)
			SubTable[i][j] = j;
	}

	// режим, 1 - зашифрование, 2 - расшифрование
	int mode;
	sscanf(argv[1], "%d", &mode);

	FILE *f_in, *f_out; // потоки для входного и выходного файлов

	// открываем файлы
	f_in = fopen(argv[2], "rb");
	f_out = fopen(argv[3], "wb");

	// считываем входные данные
	long inputsize = filesize(f_in);

	// Определяем размер входные данных
	float blokoff;
	blokoff = (float)8 * inputsize;
	blokoff = blokoff / 64;
	int blockN = (int)blokoff;
	if (blokoff - blockN>0) blockN++;

	int sh;
	if (inputsize >= 4) sh = 4; else sh = inputsize;
	int sh1 = 0;
	int flag = 0;

	uint8_t blockin[9];
	uint8_t blockout[9];
	blockin[8] = '\0';
	blockout[8] = '\0';

	for (int i = 0; i < blockN; i++)
	{
		for (int q = 0; q < 8; q++)
		{
			*((uint8_t *)&blockin + q) = 0x00;
			*((uint8_t *)&blockout + q) = 0x00;
		}

		if ((sh1 + sh) < inputsize)
		{
			fread(&blockin[0], sh, 1, f_in);
			sh1 += sh;
		}
		else
		{
			sh = inputsize - sh1;
			fread(&blockin[0], sh, 1, f_in);
			flag = 1;
		}

		if ((sh1 + sh) < inputsize)
		{
			fread(&blockin[4], sh, 1, f_in);
			sh1 += sh;
		}
		else
		{
			if (flag == 0)
			{
				sh = inputsize - sh1;
				fread(&blockin[4], sh, 1, f_in);
			}
		}

		if (mode == 1)
			Gost28147_encode(blockin, blockout, Key, SubTable);
		else if (mode == 2)
			Gost28147_decode(blockin, blockout, Key, SubTable);
		else
		{
			printf("Wrong mode. Exit. Press any key to continue...");
			getchar();
			return EXIT_FAILURE;
		}

		// Выводим результат в файл
		fwrite(&blockout[0], 8, 1, f_out);
	}

	// закрываем файлы
	fclose(f_in);
	fclose(f_out);

	return EXIT_SUCCESS;
}

long filesize(FILE * stream)
{
	long curpos, length;
	curpos = ftell(stream);
	fseek(stream, 0L, SEEK_END);
	length = ftell(stream);
	fseek(stream, curpos, SEEK_SET);
	return length;
}