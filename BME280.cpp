#include <stdio.h>
#include "BME280.h"

struct bme280_calib_data
{
 	unsigned short dig_T1;
	signed short   dig_T2;
	signed short   dig_T3;

	unsigned short dig_P1;
	signed short   dig_P2;
	signed short   dig_P3;
	signed short   dig_P4;
	signed short   dig_P5;
	signed short   dig_P6;
	signed short   dig_P7;
	signed short   dig_P8;
	signed short   dig_P9;

	unsigned char  dig_H1;
	signed short   dig_H2;
	unsigned char  dig_H3;
	signed short   dig_H4;
	signed short   dig_H5;
	char           dig_H6;

	int            t_fine;
};

static double compensate_h(int uncomp_h, const struct bme280_calib_data *calib_data)
{
	double humidity;
	double humidity_min = 0.0;
	double humidity_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = ((double)calib_data->t_fine) - 76800.0;
	var2 = (((double)calib_data->dig_H4) * 64.0 + (((double)calib_data->dig_H5) / 16384.0) * var1);
	var3 = uncomp_h - var2;
	var4 = ((double)calib_data->dig_H2) / 65536.0;
	var5 = (1.0 + (((double)calib_data->dig_H3) / 67108864.0) * var1);
	var6 = (1.0 + (((double)calib_data->dig_H6) / 67108864.0) * var1 * var5);
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)calib_data->dig_H1) * var6 / 524288.0);

 	if (humidity > humidity_max)
		humidity = humidity_max;
	else if (humidity < humidity_min)
		humidity = humidity_min;

	return humidity;
}


static double compensate_t(int uncomp_t, struct bme280_calib_data *calib_data)
{
	double var1;
	double var2;
	double temperature;
	double temperature_min = -40;
	double temperature_max = 85;

	var1 = ((double)uncomp_t) / 16384.0 - ((double)calib_data->dig_T1) / 1024.0;
	var1 = var1 * ((double)calib_data->dig_T2);
	var2 = (((double)uncomp_t) / 131072.0 - ((double)calib_data->dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data->dig_T3);
	calib_data->t_fine = (int)(var1 + var2);
	temperature        =      (var1 + var2) / 5120.0;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

static double compensate_p(int uncomp_p, const struct bme280_calib_data *calib_data)
{
	double var1;
	double var2;
	double var3;
	double pressure;
	double pressure_min = 30000.0;
	double pressure_max = 110000.0;

	var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data->dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data->dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data->dig_P4) * 65536.0);
	var3 = ((double)calib_data->dig_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_P1);
	/* avoid exception caused by division by zero */
	if (var1) {
		pressure = 1048576.0 - (double) uncomp_p;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calib_data->dig_P9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((double)calib_data->dig_P8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((double)calib_data->dig_P7)) / 16.0;

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else { /* Invalid case */
		pressure = pressure_min;
	}

	return pressure;
}

int bme280 (unsigned char * spReg, unsigned char * spCal, double &T, double &P, double &H)
{
struct bme280_calib_data calib_data;

unsigned int uncomp_t;
unsigned int uncomp_h;
unsigned int uncomp_p;

unsigned int data_msb;
unsigned int data_lsb;
unsigned int data_xlsb;

data_msb  = (unsigned int) spReg[0] << 12;     // p msb  0xf7
data_lsb  = (unsigned int) spReg[1] << 4;      // p lsb  0xf8
data_xlsb = (unsigned int) spReg[2] >> 4;      // p xlsb 0xf9

uncomp_p  = data_msb | data_lsb | data_xlsb;


data_msb  = (unsigned int) spReg[3] << 12;     // t msb  0xfa
data_lsb  = (unsigned int) spReg[4] << 4;      // t lsb  0xfb
data_xlsb = (unsigned int) spReg[5] >> 4;      // t xlsb 0xfc
uncomp_t  = data_msb | data_lsb | data_xlsb;

data_msb  = (unsigned int) spReg[6] << 8;      // p msb  0xfd
data_lsb  = (unsigned int) spReg[7];           // p lsb  0xfe

uncomp_h  = data_msb | data_lsb;

// Calibration data in spCal[]:
// 2D 6F 35 68 32 00 BF 8F A4 D6 D0 0B 56 1E 00 00
// F9 FF AC 26 0A D8 BD 10 00 4B F7 00 00 00 00 00
// 00 00 00 00 33 00 00 C0 00 54 00 00 00 00 60 02
// 00 01 FF FF 1F 4E 08 00 00 40 4B FF 00 00 00 00
// 02 00 00 00 00 00 00 00 60 00 00 00 00 00 00 00
// 00 00 00 00 00 00 00 00 00 64 01 00 14 2E 03 1E

calib_data.dig_T1 =                (spCal[1]  << 8) + spCal[0];		// 0x88 (lsb) & 0x89 (msb) unsigned short - spCal[0]  & spCal[1]  - 0x6f2d
calib_data.dig_T2 =                (spCal[3]  << 8) + spCal[2];		// 0x8a (lsb) & 0x8b (msb) signed short   - spCal[3]  & spCal[2]  - 0x6835
calib_data.dig_T3 =                (spCal[5]  << 8) + spCal[4];		// 0x8c (lsb) & 0x8d (msb) signed short   - spCal[5]  & spCal[4]  - 0x0032
calib_data.dig_P1 =                (spCal[7]  << 8) + spCal[6];		// 0x8e (lsb) & 0x8f (msb) unsigned short - spCal[7]  & spCal[6]  - 0x8fbf
calib_data.dig_P2 = (signed short) (spCal[9]  << 8) + spCal[8];  	// 0x90 (lsb) & 0x91 (msb) signed short   - spCal[9]  & spCal[8]  - 0xd6a4
calib_data.dig_P3 =                (spCal[11] << 8) + spCal[10];    // 0x92 (lsb) & 0x93 (msb) signed short   - spCal[11] & spCal[10] - 0x0bd0
calib_data.dig_P4 =                (spCal[13] << 8) + spCal[12];	// 0x94 (lsb) & 0x95 (msb) signed short   - spCal[13] & spCal[12] - 0x1e56
calib_data.dig_P5 =                (spCal[15] << 8) + spCal[14];	// 0x96 (lsb) & 0x97 (msb) signed short   - spCal[15] & spCal[14] - 0x0000
calib_data.dig_P6 = (signed short) (spCal[17] << 8) + spCal[16];	// 0x98 (lsb) & 0x99 (msb) signed short   - spCal[17] & spCal[16] - 0xfff9
calib_data.dig_P7 =                (spCal[19] << 8) + spCal[18];    // 0x9a (lsb) & 0x9b (msb) signed short   - spCal[19] & spCal[18] - 0x26ac
calib_data.dig_P8 = (signed short) (spCal[21] << 8) + spCal[20];    // 0x9c (lsb) & 0x9d (msb) signed short   - spCal[21] & spCal[20] - 0xd80a
calib_data.dig_P9 =                (spCal[23] << 8) + spCal[22];    // 0x9e (lsb) & 0x9f (msb) signed short   - spCal[23] & spCal[22] - 0x10bd
calib_data.dig_H1 =                 spCal[25];           			// 0xa1                    unsigned char  - spCal[25]             - 0x4b
calib_data.dig_H2 =                (spCal[90] << 8) + spCal[89];    // 0xe1 (lsb) & 0xe2 (msb) unsigned short - spCal[90] & spCal[89] - 0x0164
calib_data.dig_H3 =                 spCal[91];           			// 0xe3                    unsigned char  - spCal[91]             - 0x00

short dig_H4_msb = (short)(char)    spCal[92] * 16;    	// value = 0x14
short dig_H4_lsb = (short)         (spCal[93] & 0x0F); 	// value = 0x2e
calib_data.dig_H4 = dig_H4_msb | dig_H4_lsb;						// 0xe4 / 0xe5[3:0]  [11:4]/ [3:0] signed short - 14:2e:03 -> 014e - 0x014e

short dig_H5_msb = (short)(char)    spCal[94] * 16;		// value = 0x03
short dig_H5_lsb = (short)         (spCal[93] >> 4);	// value = 0x2e
calib_data.dig_H5 = dig_H5_msb | dig_H5_lsb;						// 0xe5[7:4]/0xe6     [3:0]/[11:4] signed short   14:2e:03 -> 0032 - 0x0032

calib_data.dig_H6 =                 spCal[95];          			// 0xe7                    signed char    - spCal[95]              - 0x1e

T = compensate_t(uncomp_t, &calib_data);
P = compensate_p(uncomp_p, &calib_data) / 100;
H = compensate_h(uncomp_h, &calib_data);

return 0;
}

int bme280(CBME280 & cS)
{
   return (bme280(cS.spData, cS.spCal, cS.T, cS.P, cS.H));
}
