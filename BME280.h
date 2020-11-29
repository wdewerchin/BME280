/* BME280.h - Created on: 3 Oct 2017 - Author: wd  */

#ifndef BME280_H_
#define BME280_H_

class CBME280
{
public:
   double T, P, H;
   unsigned char spCal[100];
   bool          bCal = false;
   unsigned char spData[8];
   bool          bData = false;
};

int bme280 (unsigned char * spReg, unsigned char * spCal, double &T, double &P, double &H);
int bme280 (CBME280 &);

#endif /* BME280_H_ */
