#include <HTU21.h>

void HTU21_Reset(){
	uint8_t Data=HTU21_RESET;
	HAL_Delay(15);
	HAL_I2C_Master_Transmit(&hi2c1, HTU21_I2CADDR, &Data, 1, 1000);
	HAL_Delay(15);
}

char HTU21_Init(){
	uint8_t check;
	uint8_t Data=HTU21_READREG;

	HAL_I2C_Master_Transmit(&hi2c1, HTU21_I2CADDR, &Data, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, HTU21_I2CADDR, &check, 1, 1000);
	return check;
}

float HTU21_GetTemp()
{
	uint8_t Temp_Data[2];
	uint8_t Data=HTU21_READTEMP;

	HAL_I2C_Master_Transmit(&hi2c1, HTU21_I2CADDR, &Data, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, HTU21_I2CADDR, Temp_Data, 2, 1000);

	uint16_t Raw_Temp =(uint16_t)(Temp_Data[0] << 8 | (Temp_Data[1]& 0b11111100));

	float temp = Raw_Temp;
	temp *= 175.72f;
	temp /= 65536.0f;
	temp -= 46.85f;
	return temp;
}

float HTU21_GetHum()
{
	uint8_t Hum_Data[2];
	uint8_t Data=HTU21_READHUM;

	HAL_I2C_Master_Transmit(&hi2c1, HTU21_I2CADDR, &Data, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, HTU21_I2CADDR, Hum_Data, 2, 1000);

	uint16_t Raw_Hum =(uint16_t)(Hum_Data[0] << 8 | (Hum_Data[1]& 0b11110000));

	float hum = Raw_Hum;
	hum *= 175.72f;
	hum /= 65536.0f;
	hum -= 46.85f;
	return hum;



}


