/*
	MS5611.h - Library for accessing MS5611 sensors via I2C
	Copyright (c) 2016 Roman Schmitz

	This file is part of arduino-MS5xxx.

	arduino-MS5xxx is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	arduino-MS5xxx is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with arduino-MS5xxx.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "MS5611.h"

MS5611::MS5611(TwoWire *aWire) : MS5xxx(aWire)
{
	setI2Caddr(I2C_MS5611);
	// ReadProm();
	// Readout_5611();
}

void MS5611::Readout_5611()
{

	D2 = read_adc(MS5xxx_CMD_ADC_D2 + MS5xxx_CMD_ADC_256);
	D1 = read_adc(MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_256);

	// calculate 1st order pressure and temperature (MS5611 1st order algorithm)
	dT = D2 - C[5] * pow(2, 8);
	OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7);
	SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8);
	TEMP = (2000 + (dT * C[6]) / pow(2, 23));
	P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));

	// perform higher order corrections

	if (TEMP < 2000)
	{
		T2 = dT * dT / pow(2, 31);
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 1);
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 2);
		if (TEMP < -1500)
		{
			OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 += 11 * (TEMP + 1500) * (TEMP + 1500) / pow(2, 1);
		}
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;
	P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));
}

double MS5611::get_P_5611()
{
	unsigned long value = 0;
	unsigned long c = 0;
	send_cmd(MS5xxx_CMD_ADC_READ); // read out values
	_Wire->requestFrom(i2caddr, 3);
	c = _Wire->read();
	value = (c << 16);
	c = _Wire->read();
	value += (c << 8);
	c = _Wire->read();
	value += c;
	_Wire->endTransmission(0);
	D1 = value;
	return D1;
}

double MS5611::get_T_5611()
{
	unsigned long value = 0;
	unsigned long c = 0;
	send_cmd(MS5xxx_CMD_ADC_READ); // read out values
	_Wire->requestFrom(i2caddr, 3);
	c = _Wire->read();
	value = (c << 16);
	c = _Wire->read();
	value += (c << 8);
	c = _Wire->read();
	value += c;
	_Wire->endTransmission(0);
	D2 = value;
	return D2;
}

double MS5611::calcu_P_5611()
{
	// calculate 1st order pressure and temperature (MS5611 1st order algorithm)
	dT = D2 - C[5] * pow(2, 8);
	OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7);
	SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8);
	TEMP = (2000 + (dT * C[6]) / pow(2, 23));
	P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));

	// perform higher order corrections

	if (TEMP < 2000)
	{
		T2 = dT * dT / pow(2, 31);
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 1);
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 2);
		if (TEMP < -1500)
		{
			OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 += 11 * (TEMP + 1500) * (TEMP + 1500) / pow(2, 1);
		}
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;
	P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));
	return P;
}