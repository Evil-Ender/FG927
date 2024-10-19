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

#ifndef MS5611_h
#define MS5611_h

#include "MS5xxx.h"
class MS5611 : public MS5xxx
{
protected:
  unsigned long D1 = 0, D2 = 0;
  double dT;
  double OFF;
  double SENS;
  double T2 = 0., OFF2 = 0., SENS2 = 0.;

public:
  MS5611(TwoWire *aWire = &Wire);
  virtual void Readout_5611();
  double get_P_5611();
  double get_T_5611();
  double calcu_P_5611();
};

#endif
