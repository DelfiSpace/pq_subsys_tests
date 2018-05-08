/*
 * Copyright (c) 2016 by Stefano Speretta <s.speretta@tudelft.nl>
 *
 * INA226: a library to provide high level APIs to interface with the
 * TI INA226 current sensor. It is possible to use this library in
 * Energia (the Arduino port for MSP microcontrollers) or in other
 * toolchains.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License
 * version 3, both as published by the Free Software Foundation.
 *
 */

#ifndef __INA226_H__
#define __INA226_H__


#include <stdint.h>
#include <stdbool.h>

typedef int dev_id;


typedef enum
{
    INA226_AVERAGES_1             = 0b000,
    INA226_AVERAGES_4             = 0b001,
    INA226_AVERAGES_16            = 0b010,
    INA226_AVERAGES_64            = 0b011,
    INA226_AVERAGES_128           = 0b100,
    INA226_AVERAGES_256           = 0b101,
    INA226_AVERAGES_512           = 0b110,
    INA226_AVERAGES_1024          = 0b111
} ina226_averages_t;

typedef enum
{
    INA226_BUS_CONV_TIME_140US    = 0b000,
    INA226_BUS_CONV_TIME_204US    = 0b001,
    INA226_BUS_CONV_TIME_332US    = 0b010,
    INA226_BUS_CONV_TIME_588US    = 0b011,
    INA226_BUS_CONV_TIME_1100US   = 0b100,
    INA226_BUS_CONV_TIME_2116US   = 0b101,
    INA226_BUS_CONV_TIME_4156US   = 0b110,
    INA226_BUS_CONV_TIME_8244US   = 0b111
} ina226_busConvTime_t;

typedef enum
{
    INA226_SHUNT_CONV_TIME_140US   = 0b000,
    INA226_SHUNT_CONV_TIME_204US   = 0b001,
    INA226_SHUNT_CONV_TIME_332US   = 0b010,
    INA226_SHUNT_CONV_TIME_588US   = 0b011,
    INA226_SHUNT_CONV_TIME_1100US  = 0b100,
    INA226_SHUNT_CONV_TIME_2116US  = 0b101,
    INA226_SHUNT_CONV_TIME_4156US  = 0b110,
    INA226_SHUNT_CONV_TIME_8244US  = 0b111
} ina226_shuntConvTime_t;

typedef enum
{
    INA226_MODE_POWER_DOWN      = 0b000,
    INA226_MODE_SHUNT_TRIG      = 0b001,
    INA226_MODE_BUS_TRIG        = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA226_MODE_ADC_OFF         = 0b100,
    INA226_MODE_SHUNT_CONT      = 0b101,
    INA226_MODE_BUS_CONT        = 0b110,
    INA226_MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;


bool INA226_configure2(const dev_id id, ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode);

bool INA226_calibrate2(const dev_id id, float rShuntValue, float iMaxExpected);

float INA226_readBusPower2(const dev_id id);
float INA226_readShuntCurrent2(const dev_id id);
float INA226_readShuntVoltage2(const dev_id id);
float INA226_readBusVoltage2(const dev_id id);

uint16_t INA226_getConfig(const dev_id id);

uint16_t INA226_getCalibration(const dev_id id);

void ina_reset(dev_id id);

bool ina_readDeviceID(dev_id id);

void ina_setShuntResistor(dev_id id, double shunt);

uint16_t ina_getVoltage(dev_id id);

int16_t ina_getShuntVoltage(dev_id id);

int16_t ina_getCurrent(dev_id id);

uint16_t ina_getPower(dev_id id);

#endif // __INA226_H__
