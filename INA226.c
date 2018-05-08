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

#include "INA226.h"
#include <stdio.h>
#include <math.h>


/**
 *
 *   Returns the value of the selected internal register
 *
 *
 *   Parameters:
 *   unsigned char reg     register number
 *
 *   Returns:
 *   unsigned short        register value
 *
 */
uint8_t ina_readRegister(const dev_id id, const uint8_t reg, uint16_t *res)
{
    uint8_t rx_buf[3] = { reg, 0, 0 };

    HAL_I2C_readWrite(id, rx_buf, 1, res, 2);

    uint16_t temp;
    temp = *res;
    *res = ((0x00ff & temp) << 8);
    *res |= (temp >> 8);

    return 0;
}

/**
 *
 *   Sets the value of the selected internal register
 *
 *
 *   Parameters:
 *   unsigned char reg     register number
 *   unsigned short        register value
 *
 */
void ina_writeRegister(const dev_id id, const uint8_t reg, const uint16_t val)
{
    uint8_t tx_buf[3] = { reg, ((val >> 8) & 0xFF), (val & 0xFF) };

    HAL_I2C_readWrite(id, tx_buf, 3, NULL, 0);

}


#define CONFIGURATION    0x00
#define SHUNT            0x01
#define VOLTAGE          0x02
#define POWER            0x03
#define CURRENT          0x04
#define CALIBRATION      0x05
#define ID               0xFF

#define DEVICE_ID        0x2260
#define RESET            0x8000
#define CALIBRATION_REF  40.96f


#define INA226_REG_CONFIG           (0x00)
#define INA226_REG_SHUNTVOLTAGE     (0x01)
#define INA226_REG_BUSVOLTAGE       (0x02)
#define INA226_REG_POWER            (0x03)
#define INA226_REG_CURRENT          (0x04)
#define INA226_REG_CALIBRATION      (0x05)
#define INA226_REG_MASKENABLE       (0x06)
#define INA226_REG_ALERTLIMIT       (0x07)

#define INA226_BIT_SOL              (0x8000)
#define INA226_BIT_SUL              (0x4000)
#define INA226_BIT_BOL              (0x2000)
#define INA226_BIT_BUL              (0x1000)
#define INA226_BIT_POL              (0x0800)
#define INA226_BIT_CNVR             (0x0400)
#define INA226_BIT_AFF              (0x0010)
#define INA226_BIT_CVRF             (0x0008)
#define INA226_BIT_OVF              (0x0004)
#define INA226_BIT_APOL             (0x0002)
#define INA226_BIT_LEN              (0x0001)

static float vShuntMax, vBusMax, rShunt;
float currentLSB, powerLSB;

uint16_t INA226_getConfig(const dev_id id)
{
    uint16_t value;
    ina_readRegister(id, INA226_REG_CONFIG, &value);

    return value;
}

uint16_t INA226_getCalibration(const dev_id id)
{
    uint16_t value;
    ina_readRegister(id, INA226_REG_CALIBRATION, &value);

    return value;
}

bool INA226_configure2(const dev_id id, ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36;
    vShuntMax = 0.08192f;

    ina_writeRegister(id, INA226_REG_CONFIG, config);

    return true;
}

bool INA226_calibrate2(const dev_id id, float rShuntValue, float iMaxExpected)
{
    uint16_t calibrationValue;
    rShunt = rShuntValue;

    float iMaxPossible, minimumLSB;

    iMaxPossible = vShuntMax / rShunt;

    minimumLSB = iMaxExpected / 32767;

    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = ceil(currentLSB);
    currentLSB *= 0.0001;

    powerLSB = currentLSB * 25;

    calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

    ina_writeRegister(id, INA226_REG_CALIBRATION, calibrationValue);

    return true;
}

float INA226_readBusPower2(const dev_id id)
{
    int16_t power;

    ina_readRegister(id, INA226_REG_POWER, &power);

    return (power * powerLSB);
}

float INA226_readShuntCurrent2(const dev_id id)
{
    int16_t current;

    ina_readRegister(id, INA226_REG_CURRENT, &current);

    return (current * currentLSB);
}

float INA226_readShuntVoltage2(const dev_id id)
{
    int16_t voltage;

    ina_readRegister(id, INA226_REG_SHUNTVOLTAGE, &voltage);

    return (voltage * 0.0000025);
}

float INA226_readBusVoltage2(const dev_id id)
{
    uint16_t voltage;

    ina_readRegister(id, INA226_REG_BUSVOLTAGE, &voltage);

    return (voltage * 0.00125);
}

/**
 *
 *   Reset the INA226
 *
 */
void ina_reset(dev_id id)
{
   ina_writeRegister(id, CONFIGURATION, RESET);
}

/**
 *
 *   Verify if INA226 is present
 *
 *   Returns:
 *   true                  INA226 is present
 *   false                 otherwise
 *
 */
bool ina_readDeviceID(dev_id id)
{
    uint16_t res = 0;
    uint8_t reg = ID;

    ina_readRegister(id, &reg, &res);

    return res == DEVICE_ID;
}

/**
 *
 *   Sets the shunt resistor value in Ohm
 *
 *   Parameters:
 *   double shunt          shunt resistor value in Ohm
 *
 */
void ina_setShuntResistor(dev_id id, double shunt)
{
    ina_writeRegister(id, CALIBRATION, (uint16_t)(CALIBRATION_REF / shunt));
}

/**
 *
 *   Returns the bus voltage in mV
 *
 *   Returns:
 *   unsigned short        bus voltage in mV
 *
 */
uint16_t ina_getVoltage(dev_id id)
{
    uint16_t res = 0;
    ina_readRegister(id, VOLTAGE, &res);
    return res + (res >> 2);
}

/**
 *
 *   Returns the voltage across the shunt resistor
 *
 *   Returns:
 *   signed short        bus voltage (LSB = 2.5 uV)
 *
 */
int16_t ina_getShuntVoltage(dev_id id)
{
    uint16_t res = 0;
    ina_readRegister(id, SHUNT, &res);
    return res;
}

/**
 *
 *   Returns the current through the shunt resistor
 *
 *   Returns:
 *   signed short         current in mA
 *
 */
int16_t ina_getCurrent(dev_id id)
{
    uint16_t res = 0;
    ina_readRegister(id, CURRENT, &res);
    return (res >> 3);
}

/**
 *
 *   Returns the power across the load in mW
 *
 *   Returns:
 *   unsigned short        power in mW
 *
 */
uint16_t ina_getPower(dev_id id)
{
    uint16_t res = 0;
    ina_readRegister(id, POWER, &res);
    return (res * 3) + (res >> 3);
}
