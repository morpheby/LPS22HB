/**
 ******************************************************************************
 * @file    LPS22HBSensor.cpp
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Implementation of a LPS22HB Pressure sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "Arduino.h"
#include "Wire.h"
#include "LPS22HBSensor.h"


/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LPS22HBSensor::LPS22HBSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  isEnabled = 0;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LPS22HBSensor::LPS22HBSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  dev_i2c = NULL;
  address = 0;
  isEnabled = 0;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::begin(void)
{
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); 
  }

  if ( LPS22HB_Set_PowerMode( (void *)this, LPS22HB_LowNoise) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)this, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if( LPS22HB_Set_LowPassFilter( (void *)this, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Set low-pass filter cutoff configuration*/
  if( LPS22HB_Set_LowPassFilterCutoff( (void *)this, LPS22HB_ODR_9) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Set block data update mode */
  if ( LPS22HB_Set_Bdu( (void *)this, LPS22HB_BDU_NO_UPDATE ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Disable automatic increment for multi-byte read/write */
  if( LPS22HB_Set_AutomaticIncrementRegAddress( (void *)this, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  isEnabled = 0;
  Last_ODR = 25.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::end(void)
{
  /* Disable pressure and temperature sensor */
  if (Disable() != LPS22HB_STATUS_OK)
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Reset CS configuration */
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, INPUT); 
  }

  return LPS22HB_STATUS_OK;
}


/**
 * @brief  Enable LPS22HB
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Enable(void)
{
  /* Check if the component is already enabled */
  if ( isEnabled == 1 )
  {
    return LPS22HB_STATUS_OK;
  }

  if(SetODR_When_Enabled(Last_ODR) == LPS22HB_STATUS_ERROR)
  {
    return LPS22HB_STATUS_ERROR;
  }

  isEnabled = 1;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Disable LPS22HB
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Disable(void)
{
  /* Check if the component is already disabled */
  if ( isEnabled == 0 )
  {
    return LPS22HB_STATUS_OK;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)this, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  isEnabled = 0;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read ID address of LPS22HB
 * @param  ht_id the pointer where the ID of the device is stored
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::ReadID(uint8_t *p_id)
{
  if(!p_id)
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Read WHO AM I register */
  if ( LPS22HB_Get_DeviceID( (void *)this, p_id ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Reboot memory content of LPS22HB
 * @param  None
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Reset(void)
{
  /* Read WHO AM I register */
  if ( LPS22HB_MemoryBoot((void *)this) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register, and calculate the pressure in mbar
 * @param  pfData the pressure value in hPa
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetPressure(float* pfData)
{
  int32_t int32data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_RawPressure( (void *)this, &int32data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  *pfData = ( float )int32data / 4096.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Get LPS22HB sensitivity in LSB/hPa
 * @param  pfData the sensitivity value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetPressureSensitivity(int16_t* pfData)
{
  pfData[0] = 4096;
  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register
 * @param  pfData the pressure value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetPressureRaw(int32_t* pfData)
{
  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_RawPressure( (void *)this, pfData ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register, and calculate the temperature
 * @param  pfData the temperature value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetTemperature(float *pfData)
{
  int16_t int16data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_RawTemperature( (void *)this, &int16data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  *pfData = ( float )int16data / 100.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Get LPS22HB sensitivity in LSB/degC
 * @param  pfData the sensitivity value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetTemperatureSensitivity(int16_t* pfData)
{
  pfData[0] = 100;
  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register
 * @param  pfData the temperature value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetTemperatureRaw(int16_t *pfData)
{
  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_RawTemperature( (void *)this, pfData ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output data rate
 * @param  odr the pointer to the output data rate
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetODR(float* odr)
{
  LPS22HB_Odr_et odr_low_level;

  if ( LPS22HB_Get_Odr( (void *)this, &odr_low_level ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case LPS22HB_ODR_ONE_SHOT:
      *odr = 0.0f;
      break;
    case LPS22HB_ODR_1HZ:
      *odr = 1.0f;
      break;
    case LPS22HB_ODR_10HZ:
      *odr = 10.0f;
      break;
    case LPS22HB_ODR_25HZ:
      *odr = 25.0f;
      break;
    case LPS22HB_ODR_50HZ:
      *odr = 50.0f;
      break;
    case LPS22HB_ODR_75HZ:
      *odr = 75.0f;
      break;
    default:
      *odr = -1.0f;
      return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output data rate
 * @param  odr the pointer to the output data rate
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetODRRaw(LPS22HB_Odr_et* odr)
{
  if ( LPS22HB_Get_Odr( (void *)this, odr ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Set ODR
 * @param  odr the output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR(float odr)
{
  if(isEnabled == 1)
  {
    if(SetODR_When_Enabled(odr) == LPS22HB_STATUS_ERROR)
    {
      return LPS22HB_STATUS_ERROR;
    }
  }
  else
  {
    if(SetODR_When_Disabled(odr) == LPS22HB_STATUS_ERROR)
    {
      return LPS22HB_STATUS_ERROR;
    }
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Set ODR
 * @param  odr the output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODRRaw(LPS22HB_Odr_et odr)
{
  if ( LPS22HB_Set_Odr( (void *)this, odr ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  if (odr != LPS22HB_ODR_ONE_SHOT) {
    isEnabled = 1;
  } else {
    isEnabled = false;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param odr the functional output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR_When_Enabled( float odr )
{
  LPS22HB_Odr_et new_odr;

  new_odr = ( odr <=  1.0f ) ? LPS22HB_ODR_1HZ
          : ( odr <= 10.0f ) ? LPS22HB_ODR_10HZ
          : ( odr <= 25.0f ) ? LPS22HB_ODR_25HZ
          : ( odr <= 50.0f ) ? LPS22HB_ODR_50HZ
          :                    LPS22HB_ODR_75HZ;

  if ( LPS22HB_Set_Odr( (void *)this, new_odr ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  if ( GetODR( &Last_ODR ) == LPS22HB_STATUS_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param odr the functional output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR_When_Disabled( float odr )
{
  Last_ODR = ( odr <=  1.0f ) ? 1.0f
           : ( odr <= 10.0f ) ? 10.0f
           : ( odr <= 25.0f ) ? 25.0f
           : ( odr <= 50.0f ) ? 50.0f
           :                    75.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB filter state
 * @param  state the pointer to the output state
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetFilter(LPS22HB_State_et* state)
{
  if ( LPS22HB_Get_LowPassFilter( (void *)this, state ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Set LPS22HB filter state
 * @param  state Filter state
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetFilter(LPS22HB_State_et state)
{
  if ( LPS22HB_Set_LowPassFilter( (void *)this, state ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB filter cutoff value
 * @param  state the pointer to the output cutoff
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetFilterCutoff(LPS22HB_LPF_Cutoff_et* cutoff)
{
  if ( LPS22HB_Get_LowPassFilterCutoff( (void *)this, cutoff ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Set LPS22HB filter cutoff value
 * @param  state Filter cutoff
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetFilterCutoff(LPS22HB_LPF_Cutoff_et cutoff)
{
  if ( LPS22HB_Set_LowPassFilterCutoff( (void *)this, cutoff ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::ReadReg( uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_ReadReg( (void *)this, reg, 1, data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::WriteReg( uint8_t reg, uint8_t data )
{

  if ( LPS22HB_WriteReg( (void *)this, reg, 1, &data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}


uint8_t LPS22HB_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  return ((LPS22HBSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

uint8_t LPS22HB_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return ((LPS22HBSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
