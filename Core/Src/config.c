/*
 * config.c
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VHEMaster
 */

#include <string.h>
#include "config.h"
#include "sst25vf032b.h"
#include "crc.h"

#define CONFIG_VERSION 0x00000004

const uint32_t flash_addresses[2] = {0x00000000, 0x00010000};

static sAcisConfig config_safe;
static sAcisConfig config_check;

HAL_StatusTypeDef config_load(sAcisConfig * config)
{
  uint32_t size = (uint32_t)&config->crc - (uint32_t)config;

  for(int region = 0; region < 2; region++)
  {
    uint8_t spistatus = SST25_Read(flash_addresses[region], sizeof(sAcisConfig), (uint8_t*)&config_check);
    if(spistatus)
    {
      uint16_t crc16 = CRC16_Generate((uint8_t*)&config_check, size);
      if(crc16 == config_check.crc && config_check.version == CONFIG_VERSION) {
        memcpy(config, &config_check, sizeof(sAcisConfig));
        memcpy(&config_safe, &config_check, sizeof(sAcisConfig));
        return HAL_OK;
      }
      if(region > 0)
        return HAL_ERROR;
    }
  }

  return HAL_BUSY;
}

static const float default_rotates[TABLE_ROTATES_MAX] = {
    600, 840, 1170, 1650, 2310, 3210, 4530, 6360,
};

static const float default_pressures[TABLE_PRESSURES_MAX] = {
    13300, 26100, 38900, 51700, 64600, 77400, 90200, 103000,
};

static const float default_ignitions[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX] = {
    { 16.1f, 20.1f, 26.6f, 38.1f, 42.6f, 43.0f, 44.3f, 44.8f },
    { 18.7f, 22.2f, 27.7f, 38.2f, 42.4f, 43.7f, 44.0f, 44.7f },
    { 17.4f, 20.9f, 26.2f, 36.3f, 40.6f, 41.9f, 42.1f, 42.9f},
    { 12.2f, 15.9f, 21.5f, 31.9f, 36.8f, 38.9f, 39.5f, 40.8f },
    { 8.5f, 10.8f, 15.3f, 23.0f, 30.0f, 35.4f, 36.7f, 37.5f },
    { 6.3f, 8.1f, 11.1f, 18.0f, 26.9f, 31.6f, 32.7f, 33.8f },
    { 6.0f, 7.1f, 9.1f, 15.8f, 24.4f, 28.7f, 28.7f, 30.7f },
    { 6.0f, 7.0f, 8.5f, 12.3f, 19.5f, 24.8f, 26.0f, 28.0f},
};


static const float default_idle_rotates[TABLE_ROTATES_MAX] = {
    417, 455, 476, 500, 525, 556, 588, 625, 667, 714, 769, 833, 909, 1000, 1111, 1250, 1429, 1667, 2000, 2500,
};

static const float default_idle_ignitions[TABLE_ROTATES_MAX] = {
    10, 11, 12, 13, 14, 16, 18, 21, 24, 27, 28, 27, 21, 13, 10, 9, 10, 12, 14, 17
};

static void setconfig_standart16l(sAcisConfig * config, uint8_t i)
{

  config->tables[i].rotates_count = 8;
  config->tables[i].pressures_count = 8;

  memcpy(config->tables[i].rotates, default_rotates, sizeof(default_rotates));
  memcpy(config->tables[i].pressures, default_pressures, sizeof(default_pressures));
  memcpy(config->tables[i].ignitions, default_ignitions, sizeof(default_ignitions));
}

HAL_StatusTypeDef config_default(sAcisConfig * config)
{
  HAL_StatusTypeDef status = HAL_OK;

  memset(config, 0, sizeof(sAcisConfig));

  config->params.isCutoffEnabled = 1;
  config->params.isTemperatureEnabled = 0;
  config->params.isEconomEnabled = 0;
  config->params.isAutostartEnabled = 0;
  config->params.isIgnitionByHall = 0;
  config->params.isForceIdle = 0;
  config->params.isHallLearningMode = 0;
  config->params.isSwitchByExternal = 0;
  config->params.isEconOutAsStrobe = 0;

  config->params.switchPos1Table = 0;
  config->params.switchPos0Table = 2;
  config->params.switchPos2Table = 1;

  config->params.EconRpmThreshold = 2000;
  config->params.CutoffRPM = 6000;
  config->params.CutoffMode = 1;
  config->params.CutoffAngle = 5;

  config->params.isForceTable = 0;
  config->params.forceTableNumber = 0;
  config->params.hallLearningTable = 3;
  config->params.isForceIgnition = 0;
  config->params.forceIgnitionAngle = 10;

  config->params.engineVolume = 1550;

  for(int i = 0; i < TABLE_SETUPS_MAX; i++)
    memset(&config->tables[i], 0, sizeof(sAcisIgnTable));

  config->tables_count = TABLE_SETUPS_MAX;
  strcpy(config->tables[0].name, "Default 1");
  strcpy(config->tables[1].name, "Default 2");
  strcpy(config->tables[2].name, "Default 3");
  strcpy(config->tables[3].name, "Default 4");

  for(int i = 0; i < 4; i++)
  {
    strcpy(config->tables[i].name, i == 0 ? "Pertol-A95" : i == 1 ? "Propane" : i == 2 ? "Empty" : "Reserved");
    config->tables[i].valve_channel = i == 0 ? ValvePetrol : i == 1 ? ValvePropane : ValveAllClosed;
    config->tables[i].valve_timeout = 0;

    config->tables[i].initial_ignition = 10.0f;
    config->tables[i].octane_corrector = 0.0f;

    config->tables[i].idles_count = 20;
    memcpy(config->tables[i].idle_rotates, default_idle_rotates, sizeof(default_idle_rotates));
    memcpy(config->tables[i].idle_ignitions, default_idle_ignitions, sizeof(default_idle_ignitions));

    setconfig_standart16l(config,i);

    config->tables[i].temperatures_count = 12;
    config->tables[i].temperatures[0] = -20.0f;
    config->tables[i].temperatures[1] = -15.0f;
    config->tables[i].temperatures[2] = -7.0f;
    config->tables[i].temperatures[3] = 0.0f;
    config->tables[i].temperatures[4] = 10.0f;
    config->tables[i].temperatures[5] = 20.0f;
    config->tables[i].temperatures[6] = 40.0f;
    config->tables[i].temperatures[7] = 55.0f;
    config->tables[i].temperatures[8] = 70.0f;
    config->tables[i].temperatures[9] = 90.0f;
    config->tables[i].temperatures[10] = 100.0f;
    config->tables[i].temperatures[11] = 110.0f;

    config->tables[i].temperature_ignitions[0] = 6.5f;
    config->tables[i].temperature_ignitions[1] = 6.0f;
    config->tables[i].temperature_ignitions[2] = 5.5f;
    config->tables[i].temperature_ignitions[3] = 5.0f;
    config->tables[i].temperature_ignitions[4] = 4.0f;
    config->tables[i].temperature_ignitions[5] = 3.0f;
    config->tables[i].temperature_ignitions[6] = 2.0f;
    config->tables[i].temperature_ignitions[7] = 1.0f;
    config->tables[i].temperature_ignitions[8] = 0.0f;
    config->tables[i].temperature_ignitions[9] = 0.0f;
    config->tables[i].temperature_ignitions[10] = -1.0f;
    config->tables[i].temperature_ignitions[11] = -5.0f;

    config->tables[i].servo_acc[0] = 0.0f;
    config->tables[i].servo_acc[1] = 0.0f;
    config->tables[i].servo_acc[2] = 0.0f;
    config->tables[i].servo_acc[3] = 0.0f;
    config->tables[i].servo_acc[4] = 0.0f;
    config->tables[i].servo_acc[5] = 0.0f;
    config->tables[i].servo_acc[6] = 0.0f;
    config->tables[i].servo_acc[7] = 0.0f;
    config->tables[i].servo_acc[8] = 0.0f;
    config->tables[i].servo_acc[9] = 0.0f;
    config->tables[i].servo_acc[10] = 0.0f;
    config->tables[i].servo_acc[11] = 0.0f;

    config->tables[i].servo_choke[0] = 0.0f;
    config->tables[i].servo_choke[1] = 0.0f;
    config->tables[i].servo_choke[2] = 0.0f;
    config->tables[i].servo_choke[3] = 0.0f;
    config->tables[i].servo_choke[4] = 0.0f;
    config->tables[i].servo_choke[5] = 0.0f;
    config->tables[i].servo_choke[6] = 0.0f;
    config->tables[i].servo_choke[7] = 0.0f;
    config->tables[i].servo_choke[8] = 0.0f;
    config->tables[i].servo_choke[9] = 0.0f;
    config->tables[i].servo_choke[10] = 0.0f;
    config->tables[i].servo_choke[11] = 0.0f;

    config->tables[i].fuel_rate = 14.7; //Stoichiometric mixture
    config->tables[i].fuel_volume = 0.75f; //kg/l (0.75 A-95, 0.5 Propane)
  }

  return status;
}

HAL_StatusTypeDef config_save(sAcisConfig * config)
{
  HAL_StatusTypeDef status = HAL_BUSY;
  static uint8_t state = 0;
  static uint32_t size = 0;
  static uint16_t crc16 = 0;
  static uint16_t sectors = 0;
  static uint16_t erased = 0;
  static uint8_t region = 0;
  uint16_t crc16_check;
  uint8_t spistatus = 0;
  do
  {
    switch (state)
    {
      case 0:
        config->version = CONFIG_VERSION;
        size = (uint32_t)&config->crc - (uint32_t)config;
        memcpy(&config_safe, config, sizeof(sAcisConfig));
        crc16 = CRC16_Generate((uint8_t*)&config_safe, size);
        config_safe.crc = crc16;
        state++;
        sectors = (sizeof(sAcisConfig) / SST25_SECTORSIZE) + ((sizeof(sAcisConfig) % SST25_SECTORSIZE > 0) ? (1) : (0));
        erased = 0;
        continue;
      case 1:
        if(erased < sectors)
        {
          spistatus = SST25_Erase4KSector(erased * SST25_SECTORSIZE + flash_addresses[region]);
          if(spistatus)
          {
            erased++;
            continue;
          }
        }
        else
        {
          state++;
          continue;
        }
        break;
      case 2:
        spistatus = SST25_Write(flash_addresses[region], sizeof(sAcisConfig), (uint8_t*)&config_safe);
        if(spistatus)
        {
          state++;
          continue;
        }
        break;
      case 3:
        spistatus = SST25_Read(flash_addresses[region], sizeof(sAcisConfig), (uint8_t*)&config_check);
        if(spistatus)
        {
          size = (uint32_t)&config_check.crc - (uint32_t)&config_check;
          crc16_check = CRC16_Generate((uint8_t*)&config_check, size);
          if(crc16_check == config_check.crc && crc16_check == crc16)
          {
            if(region == 0) {
              region++;
            }
            else {
              region = 0;
              status = HAL_OK;
            }
          }
          else status = HAL_ERROR;
          state = 0;
        }
        break;
      default:
        state = 0;
        continue;
    }
  } while(0);

  return status;
}

HAL_StatusTypeDef config_save_particial(sAcisConfig * config, void * from, uint32_t length)
{
  HAL_StatusTypeDef status = HAL_BUSY;
  static uint8_t state = 0;
  static uint32_t size = 0;
  static uint16_t crc16 = 0;
  static uint16_t sectors = 0;
  static uint16_t erased = 0;
  static uint8_t region = 0;
  uint16_t crc16_check;
  uint8_t spistatus = 0;

  if((uint32_t)config > (uint32_t)from || length > sizeof(sAcisConfig))
    return HAL_ERROR;

  do
  {
    switch (state)
    {
      case 0:
        config->version = CONFIG_VERSION;
        config_safe.version = CONFIG_VERSION;
        memcpy((void*)((uint32_t)&config_safe + (uint32_t)from - (uint32_t)config), from, length);
        size = (uint32_t)&config_safe.crc - (uint32_t)&config_safe;
        crc16 = CRC16_Generate((uint8_t*)&config_safe, size);
        config_safe.crc = crc16;
        state++;
        sectors = (sizeof(sAcisConfig) / SST25_SECTORSIZE) + ((sizeof(sAcisConfig) % SST25_SECTORSIZE > 0) ? (1) : (0));
        erased = 0;
        continue;
      case 1:
        if(erased < sectors)
        {
          spistatus = SST25_Erase4KSector(erased * SST25_SECTORSIZE + flash_addresses[region]);
          if(spistatus)
          {
            erased++;
            continue;
          }
        }
        else
        {
          state++;
          continue;
        }
        break;
      case 2:
        spistatus = SST25_Write(flash_addresses[region], sizeof(sAcisConfig), (uint8_t*)&config_safe);
        if(spistatus)
        {
          state++;
          continue;
        }
        break;
      case 3:
        spistatus = SST25_Read(flash_addresses[region], sizeof(sAcisConfig), (uint8_t*)&config_check);
        if(spistatus)
        {
          size = (uint32_t)&config_check.crc - (uint32_t)&config_check;
          crc16_check = CRC16_Generate((uint8_t*)&config_check, size);
          if(crc16_check == config_check.crc && crc16_check == crc16)
          {
            if(region == 0) {
              region++;
            }
            else {
              region = 0;
              status = HAL_OK;
            }
          }
          else status = HAL_ERROR;
          state = 0;
        }
        break;
      default:
        state = 0;
        continue;
    }
  } while(0);

  return status;
}

