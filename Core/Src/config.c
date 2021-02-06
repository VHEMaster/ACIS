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

HAL_StatusTypeDef config_load(sAcisConfig * config)
{
  HAL_StatusTypeDef status = HAL_BUSY;
  uint32_t size = (uint32_t)&config->crc - (uint32_t)config;
  uint8_t spistatus = SST25_Read(0, sizeof(sAcisConfig), (uint8_t*)config);
  if(spistatus)
  {
    uint16_t crc16 = CRC16_Generate((uint8_t*)config, size);
    if(crc16 == config->crc)
      status = HAL_OK;
    else status = HAL_ERROR;
  }

  return status;
}

/*
static void setconfig_microplex(sAcisConfig * config, uint8_t i)
{

  config->tables[i].rotates_count = 13;
  config->tables[i].rotates[0] = 0;
  config->tables[i].rotates[1] = 500;
  config->tables[i].rotates[2] = 1000;
  config->tables[i].rotates[3] = 1500;
  config->tables[i].rotates[4] = 2000;
  config->tables[i].rotates[5] = 2500;
  config->tables[i].rotates[6] = 3000;
  config->tables[i].rotates[7] = 3500;
  config->tables[i].rotates[8] = 4000;
  config->tables[i].rotates[9] = 4500;
  config->tables[i].rotates[10] = 5000;
  config->tables[i].rotates[11] = 5500;
  config->tables[i].rotates[12] = 6000;

  config->tables[i].pressures_count = 8;
  config->tables[i].pressures[0] = 40000.0f;
  config->tables[i].pressures[1] = 50000.0f;
  config->tables[i].pressures[2] = 60000.0f;
  config->tables[i].pressures[3] = 70000.0f;
  config->tables[i].pressures[4] = 81000.0f;
  config->tables[i].pressures[5] = 92000.0f;
  config->tables[i].pressures[6] = 102000.0f;
  config->tables[i].pressures[7] = 112000.0f;

  config->tables[i].ignitions[7][0] = 10.0f;
  config->tables[i].ignitions[7][1] = 14.0f;
  config->tables[i].ignitions[7][2] = 16.0f;
  config->tables[i].ignitions[7][3] = 16.0f;
  config->tables[i].ignitions[7][4] = 22.0f;
  config->tables[i].ignitions[7][5] = 24.5f;
  config->tables[i].ignitions[7][6] = 27.0f;
  config->tables[i].ignitions[7][7] = 28.0f;
  config->tables[i].ignitions[7][8] = 28.0f;
  config->tables[i].ignitions[7][9] = 29.0f;
  config->tables[i].ignitions[7][10] = 29.0f;
  config->tables[i].ignitions[7][11] = 29.0f;
  config->tables[i].ignitions[7][12] = 29.0f;

  config->tables[i].ignitions[6][0] = 10.0f;
  config->tables[i].ignitions[6][1] = 15.0f;
  config->tables[i].ignitions[6][2] = 18.0f;
  config->tables[i].ignitions[6][3] = 18.0f;
  config->tables[i].ignitions[6][4] = 24.0f;
  config->tables[i].ignitions[6][5] = 26.0f;
  config->tables[i].ignitions[6][6] = 28.0f;
  config->tables[i].ignitions[6][7] = 28.5f;
  config->tables[i].ignitions[6][8] = 29.0f;
  config->tables[i].ignitions[6][9] = 33.0f;
  config->tables[i].ignitions[6][10] = 33.0f;
  config->tables[i].ignitions[6][11] = 33.0f;
  config->tables[i].ignitions[6][12] = 33.0f;

  config->tables[i].ignitions[5][0] = 10.0f;
  config->tables[i].ignitions[5][1] = 16.0f;
  config->tables[i].ignitions[5][2] = 20.0f;
  config->tables[i].ignitions[5][3] = 23.0f;
  config->tables[i].ignitions[5][4] = 25.5f;
  config->tables[i].ignitions[5][5] = 28.5f;
  config->tables[i].ignitions[5][6] = 29.0f;
  config->tables[i].ignitions[5][7] = 30.0f;
  config->tables[i].ignitions[5][8] = 30.0f;
  config->tables[i].ignitions[5][9] = 33.0f;
  config->tables[i].ignitions[5][10] = 33.0f;
  config->tables[i].ignitions[5][11] = 33.0f;
  config->tables[i].ignitions[5][12] = 33.0f;

  config->tables[i].ignitions[4][0] = 10.0f;
  config->tables[i].ignitions[4][1] = 16.0f;
  config->tables[i].ignitions[4][2] = 20.0f;
  config->tables[i].ignitions[4][3] = 25.0f;
  config->tables[i].ignitions[4][4] = 27.5f;
  config->tables[i].ignitions[4][5] = 29.5f;
  config->tables[i].ignitions[4][6] = 30.5f;
  config->tables[i].ignitions[4][7] = 31.0f;
  config->tables[i].ignitions[4][8] = 31.0f;
  config->tables[i].ignitions[4][9] = 33.0f;
  config->tables[i].ignitions[4][10] = 33.0f;
  config->tables[i].ignitions[4][11] = 33.0f;
  config->tables[i].ignitions[4][12] = 33.0f;

  config->tables[i].ignitions[3][0] = 10.0f;
  config->tables[i].ignitions[3][1] = 16.0f;
  config->tables[i].ignitions[3][2] = 20.0f;
  config->tables[i].ignitions[3][3] = 24.5f;
  config->tables[i].ignitions[3][4] = 29.0f;
  config->tables[i].ignitions[3][5] = 31.0f;
  config->tables[i].ignitions[3][6] = 32.0f;
  config->tables[i].ignitions[3][7] = 33.0f;
  config->tables[i].ignitions[3][8] = 33.0f;
  config->tables[i].ignitions[3][9] = 33.0f;
  config->tables[i].ignitions[3][10] = 33.0f;
  config->tables[i].ignitions[3][11] = 33.0f;
  config->tables[i].ignitions[3][12] = 33.0f;

  config->tables[i].ignitions[2][0] = 10.0f;
  config->tables[i].ignitions[2][1] = 16.0f;
  config->tables[i].ignitions[2][2] = 20.0f;
  config->tables[i].ignitions[2][3] = 26.0f;
  config->tables[i].ignitions[2][4] = 32.0f;
  config->tables[i].ignitions[2][5] = 34.0f;
  config->tables[i].ignitions[2][6] = 34.0f;
  config->tables[i].ignitions[2][7] = 34.0f;
  config->tables[i].ignitions[2][8] = 34.0f;
  config->tables[i].ignitions[2][9] = 34.0f;
  config->tables[i].ignitions[2][10] = 34.0f;
  config->tables[i].ignitions[2][11] = 33.0f;
  config->tables[i].ignitions[2][12] = 33.0f;

  config->tables[i].ignitions[1][0] = 10.0f;
  config->tables[i].ignitions[1][1] = 16.0f;
  config->tables[i].ignitions[1][2] = 20.0f;
  config->tables[i].ignitions[1][3] = 28.5f;
  config->tables[i].ignitions[1][4] = 37.0f;
  config->tables[i].ignitions[1][5] = 37.0f;
  config->tables[i].ignitions[1][6] = 37.0f;
  config->tables[i].ignitions[1][7] = 37.0f;
  config->tables[i].ignitions[1][8] = 37.0f;
  config->tables[i].ignitions[1][9] = 37.0f;
  config->tables[i].ignitions[1][10] = 37.0f;
  config->tables[i].ignitions[1][11] = 33.0f;
  config->tables[i].ignitions[1][12] = 33.0f;

  config->tables[i].ignitions[0][0] = 10.0f;
  config->tables[i].ignitions[0][1] = 16.0f;
  config->tables[i].ignitions[0][2] = 20.0f;
  config->tables[i].ignitions[0][3] = 28.5f;
  config->tables[i].ignitions[0][4] = 37.0f;
  config->tables[i].ignitions[0][5] = 38.5f;
  config->tables[i].ignitions[0][6] = 40.0f;
  config->tables[i].ignitions[0][7] = 40.0f;
  config->tables[i].ignitions[0][8] = 40.0f;
  config->tables[i].ignitions[0][9] = 40.0f;
  config->tables[i].ignitions[0][10] = 40.0f;
  config->tables[i].ignitions[0][11] = 33.0f;
  config->tables[i].ignitions[0][12] = 33.0f;
}
*/

static void setconfig_standart16l(sAcisConfig * config, uint8_t i)
{

  config->tables[i].rotates_count = 16;
  config->tables[i].rotates[0] = 600;
  config->tables[i].rotates[1] = 720;
  config->tables[i].rotates[2] = 840;
  config->tables[i].rotates[3] = 990;
  config->tables[i].rotates[4] = 1170;
  config->tables[i].rotates[5] = 1380;
  config->tables[i].rotates[6] = 1650;
  config->tables[i].rotates[7] = 1950;
  config->tables[i].rotates[8] = 2310;
  config->tables[i].rotates[9] = 2730;
  config->tables[i].rotates[10] = 3210;
  config->tables[i].rotates[11] = 3840;
  config->tables[i].rotates[12] = 4530;
  config->tables[i].rotates[13] = 5370;
  config->tables[i].rotates[14] = 6360;
  config->tables[i].rotates[15] = 7500;

  config->tables[i].pressures_count = 16;
  config->tables[i].pressures[0] = 28000.0f;
  config->tables[i].pressures[1] = 32800.0f;
  config->tables[i].pressures[2] = 37600.0f;
  config->tables[i].pressures[3] = 42400.0f;
  config->tables[i].pressures[4] = 47200.0f;
  config->tables[i].pressures[5] = 52000.0f;
  config->tables[i].pressures[6] = 56800.0f;
  config->tables[i].pressures[7] = 61600.0f;
  config->tables[i].pressures[8] = 66400.0f;
  config->tables[i].pressures[9] = 71200.0f;
  config->tables[i].pressures[10] = 76000.0f;
  config->tables[i].pressures[11] = 80800.0f;
  config->tables[i].pressures[12] = 85600.0f;
  config->tables[i].pressures[13] = 90400.0f;
  config->tables[i].pressures[14] = 95200.0f;
  config->tables[i].pressures[15] = 100000.0f;

  config->tables[i].ignitions[0][0]  = 14.5f;
  config->tables[i].ignitions[0][1]  = 16.1f;
  config->tables[i].ignitions[0][2]  = 18.0f;
  config->tables[i].ignitions[0][3]  = 20.9f;
  config->tables[i].ignitions[0][4]  = 24.7f;
  config->tables[i].ignitions[0][5]  = 29.7f;
  config->tables[i].ignitions[0][6]  = 37.8f;
  config->tables[i].ignitions[0][7]  = 40.9f;
  config->tables[i].ignitions[0][8]  = 42.6f;
  config->tables[i].ignitions[0][9]  = 43.4f;
  config->tables[i].ignitions[0][10] = 44.0f;
  config->tables[i].ignitions[0][11] = 44.4f;
  config->tables[i].ignitions[0][12] = 44.3f;
  config->tables[i].ignitions[0][13] = 44.3f;
  config->tables[i].ignitions[0][14] = 44.8f;
  config->tables[i].ignitions[0][15] = 45.0f;

  config->tables[i].ignitions[1][0]  = 16.1f;
  config->tables[i].ignitions[1][1]  = 18.0f;
  config->tables[i].ignitions[1][2]  = 20.1f;
  config->tables[i].ignitions[1][3]  = 23.1f;
  config->tables[i].ignitions[1][4]  = 26.6f;
  config->tables[i].ignitions[1][5]  = 31.0f;
  config->tables[i].ignitions[1][6]  = 38.1f;
  config->tables[i].ignitions[1][7]  = 41.0f;
  config->tables[i].ignitions[1][8]  = 42.6f;
  config->tables[i].ignitions[1][9]  = 43.4f;
  config->tables[i].ignitions[1][10] = 43.0f;
  config->tables[i].ignitions[1][11] = 44.4f;
  config->tables[i].ignitions[1][12] = 44.3f;
  config->tables[i].ignitions[1][13] = 44.3f;
  config->tables[i].ignitions[1][14] = 44.8f;
  config->tables[i].ignitions[1][15] = 45.0f;

  config->tables[i].ignitions[2][0]  = 17.8f;
  config->tables[i].ignitions[2][1]  = 19.7f;
  config->tables[i].ignitions[2][2]  = 21.5f;
  config->tables[i].ignitions[2][3]  = 24.2f;
  config->tables[i].ignitions[2][4]  = 27.6f;
  config->tables[i].ignitions[2][5]  = 31.7f;
  config->tables[i].ignitions[2][6]  = 38.3f;
  config->tables[i].ignitions[2][7]  = 41.1f;
  config->tables[i].ignitions[2][8]  = 42.6f;
  config->tables[i].ignitions[2][9]  = 43.4f;
  config->tables[i].ignitions[2][10] = 43.0f;
  config->tables[i].ignitions[2][11] = 44.4f;
  config->tables[i].ignitions[2][12] = 44.3f;
  config->tables[i].ignitions[2][13] = 44.3f;
  config->tables[i].ignitions[2][14] = 44.8f;
  config->tables[i].ignitions[2][15] = 45.0f;

  config->tables[i].ignitions[3][0]  = 18.7f;
  config->tables[i].ignitions[3][1]  = 20.5f;
  config->tables[i].ignitions[3][2]  = 22.2f;
  config->tables[i].ignitions[3][3]  = 24.6f;
  config->tables[i].ignitions[3][4]  = 27.7f;
  config->tables[i].ignitions[3][5]  = 31.8f;
  config->tables[i].ignitions[3][6]  = 38.2f;
  config->tables[i].ignitions[3][7]  = 40.9f;
  config->tables[i].ignitions[3][8]  = 42.4f;
  config->tables[i].ignitions[3][9]  = 43.2f;
  config->tables[i].ignitions[3][10] = 43.7f;
  config->tables[i].ignitions[3][11] = 44.1f;
  config->tables[i].ignitions[3][12] = 44.0f;
  config->tables[i].ignitions[3][13] = 44.1f;
  config->tables[i].ignitions[3][14] = 44.7f;
  config->tables[i].ignitions[3][15] = 44.9f;

  config->tables[i].ignitions[4][0]  = 18.7f;
  config->tables[i].ignitions[4][1]  = 20.4f;
  config->tables[i].ignitions[4][2]  = 21.9f;
  config->tables[i].ignitions[4][3]  = 24.3f;
  config->tables[i].ignitions[4][4]  = 27.3f;
  config->tables[i].ignitions[4][5]  = 31.3f;
  config->tables[i].ignitions[4][6]  = 37.6f;
  config->tables[i].ignitions[4][7]  = 40.3f;
  config->tables[i].ignitions[4][8]  = 41.7f;
  config->tables[i].ignitions[4][9]  = 42.4f;
  config->tables[i].ignitions[4][10] = 42.9f;
  config->tables[i].ignitions[4][11] = 43.2f;
  config->tables[i].ignitions[4][12] = 43.1f;
  config->tables[i].ignitions[4][13] = 43.3f;
  config->tables[i].ignitions[4][14] = 43.0f;
  config->tables[i].ignitions[4][15] = 44.2f;

  config->tables[i].ignitions[5][0]  = 17.4f;
  config->tables[i].ignitions[5][1]  = 19.4f;
  config->tables[i].ignitions[5][2]  = 20.9f;
  config->tables[i].ignitions[5][3]  = 23.2f;
  config->tables[i].ignitions[5][4]  = 26.2f;
  config->tables[i].ignitions[5][5]  = 30.1f;
  config->tables[i].ignitions[5][6]  = 36.3f;
  config->tables[i].ignitions[5][7]  = 39.1f;
  config->tables[i].ignitions[5][8]  = 40.6f;
  config->tables[i].ignitions[5][9]  = 41.3f;
  config->tables[i].ignitions[5][10] = 41.9f;
  config->tables[i].ignitions[5][11] = 42.2f;
  config->tables[i].ignitions[5][12] = 42.1f;
  config->tables[i].ignitions[5][13] = 42.3f;
  config->tables[i].ignitions[5][14] = 42.9f;
  config->tables[i].ignitions[5][15] = 43.1f;

  config->tables[i].ignitions[6][0]  = 14.9f;
  config->tables[i].ignitions[6][1]  = 17.2f;
  config->tables[i].ignitions[6][2]  = 18.7f;
  config->tables[i].ignitions[6][3]  = 21.1f;
  config->tables[i].ignitions[6][4]  = 24.3f;
  config->tables[i].ignitions[6][5]  = 28.4f;
  config->tables[i].ignitions[6][6]  = 34.3f;
  config->tables[i].ignitions[6][7]  = 37.3f;
  config->tables[i].ignitions[6][8]  = 38.9f;
  config->tables[i].ignitions[6][9]  = 39.8f;
  config->tables[i].ignitions[6][10] = 40.6f;
  config->tables[i].ignitions[6][11] = 41.0f;
  config->tables[i].ignitions[6][12] = 41.0f;
  config->tables[i].ignitions[6][13] = 41.3f;
  config->tables[i].ignitions[6][14] = 41.8f;
  config->tables[i].ignitions[6][15] = 42.0f;

  config->tables[i].ignitions[7][0]  = 12.2f;
  config->tables[i].ignitions[7][1]  = 14.5f;
  config->tables[i].ignitions[7][2]  = 15.9f;
  config->tables[i].ignitions[7][3]  = 17.9f;
  config->tables[i].ignitions[7][4]  = 21.5f;
  config->tables[i].ignitions[7][5]  = 26.0f;
  config->tables[i].ignitions[7][6]  = 31.9f;
  config->tables[i].ignitions[7][7]  = 35.0f;
  config->tables[i].ignitions[7][8]  = 36.8f;
  config->tables[i].ignitions[7][9]  = 37.9f;
  config->tables[i].ignitions[7][10] = 38.9f;
  config->tables[i].ignitions[7][11] = 39.5f;
  config->tables[i].ignitions[7][12] = 39.5f;
  config->tables[i].ignitions[7][13] = 40.2f;
  config->tables[i].ignitions[7][14] = 40.8f;
  config->tables[i].ignitions[7][15] = 41.0f;

  config->tables[i].ignitions[8][0]  = 10.2f;
  config->tables[i].ignitions[8][1]  = 11.9f;
  config->tables[i].ignitions[8][2]  = 13.1f;
  config->tables[i].ignitions[8][3]  = 14.9f;
  config->tables[i].ignitions[8][4]  = 18.3f;
  config->tables[i].ignitions[8][5]  = 22.8f;
  config->tables[i].ignitions[8][6]  = 28.4f;
  config->tables[i].ignitions[8][7]  = 31.9f;
  config->tables[i].ignitions[8][8]  = 34.2f;
  config->tables[i].ignitions[8][9]  = 35.8f;
  config->tables[i].ignitions[8][10] = 37.2f;
  config->tables[i].ignitions[8][11] = 38.1f;
  config->tables[i].ignitions[8][12] = 38.7f;
  config->tables[i].ignitions[8][13] = 39.1f;
  config->tables[i].ignitions[8][14] = 39.7f;
  config->tables[i].ignitions[8][15] = 39.9f;

  config->tables[i].ignitions[9][0]  = 8.5f;
  config->tables[i].ignitions[9][1]  = 9.7f;
  config->tables[i].ignitions[9][2]  = 10.8f;
  config->tables[i].ignitions[9][3]  = 12.4f;
  config->tables[i].ignitions[9][4]  = 15.3f;
  config->tables[i].ignitions[9][5]  = 19.2f;
  config->tables[i].ignitions[9][6]  = 23.0f;
  config->tables[i].ignitions[9][7]  = 27.9f;
  config->tables[i].ignitions[9][8]  = 30.0f;
  config->tables[i].ignitions[9][9]  = 33.2f;
  config->tables[i].ignitions[9][10] = 35.4f;
  config->tables[i].ignitions[9][11] = 36.5f;
  config->tables[i].ignitions[9][12] = 36.7f;
  config->tables[i].ignitions[9][13] = 37.0f;
  config->tables[i].ignitions[9][14] = 37.5f;
  config->tables[i].ignitions[9][15] = 37.7f;

  config->tables[i].ignitions[10][0]  = 7.1f;
  config->tables[i].ignitions[10][1]  = 8.2f;
  config->tables[i].ignitions[10][2]  = 9.2f;
  config->tables[i].ignitions[10][3]  = 10.6f;
  config->tables[i].ignitions[10][4]  = 12.9f;
  config->tables[i].ignitions[10][5]  = 16.1f;
  config->tables[i].ignitions[10][6]  = 20.8f;
  config->tables[i].ignitions[10][7]  = 24.0f;
  config->tables[i].ignitions[10][8]  = 28.3f;
  config->tables[i].ignitions[10][9]  = 30.8f;
  config->tables[i].ignitions[10][10] = 33.4f;
  config->tables[i].ignitions[10][11] = 34.2f;
  config->tables[i].ignitions[10][12] = 33.7f;
  config->tables[i].ignitions[10][13] = 33.9f;
  config->tables[i].ignitions[10][14] = 34.2f;
  config->tables[i].ignitions[10][15] = 34.3f;

  config->tables[i].ignitions[11][0]  = 6.3f;
  config->tables[i].ignitions[11][1]  = 7.2f;
  config->tables[i].ignitions[11][2]  = 8.1f;
  config->tables[i].ignitions[11][3]  = 9.3f;
  config->tables[i].ignitions[11][4]  = 11.1f;
  config->tables[i].ignitions[11][5]  = 14.0f;
  config->tables[i].ignitions[11][6]  = 18.0f;
  config->tables[i].ignitions[11][7]  = 23.3f;
  config->tables[i].ignitions[11][8]  = 26.9f;
  config->tables[i].ignitions[11][9]  = 29.1f;
  config->tables[i].ignitions[11][10] = 31.6f;
  config->tables[i].ignitions[11][11] = 32.0f;
  config->tables[i].ignitions[11][12] = 31.0f;
  config->tables[i].ignitions[11][13] = 31.3f;
  config->tables[i].ignitions[11][14] = 31.7f;
  config->tables[i].ignitions[11][15] = 31.8f;

  config->tables[i].ignitions[12][0]  = 6.1f;
  config->tables[i].ignitions[12][1]  = 6.7f;
  config->tables[i].ignitions[12][2]  = 7.3f;
  config->tables[i].ignitions[12][3]  = 8.4f;
  config->tables[i].ignitions[12][4]  = 9.9f;
  config->tables[i].ignitions[12][5]  = 12.5f;
  config->tables[i].ignitions[12][6]  = 17.7f;
  config->tables[i].ignitions[12][7]  = 22.0f;
  config->tables[i].ignitions[12][8]  = 25.9f;
  config->tables[i].ignitions[12][9]  = 27.9f;
  config->tables[i].ignitions[12][10] = 30.1f;
  config->tables[i].ignitions[12][11] = 30.3f;
  config->tables[i].ignitions[12][12] = 29.2f;
  config->tables[i].ignitions[12][13] = 29.4f;
  config->tables[i].ignitions[12][14] = 30.0f;
  config->tables[i].ignitions[12][15] = 30.2f;

  config->tables[i].ignitions[13][0]  = 6.0f;
  config->tables[i].ignitions[13][1]  = 6.5f;
  config->tables[i].ignitions[13][2]  = 7.1f;
  config->tables[i].ignitions[13][3]  = 7.9f;
  config->tables[i].ignitions[13][4]  = 9.1f;
  config->tables[i].ignitions[13][5]  = 11.3f;
  config->tables[i].ignitions[13][6]  = 15.8f;
  config->tables[i].ignitions[13][7]  = 20.3f;
  config->tables[i].ignitions[13][8]  = 24.4f;
  config->tables[i].ignitions[13][9]  = 26.6f;
  config->tables[i].ignitions[13][10] = 28.7f;
  config->tables[i].ignitions[13][11] = 28.8f;
  config->tables[i].ignitions[13][12] = 27.6f;
  config->tables[i].ignitions[13][13] = 27.9f;
  config->tables[i].ignitions[13][14] = 28.6f;
  config->tables[i].ignitions[13][15] = 28.8f;

  config->tables[i].ignitions[14][0]  = 6.0f;
  config->tables[i].ignitions[14][1]  = 6.5f;
  config->tables[i].ignitions[14][2]  = 7.0f;
  config->tables[i].ignitions[14][3]  = 7.7f;
  config->tables[i].ignitions[14][4]  = 8.7f;
  config->tables[i].ignitions[14][5]  = 10.5f;
  config->tables[i].ignitions[14][6]  = 14.1f;
  config->tables[i].ignitions[14][7]  = 18.2f;
  config->tables[i].ignitions[14][8]  = 22.1f;
  config->tables[i].ignitions[14][9]  = 24.5f;
  config->tables[i].ignitions[14][10] = 26.8f;
  config->tables[i].ignitions[14][11] = 27.0f;
  config->tables[i].ignitions[14][12] = 25.9f;
  config->tables[i].ignitions[14][13] = 26.2f;
  config->tables[i].ignitions[14][14] = 26.8f;
  config->tables[i].ignitions[14][15] = 27.0f;

  config->tables[i].ignitions[15][0]  = 6.0f;
  config->tables[i].ignitions[15][1]  = 6.5f;
  config->tables[i].ignitions[15][2]  = 7.0f;
  config->tables[i].ignitions[15][3]  = 7.6f;
  config->tables[i].ignitions[15][4]  = 8.5f;
  config->tables[i].ignitions[15][5]  = 9.9f;
  config->tables[i].ignitions[15][6]  = 12.3f;
  config->tables[i].ignitions[15][7]  = 15.8f;
  config->tables[i].ignitions[15][8]  = 19.5f;
  config->tables[i].ignitions[15][9]  = 22.2f;
  config->tables[i].ignitions[15][10] = 24.8f;
  config->tables[i].ignitions[15][11] = 25.1f;
  config->tables[i].ignitions[15][12] = 24.0f;
  config->tables[i].ignitions[15][13] = 24.3f;
  config->tables[i].ignitions[15][14] = 24.8f;
  config->tables[i].ignitions[15][15] = 25.0f;
}

HAL_StatusTypeDef config_default(sAcisConfig * config)
{
  HAL_StatusTypeDef status = HAL_OK;

  for(int i = 0; i < sizeof(sAcisConfig); i++)
    ((uint8_t*)config)[i] = 0;

  config->params.isCutoutEnabled = 1;
  config->params.isTemperatureEnabled = 1;
  config->params.isEconomEnabled = 1;
  config->params.isAutostartEnabled = 0;
  config->params.isIgnitionByHall = 0;
  config->params.isForceTable = 0;
  config->params.isHallLearningMode = 0;
  config->params.isSwitchByExternal = 1;
  config->params.isEconOutAsStrobe = 0;

  config->params.switchPos1Table = 0;
  config->params.switchPos0Table = 0;
  config->params.switchPos2Table = 0;
  config->params.forceTableNumber = 0;

  config->params.InitialRpmThreshold = 400;
  config->params.EconRpmThreshold = 2000;
  config->params.CutoutRPM = 5000;

  config->tables_count = 1;
  for(int i = 0; i < config->tables_count; i++)
  {

    strcpy(config->tables[i].name, "Default 1");

    config->tables[i].valve_channel = ValvePetrol;
    config->tables[i].valve_timeout = 0;

    config->tables[i].initial_ignition = 0.0f;
    config->tables[i].octane_corrector = 0.0f;

    config->tables[i].idles_count = 20;
    config->tables[i].idle_rotates[0] = 417;
    config->tables[i].idle_rotates[1] = 455;
    config->tables[i].idle_rotates[2] = 476;
    config->tables[i].idle_rotates[3] = 500;
    config->tables[i].idle_rotates[4] = 525;
    config->tables[i].idle_rotates[5] = 556;
    config->tables[i].idle_rotates[6] = 588;
    config->tables[i].idle_rotates[7] = 625;
    config->tables[i].idle_rotates[8] = 667;
    config->tables[i].idle_rotates[9] = 714;
    config->tables[i].idle_rotates[10] = 769;
    config->tables[i].idle_rotates[11] = 833;
    config->tables[i].idle_rotates[12] = 909;
    config->tables[i].idle_rotates[13] = 1000;
    config->tables[i].idle_rotates[14] = 1111;
    config->tables[i].idle_rotates[15] = 1250;
    config->tables[i].idle_rotates[16] = 1429;
    config->tables[i].idle_rotates[17] = 1667;
    config->tables[i].idle_rotates[18] = 2000;
    config->tables[i].idle_rotates[19] = 2500;

    config->tables[i].idle_ignitions[0] = 10;
    config->tables[i].idle_ignitions[1] = 11;
    config->tables[i].idle_ignitions[2] = 12;
    config->tables[i].idle_ignitions[3] = 13;
    config->tables[i].idle_ignitions[4] = 14;
    config->tables[i].idle_ignitions[5] = 16;
    config->tables[i].idle_ignitions[6] = 18;
    config->tables[i].idle_ignitions[7] = 20;
    config->tables[i].idle_ignitions[8] = 20;
    config->tables[i].idle_ignitions[9] = 19;
    config->tables[i].idle_ignitions[10] = 15;
    config->tables[i].idle_ignitions[11] = 11;
    config->tables[i].idle_ignitions[12] = 9;
    config->tables[i].idle_ignitions[13] = 8;
    config->tables[i].idle_ignitions[14] = 8;
    config->tables[i].idle_ignitions[15] = 9;
    config->tables[i].idle_ignitions[16] = 10;
    config->tables[i].idle_ignitions[17] = 12;
    config->tables[i].idle_ignitions[18] = 14;
    config->tables[i].idle_ignitions[19] = 17;

    //setconfig_microplex(config, i);
    setconfig_standart16l(config,i);

    config->tables[i].temperatures_count = 11;
    config->tables[i].temperatures[0] = -20.0f;
    config->tables[i].temperatures[1] = -10.0f;
    config->tables[i].temperatures[2] = 0.0f;
    config->tables[i].temperatures[3] = 10.0f;
    config->tables[i].temperatures[4] = 20.0f;
    config->tables[i].temperatures[5] = 30.0f;
    config->tables[i].temperatures[6] = 40.0f;
    config->tables[i].temperatures[7] = 50.0f;
    config->tables[i].temperatures[8] = 60.0f;
    config->tables[i].temperatures[9] = 70.0f;
    config->tables[i].temperatures[10] = 80.0f;

    config->tables[i].temperature_ignitions[0] = 5.0f;
    config->tables[i].temperature_ignitions[1] = 4.0f;
    config->tables[i].temperature_ignitions[2] = 3.0f;
    config->tables[i].temperature_ignitions[3] = 2.0f;
    config->tables[i].temperature_ignitions[4] = 2.0f;
    config->tables[i].temperature_ignitions[5] = 1.0f;
    config->tables[i].temperature_ignitions[6] = 1.0f;
    config->tables[i].temperature_ignitions[7] = 0.0f;
    config->tables[i].temperature_ignitions[8] = 0.0f;
    config->tables[i].temperature_ignitions[9] = 0.0f;
    config->tables[i].temperature_ignitions[10] = 0.0f;

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
  uint16_t crc16_check;
  uint8_t spistatus = 0;
  do
  {
    switch (state)
    {
      case 0:
        size = (uint32_t)&config->crc - (uint32_t)config;
        crc16 = CRC16_Generate((uint8_t*)config, size);
        config->crc = crc16;
        state++;
        sectors = (sizeof(sAcisConfig) / SST25_SECTORSIZE) + ((sizeof(sAcisConfig) % SST25_SECTORSIZE > 0) ? (1) : (0));
        erased = 0;
        continue;
      case 1:
        if(erased < sectors)
        {
          spistatus = SST25_Erase4KSector(erased * SST25_SECTORSIZE);
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
        spistatus = SST25_Write(0, sizeof(sAcisConfig), (uint8_t*)config);
        if(spistatus)
        {
          state++;
          continue;
        }
        break;
      case 3:
        spistatus = SST25_Read(0, sizeof(sAcisConfig), (uint8_t*)config);
        if(spistatus)
        {
          size = (uint32_t)&config->crc - (uint32_t)config;
          crc16_check = CRC16_Generate((uint8_t*)config, size);
          if(crc16_check == config->crc && crc16_check == crc16)
            status = HAL_OK;
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

