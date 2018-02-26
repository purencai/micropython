/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __FILE_RECEVIE_H__
#define __FILE_RECEVIE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"

#define FILE_TYPE_FRAME  0x4d54
#define FILE_START_FRAME 0x6130 
#define FILE_DATA_FRAME  0x4c42
#define FILE_END_FRAME   0x524e
#define NEXT_FRAME_TIME_OUT    128
#define FILE_RECEIVE_TIME_OUT  5000

#define NO_ERROR       0
#define CRC_ERROR      1
#define TIME_OUT_ERROR 2
#define SERIOUS_ERROR  100

#ifdef __cplusplus
}
#endif

#endif //__FILE_RECEVIE_H__
