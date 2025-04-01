#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define CAN_ERROR -1
#define CAN_SUCCESS 0


typedef void (*CAN_RxCallback)(uint32_t id, bool rtr, uint8_t *data, uint8_t dataLen);

int CAN_Initialize(CAN_RxCallback rxCallback);
int CAN_Send(uint32_t id, uint8_t *data, uint8_t dataLen);

#ifdef __cplusplus
}
#endif