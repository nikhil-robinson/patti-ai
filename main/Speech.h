#pragma once



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_mn_iface.h"

#define MAX_STRING_LENGTH 100
#define MAX_COMMANDS 5

typedef struct {
  char token[MAX_STRING_LENGTH];
  void (*fun)(int); // Function pointer member
} voice_mapping_t;


#ifdef __cplusplus
extern "C" {
#endif


void enable_tts();
void app_sr_init(const voice_mapping_t *maps,const QueueHandle_t xQueue);
void say_this(char *received_message, size_t len);

#ifdef __cplusplus
}
#endif