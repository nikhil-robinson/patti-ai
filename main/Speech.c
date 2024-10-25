#include "Speech.h"
#include "bsp/esp-bsp.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_board_init.h"
#include "esp_log.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"

#define TAG "SAVY/Speech"

#define TTS_CORE 1

static volatile bool detect_flag = false;
static volatile bool tts_running = false;
static esp_afe_sr_iface_t *afe_handle = NULL;

static srmodel_list_t *models = NULL;
static TaskHandle_t voice_handel = NULL;
static TaskHandle_t detect_handel = NULL;
static QueueHandle_t xQueueResult = NULL;

voice_mapping_t voice_lookup[MAX_COMMANDS];

static void on_samples(int16_t *buf, unsigned count) {
  esp_audio_play(buf, count * 2, 0);
}

static void on_tts_idel() { tts_running = false; }

static void wait_for_tts() {
  while (tts_running) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void say_this(char *received_message, size_t len) {
  vTaskSuspend(voice_handel);
  while (tts_running) {
    vTaskDelay(1);
  }
  picotts_add(received_message, len);
  tts_running = true;
  while (tts_running) {
    vTaskDelay(1);
  }
  vTaskResume(voice_handel);
}

void feed_Task(void *arg) {
  esp_afe_sr_data_t *afe_data = arg;
  int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
  int nch = afe_handle->get_channel_num(afe_data);
  int feed_channel = esp_get_feed_channel();
  assert(nch <= feed_channel);
  int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
  assert(i2s_buff);

  while (true) {

    esp_get_feed_data(false, i2s_buff,audio_chunksize * sizeof(int16_t) * feed_channel);
    afe_handle->feed(afe_data, i2s_buff);
  }
}

void enable_tts()
{
  unsigned prio = uxTaskPriorityGet(NULL);
  picotts_init(prio, on_samples, TTS_CORE);
  picotts_set_idle_notify(on_tts_idel);
}

void detect_Task(void *arg) {
  esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
  int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
  char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
  ESP_LOGI(TAG,"multinet:%s\n", mn_name);
  esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
  model_iface_data_t *model_data = multinet->create(mn_name, 6000);

  esp_mn_commands_clear(); // Clear commands that already exist
  for (size_t i = 1; i < MAX_COMMANDS; i++) {
    voice_mapping_t *voice = &voice_lookup[i];
    esp_mn_commands_add(i, voice->token);
  }
  esp_mn_commands_update(); // update commands
  int mu_chunksize = multinet->get_samp_chunksize(model_data);
  assert(mu_chunksize == afe_chunksize);
  multinet->print_active_speech_commands(model_data);


  ESP_LOGI(TAG,"------------detect start------------\n");

  while (true) {
    wait_for_tts();
    afe_fetch_result_t *res = afe_handle->fetch(afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
      ESP_LOGI(TAG,"fetch error!\n");
      // break;
      continue;
    }

    esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

    if (mn_state == ESP_MN_STATE_DETECTING) {
      continue;
    }

    if (mn_state == ESP_MN_STATE_DETECTED) {
      esp_mn_results_t *mn_result = multinet->get_results(model_data);
      xQueueSend(xQueueResult, mn_result, portMAX_DELAY);

      ESP_LOGI(TAG,"-----------listening-----------\n");
    }

    if (mn_state == ESP_MN_STATE_TIMEOUT) {
      esp_mn_results_t *mn_result = multinet->get_results(model_data);
      ESP_LOGI(TAG,"timeout, string:%s\n", mn_result->string);
      afe_handle->enable_wakenet(afe_data);
      afe_handle->disable_wakenet(afe_data);
      multinet->clean(model_data);
      detect_flag = false;
      ESP_LOGI(TAG,"\n-----------awaits to be waken up-----------\n");
      continue;
    }
  }
  if (model_data) {
    multinet->destroy(model_data);
    model_data = NULL;
  }
  ESP_LOGI(TAG,"detect exit\n");
  vTaskDelete(NULL);
}

void app_sr_init(const voice_mapping_t *maps, QueueHandle_t xQueue) {

  xQueueResult = xQueue;

  for (size_t i = 1; i < MAX_COMMANDS; i++) {
    memcpy(voice_lookup[i].token,maps[i].token,strlen(maps[i].token));
  }

  models =esp_srmodel_init("model"); // partition label defined in partitions.csv
  ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));
  esp_audio_set_play_vol(100);
  afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
  afe_config_t afe_config = AFE_CONFIG_DEFAULT();
  afe_config.wakenet_model_name =
      esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
  afe_config.aec_init = false;
  esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
  xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void *)afe_data, 5, &detect_handel, 1);
  xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void *)afe_data,  5,&voice_handel, 1);
}