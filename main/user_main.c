/* Fun control FOR ESP8266 01

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "main";

/**
 *
 * GPIO status:
 * GPIO0: output
 * GPIO2: input, pulled up, interrupt from rising edge.
 *
 */

#define GPIO_OUTPUT_IO_0 0
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0))

#define GPIO_INPUT_IO_0 2
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0))

#define TIMER_ONE_SHOT false
#define TIMER_RELOAD true

#define TIMER_PART_US 0x16E360

#define ENABLE_FUN_DELAY_CNT 20  // 30sec / 1.5sec = 20 parts
#define DISABLE_FUN_DELAY_CNT 20 // 30sec / 1.5sec = 20 parts

#define HIGH_LEVEL 1
#define LOW_LEVEL 0

#define D_DISABLE_FAN 1
#define D_ENABLE_FAN 2

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_evt_queue = NULL;
static xQueueHandle control_fun_evt_queue = NULL;

enum fun_state
{
  Disabled,
  Await_enable, // когда таймер считает до включения
  Enabled,
  Await_disable // когда таймер считает до выключения
};

static enum fun_state state = Disabled;
static uint32_t time_past_parts_cnt = 0;

void hw_timer_part_callback(void *arg)
{
  uint32_t data = 0;
  xQueueSendFromISR(timer_evt_queue, &data, NULL);
}

void gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t)arg;

  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void enable_timer(void)
{
  if (!hw_timer_get_enable())
  {
    time_past_parts_cnt = 0;
    ESP_ERROR_CHECK(hw_timer_enable(true));
    ESP_ERROR_CHECK(hw_timer_alarm_us(TIMER_PART_US, TIMER_RELOAD));
  }
}

void disable_timer(void)
{
  if (hw_timer_get_enable())
  {
    ESP_ERROR_CHECK(hw_timer_enable(false));
    time_past_parts_cnt = 0;
  }
}

// when light
int run_timer_to_enable()
{
  // не дожидаясь выключаем таймер, если ждали выключения, и включаем вентилятор
  if (Await_disable == state)
  {
    ESP_LOGI(TAG, "SENSOR TASK: Light enabled. State is %d (Await_disable). Disable timer and set state Enabled\n", state);

    state = Enabled;
    disable_timer();

    uint32_t data = D_ENABLE_FAN;
    xQueueSend(control_fun_evt_queue, &data, (TickType_t)10);
    return 0;
  }

  state = Await_enable;
  ESP_LOGI(TAG, "SENSOR TASK: Light enabled. State is set %d (Await_enable). Run timer!\n", state);
  enable_timer();

  return 1;
}

// when dark
int run_timer_to_disable(void)
{
  // не дожидаясь выключаем таймер, если ждали включения
  if (Await_enable == state)
  {
    ESP_LOGI(TAG, "SENSOR TASK: Dark enabled. State is %d (Await_enable). Disable timer. Next state is Disabled and continue\n", state);

    state = Disabled;
    disable_timer();

    uint32_t data = D_DISABLE_FAN;
    xQueueSend(control_fun_evt_queue, &data, (TickType_t)10);
    return 0;
  }

  // ждем некоторое время и затем выключаем вентилятор
  state = Await_disable;
  ESP_LOGI(TAG, "SENSOR TASK: Dark enabled. State is %d (Await_disable). Enable timer.\n", state);
  enable_timer();

  return 1;
}

// Handles events from light sensor
void handle_sensor_task(void *arg)
{
  uint32_t io_num = 0;

  for (;;)
  {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
    {
      int io_level = gpio_get_level(io_num);

      if (LOW_LEVEL == io_level)
      {
        // Включили свет
        if (state == Disabled || state == Await_disable)
        {
          run_timer_to_enable();
        }
      }
      else
      {
        // Выключили свет
        if (state == Enabled || state == Await_enable)
        {
          run_timer_to_disable();
        }
      }
      // Задержка, если получили и обработали сигнал, то даем паузу, чтобы обработать корректно текущий сигнал
      vTaskDelay(2000 / portTICK_RATE_MS);
    }
  }
}

void handle_timer_events_task(void *arg)
{
  uint32_t data = 0;
  for (;;)
  {
    if (xQueueReceive(timer_evt_queue, &data, portMAX_DELAY))
    {
      if (Enabled == state || Disabled == state)
      {
        continue;
      }

      ESP_LOGI(TAG, "TIMER TASK: %d, state is: %d\n", time_past_parts_cnt * TIMER_PART_US, state);
      uint32_t delay = state == Await_disable ? DISABLE_FUN_DELAY_CNT : ENABLE_FUN_DELAY_CNT;

      if (time_past_parts_cnt < delay)
      {
        time_past_parts_cnt++;
      }
      else
      {
        if (Await_disable == state)
        {
          state = Disabled;
          data = D_DISABLE_FAN;
        }

        if (Await_enable == state)
        {
          state = Enabled;
          data = D_ENABLE_FAN;
        }

        time_past_parts_cnt = 0;
        xQueueSend(control_fun_evt_queue, &data, (TickType_t)10);

        if (ESP_OK != hw_timer_enable(false))
        {
          ESP_LOGE(TAG, "Unable to disable timer");
          // Аварийное завершение, отключаем вентилятор и переводим в состояние выключено
          state = Disabled;
          data = D_DISABLE_FAN;
          xQueueSend(control_fun_evt_queue, &data, (TickType_t)10);
        }
      }
    }
  }
}

// Непосредственное управление вентилятором
// В цикле проверяем очередь событий на включение
void control_fan_task(void *arg)
{
  uint32_t data = 0;
  for (;;)
  {
    if (xQueueReceive(control_fun_evt_queue, &data, portMAX_DELAY))
    {
      if (Await_disable == state || Await_enable == state)
      {
        continue;
      }

      if (data == D_ENABLE_FAN)
      {
        ESP_LOGI(TAG, "Enable fun!\n");
        gpio_set_level(GPIO_OUTPUT_IO_0, 1);
      }
      else if (data == D_DISABLE_FAN)
      {
        ESP_LOGI(TAG, "Disable fun!\n");
        gpio_set_level(GPIO_OUTPUT_IO_0, 0);
      }

      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }
}

void app_main(void)
{
  gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));
  timer_evt_queue = xQueueCreate(2, sizeof(uint32_t));
  control_fun_evt_queue = xQueueCreate(2, sizeof(uint32_t));

  if (pdPASS != xTaskCreate(handle_sensor_task, "handle_sensor_task", 2048, NULL, 10, NULL))
  {
    ESP_LOGE(TAG, "Unable to create handle_sensor_task!\n");
  }

  if (pdPASS != xTaskCreate(handle_timer_events_task, "handle_timer_events_task", 2048, NULL, 10, NULL))
  {
    ESP_LOGE(TAG, "Unable to create handle_timer_events_task!\n");
  }

  if (pdPASS != xTaskCreate(control_fan_task, "control_fan_task", 2048, NULL, 10, NULL))
  {
    ESP_LOGE(TAG, "Unable to create control_fan_task!\n");
  }

  // Init timer
  ESP_LOGI(TAG, "Initialize hw_timer for hw_timer_part_callback");
  ESP_ERROR_CHECK(hw_timer_init(hw_timer_part_callback, NULL));
  ESP_ERROR_CHECK(hw_timer_enable(false));

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0));

  while (1)
  {
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}
