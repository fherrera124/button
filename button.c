/**
  ******************************************************************************
  * @file           : button.c
  * @authors        : Mauricio Barroso Benavides - Francisco Herrera
  * @date           : Mar 20, 2022
  * @brief          : This file provides code for the configuration and control
  *                   of bounce of the buttons instances
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides - Francisco Herrera
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "button.h"
#include "esp_log.h"
#include "rotary_encoder.h" /* esp32-rotary-encoder component */

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* This button defaults to falling mode */
#define PRESSED		 DOWN_STATE
#define NOT_PRESSED UP_STATE

/* Private function prototypes -----------------------------------------------*/
static void IRAM_ATTR isr_handler(void * arg);
static void button_task(void * arg);
static void timer_handler(TimerHandle_t timer);
static void default_queue(QueueHandle_t queue, TickType_t ticks);
static void rotary_queue(QueueHandle_t queue, TickType_t ticks);

/* Private variables ---------------------------------------------------------*/
/* Tag for debug */
static const char * TAG = "button";

/* Exported functions --------------------------------------------------------*/
esp_err_t button_init(button_t * const me,
		gpio_num_t gpio,
		QueueHandle_t queue) {

	/* Error code variable */
	esp_err_t ret;

	/* If no queue handle is passed by parameter, check
	   if this button instance has already one */
	if (queue == NULL && me->queue == NULL) {
		ESP_LOGE(TAG, "Error in gpio number argument");

		return ESP_ERR_INVALID_ARG;
	}

	/* Initialize button GPIO */
	if(gpio < GPIO_NUM_0 || gpio >= GPIO_NUM_MAX) {
		ESP_LOGE(TAG, "Error in gpio number argument");

		return ESP_ERR_INVALID_ARG;
	}

	me->gpio = gpio;

	gpio_config_t gpio_conf;
	gpio_conf.pin_bit_mask = 1ULL << me->gpio;
	gpio_conf.mode = GPIO_MODE_INPUT;

//	if(mode != FALLING_MODE && mode != RISING_MODE) {
//		ESP_LOGE(TAG, "Error in mode argument");
//
//		return ESP_ERR_INVALID_ARG;
//	}

	me->mode = FALLING_MODE;

	if(me->mode == FALLING_MODE) {
		gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
		gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		me->state = NOT_PRESSED;
		gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
	}
//	else {
//		gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
//		gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
//		me->state = UP_STATE;
//		gpio_conf.intr_type = GPIO_INTR_POSEDGE;
//	}

//	gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
	ret = gpio_config(&gpio_conf);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Error configuring GPIO");

		return ret;
	}

	/* Install ISR service */
	ret = gpio_install_isr_service(0);

	if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "Error configuring GPIO ISR service");

		return ret;
	}

	/* Add ISR handler */
	ret = gpio_isr_handler_add(me->gpio, isr_handler, &me->timer);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Error adding GPIO ISR handler");

		return ret;
	}

	return ESP_OK;
}

esp_err_t button_default_init(button_t *const me,
		  gpio_num_t gpio,
		  UBaseType_t task_priority,
		  uint32_t task_stack_size) {
	ESP_LOGI(TAG, "Initializing button component...");

	/* Error code variable */
	esp_err_t ret;

	/* Initialize callback struct pointer */
	me->cbs = NULL;

	/* Attach the function responsible for writing
	   the press event into the queue */
	me->event_fn = default_queue;

	/* Create button queue */
	me->queue = xQueueCreate(1, sizeof(button_press_e));

	if (me->queue == NULL) {
		ESP_LOGE(TAG, "Error allocating memory to create queue");

		return ESP_ERR_NO_MEM;
	}

	/* Setting up gpio interrupt */
	ret = button_init(me, gpio, NULL);
		
	if (ret != ESP_OK) {
		return ret;
	}

	/* Allocate memory for callback struct */
	me->cbs = calloc(0, sizeof(f_cbs));

	if (me->cbs == NULL) {
		ESP_LOGE(TAG, "Error allocating memory for callback struct");

		return ESP_ERR_NO_MEM;
	}
		
	/* Register queue event function callback */
	me->event_fn = default_queue;

	/* Create RTOS task */
    if(pdPASS != xTaskCreate(button_task,
    	"Button Task",
		task_stack_size,
		(void *)me,
		task_priority,
		NULL)) {
    	ESP_LOGE(TAG, "Error allocating memory to create task");

    	return ESP_ERR_NO_MEM;
    }

	/* Creater FreeRTOS software timer to avoid bounce button */
	me->timer = xTimerCreate("Test timer",
    		BUTTON_SHORT_TICKS,
    		pdFALSE,
    		(void *)me,
    		timer_handler); /* todo: implement error handler */

	/* Return error code */
	return ret;
}

esp_err_t button_register_cb(button_t *const me,
		button_press_e press_e,
		button_cb_t function,
		void * arg) {
	ESP_LOGI(TAG, "Registering a callback for button...");

	if(function == NULL && arg == NULL) {
		ESP_LOGI(TAG, "Error in function argument");
		return ESP_ERR_INVALID_ARG;
	}

	if(me->cbs == NULL) {
		ESP_LOGW(TAG, "No callback struct instance on this button. Skipping.");
		return ESP_ERR_NOT_SUPPORTED;
	}

	/* Select callback by press duration */
	switch(press_e) {
		case SHORT_PRESS:
			me->cbs->short_f.function = function;
			me->cbs->short_f.arg = arg;

			break;

		case MEDIUM_PRESS:
			me->cbs->medium_f.function = function;
			me->cbs->medium_f.arg = arg;

			break;

		case LONG_PRESS:
			me->cbs->long_f.function = function;
			me->cbs->long_f.arg = arg;

			break;

		default:
			ESP_LOGI(TAG, "Invalid time value");
			return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

esp_err_t button_unregister_cb(button_t * const me, button_press_e press_e) {
	ESP_LOGI(TAG, "Initializing button component...");

	if (me->cbs == NULL) {
		ESP_LOGW(TAG, "No callback struct instance on this button. Skipping.");
		return ESP_ERR_NOT_SUPPORTED;
	}

	/* Select callback by press duration and clear
	   button callback function and argument */
	switch(press_e) {
		case SHORT_PRESS:
			me->cbs->short_f.function = NULL;
			me->cbs->short_f.arg = NULL;

			break;

		case MEDIUM_PRESS:
			me->cbs->medium_f.function = NULL;
			me->cbs->medium_f.arg = NULL;

			break;

		case LONG_PRESS:
			me->cbs->long_f.function = NULL;
			me->cbs->long_f.arg = NULL;

			break;

		default:
			ESP_LOGI(TAG, "Error in arg argument");
			return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

esp_err_t re_button_init(button_t * me,
			gpio_num_t gpio,
			QueueHandle_t queue) {

	/* Error code variable */
	esp_err_t ret;

	ret = button_init(me, gpio, queue);

	if(ret != ESP_OK) {
		return ret;
	}

	/* Attach the function responsible for writing
	   the press event into the queue */
	me->event_fn = rotary_queue;

	return ESP_OK;
}

/* Private functions ---------------------------------------------------------*/
static void IRAM_ATTR isr_handler(void *arg) {
	TimerHandle_t timer = *(TimerHandle_t *)arg;

	ESP_DRAM_LOGD(TAG, "ISR ROUTINE CALLED");

	/* Start debounce timer */
	BaseType_t task_woken = pdFALSE;
	xTimerStartFromISR(timer, &task_woken);
	if (task_woken) {
		portYIELD_FROM_ISR();
	}
}

static void button_task(void *arg) {
	button_t * btn = (button_t *)arg;
	button_press_e event;

	while (pdTRUE == xQueueReceive(btn->queue, &event, portMAX_DELAY)) {
		switch (event) {
			case SHORT_PRESS:
				ESP_LOGI(TAG, "BUTTON_SHORT_PRESS_BIT set");
				// ESP_LOGI(TAG, "Button %d", btn->gpio);

				/* Execute callback function */
				if (btn->cbs->short_f.function != NULL) {
					btn->cbs->short_f.function(btn->cbs->short_f.arg);
				} else {
					ESP_LOGW(TAG, "Not defined callback function");
				}
				break;

			case MEDIUM_PRESS:
				ESP_LOGI(TAG, "BUTTON_MEDIUM_PRESS_BIT set");
				// ESP_LOGI(TAG, "Button %d", btn->gpio);

				/* Execute callback function */
				if (btn->cbs->medium_f.function != NULL) {
					btn->cbs->medium_f.function(btn->cbs->medium_f.arg);
				} else {
					ESP_LOGW(TAG, "Not defined callback function");
				}
				break;

			case LONG_PRESS:
				ESP_LOGI(TAG, "BUTTON_LONG_PRESS_BIT set");
				// ESP_LOGI(TAG, "Button %d", btn->gpio);

				/* Execute callback function */
				if (btn->cbs->long_f.function != NULL) {
					btn->cbs->long_f.function(btn->cbs->long_f.arg);
				} else {
					ESP_LOGW(TAG, "Not defined callback function");
				}
				break;

			default:
				ESP_LOGI(TAG, "Button unexpected event");
				break;
		}
	}
	/* An error ocurred while waiting for the queue. Cleaning up. */
	vQueueDelete(btn->queue);
	free(btn->cbs);
	vTaskDelete(NULL);
}

static void timer_handler(TimerHandle_t timer) {
	button_t *button = (button_t *)pvTimerGetTimerID(timer);

	if (button->state == NOT_PRESSED) {
		if (gpio_get_level(button->gpio) == PRESSED) {
			ESP_DRAM_LOGI(TAG, "PRESSED");
			button->state		= PRESSED;
			button->tick_counter = xTaskGetTickCount();
		}
	} else { /* state == PRESSED */
		if (gpio_get_level(button->gpio) == NOT_PRESSED) {
			ESP_DRAM_LOGI(TAG, "RELEASED");
			button->state = NOT_PRESSED;

			/* Calculate elapsed time pressed on ticks */
			TickType_t ticks = xTaskGetTickCount() - button->tick_counter;

			ESP_DRAM_LOGI(TAG, "Pressed for %d ms", pdTICKS_TO_MS(ticks));

			button->event_fn(button->queue, ticks);
		}
	}
}

static void default_queue(QueueHandle_t queue, TickType_t ticks) {
	button_press_e btn_event;

	if (ticks < BUTTON_MEDIUM_TICKS) {
		btn_event = SHORT_PRESS;
	} else if (ticks < BUTTON_LONG_TICKS) {
		btn_event = MEDIUM_PRESS;
	} else {
		btn_event = LONG_PRESS;
	}

	/* Copy the event to the queue */
	xQueueOverwrite(queue, &btn_event);
}

static void rotary_queue(QueueHandle_t queue, TickType_t ticks) {
	rotary_encoder_event_t queue_event = {.event_type = BUTTON_EVENT};

	if (ticks < BUTTON_MEDIUM_TICKS) {
		queue_event.btn_event = SHORT_PRESS;
	} else if (ticks < BUTTON_LONG_TICKS) {
		queue_event.btn_event = MEDIUM_PRESS;
	} else {
		queue_event.btn_event = LONG_PRESS;
	}

	/* Copy the event to the queue shared with a rotative encoder */
	xQueueOverwrite(queue, &queue_event);
}

/***************************** END OF FILE ************************************/
