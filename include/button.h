/**
  ******************************************************************************
  * @file           : button.h
  * @author         : Mauricio Barroso Benavides
  * @date           : Mar 20, 2022
  * @brief          : This file contains all the definitions, data types and
  *                   function prototypes for button.c file
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUTTON_H_
#define BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

/* Exported types ------------------------------------------------------------*/
typedef void (* button_cb_t)(void *);

/**/
typedef struct {
	button_cb_t function;
	void * arg;
} button_function_t;

typedef struct {
	button_function_t short_;	/*!< Short press callback function structure */
	button_function_t medium_;	/*!< Medium press callback function structure */
	button_function_t long_f;	/*!< Long press callback function structure */
} f_cbs;

typedef enum {
	FALLING_MODE = 0,
	RISING_MODE
} button_mode_e;

/* Button states */
typedef enum {
	DOWN_STATE = 0,
	UP_STATE,
//	FALLING_STATE,
//	RISING_STATE
} button_state_e;

/**/
typedef enum {
	SHORT_PRESS = 0,
	MEDIUM_PRESS,
	LONG_PRESS
} button_press_e;

typedef void (*event_fn)(QueueHandle_t, TickType_t);

/* Struct definition of a button */
typedef struct {
	button_state_e state;		  /*!< Button state */
	button_mode_e mode;			  /*!< Button mode activation */
	gpio_num_t gpio;			    /*!< Button GPIO number */
	TickType_t tick_counter;	/*!< Tick counter */
	TimerHandle_t timer;		  /*!< Handle of the software timer, used for debouncing */
	QueueHandle_t queue;		  /*!< Handle of the event queue */
	f_cbs * cbs;				      /*!< Optional pointer to callback function struct*/
	event_fn event_handler;   /*!< Callback function responsible for
                                 writing button events into the queue */
} button_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Button */
#define BUTTON_SHORT_TICKS  pdMS_TO_TICKS(CONFIG_BUTTON_DEBOUNCE_SHORT_TIME)
#define BUTTON_MEDIUM_TICKS pdMS_TO_TICKS(CONFIG_BUTTON_DEBOUNCE_MEDIUM_TIME)
#define BUTTON_LONG_TICKS   pdMS_TO_TICKS(CONFIG_BUTTON_DEBOUNCE_LONG_TIME)

/* Next macros are for compatibility between esp-idf and ESP8266-RTOS SDKs */

/* Not (yet) defined in the ESP8266-RTOS SDK */
#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks) ((TickType_t)((uint64_t)(xTicks)*1000 / configTICK_RATE_HZ))
#endif

/* Diferent macros for printing inside an ISR routine */
#ifndef ESP_DRAM_LOGI
#define ESP_DRAM_LOGI ESP_EARLY_LOGI
#endif
#ifndef ESP_DRAM_LOGE
#define ESP_DRAM_LOGE ESP_EARLY_LOGE
#endif
#ifndef ESP_DRAM_LOGD
#define ESP_DRAM_LOGD ESP_EARLY_LOGD
#endif
#ifndef ESP_DRAM_LOGW
#define ESP_DRAM_LOGW ESP_EARLY_LOGW
#endif

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initialize a button instance
  *
  * @param me Pointer to button_t structure
  * @param gpio GPIO number to attach button
  * @param task_priority Button task priority
  * @param task_stack_size Button task stack size
  *
  * @retval
  * 	- ESP_OK on success
  * 	- ESP_ERR_INVALID_ARG if the argument is invalid
  */
esp_err_t button_init(button_t * const me,
		gpio_num_t gpio,
		UBaseType_t task_priority,
		uint32_t task_stack_size);

/**
  * @brief Register a button callback function
  *
  * @param me Pointer to button_t structure
  * @param press_e Button press event to register callback function
  * @param function Callback function code
  * @param arg Pointer to callback function argument
  *
  * @retval
  * 	- ESP_OK on success
  * 	- ESP_NO_MEM if is out of memory
  * 	- ESP_ERR_INVALID_ARG if the argument is invalid
  */
esp_err_t button_register_cb(button_t * const me,
		button_press_e press_e,
		button_cb_t function,
		void * arg);

/**
  * @brief Unregister a button callback function
  *
  * @param me Pointer to button_t structure
  * @param press_e Button press event to unregister callback function
  *
  *
  * @retval
  * 	- ESP_OK on success
  * 	- ESP_ERR_INVALID_ARG if the argument is invalid
  */
esp_err_t button_unregister_cb(button_t * const me, button_press_e press_e);

/**
  * @brief Initialize a button that interoperates with a rotary encoder.
  * See: https://github.com/fherrera124/esp32-rotary-encoder
  *
  * @param me Pointer to button_q_t structure
  * @param gpio GPIO number to attach button
  * @param queue Handle of the rotary encoder queue
  *
  * @retval
  * 	- ESP_OK on success
  * 	- ESP_ERR_INVALID_ARG if the argument is invalid
  */
esp_err_t re_button_init(button_t * const me,
    gpio_num_t gpio,
    QueueHandle_t queue);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H_ */

/***************************** END OF FILE ************************************/
