
# ESP-IDF Button Component


## NEW: Rotary encoder button
- Added support for instantiating a rotary encoder button that will write press events to a FreeRTOS queue. That queue is shared with a rotary-encoder instance of the [esp32-rotary-encoder component](https://github.com/fherrera124/esp32-rotary-encoder). You can instantiate as many rotary encoder buttons as there are rotary encoders in your particular project.
- Added some MACRO definitions to maintain compatibility with the ESP8266-RTOS-SDK library.

## Features
- Each button can have three different callback functions depending on the time the button is pressed (short, medium and long)
- Debounce algorithm is based on FSM (Finite State Machine), FreeRTOS software timers, FreeRTOS event groups and GPIO interrupts.
- Multiple instances

## How to use
To use this component follow the next steps:

1. Configure the component in the project configuration menu (`idf.py menuconfig`)

`
Component config-->Button Configuration
`

2. Include the component header
```c
#include "button.h"
```
3. Create one or more instance of the component
```c
button_t button1;
button_t button2; 
```

4. Define button callback functions
```c
/* Callback function to handle press of button 1 without argument*/
void button1_cb(void * arg) {
    printf("Button 1 short press");
}

/* Callback function to handle press of button 2 with argument*/
void button2_cb(void * arg) {
    char * string = (char *)arg;
    printf("%s\n", string);
}
```

5. Initialize the component instances
```c
/* Initialize button instances */
ESP_ERROR_CHECK(button_init(&button1, /* button 1 instance */
    GPIO_NUM_0, /* GPIO number */
    tskIDLE_PRIORITY + 10, /* button FreeRTOS task priority */
    configMINIMAL_STACK_SIZE * 4)); /* button FreeRTOS task stack size */

ESP_ERROR_CHECK(button_init(&button2, /* button 2 instance */
    GPIO_NUM_21, /* GPIO number */
    tskIDLE_PRIORITY + 11, /* button FreeRTOS task priority */
    configMINIMAL_STACK_SIZE * 4)); /* button FreeRTOS task stack size */
```

6. Register the callback functions defined in 3
```c
 /* Register button1 callback for short press without argument */
button_register_cb(&button1, SHORT_TIME, button1_cb, NULL);
 
 /* Register button2 callbacks for medium and long press with different arguments */
button_register_cb(&button2, MEDIUM_TIME, button2_cb, "Button 2 medium press");
button_register_cb(&button2, LONG_TIME, button2_cb, "Button 2 long press");
```

## License
MIT License

Copyright (c) 2022 Mauricio Barroso Benavides

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

