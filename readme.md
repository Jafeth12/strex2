# Microbit + FreeRTOS integration

## Download FreeRTOS

In vscode+platfromio, create a new project: 
    
    board = BBC micro:bit V2
    framework = Arduino

From FreeRTOS webpage https://www.freertos.org/a00104.html
download the current files. 

No need to download samples, so LTS version is preferred.

Current version is FreeRTOS 202210.01 LTS (v10.5.1).

Unzip the FreeRTOSv202210.01-LTS.zip file.

Copy the FreeRTOS-Kernel folder and move it to ../lib/FreeRTOS folder. It will be better to rename it as FreeRTOS.

Remove everything inside the ../portable folder except the files inside 
../FreeRTOSv202210.01-LTS/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F folder.

Copy port.c and potmacro.h into the ../lib/FreeRTOs/portable folder.

Copy the ../MemMang folder and move it to ../lib/FreeRTOs/portable folder.

Somewhere, somehow the FreeRTOSConfig.h fie appears. It is important (and mandatory) to change:
        
    `#define configTICK_SOURCE FREERTOS_USE_SYSTICK //STR FREERTOS_USE_RTC`

because RTC is used by delay funcion in Arduino framework. The compilation shows 
`multiple definition of RTC1_IRQHandler`

Create some tasks, code them and start the scheduler.

Note: Comment line 94, 95, 96 and 97 in nrf.h by clicking on Arduino.h and then in line 10 clicking on nrf.h. By doing this, the microbit v2 pinout is correct


