clear all;
SystemCoreClock=64000000%From Serial.println(SystemCoreClock);
configSYSTICK_CLOCK_HZ=SystemCoreClock
configTICK_RATE_HZ=100;
portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );% - 1
tick2milliseconds=1/configTICK_RATE_HZ*1000/portNVIC_SYSTICK_LOAD_REG
vpa(tick2milliseconds,20)

