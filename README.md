# M480BSP_PWD_WAKUP
 M480BSP_PWD_WAKUP

update @ 2020/12/04

1. Add deep power down mode , base on \M480BSP\SampleCode\StdDriver\DYS_SPDMode_Wakeup

2. check entry power mode , by #define USE_DPD , #define USE_SPD0 

3. check wake up source , by #define USE_RTC_TICK , #define USE_WAKEUP_TIMER

when under DPD mode , RTC_TICK wake up function only available for M48xGC/ M48xE8 , check RM Table 6.2-6 Re-Entering Power-down Mode Condition 

4. Add data flash emulate EEPROM sample code , to record wake up times from DPD mode

![image](https://github.com/released/M480BSP_PWD_WAKUP/blob/main/Wakeup_Timer_DPD.jpg)



update @ 2020/12/03

1. Base on \M480BSP\SampleCode\StdDriver\SYS_SPDMode_Wakeup

2. add timer , entry power down when power on 3 sec , and increase RAM address data

3. add UART (digit 1 , digit 2) , to entry power down (RTC tick , wake up timer)

4. Below is terminal screen capture after wake up from power on


RTC tick 1 sec

![image](https://github.com/released/M480BSP_PWD_WAKUP/blob/main/RTC_Tick.jpg)


wake up timer 

![image](https://github.com/released/M480BSP_PWD_WAKUP/blob/main/Wakeup_Timer.jpg)

