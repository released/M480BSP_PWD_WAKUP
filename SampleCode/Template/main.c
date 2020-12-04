/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "EEPROM_Emulate.h"

#define LED_R									(PH0)
#define LED_Y									(PH1)
#define LED_G									(PH2)

#define SIGNATURE      	 						(0x125ab234)
#define FLAG_ADDR       						(0x20001000)

//#define USE_DPD
#define USE_SPD0

//#define USE_RTC_TICK
#define USE_WAKEUP_TIMER

#define DATA_FLASH_AMOUNT						(24)
#define DATA_FLASH_PAGE  						(2)


#define EEP_ADDR_WAKEUP     					(0x05)	// max : DATA_FLASH_AMOUNT 

typedef enum{
	flag_DEFAULT = 0 ,

	flag_entry_power_down , 
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint32_t conter_tick = 0;

/*----------------------------------------------------------------------------*/


void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void set_tick(uint32_t t)
{
	conter_tick = t;
}


void Emulate_EEPROM_Write(uint8_t idx , uint8_t value)
{
    SYS_UnlockReg();
    FMC_Open();
	
	Write_Data(idx,value);

    FMC_Close();
    SYS_LockReg();
}

void Emulate_EEPROM_RecordTimes(void)
{
	uint8_t cnt = 0;
	
    SYS_UnlockReg();
    FMC_Open();

	Read_Data(EEP_ADDR_WAKEUP , &cnt);
	printf("addr : 0x%4X , value : 0x%X\r\n" , EEP_ADDR_WAKEUP ,cnt);

	cnt++;
	Write_Data(EEP_ADDR_WAKEUP , cnt );

    FMC_Close();
    SYS_LockReg();		
}

// need to set data flash index in ICP tool , ex : 0x1E000
void Emulate_EEPROM_Init(void)
{
	Init_EEPROM(DATA_FLASH_AMOUNT, DATA_FLASH_PAGE);
	Search_Valid_Page();	
}


void CheckPowerSource(void)
{

    uint32_t u32RegRstsrc;
    u32RegRstsrc = CLK_GetPMUWKSrc();

    printf("Power manager Power Manager Status 0x%x\r\n", u32RegRstsrc);

    if((u32RegRstsrc & CLK_PMUSTS_RTCWK_Msk) != 0)
    {
        printf("Wake-up source is RTC.\r\n");

		#if defined (USE_DPD)
//		Emulate_EEPROM_RecordTimes();
		
		#elif defined (USE_SPD0)
		M32(FLAG_ADDR) += 1;
//		printf("0x%4X\r\n" , M32(FLAG_ADDR));
		printf("addr : 0x%4X , value : 0x%X\r\n" , FLAG_ADDR ,M32(FLAG_ADDR));	
		#endif		
    }
    if((u32RegRstsrc & CLK_PMUSTS_TMRWK_Msk) != 0)
    {
        printf("Wake-up source is Wake-up Timer.\r\n");

		#if defined (USE_DPD)
		Emulate_EEPROM_RecordTimes();
		
		#elif defined (USE_SPD0)
		M32(FLAG_ADDR) += 1;
//		printf("0x%4X\r\n" , M32(FLAG_ADDR));
		printf("addr : 0x%4X , value : 0x%X\r\n" , FLAG_ADDR ,M32(FLAG_ADDR));	
		#endif		

		
    }
    if((u32RegRstsrc & CLK_PMUSTS_PINWK_Msk) != 0)
        printf("Wake-up source is Wake-up Pin.\r\n");

    /* Clear all wake-up flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;
	
}

void WakeUpRTCTickFunction(uint32_t u32PDMode)
{
    SYS_UnlockReg();

	printf("entry power down (RTCTick) \r\n" );

    /* Check if all the debug messages are finished */
    while(!UART_IS_TX_EMPTY(UART0));


    /* enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
    /* RTC clock source select 32KHz */
    CLK->CLKSEL3 &= ~(CLK_CLKSEL3_RTCSEL_Msk);

    /* Open RTC and start counting */
    RTC->INIT = RTC_INIT_KEY;

    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;
        while(RTC->INIT != RTC_INIT_ACTIVE_Msk);
    }

    RTC_WaitAccessEnable();
    /* clear tick status */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC Tick interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_WaitAccessEnable();

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Wait for Power-down mode wake-up reset happen */
    while(1);
}

void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
	SYS_UnlockReg();

	printf("entry power down (WakeUpTimer) \r\n" );

    /* Check if all the debug messages are finished */
    while(!UART_IS_TX_EMPTY(UART0));
	

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

	#if defined (USE_DPD)
	if (u32PDMode == CLK_PMUCTL_PDMSEL_DPD)
	{
	    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
	    /* Waiting for LIRC clock ready */
	    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	
	}
	#endif


    /* Set Wake-up Timer Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode */
    CLK_PowerDown();

    while(1);
}

void Loop_Process(void)
{

	LED_Y ^= 1;

	if (is_flag_set(flag_entry_power_down))
	{
		set_flag(flag_entry_power_down , DISABLE);

		#if defined (USE_RTC_TICK)

		#if defined (USE_DPD)		// only available for (M48xGC/ M48xE8 , check RM Table 6.2-6 Re-Entering Power-down Mode Condition 
		WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_DPD);
		#elif defined (USE_SPD0)
		WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_SPD0);
		#endif		

		#endif	/*USE_RTC_TICK*/

		#if defined (USE_WAKEUP_TIMER)

		#if defined (USE_DPD)
		WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_PMUCTL_WKTMRIS_16384);	
		#elif defined (USE_SPD0)
		WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_SPD0, CLK_PMUCTL_WKTMRIS_16384);	
		#endif
		
		#endif	/*USE_WAKEUP_TIMER*/	
	}
	
}


void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':
				#if defined (USE_DPD)
				WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_DPD);
				#elif defined (USE_SPD0)
				WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_SPD0);
				#endif		
			
				break;	

			case '2':
				#if defined (USE_DPD)
				WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_PMUCTL_WKTMRIS_16384);	
				#elif defined (USE_SPD0)
				WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_SPD0, CLK_PMUCTL_WKTMRIS_16384);	
				#endif
			
				break;

			case '3':
			
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':

				#if defined (USE_DPD)
				Emulate_EEPROM_Write(EEP_ADDR_WAKEUP , 0x00);
				#elif defined (USE_SPD0)
				M32(FLAG_ADDR) = 0;
				#endif
			
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}



void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 3000) == 0)
		{
			set_flag(flag_entry_power_down , ENABLE);
		}
		
		if (CNT++ > 1000)
		{		
			CNT = 0;
			printf("%s : %2d\r\n" , __FUNCTION__ , log++);

			//Emulate_EEPROM_RecordTimes();	// for test
			
			LED_R ^= 1;
		}
		
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_LIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void GpioPinSetting(void)
{
    /* Set function pin to GPIO mode */
    SYS->GPA_MFPH = 0;
    SYS->GPA_MFPL = 0;
    SYS->GPB_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPD_MFPH = 0;
    SYS->GPD_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPF_MFPH = 0;
    SYS->GPF_MFPL = 0x000000EE; //ICE pin
    SYS->GPG_MFPH = 0;
    SYS->GPG_MFPL = 0;
    SYS->GPH_MFPH = 0;
    SYS->GPH_MFPL = 0;

    /* Set all GPIOs are output mode */
    PA->MODE = 0x55555555;
    PB->MODE = 0x55555555;
    PC->MODE = 0x55555555;
    PD->MODE = 0x55555555;
    PE->MODE = 0x55555555;
    PF->MODE = 0x55555555;
    PG->MODE = 0x55555555;
    PH->MODE = 0x55555555;

    /* Set all GPIOs are output high */
    PA->DOUT = 0xFFFFFFFF;
    PB->DOUT = 0xFFFFFFFF;
    PC->DOUT = 0xFFFFFFFF;
    PD->DOUT = 0xFFFFFFFF;
    PE->DOUT = 0xFFFFFFFF;
    PF->DOUT = 0xFFFFFFFF;
    PG->DOUT = 0xFFFFFFFF;
    PH->DOUT = 0xFFFFFFFF;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable HIRC clock (Internal RC 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

//	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

    if ((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x0) // M480MD
    {
        /* Set IO State and all IPs clock disable for power consumption */
        GpioPinSetting();
    }

    CLK->APBCLK1 = 0x00000000;
    CLK->APBCLK0 = 0x00000000;

	/* ---------- Turn off RTC  -------- */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
    RTC_WaitAccessEnable();
    RTC->INTEN = 0;
    RTC_Close();	

    SYS_Init();

	UART0_Init();

	#if defined (USE_DPD)
	Emulate_EEPROM_Init();
	#endif
	
    CheckPowerSource();

	LED_Init();
	TIMER1_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(100);
		Loop_Process();

    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
