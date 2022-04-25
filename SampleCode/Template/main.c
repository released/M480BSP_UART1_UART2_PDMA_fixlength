/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

#define ENABLE_UART1
#define ENABLE_UART2

#define UART1_RX_DMA_CH 		    (0)
#define UART2_RX_DMA_CH 		    (1)
#define UART_PDMA_OPENED_CH   	    ((1 << UART1_RX_DMA_CH) | (1 << UART2_RX_DMA_CH))

#define UART1_PDMA_OPENED_CH_RX   	(1 << UART1_RX_DMA_CH)
#define UART2_PDMA_OPENED_CH_RX   	(1 << UART2_RX_DMA_CH)

#define PDMA_TIME                   (0x100)
#define RXBUFSIZE                   (64)

uint8_t UART1_RxBuffer[RXBUFSIZE] = {0};
uint8_t UART2_RxBuffer[RXBUFSIZE] = {0};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void UART2_RX_PDMA_set(void)
{
    //RX	
    PDMA_SetTransferCnt(PDMA,UART2_RX_DMA_CH, PDMA_WIDTH_8, RXBUFSIZE);
    PDMA_SetTransferAddr(PDMA,UART2_RX_DMA_CH, UART2_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART2_RxBuffer[0])), PDMA_DAR_INC);	
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART2_RX_DMA_CH, PDMA_UART2_RX, FALSE, 0);
      
}

void UART1_RX_PDMA_set(void)
{
    //RX	
    PDMA_SetTransferCnt(PDMA,UART1_RX_DMA_CH, PDMA_WIDTH_8, RXBUFSIZE);
    PDMA_SetTransferAddr(PDMA,UART1_RX_DMA_CH, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART1_RxBuffer[0])), PDMA_DAR_INC);	
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART1_RX_DMA_CH, PDMA_UART1_RX, FALSE, 0);
     
}


void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        printf("target abort interrupt !!:\r\n");
		#if 0
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else

        #if defined (ENABLE_UART1)            
        if (PDMA_GET_ABORT_STS(PDMA) & (UART1_PDMA_OPENED_CH_RX))
        {
            printf("UART1_PDMA_OPENED_CH_RX\r\n");
            PDMA_CLR_ABORT_FLAG(PDMA, (UART1_PDMA_OPENED_CH_RX));
        }
        #endif
        
        #if defined (ENABLE_UART2) 
        if (PDMA_GET_ABORT_STS(PDMA) & (UART2_PDMA_OPENED_CH_RX))
        {
            printf("UART2_PDMA_OPENED_CH_RX\r\n");
            PDMA_CLR_ABORT_FLAG(PDMA, (UART2_PDMA_OPENED_CH_RX));
        }
        #endif
        
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
		#if 1

        #if defined (ENABLE_UART1)            
        if (PDMA_GET_TD_STS(PDMA) & UART1_PDMA_OPENED_CH_RX)
        {
            PDMA_CLR_TD_FLAG(PDMA, UART1_PDMA_OPENED_CH_RX);
        } 
        #endif

        #if defined (ENABLE_UART2) 
        if (PDMA_GET_TD_STS(PDMA) & UART2_PDMA_OPENED_CH_RX)
        {
            PDMA_CLR_TD_FLAG(PDMA, UART2_PDMA_OPENED_CH_RX);
        } 
        #endif
		
		#else
        if((PDMA_GET_TD_STS(PDMA) & UART_PDMA_OPENED_CH) == UART_PDMA_OPENED_CH)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, UART_PDMA_OPENED_CH);
			//insert process
			/*
                DISABLE TRIGGER
            */

        } 
		#endif
    }
   #if defined (ENABLE_UART1)     
    else if (status & (PDMA_INTSTS_REQTOF0_Msk))     /* Check the DMA time-out interrupt flag */
    {
        // printf("UART1_RX time-out !!\r\n");
        /* Update receive count */

        // PDMA_SET_TRANS_CNT(PDMA, UART1_RX_DMA_CH,1); 
        /* restart timeout */
        PDMA_SetTimeOut(PDMA, UART1_RX_DMA_CH, DISABLE, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA, UART1_RX_DMA_CH);
        PDMA_SetTimeOut(PDMA, UART1_RX_DMA_CH, ENABLE, PDMA_TIME);    
        set_flag(flag_UART1_RX_end,ENABLE);                             
    }
    #endif
    #if defined (ENABLE_UART2)     
    else if (status & (PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        // printf("UART2_RX time-out !!\r\n");
        /* Update receive count */
        
        // PDMA_SET_TRANS_CNT(PDMA, UART2_RX_DMA_CH,1);    
        /* restart timeout */
        PDMA_SetTimeOut(PDMA, UART2_RX_DMA_CH, DISABLE, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA, UART2_RX_DMA_CH);
        PDMA_SetTimeOut(PDMA, UART2_RX_DMA_CH, ENABLE, PDMA_TIME);    
        set_flag(flag_UART2_RX_end,ENABLE);                   
    }    
    #endif    
    else
    {
        printf("unknown interrupt !!\r\n");
    }	
}

/*
    UART1 RX :PA2
    UART2 RX :PB0
*/
void UART1_UART2_Init(void)
{
    #if defined (ENABLE_UART1)     
    SYS_ResetModule(UART1_RST);
    UART_Open(UART1, 115200);
    UART_PDMA_ENABLE(UART1,UART_INTEN_RXPDMAEN_Msk);    
    #endif

    #if defined (ENABLE_UART2)     
    SYS_ResetModule(UART2_RST);    
    UART_Open(UART2, 115200);
    UART_PDMA_ENABLE(UART2,UART_INTEN_RXPDMAEN_Msk);    
    #endif
}

void UART_PDMA_Init(void)
{
	set_flag(flag_UART1_RX_end,DISABLE);
	set_flag(flag_UART2_RX_end,DISABLE);

    SYS_ResetModule(PDMA_RST);

    #if defined (ENABLE_UART1)    
    PDMA_Open(PDMA, UART1_PDMA_OPENED_CH_RX);

    PDMA_SetTransferCnt(PDMA,UART1_RX_DMA_CH, PDMA_WIDTH_8, RXBUFSIZE);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART1_RX_DMA_CH, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART1_RxBuffer[0])), PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART1_RX_DMA_CH, PDMA_UART1_RX, FALSE, 0);
    /* Single request type. */
    PDMA_SetBurstType(PDMA,UART1_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_DisableInt(PDMA,UART1_RX_DMA_CH, PDMA_INT_TEMPTY );//PDMA->DSCT[UART1_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;  

    PDMA_EnableInt(PDMA, UART1_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, UART1_RX_DMA_CH, PDMA_INT_TIMEOUT);

    // PDMA->TOUTPSC = (PDMA->TOUTPSC & (~PDMA_TOUTPSC_TOUTPSC0_Msk)) | (0x5 << PDMA_TOUTPSC_TOUTPSC0_Pos);
    PDMA_SetTimeOut(PDMA,UART1_RX_DMA_CH, ENABLE, PDMA_TIME );

    #endif

    #if defined (ENABLE_UART2)     
    PDMA_Open(PDMA, UART2_PDMA_OPENED_CH_RX);

    PDMA_SetTransferCnt(PDMA,UART2_RX_DMA_CH, PDMA_WIDTH_8, RXBUFSIZE);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART2_RX_DMA_CH, UART2_BASE, PDMA_SAR_FIX, ((uint32_t) (&UART2_RxBuffer[0])), PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,UART2_RX_DMA_CH, PDMA_UART2_RX, FALSE, 0);
    /* Single request type. */
    PDMA_SetBurstType(PDMA,UART2_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA_DisableInt(PDMA,UART2_RX_DMA_CH, PDMA_INT_TEMPTY );//PDMA->DSCT[UART2_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;  

    PDMA_EnableInt(PDMA, UART2_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, UART2_RX_DMA_CH, PDMA_INT_TIMEOUT); 

    // PDMA->TOUTPSC = (PDMA->TOUTPSC & (~PDMA_TOUTPSC_TOUTPSC1_Msk)) | (0x5 << PDMA_TOUTPSC_TOUTPSC1_Pos);
    PDMA_SetTimeOut(PDMA,UART2_RX_DMA_CH, ENABLE, PDMA_TIME );     
    #endif
    
     /*
        UART data freq : 1.6KHz	0.625	ms

        Set PDMA CH 0/1 timeout to about 
        2 ms (5/(72M/(2^15)))
        0.56 ms (5/(72M/(2^13)))

        target (ms)	    u32TimeOutCnt	clk div	    prescale	
        2.275555556	    5	            72000000	32768	15      111
        1.137777778	    5	            72000000	16384	14      110
        0.568888889	    5	            72000000	8192	13      101
        0.284444444	    5	            72000000	4096	12      100
        0.142222222	    5	            72000000	2048	11      011
        0.071111111	    5	            72000000	1024	10      010
        0.035555556	    5	            72000000	512	    9       001
        0.017777778	    5	            72000000	256	    8       000  

    */

    NVIC_EnableIRQ(PDMA_IRQn);
}


void loop(void)
{
    uint32_t i = 0;

    /* Add data processing code */

    if (is_flag_set(flag_UART1_RX_end))
    {
        set_flag(flag_UART1_RX_end, DISABLE);

        #if 1
        printf("UART1_RX : \r\n");
        for (i = 0 ; i < strlen((char*)UART1_RxBuffer); i++)
        {
            printf("%c[0x%2X],",UART1_RxBuffer[i],UART1_RxBuffer[i]);
            if ((i+1)%8 ==0)
            {
                printf("\r\n");
            }              
        }
        printf("\r\n");
        #else
        printf("%s\r\n",UART1_RxBuffer);
        #endif

        reset_buffer(UART1_RxBuffer,0x00,RXBUFSIZE);  

        UART1_RX_PDMA_set();        
    }

    if (is_flag_set(flag_UART2_RX_end))
    {
        set_flag(flag_UART2_RX_end, DISABLE);

        printf("UART2_RX : \r\n");
        printf("%s\r\n",UART2_RxBuffer);

        reset_buffer(UART2_RxBuffer,0x00,RXBUFSIZE);  

        UART2_RX_PDMA_set();   
    }
}

void TMR1_IRQHandler(void)
{
	// static uint32_t LOG = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
			// printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PH0 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
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

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif
}

void Custom_Init(void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    #if defined (ENABLE_UART1) 
    CLK_EnableModuleClock(UART1_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    #endif

    #if defined (ENABLE_UART2) 
    CLK_EnableModuleClock(UART2_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    #endif

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    #if defined (ENABLE_UART1) 
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA2MFP_UART1_RXD;
    #endif

    #if defined (ENABLE_UART2) 
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_UART2_RXD;    
    #endif

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

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
    SYS_Init();

	UART0_Init();
	Custom_Init();	
	TIMER1_Init();

    UART_PDMA_Init();
    UART1_UART2_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
