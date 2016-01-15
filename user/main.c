#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "string.h"
#include "stdio.h"
#include "command.h"
#include "stm32f10x_flash.h"




/**************************************
*
*
*
*************************************/

// the beginning address of the user's application 
#define APP_DEFAULT_ADD (0x08000000 +  0x5000)

#define PAGE_SIZE   2048       //Ò»ï¿½ï¿½page ï¿½ï¿½2048ï¿½ï¿½ï¿½Ö½ï¿½



typedef void (*pFunction)(void);      // typedef a function type for jump into the user application.

void usart1Init(void);




/***********************************
*global is defined for uart1
*
*
*
*************************************/

uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        64  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

volatile uint8_t gRxBuf[300]/* = {0}*/;
volatile uint8_t *gpu8RxBuf = gRxBuf;
volatile uint32_t gu32RxSize = 0;
volatile uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;
volatile int	upload = 0;
//cortex-m0
//#define SIGNATURE_BYTES 0x1E900A

//mega2560
#define SIGNATURE_BYTES 0x1E9801
#define APP_OFFSET (1024*12)

u8 msgParseState=ST_START;
u32		left_address = 0, address	=	0;
uint8_t iswrite = 0, isread = 0;
u32		eraseAddress	=	0;
unsigned char	msgParseState;
unsigned int	ii				=	0;
unsigned char	checksum		=	0;
unsigned char	seqNum			=	0;
unsigned int	msgLength		=	0;
unsigned char	msgBuffer[285];
//unsigned char	tmpBuffer[285];
unsigned char	c, *p;
unsigned char   isLeave = 0;

typedef unsigned int address_t;

#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A


unsigned char pageCache[512];
unsigned char pageEnd = 0;
extern volatile int upload;


int USB_Read(u8 *buf)
{
	int ret=0;
	//if(gi8BulkOutReady){
		memcpy((void *)buf,(const void*)gpu8RxBuf,(unsigned int)gu32RxSize);
		ret = gu32RxSize;
		gu32RxSize = 0;
		//gi8BulkOutReady = 0;
	
	return ret;
}

void USB_Write(const u8 *buf,int len)
{
	int i;
	for(i=0;i<len;i++){
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
          USART_SendData(USART1, buf[i]);
	}
}

void getPacket(void)
{
	int total = 0,i,len;
	u8 checksum = 0;
    
//	    start    seqno    size1  size2  token  data   checksum
//     . [1b]   . [01]  . [00]  . [01] . [0e] . [01]   . [14] 
    
    
	while(msgParseState	!=	ST_PROCESS){
		//total += USB_Read(msgBuffer+total);
		total = gu32RxSize;
		len = (gpu8RxBuf[2]<<8) + gpu8RxBuf[3]+6;
			if(len != 0 && len == total){   // the complete frame has received now.
				//check sum the received frame.
                
				for(i=0;i<total-1;i++){
                    
					checksum ^= gpu8RxBuf[i];
				}
				if(checksum == gpu8RxBuf[total-1]){
					seqNum = gpu8RxBuf[1];
					msgParseState	=	ST_PROCESS;
					//memcpy(msgBuffer,msgBuffer+5, (msgBuffer[2]<<8) + msgBuffer[3]);
					if(seqNum == 6)
						seqNum = 6;
					for(i=0;i<len - 6;i++){
						msgBuffer[i] = gpu8RxBuf[i+5];
					}
				}else{    //the frame is error and the gu32RxSize should be zero.
					msgParseState = ST_START;
					
				}
                // must be set as zero here.
                gu32RxSize = 0;
			}
			else{
				continue;
			}
		}// end if(total>=6)
}// end while(msgParseState	!=	ST_PROCESS)
  

















uint16_t Read(uint32_t Addr)
{

    return *(uint16_t *)(Addr);

}


void  Readbuf(uint32_t Addr, void *data, uint32_t NumByteToWrite)
{

 

    memcpy(data, (void *)(Addr), NumByteToWrite);

  

}


FLASH_Status Write(uint32_t WriteAddr, uint16_t data)
{

    FLASH_Status state;

    assert_param(IS_FLASH_ADDRESS(WriteAddr));

   // FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);	

    state = FLASH_ProgramHalfWord(WriteAddr, data);

    //FLASH_Lock();

    return state;

}

unsigned char Buffer[2048];


static FLASH_Status WriteBuffer(uint32_t WriteAddr, void *data, uint32_t len)
{
    char readbuf[300];
    FLASH_Status state;
    int i;
    static int flag = 1;
    
    if (flag){
                for (i = 0; i < 20; i++){
               
                FLASH_Status status = FLASH_ErasePage(APP_DEFAULT_ADD + i * 2048);
                if  (FLASH_COMPLETE != status){
                    i++;
                    break;
                
                }
             
             }
             
       flag = 0;
   
    }

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);	
   // FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | 
    //              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
    for (i = 0; i <((len + 1) / 2) ; i += 1)
    {
        state = FLASH_ProgramHalfWord(WriteAddr + i * 2, ((uint16_t *)data)[i]);
        
        
        if (state != FLASH_COMPLETE)
        {
            i++;
            break;

        }
    }
    //read back
    
    memcpy(readbuf, (void *)WriteAddr, len);
    if (memcmp(readbuf, data, len)){
       //printf("ok");
      i++;
    }

    //FLASH_Lock();


    return state;



}


uint32_t Writebuf(uint32_t WriteAddr, void *data, uint32_t NumByteToWrite)
{

    uint32_t pageRemain, pageAddr, pageOff, pagepos; 
    char *data1 = (char *)data;

    pageRemain = PAGE_SIZE - (WriteAddr - APP_DEFAULT_ADD) % PAGE_SIZE; //ï¿½ï¿½Ò³Ê£ï¿½ï¿½ï¿½ï¿½Ö½ï¿½ï¿½ï¿?	

    pageAddr = (((WriteAddr - APP_DEFAULT_ADD) / PAGE_SIZE ) * PAGE_SIZE) + APP_DEFAULT_ADD; //WriteAddr ï¿½ï¿½ï¿½ï¿½Ò³ï¿½ï¿½Ò³ï¿½ï¿½Ö·ï¿½ï¿½
    pageOff = WriteAddr - pageAddr;
    pagepos = (WriteAddr - APP_DEFAULT_ADD) / PAGE_SIZE;

    if (NumByteToWrite <= pageRemain)
        pageRemain = NumByteToWrite;

    while(1)
    {	 
        int i = 0;

        Readbuf(pagepos * PAGE_SIZE + APP_DEFAULT_ADD , Buffer, PAGE_SIZE); //ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½pageï¿½ï¿½ï¿½ï¿½ï¿½Ý¡ï¿½

        for ( i = 0; i < pageRemain; i += 1)
        {
            if (Buffer[i + pageOff] != 0xFF)
                break;
        }

        if (i < pageRemain)
        {
            FLASH_Status status = FLASH_ErasePage(pageAddr);
            for (i = 0; i < pageRemain; i += 1)
                Buffer[i + pageOff] = data1[i];


            WriteBuffer(( pagepos * PAGE_SIZE + APP_DEFAULT_ADD), Buffer, PAGE_SIZE);
        }
        else
        {

            WriteBuffer((pageOff + pagepos * PAGE_SIZE + APP_DEFAULT_ADD), data1, pageRemain);


        }

        if(NumByteToWrite == pageRemain)
            break;//Ð´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
        else //NumByteToWrite>pageremain
        {

            pageOff = 0;
            pagepos++;
            pageAddr = pagepos * PAGE_SIZE + APP_DEFAULT_ADD;
            data1 += pageRemain;
            WriteAddr +=pageRemain; 
            NumByteToWrite -=pageRemain;	  //ï¿½ï¿½È¥ï¿½Ñ¾ï¿½Ð´ï¿½ï¿½ï¿½Ëµï¿½ï¿½Ö½ï¿½ï¿½ï¿½
            if(NumByteToWrite > PAGE_SIZE)
                pageRemain = PAGE_SIZE; //ï¿½Î¿ï¿½ï¿½ï¿½Ð´ï¿½ï¿½2048ï¿½ï¿½ï¿½Ö½ï¿½
            else 
                pageRemain = NumByteToWrite;	 //ï¿½ï¿½ï¿½ï¿½2048ï¿½ï¿½ï¿½Ö½ï¿½ï¿½ï¿½
        }

    }   

    return 0;
}






void WritePage(uint32_t addr,void* buf,uint32_t len)
{
  //Writebuf(addr, buf, len);
  WriteBuffer(addr,buf, len);
}

void Protocol_Deal(void){






}
volatile int signOnCounter=0;

void VCOM_TransferData(void)
{
	while(!isLeave){
		getPacket();
		while(msgParseState	!=	ST_PROCESS);
        __disable_irq();
		upload = 1;
		switch (msgBuffer[0]){
			case CMD_SPI_MULTI:{
				unsigned char answerByte;
				unsigned char flag=0;

				if ( msgBuffer[4]== 0x30 ){
							unsigned char signatureIndex	=	msgBuffer[6];

							if ( signatureIndex == 0 ){
								answerByte	=	(SIGNATURE_BYTES >> 16) & 0x000000FF;
							}else if ( signatureIndex == 1 ){
								answerByte	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
							}else{
								answerByte	=	SIGNATURE_BYTES & 0x000000FF;
							}
						}else if ( msgBuffer[4] & 0x50 ){
							if (msgBuffer[4] == 0x50){
								answerByte	=	0xa0;//boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
							}else if (msgBuffer[4] == 0x58){
								answerByte	=	0xa0;//boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
							}else{
								answerByte	=	0;
							}
						}else{
							answerByte	=	0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
						}
						if ( !flag ){
							msgLength		=	7;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	0;
							msgBuffer[3]	=	msgBuffer[4];
							msgBuffer[4]	=	0;
							msgBuffer[5]	=	answerByte;
							msgBuffer[6]	=	STATUS_CMD_OK;
						}
					}
					break;

				case CMD_SIGN_ON:
                    signOnCounter++;
					msgLength		=	11;
					msgBuffer[1] 	=	STATUS_CMD_OK;
					msgBuffer[2] 	=	8;
					msgBuffer[3] 	=	'A';msgBuffer[4] 	=	'V';
					msgBuffer[5] 	=	'R';msgBuffer[6] 	=	'I';
					msgBuffer[7] 	=	'S';msgBuffer[8] 	=	'P';
					msgBuffer[9] 	=	'_';msgBuffer[10]	=	'2';
					break;

				case CMD_GET_PARAMETER:
					{
						unsigned char value;

						switch(msgBuffer[1]){
						case PARAM_BUILD_NUMBER_LOW:
							value	=	CONFIG_PARAM_BUILD_NUMBER_LOW;
							break;
						case PARAM_BUILD_NUMBER_HIGH:
							value	=	CONFIG_PARAM_BUILD_NUMBER_HIGH;
							break;
						case PARAM_HW_VER:
							value	=	CONFIG_PARAM_HW_VER;
							break;
						case PARAM_SW_MAJOR:
							value	=	CONFIG_PARAM_SW_MAJOR;
							break;
						case PARAM_SW_MINOR:
							value	=	CONFIG_PARAM_SW_MINOR;
							break;
						default:
							value	=	0;
							break;
						}
                        signOnCounter;
						msgLength		=	3;
						msgBuffer[1]	=	STATUS_CMD_OK;
						msgBuffer[2]	=	value;
					}
					break;

				case CMD_LEAVE_PROGMODE_ISP:
					isLeave	=	1;
				case CMD_SET_PARAMETER:
				case CMD_ENTER_PROGMODE_ISP:
					msgLength		=	2;
					msgBuffer[1]	=	STATUS_CMD_OK;
					break;

				case CMD_READ_SIGNATURE_ISP:
					{
						unsigned char signatureIndex	=	msgBuffer[4];
						unsigned char signature;

						if ( signatureIndex == 0 )
							signature	=	(SIGNATURE_BYTES >>16) & 0x000000FF;
						else if ( signatureIndex == 1 )
							signature	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
						else
							signature	=	SIGNATURE_BYTES & 0x000000FF;

						msgLength		=	4;
						msgBuffer[1]	=	STATUS_CMD_OK;
						msgBuffer[2]	=	signature;
						msgBuffer[3]	=	STATUS_CMD_OK;
					}
					break;

				case CMD_READ_LOCK_ISP:
					msgLength		=	4;
					msgBuffer[1]	=	STATUS_CMD_OK;
					//msgBuffer[2]	=	boot_lock_fuse_bits_get( GET_LOCK_BITS );
					msgBuffer[3]	=	STATUS_CMD_OK;
					break;

				case CMD_READ_FUSE_ISP:
					{
						unsigned char fuseBits=0;

						msgLength		=	4;
						msgBuffer[1]	=	STATUS_CMD_OK;
						msgBuffer[2]	=	fuseBits;
						msgBuffer[3]	=	STATUS_CMD_OK;
					}
					break;

				case CMD_PROGRAM_LOCK_ISP:
					{
						unsigned char lockBits	=	msgBuffer[4];

						lockBits	=	(~lockBits) & 0x3C;	// mask BLBxx bits
						//boot_lock_bits_set(lockBits);		// and program it
						//boot_spm_busy_wait();

						msgLength		=	3;
						msgBuffer[1]	=	STATUS_CMD_OK;
						msgBuffer[2]	=	STATUS_CMD_OK;
					}
					break;

				case CMD_CHIP_ERASE_ISP:
					eraseAddress	=	0;
					msgLength		=	2;
				//	msgBuffer[1]	=	STATUS_CMD_OK;
					msgBuffer[1]	=	STATUS_CMD_FAILED;	//*	isue 543, return FAILED instead of OK
					break;

				case CMD_LOAD_ADDRESS:
                 
                  
					address	=	( ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]) );
					address &= 0x00ffffff;
					address <<= 1;
					address += APP_DEFAULT_ADD;
                    
                 /*
                     if (address == 0){
                         if (iswrite == 0){
                              iswrite = 1;
                              isread = 0;

                         }else{
                              isread = 1;
                              iswrite = 0;
                         }   
                              
                     }
                    
                    
                    if (iswrite == 1){
                    
                     
                      address = APP_DEFAULT_ADD + 256 * count_write++;
                    
                    }
                    if (isread == 1){
                    
                       address = APP_DEFAULT_ADD + 256 * count_read++;
                    }
                   */
                    
					msgLength		=	2;
					msgBuffer[1]	=	STATUS_CMD_OK;
					break;

				case CMD_PROGRAM_FLASH_ISP:
				case CMD_PROGRAM_EEPROM_ISP:
					{
                      
						unsigned int	size	=	((msgBuffer[1])<<8) | msgBuffer[2];
						unsigned char	*p	=	msgBuffer+10;
                     				
					WritePage(address, p,size);
						
					msgLength		=	2;
					msgBuffer[1]	=	STATUS_CMD_OK;
                    }
					break;

				case CMD_READ_FLASH_ISP:
				case CMD_READ_EEPROM_ISP:
					{
						unsigned int	size	=	((msgBuffer[1])<<8) | msgBuffer[2];
						unsigned char	*p		=	msgBuffer+1;
						msgLength				=	size+3;
						
						*p++	=	STATUS_CMD_OK;
						if (msgBuffer[0] == CMD_READ_FLASH_ISP )
						{
							memcpy(p,(void*)(address),size);
							p+=size;
							//address += size;
						
						}
                        else
                        {
							while(1);
						}

						
						*p++	=	STATUS_CMD_OK;
					}
					break;

				default:
					msgLength		=	2;
					msgBuffer[1]	=	STATUS_CMD_FAILED;
					break;
			}
			
			
			/*
			 * Now send answer message back
			 */
			{
				unsigned char tmpBuffer[285],checksum = 0;
				int i;
				tmpBuffer[0] = MESSAGE_START;
				tmpBuffer[1] = seqNum;
				tmpBuffer[2] = ((msgLength>>8)&0xFF);
				tmpBuffer[3] = msgLength&0x00FF;
				tmpBuffer[4] = TOKEN;
				memcpy(&tmpBuffer[5],msgBuffer,msgLength);				
				for(i=0;i<msgLength+5;i++)
					checksum	^=	tmpBuffer[i];
				tmpBuffer[i] = checksum;
                memset((void *)gpu8RxBuf, 0, sizeof(gpu8RxBuf));
				USB_Write(tmpBuffer,msgLength+6);
                __enable_irq();
			}
			msgParseState = ST_START;
	
        
    }
}



void usart_send(uint8_t *data){

    USART_SendData(USART1, *data);


}


void TIM2_Configuration(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);



    // RCC_Configuration();

    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);



    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Prescaler configuration */
    TIM_PrescalerConfig(TIM2, 71, TIM_PSCReloadMode_Immediate);



    /* TIM IT enable */
    TIM_ITConfig(TIM2,TIM_IT_Update , ENABLE);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
  
}


 __IO uint32_t TIM2_TimingMillis;

void delay_ms(uint32_t ms){

  TIM2_TimingMillis = 0;
   /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  while (TIM2_TimingMillis < ms );
   /* TIM2 Disable counter */
  TIM_Cmd(TIM2, DISABLE);

}

 
volatile uint32_t ISusartDataDomming = 0;


int main(void){
   uint32_t time_delay = 0;
   pFunction Jump_To_Application = NULL;
   uint32_t JumpAddress;
   __IO uint32_t __sp;
   
   usart1Init();
   FLASH_Unlock();
   USB_Write("heloo", 5);

    __sp = *(__IO uint32_t*)APP_DEFAULT_ADD;
    
   while ( time_delay++ < 200000){    // around xs  ¶ÔÕâ¸öÊ±¼äµÄÒªÇó ÒòÎªÉÏÎ»»úÒ²»áÓÐ³¬Ê±
      
     if (ISusartDataDomming == 1 || !(__sp >= 0x20000000 &&__sp <= 0x20010000)){      //ÅÐ¶Ï´®¿ÚÊÇ·ñÓÐÊý¾Ýµ½À´£¬»òÕßÅÐ¶ÏÓÃÓÚappÆðÊ¼µØÖ·³öµÄÕ»¶¥µØÖ·ÊÇ·ñÔÚ0x20000000 ~ 0x20010000ÄÚ¡£
            
              /* TIM2 Disable counter */
           //   TIM_Cmd(TIM2, DISABLE);
              VCOM_TransferData();    //½øÈë´®¿ÚÉý¼¶½»»¥³ÌÐò¡£
             
              break;
          
          }
        
      }
   
       /* Test if user code is programmed starting from address 0x800C000£¬Ìø×ªÖÁÓÃ»§³ÌÐòÔËÐÐ */
        if (((*(__IO uint32_t*)APP_DEFAULT_ADD) > 0x20000000 &&  *(__IO uint32_t*)APP_DEFAULT_ADD) < 0x20018000) //¼ì²â0x0800C000´¦±£´æµÄÕ»¶¥Ö¸ÕëÊÇ·ñÔÚ0x2000000 - 0x20018000Ö®¼ä¼´ 128K RAM 
        { /* Jump to user application */
             
             /* Initialize user application's Stack Pointer */
       
            JumpAddress = *(__IO uint32_t*) (APP_DEFAULT_ADD + 4); //¶ÁÈ¡0x0800C0001£»
            Jump_To_Application = (pFunction) JumpAddress; //
            __set_MSP(*(__IO uint32_t*) APP_DEFAULT_ADD);
            Jump_To_Application();
            
        }else{
        
          while (1){
 
          }
        
        }
    

  return 0;

}



void usart1Init(void){
// setup clock for periph

	RCC->APB2ENR |= RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1;


	//config GPIO  as uart alternative function;
	// gpio 9
	// uart alternative function 
    GPIOA->CRH = GPIOA->CRH & ~(3 << 6) | 2 << 6;
	//gpio speed
	GPIOA->CRH |= 3 << 4;
	//gpio10
	GPIOA->CRH = GPIOA->CRH & ~(3 << 10) | 1 << 10;
	GPIOA->CRH &= ~(3 << 8);
    
    //GPIOA->CRH |= 0x888444B4;
    
	// uart config 
	USART1->CR1 &= 0xE9F3;

	//USART1->CR1 |= USART_Parity_No | USART_WordLength_8b  | USART_Mode_Rx | USART_Mode_Tx;
	USART1->CR1 = 0x0C;

	USART1->CR2 &= 0xCFFF;
	USART1->CR2 = USART_StopBits_1; 

	// HardwareFlowControl
	USART1->CR3 &= 0xFCFF;
	USART1->CR3 =USART_HardwareFlowControl_None;

	//baudrate
	USART1->BRR = 69;
    
    *(__IO uint32_t*)0x4001380C |= 32;

	NVIC->IP[USART1_IRQn] = 0;
	NVIC->ISER[USART1_IRQn >> 0x05] = (uint32_t)0x01 << (USART1_IRQn & (uint8_t)0x1F);
    USART1->CR1 |=0x2000;
}



















