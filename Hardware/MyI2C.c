#include "stm32f10x.h"                  // Device header
#include "Delay.h"


void MyI2C_W_SCL(uint8_t Bitvalue)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction) Bitvalue);
    Delay_us(10); 
}

void MyI2C_W_SDA(uint8_t Bitvalue)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction) Bitvalue);
    Delay_us(10); 
}


uint8_t MyI2C_R_SDA(void)
{
    uint8_t Bit=0;
    Bit = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
    Delay_us(10); 
    return Bit;
}


void MyI2C_Init(void)
{
    /*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;            //
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB10和PB11引脚初始化为开漏输出
	 
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);	
}

void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);  
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i=0;
    for(i=0; i<8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80>>i));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
    
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i=0 , Bytevalue=0x00;
    
    MyI2C_W_SDA(1);
    
    for(i=0; i<8; i++)
    {
        MyI2C_W_SCL(1);
        if(MyI2C_R_SDA()==1){Bytevalue |= (0x80>>i);}
        MyI2C_W_SCL(0);
    }
    
    return Bytevalue;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit=0;
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
    AckBit =MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}
