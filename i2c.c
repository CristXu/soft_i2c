#include "i2c.h"

#define I2C_SDA_1()  SDA_HIGH()
#define I2C_SDA_0()  SDA_LOW()

#define I2C_SCL_1()  SCL_HIGH()
#define I2C_SCL_0()  SCL_LOW()

#define I2C_SDA_READ()  (SDA_STATE())

extern void delay();
#define Delay()    delay();

void i2c_start(void)
{
    I2C_SDA_1();
    I2C_SCL_1();
    Delay();
    I2C_SDA_0();
    Delay();
    I2C_SCL_0();
    Delay();
}

void i2c_stop(void)
{
    I2C_SDA_0();
    I2C_SCL_1();
    Delay();
    I2C_SDA_1();
}

void i2c_Ack(void)
{
    I2C_SDA_0();
    Delay();
    I2C_SCL_1();
    Delay();
    I2C_SCL_0();
    Delay();
    I2C_SDA_1();	
}

void i2c_NAck(void)
{
    I2C_SDA_1();
    Delay();
    I2C_SCL_1();
    Delay();
    I2C_SCL_0();
    Delay();
}

uint8_t i2c_WaitAck(void)
{
    uint8_t re;

    I2C_SDA_1();
    Delay();
    I2C_SCL_1();	
    Delay();
//	uint32_t safeDelay = 10000 * 150 / (1000 / 1000);
//	for (s_i = 0; s_i<safeDelay; s_i++) {}
    if (I2C_SDA_READ() != 0)
    {
		re = 1;
    }
    else
    {
		re = 0;
    }
    I2C_SCL_0();
    Delay();
    
    return re;
}

void i2c_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i=0; i<8; i++)
    {		
        if (byte & 0x80)
	{
	    I2C_SDA_1();
	}
	else
	{
	    I2C_SDA_0();
	}

	Delay();
	I2C_SCL_1();
	Delay();	
	I2C_SCL_0();
	if(i == 7)
	{
	    I2C_SDA_1(); 
	}
	byte <<= 1;
	Delay();
    }
}

void i2c_WaitAck_Delay(delay)
{
    uint8_t ack_status;

    while(delay--)
    {
        ack_status = i2c_WaitAck();
        if(ack_status)
        {
//            i2c_stop();
//			break;
        }
        else
        {
            break;
        }
    }
}

uint8_t i2c_ReadByte(void)
{
    uint8_t i;
    uint8_t value = 0;

    for (i=0; i<8; i++)
    {
		value <<= 1;
		I2C_SCL_1();
		Delay();
		if(I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		Delay();
    }

    return value;
}

uint32_t I2C_ByteWrite(u8 slave_addr, u8 WriteAddr, u8 pBuffer)
{			
    i2c_start();

    i2c_SendByte(CVT_ADDR(slave_addr, WRITE_ADDR));	     
    i2c_WaitAck_Delay(1000);
		
    i2c_SendByte(WriteAddr);
    i2c_WaitAck_Delay(1000);

    i2c_SendByte(pBuffer);
    i2c_WaitAck_Delay(1000);

    i2c_stop();
    return 1;
}
uint32_t I2C_Read(u8 slave_addr, u8 WriteAddr, u8* pBuffer, u8 numByteToRead)
{
    i2c_start();

    i2c_SendByte(CVT_ADDR(slave_addr, WRITE_ADDR));
    i2c_WaitAck_Delay(1000);

    i2c_SendByte(WriteAddr);
    i2c_WaitAck_Delay(1000);

    i2c_start();

    i2c_SendByte(CVT_ADDR(slave_addr, READ_ADDR));	
    i2c_WaitAck_Delay(1000);	

    while(numByteToRead)
    {
	*pBuffer = i2c_ReadByte();
		
	if (numByteToRead != 1)
	{
	    i2c_Ack();
	}
	else
	{
	    i2c_NAck();
	}
        
        pBuffer++;
        numByteToRead--;
    }

    i2c_stop();
    return 0;
}


#if 0
#define io_get(PIN)   (GPIO_PinRead(PIN_PORT, PIN##_PIN))
#define io_set_high(PIN) PIN##_HIGH()
#define io_set_low(PIN)  PIN##_LOW()
extern void delay();
#define delay_us(x)   delay() 

void i2c_start(void)
{
	io_set_high(SDA);	// SDA=1
	io_set_high(SCL);	// SCL=1
	delay_us(5);
	io_set_low(SDA);	// SDA=0
	delay_us(5);
}

void i2c_stop(void)
{
	if (io_get(SCL) == 1)
		io_set_low(SCL);	// SDA=0
	if (io_get(SDA) == 1)
		io_set_low(SDA);	// SDA=0
	delay_us(5);
	io_set_high(SCL);	// SCL=1
	delay_us(5);
	io_set_high(SDA);	// SDA=1
	delay_us(5);
}

extern volatile uint32_t s_i;
uint8_t i2c_read_ack(void)
{
	uint8_t ack;
	io_set_high(SCL);	// SCL=1
	uint32_t safeDelay = 10000 * 150 / (100000 / 1000);
	for (s_i = 0; s_i<safeDelay; s_i++) {}
	ack = io_get(SDA);
	delay_us(5);
	io_set_low(SCL);	// SCL=0
	delay_us(5);
	return ack;
}

void i2c_send_ack(void)
{
	io_set_low(SCL);	// SCL=0
	io_set_low(SDA);	// SDA=0
	delay_us(5);
	io_set_high(SCL);	// SCL=1
	delay_us(5);
	// TAKE CAREFULLY!!!These two orders below must be included
	// to pull down the SCL for the following opreations.
	io_set_low(SCL);	// SCL=0
	delay_us(5);
}

void i2c_send_nack(void)
{
	io_set_low(SCL);	// SCL=0
	io_set_high(SDA);	// SDA=1
	delay_us(5);
	io_set_high(SCL);	// SCL=1
	delay_us(5);
	io_set_low(SCL);	// SCL=0
	delay_us(5);
}

uint8_t i2c_ack_check(uint8_t ctrl_byte)
{
	i2c_start();
	i2c_write_single_byte(ctrl_byte);
	if(i2c_read_ack() == 0)
	{
		//sl_printf("i2c_read_ack() == 0.\n");
		// time delay here is not necessary, just to make waveforms more readable
		delay_us(30);
		//i2c_stop();
		return 0;
	}
	else
	{
		//sl_printf("i2c_read_ack() == 1.\n");
		// time delay here is to save computing resource
		delay_us(100);
		//io_input(SDA);		// set SDA as input
		//io_input(SCL);		// set SCL as input
		return 1;
	}
}

uint8_t i2c_read_single_byte(void)
{
	uint8_t i=8;
	uint8_t i2c_buff = 0x0;
	// ??read,????????,??MSB is low ??????(?SCL=0????????)
	// Is that because of setting SDA as input, making it HIGH at the very beginning? Maybe so.
	delay_us(5);
	delay_us(5);
	while (i--)
	{
		i2c_buff = i2c_buff<<1;
		io_set_high(SCL);		// SCL=1
		//
		if(io_get(SDA)==1)
			i2c_buff |= 0x01;							// Write 1 to LSB of i2c_buff
		else
			i2c_buff &= 0xFE;							// Write 0 to LSB of i2c_buff
		delay_us(5);
		io_set_low(SCL);		// SCL=0
		delay_us(5);
		//i2c_buff = i2c_buff<<1;						// move to the next MSB(from MSB to LEB)
	}
	//sl_printf("i2cbuf=%d\n", i2c_buff);
	return i2c_buff;
}

void i2c_write_single_byte(uint8_t i2c_buff)
{
	uint8_t i=8;
	while (i--)
	{
		io_set_low(SCL);			// SCL=0
		delay_us(5);
		if(i2c_buff & 0x80)					// MSB(i2c_buff)==1
			io_set_high(SDA);		// SDA=1
		else
			io_set_low(SDA);		// SDA=0
		io_set_high(SCL);			// SCL=1
		delay_us(5);
		i2c_buff = i2c_buff<<1;							// move to the next MSB(from MSB to LEB)
	}
	// After transfer, release the SCL line
	io_set_low(SCL);				// SCL=0
	delay_us(5);
}
#endif




