#include "fsl_gpio.h"
// MPU-9250 SCL 22, SDA 21, PORT GPIO4
#define SLAVE_ADDR (0x21)
#define PIN_PORT   (GPIO1)
#define SCL_PIN    (16U)
#define SDA_PIN    (17U)
#define SCL_LOW()    (GPIO_PinWrite(PIN_PORT, SCL_PIN, 0)) 
#define SCL_HIGH()    (GPIO_PinWrite(PIN_PORT, SCL_PIN, 1)) 
#define SDA_LOW()    (GPIO_PinWrite(PIN_PORT, SDA_PIN, 0)) 
#define SDA_HIGH()    (GPIO_PinWrite(PIN_PORT, SDA_PIN, 1)) 

#define SDA_AS_INPUT()  (PIN_PORT->GDIR &= ~(1<<SDA_PIN))
#define SDA_AS_OUTPUT() (PIN_PORT->GDIR |= (1<<SDA_PIN))
#define SDA_STATE()   (GPIO_PinRead(PIN_PORT, SDA_PIN))
#define SCL_STATE()   (GPIO_PinRead(PIN_PORT, SCL_PIN))

#define READ_ADDR      (1)
#define WRITE_ADDR     (0)

#define CVT_ADDR(addr, r_w) \
	(((addr) << 1) | ((r_w) & 0x1))

extern void i2c_start(void);
extern void i2c_stop(void);
extern uint8_t i2c_ack_check(uint8_t ctrl_byte);
extern uint8_t i2c_read_single_byte(void);
extern void i2c_write_single_byte(uint8_t i2c_buff);

#define u8 uint8_t
extern uint32_t I2C_Read(u8 slave_addr, u8 WriteAddr, u8* pBuffer, u8 numByteToRead);
extern uint32_t I2C_ByteWrite(u8 slave_addr, u8 WriteAddr, u8 pBuffer);