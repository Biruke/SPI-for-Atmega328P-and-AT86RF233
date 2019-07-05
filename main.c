/*
 * SpiApplication.c
 *
 * Created: 25/08/2016 14:56:18
 * Author : abiy
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include<stdbool.h>
#include <stddef.h>
//#include <Arduino.h>
#include <stdio.h>
//#define F_CPU 16000000
#include <math.h>
#include <avr/pgmspace.h>
//#define F_CPU 16000000ul
#define USART0SendByte(x) printf(" = %u",(x))
#define USART0SendBytee(x) printf("\n __%-20s\n",(x))
void USART0Init(void);
int USART00SendByte(char u8Data, FILE *stream);
int USART00ReceiveByte(FILE *stream);
FILE usart0_str = FDEV_SETUP_STREAM(USART00SendByte, USART00ReceiveByte, _FDEV_SETUP_RW);
#define RF_CMD_REG_R ((1<<7) | (0<<6))
#define RF_CMD_REG_W ((1<<7) | (1<<6))
#define IRQ_STATUS_REG 0x0f
#define USART_BAUDRATE 9600
#define TRX_CMD_FORCE_TRX_OFF 3
#define TRX_STATE_REG     0x02
#define TRX_STATUS_REG    0x01
#define TRX_CMD_TRX_OFF   8
//#define F_CPU 16000000ul
#define UBRR_VALUE (((16000000ul/ (USART_BAUDRATE * 16UL))) - 1)
#define CMD_TRX_OFF      (0x08)
#define PAN_ID_0_REG      0x22
#define PAN_ID_1_REG      0x23
#define PART_NUM_REG      0x1c
#define RG_TRX_STATUS   (0x01)
#define TRX_STATUS_TRX_OFF            8
#define TRX_STATUS_MASK               0x1f
#define TRX_CMD_RX_ON         6
#define RG_TRX_STATE   (0x02)
uint8_t u8TempData = 1;
uint8_t biruk=0;
#define SPIREAD1 1
#define WRITE_ACCESS_COMMAND            (0xC0)
//static uint8_t phyRxBuffer[128];
#define TRX_STATUS_P_ON               0
#define TRX_STATE_REG     0x02

#define TRX_PORT1_DDR                   (DDRB)
#define TRX_PORT2_DDR                   (DDRD)
#define TRX_PORT3_DDR                   (DDRC)

#define SLP_TR                          (PB1)
#define SS		PB2
#define SCK		PB5
#define MOSI	PB3
#define MISO	PB4

bool spi_read_packet( uint8_t *data, size_t len);
bool  spi_write_packet( const uint8_t *data, size_t len);
inline static void spi_write_single( uint8_t data);
inline static void spi_read_single(uint8_t *data);
static inline void spi_put(uint8_t data);
static inline bool spi_is_tx_ok();
static inline uint8_t spi_get();
inline static bool spi_is_rx_full();
bool  spi_write_packet( const uint8_t *data, size_t len);
uint8_t trx_reg_read(uint8_t addr);


	
#define SPI_IF_bm  0x80  /* Interrupt Flag bit mask. */
uint8_t register_value = 0;
#define CONFIG_SPI_MASTER_DUMMY              0xFF
#define READ_ACCESS_COMMAND             (0x80)
	
void USART0Init(void)
{
	// Set <a href="#">baud rate</a>
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 <a href="#">data bits</a>, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	// UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
}
int USART00SendByte(char u8Data, FILE *stream)
{
	if(u8Data == '\n')
	{
		USART00SendByte('\r', 0);
	}
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
	return 0;
}
int USART00ReceiveByte(FILE *stream)
{
	uint8_t u8Data;
	// Wait for byte to be received
	while(!(UCSR0A&(1<<RXC0))){};
	u8Data=UDR0;
	//echo input data
	USART00SendByte(u8Data,stream);
	// Return received data
	return u8Data;
}

void HAL_PhySpiSelect(void)
{
	 PORTB &= ~_BV(PORTB2);
}

void HAL_PhySpiDeselect(void)
{
PORTB |= _BV(PORTB2);
}

/*
static void phyWaitState(uint8_t state)
{
	while (state != (trx_reg_read(TRX_STATUS_REG) & TRX_STATUS_MASK));
}
*/
/******************************************** imported ***********************************************************/


uint8_t trx_reg_read(uint8_t addr)
{
	
	     addr |= READ_ACCESS_COMMAND;
		//spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
		HAL_PhySpiSelect();

		/* Send the Read command byte */
		spi_write_packet(&addr, 1);

		/* Do dummy read for initiating SPI read */
		spi_read_packet(&register_value, 1);

		/* Stop the SPI transaction by setting SEL high */
		//spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
		HAL_PhySpiDeselect();
	return register_value;
}


void trx_reg_write(uint8_t addr, uint8_t data)
{
	/*Saving the current interrupt status & disabling the global interrupt
	**/
	
	/* Prepare the command byte */
	addr |= WRITE_ACCESS_COMMAND;
		//spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
		HAL_PhySpiSelect();

		/* Send the Read command byte */
		spi_write_packet( &addr, 1);

		/* Write the byte in the transceiver data register */
		spi_write_packet(&data, 1);

		/* Stop the SPI transaction by setting SEL high */
		//spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
		HAL_PhySpiDeselect();
}
	
	


bool spi_read_packet( uint8_t *data, size_t len)
{
	while (len--) {
		spi_write_single(CONFIG_SPI_MASTER_DUMMY); //Dummy write

		while (!spi_is_rx_full()) {
		}
		
		spi_read_single( data);
		data++;
	}
	
	return true;
}
bool  spi_write_packet( const uint8_t *data, size_t len)
{
	while (len--) {
		spi_write_single( *data++);
		
		while (!spi_is_rx_full()) {
		}
	}
	
	return true;
}
inline static bool spi_is_rx_full()
{
	return spi_is_tx_ok();
}

inline static void spi_write_single( uint8_t data)
{
	spi_put(data);
}

inline static void spi_read_single(uint8_t *data)
{
	*data = spi_get();
}

static inline void spi_put(uint8_t data)
{
	SPDR = data;
}

static inline bool spi_is_tx_ok()
{
	return SPSR & SPI_IF_bm ? true : false;
}

static inline uint8_t spi_get()
{
	return SPDR;
}

void SPI_MasterInit(void) {   


	    TRX_PORT1_DDR |= _BV(SS);
	    TRX_PORT1_DDR |= _BV(SCK);
	    TRX_PORT1_DDR |= _BV(MOSI);
	    TRX_PORT2_DDR |= _BV(SLP_TR);	    
	    
	    TRX_PORT1_DDR &= ~_BV(MISO);
	    SPCR = (1<<MSTR) | (1<<SPE);
	    SPCR &= ~(_BV(SPR0) | _BV(SPR1));
	    PORTB |= 1<<SS;
	    SPSR |= (1<<SPI2X);
	  
	  } 
	   
void PHY_SetPanId(uint16_t panId)
{
	uint8_t *d = (uint8_t *)&panId;

	trx_reg_write(PAN_ID_0_REG, d[0]);
	trx_reg_write(PAN_ID_0_REG, d[0]);
}

int main(void)
{
    /* Replace with your application code */
	SPI_MasterInit() ;
	
	_delay_ms(100);
	stdin=stdout=&usart0_str;
	USART0Init();		
	
		_delay_ms(100);
	uint8_t biruk = trx_reg_read(TRX_STATE_REG);

          _delay_ms(100);
		  printf("TRX_STATE_REG=");
	USART0SendByte(biruk);
		PHY_SetPanId (0x4567);				
		   	uint8_t birukxr = trx_reg_read(RG_TRX_STATUS);
			   printf("RG_TRX_STATUS=");
		   	USART0SendByte(birukxr);
		 
	  while (1)
	  
	  {	  
		 
	   	uint8_t birukxr = trx_reg_read(RG_TRX_STATUS);
		printf("RG_TRX_STATUS=");
		 USART0SendByte(birukxr);
	 
		  uint8_t birukfc =  trx_reg_read(PART_NUM_REG);
		  _delay_ms(1000);
		  printf("PART_NUM_REG=");
		  USART0SendByte(birukfc);
		  uint8_t bi =  trx_reg_read(0x1D);
		  _delay_ms(1000);
		  		  printf("\n\n");

		   printf("Version=");
		  USART0SendByte(bi);

	  }
	 
//
	
}
