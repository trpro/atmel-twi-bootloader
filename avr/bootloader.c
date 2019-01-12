/*
 * twi/i2c bootloader.c
 *
 * Created: 08.01.2014 09:00:57
 * Author: trpro <trpro@gmx.de>
 * Version: 0.1
 */ 

#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

//#include "Functions.h"

#define CMD_WAIT 1
#define CMD_FLASH 10
#define CMD_BOOT_APP 99

void (*start_app)( void ) = 0x0000; // pointer to finish bootloader

#define TIMER_DELAY F_CPU / 1024 / 2
/*#define EnableTimer()	TCCR1A = (0 << WGM11) | (1 << WGM10);\
						TCCR1B |= (1 << WGM13) | (0 << WGM12);\
						TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);\
						OCR1A = TIMER_DELAY; \
						TCNT1 -= TIMER_DELAY;\
						TIMSK = (1 << OCIE1A)
*/
#define EnableTimer()	TCCR1A = (0 << WGM11) | (1 << WGM10);\
						TCCR1B |= (1 << WGM13) | (0 << WGM12);\
						TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);\
						OCR1A = TIMER_DELAY; \
						TCNT1 -= TIMER_DELAY;\
						TIMSK = (1 << OCIE1A)
#define TWI_SLAVE_ADDR 0x30
// TWI initialisieren
#define EnableTWI() TWAR= (TWI_SLAVE_ADDR << 1);\
					/*TWCR &= ~(1 << TWSTA) | (1 << TWSTO);*/\
					TWCR |= (1 << TWEN) | (1 << TWEA) | (1 << TWIE)

#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);

#define LED_GN_ON() PORTB |= ( 1 << PB0 )
#define LED_YE_ON() PORTB |= ( 1 << PB1 )
#define LED_RD_ON() PORTB |= ( 1 << PB2 )

#define LED_GN_OFF() PORTB &= ~(1 << PB0)
#define LED_YE_OFF() PORTB &= ~(1 << PB1)
#define LED_RD_OFF() PORTB &= ~(1 << PB2)

#define TOGGLE_LED_GN() PORTB ^= (1 << PB0);
#define TOGGLE_LED_YE() PORTB ^= (1 << PB1);
#define TOGGLE_LED_RD() PORTB ^= (1 << PB2);

#define BAT_ON() PORTD |= (1 << PD6);
#define BAT_OFF() PORTD &= ~(1 << PD6);
#define GPIO_ON() PORTD |= (1 << PD7);
#define GPIO_OFF() PORTD &= ~(1 << PD7);


volatile uint8_t CCmd = CMD_WAIT;

#define TIMER_FQ 8
volatile uint16_t BootTimeout = 1 * TIMER_FQ; // Timeout in sec.

#define TWI_NEW_CMD 0xFF
#define TWI_CMD_GETVERSION 0x01
#define TWI_CMD_GETSTATUS 0x02
#define TWI_CMD_GETPAGESIZE 0x03
#define TWI_CMD_FLASHSTART 0x05
#define TWI_CMD_FLASHIT 0x06
#define TWI_CMD_REBOOT 0x10

volatile uint8_t twi_cur_cmd = 0xFF;

#define RESP_BUF_SIZE 12
uint8_t twi_resp_buf[RESP_BUF_SIZE];
uint8_t twi_resp_buf_size = 0;
uint8_t twi_resp_buf_pos = 0;

uint8_t flash_page_buf_addr = 0x00;
uint8_t flash_page_buf[SPM_PAGESIZE];
uint16_t flash_page = 0xFFFF;

uint8_t status = 0x00;

#define STAT_WATING		0b00000001
#define STAT_READY		0b00000010
#define STAT_FLASHIT	0b00000100

#define TWI_VERSION_LEN 12
volatile uint8_t *version = "tpboot v0.1";

void ProcessCommand()
{
	switch(twi_cur_cmd)
	{
		case TWI_CMD_GETVERSION:
			for(int i = 0; i < TWI_VERSION_LEN; i++)
			{
				twi_resp_buf[i] = version[i];
			}
			twi_resp_buf_size = TWI_VERSION_LEN;
			twi_resp_buf_pos = 0;
			
			twi_cur_cmd = TWI_NEW_CMD;
		break;
		case TWI_CMD_GETSTATUS:
			// output current status
			twi_resp_buf[0] = status;
			twi_resp_buf_size = 1;
			twi_resp_buf_pos = 0;
			
			twi_cur_cmd = TWI_NEW_CMD;
		break;
		case TWI_CMD_FLASHIT:
			status |= (STAT_FLASHIT);
			flash_page = 0xFFFF;
			flash_page_buf_addr = 0x00;
			
		break;
		case TWI_CMD_GETPAGESIZE:
			twi_resp_buf[0] = SPM_PAGESIZE;
			twi_resp_buf_size = 1;
			twi_resp_buf_pos = 0;
			
			twi_cur_cmd = TWI_NEW_CMD;
		break;
		case TWI_CMD_FLASHSTART:
			// cancel boot and wait for data
			CCmd = CMD_FLASH;
			
			status = 0x00;
			status |= (STAT_READY);

			twi_cur_cmd = TWI_NEW_CMD;
		break;
		case TWI_CMD_REBOOT:
			CCmd = CMD_BOOT_APP;
		break;
	}	
}

void WriteFlashPage()
{
	uint16_t pageaddr = flash_page;	
	uint8_t size = SPM_PAGESIZE;
    uint8_t sreg;
	uint8_t i;
	flash_page_buf_addr = 0x00;
	
    // Disable interrupts 
 //   sreg = SREG;
 //   cli();
    
    eeprom_busy_wait ();
	
	boot_page_erase(pageaddr);
	boot_spm_busy_wait();
	
	for(size = 0; size < SPM_PAGESIZE; size += 2)
	{
		 uint16_t data = flash_page_buf[flash_page_buf_addr++];
	     data += flash_page_buf[flash_page_buf_addr++] << 8;
	     boot_page_fill(flash_page + size, data);
	}	
	
	boot_page_write(pageaddr);
	boot_spm_busy_wait();
	boot_rww_enable();
	
	/* Re-enable interrupts (if they were ever enabled). */
//	SREG = sreg;
//	sei();
}
ISR(TWI_vect)
{
	uint8_t data = 0x00;
	
	switch(TW_STATUS)
	{
		case TW_SR_SLA_ACK:
			TWCR_ACK;
		break;
		case TW_SR_DATA_ACK:
			data = TWDR;	
			
			// collect data
			if(status & STAT_FLASHIT)
			{
				if(flash_page == 0xFFFF)
				{
					flash_page = data;
					flash_page *= 64;
				}		
				else if(flash_page_buf_addr < SPM_PAGESIZE)
				{
					flash_page_buf[flash_page_buf_addr++] = data;
					if(flash_page_buf_addr == SPM_PAGESIZE)
					{
						WriteFlashPage();
						
						flash_page = 0xFFFF;
						flash_page_buf_addr = 0x00;
									
						status &= ~(STAT_FLASHIT);
						twi_cur_cmd = TWI_NEW_CMD;
					}
				}
			}
			else	
			{
				if(twi_cur_cmd == TWI_NEW_CMD)
				{
					twi_cur_cmd = data;
				}
				ProcessCommand();
			}			
			
			TWCR_ACK;
		break;
		
		case TW_ST_SLA_ACK:
		case TW_ST_DATA_ACK:
			data = 0xFF;
			if(twi_resp_buf_pos < twi_resp_buf_size && twi_resp_buf_pos < RESP_BUF_SIZE)
			{
				data = twi_resp_buf[twi_resp_buf_pos++];
			}
			TWDR = data;
			TWCR_ACK;
		break;
					
		case TW_SR_STOP:
			TWCR_ACK;
		break;
					
		case TW_ST_DATA_NACK:
		case TW_SR_DATA_NACK:
		case TW_ST_LAST_DATA:
		default:
			TWCR_RESET;
		break;
	}		
}

ISR(TIMER1_COMPA_vect)
{
	TOGGLE_LED_GN();
	TOGGLE_LED_YE();
	TOGGLE_LED_RD();
	
	BootTimeout--;
	
	if(BootTimeout == 0 && CCmd == CMD_WAIT)
		CCmd = CMD_BOOT_APP;
}

int main(void)
{
	// modify jump addresses
	unsigned char temp;				
	temp = GICR;					
	GICR = temp | (1<<IVCE);
	GICR = temp | (1<<IVSEL);
	
	// enable twi
	EnableTWI();
	sei();
	
	// enable and start timer
	EnableTimer();
	sei();

	// enable outputs
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);
	DDRD |= (1 << PD6) | (1 << PD7);
	
	LED_GN_ON();
	LED_YE_ON();
	LED_RD_ON();
	
	BAT_OFF();
	GPIO_OFF();

	status = 0x00;
	status |= (STAT_WATING);

	while(CCmd != CMD_BOOT_APP)
	{
	}
	
	PORTD = 0x00;
	
	cli();
	
	// disable twu
	TWCR = 0x00;
	
	// disable timer
	TCCR0 = 0x00;
	TCNT0 = 0x00;
	TIMSK = 0x00;
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	
	// correct jump address
	temp = MCUCR;					
	GICR = temp | (1<<IVCE);
	GICR = temp & ~(1<<IVSEL);
	
	start_app();
	
	return 0;
}