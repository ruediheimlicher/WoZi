//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "twislave.c"
#include "lcd.c"

#include "adc.c"
#include "onewire.c"
#include "ds18x20.c"
#include "crc8.c"

//***********************************
//WOZI							*
#define SLAVE_ADRESSE 0x66 //		*
//									*
//***********************************
#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4 // PORT C
#define SCLPIN		5

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4

#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define WDTBIT			7


#define WOZIPORT	PORTD		// Ausgang fuer WOZI
#define LAMPEBIT 0

#define SERVOPORT	PORTD		// Ausgang fuer Servo
#define SERVOPIN0 7				// Impuls für Servo
#define SERVOPIN1 6				// Enable fuer Servo, Active H

#define RADIATORPORT PORTD
#define RADIMPULSPIN		7		// Impuls für Radiator
#define RADSTATUSPIN		6		// Status fuer Radiator
#define RADSTUFE1			0x8000
#define RADSTUFE2			0xE000

// Definitionen fuer mySlave PORTD
//#define LAMPEEIN 4
//#define LAMPEAUS 5

// Definitionen Slave WOZI
#define LAMPEEIN 0
#define LAMPEAUS 1


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0xEFFF

// Define fuer Slave:
#define LOOPLED			4
#define TWILED          5

// Define fuer mySlave PORTD:
//#define LOOPLED			2
//#define TWILED			7


#define TASTE1		38
#define TASTE2		46
#define TASTE3		54
#define TASTE4		72
#define TASTE5		95
#define TASTE6		115
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		225
#define TASTE0		235
#define TASTER		245
#define TASTATURPORT PORTC

#define TASTATURPIN		3
#define POTPIN			0
#define BUZZERPIN		0

#define INNEN			1	//	Byte fuer INNENtemperatur


#define RADIATORPORT          PORTD
#define RADIATORDDR           DDRD
#define RADIATORIMPULSPIN		7		// Impuls für Radiator
#define RADIATORSTATUSPIN		6		// Status fuer Radiator


//#define MAXSENSORS 2
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;




uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);

volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

static volatile uint8_t WoZistatus=0x00;
static volatile uint8_t Radiatorstatus=0x00;

volatile uint16_t Servotakt=20;					//	Abstand der Impulspakete
volatile uint16_t Servopause=0x00;				//	Zaehler fuer Pause
volatile uint16_t Servoimpuls=0x00;				//	Zaehler fuer Impuls
volatile uint8_t Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
volatile uint8_t ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
volatile uint8_t ServoimpulsdauerSpeicher=0;	//	Speicher  fuer Servoimpulsdauer
volatile uint8_t Potwert=45;
volatile uint8_t TWI_Pause=1;
volatile uint8_t ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
uint8_t ServoimpulsNullpunkt=23;
uint8_t ServoimpulsSchrittweite=10;
uint8_t Servoposition[]={23,33,42,50,60};
volatile uint16_t ADCImpuls=0;

volatile uint8_t twi=0;
uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

// Code 1_wire start
//uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];


uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	
	ow_reset();
	
	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) 
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("No Sensor found\0" );
						
			delay_ms(800);
			lcd_clr_line(1);
			break;
		}
		
		if( diff == OW_DATA_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("Bus Error\0" );
			break;
		}
		lcd_gotoxy(4,1);

		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			{
				//lcd_gotoxy(15,1);
				//lcd_puthex(id[i]);

			gSensorIDs[nSensors][i] = id[i];
			//delay_ms(100);
			}
			
		nSensors++;
	}
	
	return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void)
{

        gTemp_measurementstatus=0;
        if ( DS18X20_start_meas(NULL) != DS18X20_OK) 
		  {
                gTemp_measurementstatus=1;
        }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
        uint8_t i;
        uint8_t subzero, cel, cel_frac_bits;
        for ( i=0; i<gNsensors; i++ ) 
		  {
			  
			  if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
											 &cel, &cel_frac_bits) == DS18X20_OK ) 
			  {
				  gTempdata[i]=cel*10;
				  gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
				  if (subzero)
				  {
					  gTempdata[i]=-gTempdata[i];
				  }
			  }
			  else
			  {
				  gTempdata[i]=0;
			  }
        }
}


// Code 1_wire end


void RingD2(uint8_t anz)
{
	uint8_t k=0;
	for (k=0;k<2*anz;k++)
	{
		PORTD |=(1<<BUZZERPIN);
		twidelay_ms(2);
		PORTD &=~(1<<BUZZERPIN);
		twidelay_ms(2);
		
	}
	PORTD &=~(1<<BUZZERPIN);
}


uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void slaveinit(void)
{
 	DDRD |= (1<<DDD0);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<DDD1);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<DDD2);		//Pin 2 von PORT D als Ausgang fuer Buzzer
 	DDRD |= (1<<DDD3);		//Pin 3 von PORT D als Ausgang fuer LED TWI
	DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
	DDRD |= (1<<DDD5);		//Pin 5 von PORT D als Ausgang fuer LED Loop
 	DDRD |= (1<<RADSTATUSPIN);	//Pin 6 von PORT D als Ausgang fuer Radiator-Enable
	DDRD |= (1<<RADIMPULSPIN);	//Pin 7 von PORT D als Ausgang fuer Radiator-Impuls
	
	PORTD |=(1<<PD0);
	delay_ms(200);
	PORTD &= ~(1<<PD0);
	
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang für Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang für Taste 2
	PORTB |= (1<<PB1);	//Pull-up
	

	//LCD
	DDRB |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	DDRB |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	DDRB |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang für SDA
	TWI_PORT |= (1<<SDAPIN); // HI
	
	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang für SCL
	TWI_PORT |= (1<<SCLPIN); // HI

	SlaveStatus=0;
	SlaveStatus |= (1<<TWI_WAIT_BIT);
	
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up


	
	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//Timer fuer Servo	
	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
	
}

void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 

ISR(TIMER0_OVF_vect)
{ 
	ADCImpuls++;
	Servopause++;
	//lcd_clr_line(1);

	//lcd_gotoxy(10,1);
	//lcd_puts("Tim\0");
	//delay_ms(400);
	//lcd_cls();
	//lcd_clr_line(0);
	//lcd_gotoxy(0,1);
	//lcd_puts("Stop Servo\0");
	//lcd_puts(" TP\0");
	//lcd_putint1(TWI_Pause);
	//	Intervall 64 us, Overflow nach 16.3 ms
	
	if (Servopause==3)	// Neues Impulspaket nach 48.9 ms
	{

		if (TWI_Pause)
		{
//			lcd_gotoxy(19,0);
//			lcd_putc(' ');
			timer2(Servoimpulsdauer);	 // setzt die Impulsdauer
			if (SERVOPORT &  (1<<SERVOPIN1)) // Servo ist ON
			{
				SERVOPORT |= (1<<SERVOPIN0); // Schaltet Impuls an SERVOPIN0 ein
			}
			SERVOPORT |= (1<<5);// Kontrolle auf PIN D5
		}
		Servopause=0;
	}
	
}


ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR2=0;
		SERVOPORT &= ~(1<<SERVOPIN0);//	SERVOPIN0 zuruecksetzen
		SERVOPORT &= ~(1<<5);// Kontrolle auf PIN D5 OFF
		//delay_ms(800);
		//lcd_clr_line(1);
		
}




void main (void) 
{
	/* 
	in Start-loop in while
	init_twi_slave (SLAVE_ADRESSE);
	sei();
	*/
	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	
	//uint16_t ADC_Wert= readKanal(0);
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("READY\0");
	
	WOZIPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
	WOZIPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
	WOZIPORT |= (1<<LAMPEAUS);
	delay_ms(10);
	WOZIPORT &= ~(1<<LAMPEAUS);

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	uint8_t Servowert=0;
	uint8_t Servorichtung=1;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	uint8_t Schalterposition=0;
	//timer0();
	
	//initADC(TASTATURPIN);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint16_t startdelay0=0x01FF;
	//uint16_t startdelay1=0;

	uint16_t twi_LO_count0=0;
	uint16_t twi_LO_count1=0;

	uint16_t radiatorcount=0;

	//uint8_t twierrcount=0;
	LOOPLEDPORT |=(1<<LOOPLED);
	
	delay_ms(800);
	//eeprom_write_byte(&WDT_ErrCount0,0);
	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0);
//	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
	uint16_t twi_HI_count0=0;

	if (eepromWDT_Count0==0xFF)
	{
		eepromWDT_Count0=0;
	
	}

		/*
	Bit 0: 1 wenn wdt ausgelöst wurde
	 
	  */ 
	  #pragma mark DS1820 init
	// DS1820 init-stuff begin
	uint8_t i=0;
	uint8_t nSensors=0;
	ow_reset();
	gNsensors = search_sensors();
	
	delay_ms(100);
	lcd_gotoxy(0,0);
	lcd_puts("Sensoren: \0");
	lcd_puthex(gNsensors);
	if (gNsensors>0)
	{
		lcd_clr_line(1);
		start_temp_meas();
	}
	i=0;
	while(i<MAXSENSORS)
	{
		gTempdata[i]=0;
		i++;
	}
	// DS1820 init-stuff end
	delay_ms(1000);
	lcd_clr_line(0);
	while (1)
	{	
		//Blinkanzeige
		loopcount0++;
		if (loopcount0==LOOPSTEP)
		{
			loopcount0=0;
			LOOPLEDPORT ^=(1<<LOOPLED);
			//delay_ms(10);
			
		}
		
		// Radiatorsteuerung
		radiatorcount++;
		if (radiatorcount==LOOPSTEP)
		{
			radiatorcount=0;
			if (Radiatorstatus & 0x03) // status ist > 0
			{
				RADIATORPORT |= (1<<RADSTATUSPIN); // Statusanzeige einschalten
				RADIATORPORT |= (1<<RADIMPULSPIN); // Radiatorheizung ein, Ausschalten je nach Code 
			}
			else
			{
				RADIATORPORT &= ~(1<<RADSTATUSPIN); // Statusanzeige ausschalten
				RADIATORPORT &= ~(1<<RADIMPULSPIN); // Kein Code, Radiatorheizung aus
			}
			
		}


		
		/**	Beginn Startroutinen	***********************/
		
		// wenn Startbedingung vom Master:  TWI_slave initiieren
		if (SlaveStatus & (1<<TWI_WAIT_BIT)) 
		{
			if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
			{
			init_twi_slave (SLAVE_ADRESSE);
			sei();
			SlaveStatus &= ~(1<<TWI_WAIT_BIT);
			SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
			
			// StartDelayBit zuruecksetzen
			
			}
		}

		/**	Ende Startroutinen	***********************/
		
		
		/* **** rx_buffer abfragen **************** */
		//rxdata=0;
		
		// RADIMPULSPIN in Loop-Schleife gesetzt, Ausschalten nach Zaehlerstand des gegebenen codes
		switch (Radiatorstatus & 0x03)
			{
				case 0:
					RADIATORPORT &= ~(1<<RADIMPULSPIN); // Heizung sicher aussschalten
               RADIATORPORT &= ~(1<<RADIATORSTATUSPIN); // Statusanzeige ausschalten
					break;
					
				case 1:
				{
					if (radiatorcount >= RADSTUFE1)
					{
						RADIATORPORT &= ~(1<<RADIMPULSPIN); // Heizung wieder aussschalten
					}
				}
					break;
					
				case 2:
            case 3:
				{
					if (radiatorcount >= RADSTUFE2)
					{
						RADIATORPORT &= ~(1<<RADIMPULSPIN); // Heizung wieder aussschalten
                  RADIATORPORT &= ~(1<<RADIATORSTATUSPIN); // Statusanzeige ausschalten
					}
				}
					break;
					
				default:
					RADIATORPORT &= ~(1<<RADIMPULSPIN); // Heizung sicher aussschalten
					RADIATORPORT &= ~(1<<RADSTATUSPIN); // Statusanzeige ausschalten
					break;
			}//switch

		
		//	Schaltuhr
		
		
		if (rxdata)
		{
#pragma mark DS lesen			
			// DS lesen start
			
			// DS1820 loop-stuff begin
			
			start_temp_meas();
			delay_ms(800);
			read_temp_meas();
			
			//Sensor 1
			lcd_gotoxy(0,1);
			lcd_puts("I:     \0");
			if (gTempdata[0]/10>=100)
			{
				lcd_gotoxy(3,1);
				lcd_putint((gTempdata[0]/10));
			}
			else
			{
				lcd_gotoxy(2,1);
				lcd_putint2((gTempdata[0]/10));
			}
			
			lcd_putc('.');
			lcd_putint1(gTempdata[0]%10);
			
			/*
			// Sensor 2
			lcd_gotoxy(10,1);
			lcd_puts("KR:     \0");
			lcd_gotoxy(14,1);
			if (gTempdata[1]/10>=100)
			{
				lcd_gotoxy(13,1);
				lcd_putint((gTempdata[1]/10));
			}
			else
			{
				lcd_gotoxy(14,1);
				lcd_putint2((gTempdata[1]/10));
			}
			
			lcd_putc('.');
			lcd_putint1(gTempdata[1]%10);
			
			lcd_gotoxy(16,2);
			lcd_puts("   \0");
			lcd_gotoxy(16,2);
			lcd_puthex(gTemp_measurementstatus);
			*/
			
			
			// DS1820 loop-stuff end
			
			// DS lesen end
			
			//txbuffer[KOLLEKTORVORLAUF]=(uint8_t)(readKanal(KOLLEKTORVORLAUF)>>2);// KOLLEKTORVORLAUF
			//lcd_gotoxy(0,0);
			//lcd_puts("t\0");
			//lcd_putint(temperaturBuffer>>2);
			
			//lcd_gotoxy(0,1);
			//			lcd_puts("A\0");
			//			lcd_put_temperatur(temperaturBuffer>>2);
			//lcd_puts("A0+\0");
			//lcd_put_tempbis99(temperaturBuffer>>1);//Doppelte Auflösung				
			
         // Byte 1
			txbuffer[INNEN]=2*((gTempdata[0]/10)& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
			
			// Halbgrad addieren
			if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
			{
				txbuffer[INNEN] +=1;
			}

			/*
			lcd_cls();
			lcd_gotoxy(7,1);
			lcd_puthex(twi);
			lcd_puthex(rxbuffer[0]);
			lcd_putc(' '):
			lcd_puthex(rxbuffer[1]);
			*/
			{
				//
				if (rxbuffer[3] < 6)
				{
					if (Servorichtung) // vorwärts
					{
							Servowert++;
							if (Servowert==4)
							{
							Servorichtung=0;
							}
					}
					else
					{
							Servowert--;
							if (Servowert==0)
							{
							Servorichtung=1;
							}
					}
					/*
					lcd_gotoxy(0,12);
					lcd_puts("R:\0");
					lcd_putint2(Servorichtung);
					lcd_puts(" W:\0");
					lcd_putint2(Servowert);
					*/
					Servowert=rxbuffer[3];
					ServoimpulsdauerPuffer=Servoposition[Servowert];
				}
         }
			//RingD2(2);
			//delay_ms(20);
			
			WoZistatus=rxbuffer[0];
			lcd_gotoxy(16,1);
			//cli();
			lcd_puts("St:\0");
			//lcd_puthex(WoZistatus);
			//sei();
			//delay_ms(1000);
			if ( WoZistatus  & (1<<LAMPEBIT))
				{
					//delay_ms(1000);
					//Schaltuhr ein
					//cli();
					lcd_gotoxy(19,1);
					lcd_putc('1');
					//sei();
					WOZIPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
					WOZIPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
					WOZIPORT |= (1<<LAMPEEIN);
					delay_ms(20);
					WOZIPORT &= ~(1<<LAMPEEIN);
				}
				else
				{
					//delay_ms(1000);
					//Schaltuhr aus
					//cli();
					lcd_gotoxy(19,1);
					lcd_putc('0');
					//sei();
					WOZIPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
					WOZIPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
					WOZIPORT |= (1<<LAMPEAUS);
					delay_ms(20);
					WOZIPORT &= ~(1<<LAMPEAUS);
				}
			
			
			Radiatorstatus = rxbuffer[1]; // Code fuer Radiator	0: Grundeinstellung 
													//								1: Absenkung 1 
													//								2: Absenkung 2
			
			lcd_gotoxy(7,1);
			//lcd_puthex(rxbuffer[1]);
			lcd_puts("R:\0");
			lcd_puthex(Radiatorstatus);

				//	PIN B4 abfragen
				txbuffer[4]=(PINB & (1<< 4));
				rxdata=0;
			//PORTD &= ~(1<<PD3);

		}
		
		
		
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P0 Down\0");
			
			if (! (TastenStatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<PB0);
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
					if (Servowert<4)
					{
						Servowert++;
						Servoimpulsdauer=Servoposition[Servowert];
						
					}
					/*
					 if (Servoimpulsdauer<61)
					 {
					 Servoimpulsdauer++;
					 SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
					 lcd_gotoxy(0,1);
					 //lcd_puts("P0 down  \0");
					 lcd_putint2(Servoimpulsdauer);
					 
					 }
					 */
					Tastencount=0;
					TastenStatus &= ~(1<<PB0);
				}
			}//else
			
		}	// Taste 0
		
		
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");
			
			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PB1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					
					if (Servowert > 0)
					{
						Servowert--;
						Servoimpulsdauer=Servoposition[Servowert];
               }
					
					if (Servoimpulsdauer>19)
					{
						Servoimpulsdauer--;
						SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
						
						lcd_gotoxy(0,1);
						lcd_putint2(Servoimpulsdauer);
					}
					Tastencount=0;
					TastenStatus &= ~(1<<PB1);
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
		
		if (Tastenwert>23)
		{
			/*
			 0: 
			 1: 
			 2: 
			 3: 
			 4: 
			 5: 
			 6: 
			 7: 
			 8: 
			 9: 
			 */
			
			TastaturCount++;
			if (TastaturCount>=50)
			{
				
				//lcd_clr_line(1);
//				lcd_gotoxy(8,1);
//				lcd_puts("T:\0");
//				lcd_putint(Tastenwert);
				
				uint8_t Taste=Tastenwahl(Tastenwert);
				
				lcd_gotoxy(18,1);
				lcd_putint2(Taste);
				//delay_ms(600);
				// lcd_clr_line(1);
				
				
				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
				
				switch (Taste)
				{
					case 0://
					{ 
						
					}break;
						
					case 1://
					{ 
					}break;
						
					case 2://
					{ 
						
					}break;
						
					case 3://
					{ 
						
					}break;
						
					case 4://
					{ 
						if (Schalterposition)
						{
							Schalterposition--;
							Servoimpulsdauer=Servoposition[Schalterposition];
						}
						
					}break;
						
					case 5://
					{ 
						
						Schalterposition=0;
						Servoimpulsdauer=Servoposition[Schalterposition];
						
					}break;
						
					case 6://
					{ 
						if (Schalterposition<4)
						{
							Schalterposition++;
							Servoimpulsdauer=Servoposition[Schalterposition];
						}
					}break;
						
					case 7://
					{ 
						if (Servoimpulsdauer>Servoposition[0])
						{
							Servoimpulsdauer--;
							lcd_gotoxy(0,16);
							lcd_putint2(Servoimpulsdauer);
						}
						
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9://
					{ 
						if (Servoimpulsdauer<Servoposition[4])
						{
							Servoimpulsdauer++;
							lcd_gotoxy(0,2);
							lcd_putint2(Servoimpulsdauer);
						}
					}break;
						
						
				}//switch Tastatur
				SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
			}//if TastaturCount	
			
		}//	if Tastenwert
		
		//	LOOPLEDPORT &= ~(1<<LOOPLED);
	}//while


// return 0;
}
