#ifndef _TWISLAVE_H
#define _TWISLAVE_H

#include <util/twi.h> //enth�lt z.B. die Bezeichnungen f�r die Statuscodes in TWSR
#include <avr/interrupt.h> //dient zur behandlung der Interrupts
#include <stdint.h> //definiert den Datentyp uint8_t


/*
Dieses Programm in einer separaten Datei (z.B. twislave.c) abspeichern und in das eigene Programm
einbinden.

Betrieb eines AVRs mit Hardware-TWI-Schnittstelle als Slave. Zu Beginn muss init_twi_slave mit der gew�nschten
Slave-Adresse als Parameter aufgerufen werden. Der Datenaustausch mit dem Master erfolgt �ber die Buffer 
rxbuffer und txbuffer, auf die von Master und Slave zugegriffen werden kann. 
rxbuffer und txbuffer sind globale Variablen (Array aus uint8_t). 
Die Ansteuerung des rxbuffers, in den der Master schreiben kann, erfolgt �hnlich wie bei einem normalen I2C-EEPROM.
Man sendet zun�chst die Bufferposition, an die man schreiben will, und dann die Daten. Die Bufferposition wird 
automatisch hochgez�hlt, sodass man mehrere Datenbytes hintereinander schreiben kann, ohne jedesmal
die Bufferadresse zu schreiben.
Um den txbuffer vom Master aus zu lesen, �bertr�gt man zun�chst in einem Schreibzugriff die gew�nschte Bufferposition und
liest dann nach einem repeated start die Daten aus. Die Bufferposition wird automatisch hochgez�hlt, sodass man mehrere
Datenbytes hintereinander lesen kann, ohne jedesmal die Bufferposition zu schreiben.

Autor: Uwe Gro�e-Wortmann (uwegw)
Status: Testphase, keine Garantie f�r ordnungsgem��e Funktion! 
letze �nderungen: 
23.03.07 Makros f�r TWCR eingef�gt. Abbruch des Sendens, wenn der TXbuffer komplett gesendet wurde. 
24.03.07 verbotene Buffergr��en abgefangen
25.03.07 n�tige externe Bibliotheken eingebunden


Abgefangene Fehlbedienung durch den Master:
- Lesen �ber die Grenze des txbuffers hinaus
- Schreiben �ber die Grenzen des rxbuffers hinaus
- Angabe einer ung�ltigen Schreib/Lese-Adresse
- Lesezuggriff, ohne vorher Leseadresse geschrieben zu haben


 */ 
 



//%%%%%%%% von Benutzer konfigurierbare Einstellungen %%%%%%%%

#define buffer_size 8 //Gr��e der Buffer in Byte (2..254)


//%%%%%%%% Globale Variablen, die vom Hauptprogramm genutzt werden %%%%%%%%

/*Der Buffer, in dem die empfangenen Daten gespeichert werden. Der Slave funktioniert �hnlich  wie ein normales
 Speicher-IC [I2C-EEPROM], man sendet die Adresse, an die man schreiben will, dann die Daten, die interne Speicher-Adresse
 wird dabei automatisch hochgez�hlt*/
volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];


//%%%%%%%% Funktionen, die vom Hauptprogramm aufgerufen werden k�nnen %%%%%%%%
 
/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter: adr: gew�nschte Slave-Adresse*/
void init_twi_slave (uint8_t adr);



//%%%%%%%% ab hier sind normalerweise keine weiteren �nderungen erforderlich! %%%%%%%%//
//____________________________________________________________________________________//

#include <util/twi.h> //enth�lt z.B. die Bezeichnungen f�r die Statuscodes in TWSR


//Bei zu alten AVR-GCC-Versionen werden die Interrupts anders genutzt, daher in diesem Fall mit Fehlermeldung abbrechen
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
	#error "This library requires AVR-GCC 3.4.5 or later, update to newer AVR-GCC compiler !"
#endif

//Schutz vor unsinnigen Buffergr��en
#if (buffer_size > 254)
	#error Buffer zu gro� gew�hlt! Maximal 254 Bytes erlaubt.
#endif

#if (buffer_size < 2)
	#error Buffer muss mindestens zwei Byte gro� sein!
#endif

void twidelay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}


volatile uint8_t buffer_adr; //"Adressregister" f�r den Buffer

/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter: adr: gew�nschte Slave-Adresse
*/
void init_twi_slave (uint8_t adr)
{
	TWAR= adr; //Adresse setzen
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE); 	
	buffer_adr=0xFF;  
	sei();
}


//Je nach Statuscode in TWSR m�ssen verschiedene Bitmuster in TWCR geschrieben werden(siehe Tabellen im Datenblatt!). 
//Makros f�r die verwendeten Bitmuster:

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  
//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
//switched to the non adressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

//Die Bitmuster f�r TWCR_ACK und TWCR_RESET sind gleich. Dies ist kein Fehler und dient nur der �bersicht!


/*ISR, die bei einem Ereignis auf dem Bus ausgel�st wird. Im Register TWSR befindet sich dann 
ein Statuscode, anhand dessen die Situation festgestellt werden kann.
*/
ISR (TWI_vect)  
{
uint8_t data=0;
PORTB |= (1<<PORTB0);
twidelay_ms(400);
PORTB &= ~(1<<PORTB0);
switch (TW_STATUS) //TWI-Statusregister pr�fen und n�tige Aktion bestimmen 
{

case TW_SR_SLA_ACK: // 0x60 Slave Receiver, wurde adressiert	
	TWCR_ACK; // n�chstes Datenbyte empfangen, ACK danach
	buffer_adr=0xFF; //Bufferposition ist undefiniert
break;
	
case TW_SR_DATA_ACK: // 0x80 Slave Receiver,Daten empfangen
	data=TWDR; //Empfangene Daten auslesen
	if (buffer_adr == 0xFF) //erster Zugriff, Bufferposition setzen
		{
			
			//Kontrolle ob gew�nschte Adresse im erlaubten bereich
			if(data<=buffer_size)
				{
					buffer_adr= data; //Bufferposition wie adressiert setzen
				}
			else
				{
				buffer_adr=0; //Adresse auf Null setzen. Ist das sinnvoll?
				}				
			TWCR_ACK;	// n�chstes Datenbyte empfangen, ACK danach, um n�chstes Byte anzufordern
		}
	else //weiterer Zugriff, Daten empfangen
		{
			rxbuffer[buffer_adr]=data; //Daten in Buffer schreiben
			buffer_adr++; //Buffer-Adresse weiterz�hlen f�r n�chsten Schreibzugriff
			if(buffer_adr<(buffer_size-1)) //im Buffer ist noch Platz f�r mehr als ein Byte
				{
					TWCR_ACK;// n�chstes Datenbyte empfangen, ACK danach, um n�chstes Byte anzufordern
				}
			else   //es kann nur noch ein Byte kommen, dann ist der Buffer voll
				{
					TWCR_NACK;//letztes Byte lesen, dann NACK, um vollen Buffer zu signaliseren
				}
		}
break;

case TW_ST_SLA_ACK: //?!?
case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, weitere Daten wurden angefordert

	if (buffer_adr == 0xFF) //zuvor keine Leseadresse angegeben! 
		{
			buffer_adr=0;
		}	
	TWDR = txbuffer[buffer_adr]; //Datenbyte senden 
	buffer_adr++; //bufferadresse f�r n�chstes Byte weiterz�hlen
	if(buffer_adr<(buffer_size-1)) //im Buffer ist mehr als ein Byte, das gesendet werden kann
		{
			TWCR_ACK; //n�chstes Byte senden, danach ACK erwarten
		}
	else
		{
			TWCR_NACK; //letztes Byte senden, danach NACK erwarten
		}
break;

case TW_ST_DATA_NACK: //0xC0 Keine Daten mehr gefordert 
case TW_SR_DATA_NACK: //0x88 
case TW_ST_LAST_DATA: //0xC8  Last data byte in TWDR has been transmitted (TWEA = �0�); ACK has been received
case TW_SR_STOP: // 0xA0 STOP empfangen
default: 	
    TWCR_RESET; //�bertragung beenden, warten bis zur n�chsten Adressierung
break;

	
} //end.switch (TW_STATUS)
} //end.ISR(TWI_vect)


#endif //#ifdef _TWISLAVE_H
////Ende von twislave.c////

