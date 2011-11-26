/*********************************************
* vim: set sw=8 ts=8 si :
* Author: Jochen Roessner Copyright: GPL v.2 
* Bootloader ueber TWI (I2C) Interface
* 
* Chip Typ           : Atmega8 
* Clock Frequenz     : 8,000000 MHz
*********************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/boot.h>
#include <inttypes.h>

/* MITLED definiert ob LED bei zugriffen blinken soll. 
 * Ohne LED ist der Code unter 512 bytes gross!
 */
#ifndef LEDPIN
#define LEDDDR DDRB
#define LEDPORT PORTB
#define LEDPIN PB5
#endif

#ifndef I2C_ADDR
#define I2C_ADDR 0x04
#endif

#define STATUSLED(s) (LEDPORT = (LEDPORT &~_BV(LEDPIN)) | ((s) ? _BV(LEDPIN) : 0))

volatile int8_t byteanzahl;
volatile uint8_t kommando;
volatile uint8_t smbuscommand;
volatile int8_t smbuscount;
volatile uint8_t Checksum;
volatile uint32_t pageaddr;
volatile uint8_t buf[SPM_PAGESIZE];
volatile uint8_t bufaddr;
volatile uint16_t timeout;

void boot_program_page (uint32_t page);

#if MITLED
void
blink(uint8_t blinki, uint8_t puls, uint8_t pause)
{
  uint8_t pulstemp = puls;
  uint8_t pausetemp = pause;
  while(blinki > 0){
    pulstemp = puls;
    pausetemp = pause;
    STATUSLED(1);//PORTB |= _BV(PB0);
    while(pulstemp > 0){
     _delay_ms(25);
      pulstemp--;
    }
    STATUSLED(0);//PORTB &= ~_BV(PB0);
    while(pausetemp > 0){
      _delay_ms(25);
      pausetemp--;
    }
    blinki--;
  }
}
#endif
/* Interruptroutine des TWI
 */
ISR ( TWI_vect )
{
  /* hier wird das TWSR darauf geprüft, ob 0x80 darin steht, 
   * dieser Wert bedeutet, dass Daten empfangen wurden,
   * 0x60 würde zB bedeuten, dass er addressiert wurde 
   * und dies bestätigt hat
   */
 
  if((TWSR & 0xF8) == 0x80){
  /* Datenbyte wurde empfangen hier Code einfügen,
   * der bearbeitet werden soll, die empfangenen 
   * Daten kann man aus TWDR auslesen
   */
    if (byteanzahl == 0){
      smbuscommand = TWDR;
    }
    if (byteanzahl == 1){
      smbuscount = TWDR;
    }

    /* empfangen der flash schreib adresse */
    if (smbuscommand == 0xF1){
      if (byteanzahl == 0){
        bufaddr = 0;
        Checksum = 0;
      }
      else if (byteanzahl == 2){
        pageaddr = TWDR;
      }
      else if (byteanzahl == 3){
        pageaddr |= TWDR<<8;
      }
    }
    /* empfangen der datenbloecke */ 
    if ( smbuscommand == 0xF2 && byteanzahl > 1 && smbuscount > 0 && bufaddr < SPM_PAGESIZE){
      buf[bufaddr++] = TWDR;
      Checksum = (Checksum + TWDR) & 0xFF;
      smbuscount--;
      if(bufaddr == SPM_PAGESIZE){
        kommando = 0x04;
        return;
      }
    }

    /* bootloader quit und start der app */
    if (smbuscommand == 0xFE && smbuscount == 1 && byteanzahl == 2 && TWDR == 0xFF){
      kommando=0xFE;
    }

    byteanzahl++;
  }
  else if((TWSR & 0xF8) == 0x60){
    /* Der Avr wurde mit seiner Adresse angesprochen, TWI Address rec'd  */
    timeout = 0xFFFF;
    byteanzahl = 0;
  }

  /* hier wird an den Master gesendet -  Master requesting data */
  else if((TWSR & 0xF8) == 0xA8){
    if(smbuscommand == 0xF3){
      TWDR = 1;
    }
    else if(smbuscommand == 0xF0){
#if MITLED
      STATUSLED(1);
#endif
      TWDR = 1;
    }
    else if(smbuscommand == 0xF4){
#if MITLED
      STATUSLED(1);
#endif
      TWDR = 0x10;
      smbuscount = 0x00;
    }
    else{
      TWDR = 0;
    }
    byteanzahl++;
  }
  else if((TWSR & 0xF8) == 0xB8){
    if(smbuscommand == 0xF3){
      TWDR = Checksum;
    }
    else if(smbuscommand == 0xF0){
      TWDR = 0xFF;
    }
    else if(smbuscommand == 0xF4){
      if(smbuscount < 0x10){
      TWDR = pgm_read_byte(pageaddr++);
      smbuscount++;
      }
#if MITLED
      else STATUSLED(0);//PORTB &= ~_BV(PB0);
#endif
    }
    else{
      TWDR = 0;
    }
    byteanzahl++;
  }

  /* wenn der Interrupt ausgelöst wird, wird der TWI des µC blockiert,
   * damit man die Daten verarbeiten kann
   * um ihn wieder zu aktivieren, muss man eben folgenden Befehl ausführe
   */
  /* Re-enable TWI interrupt. */
  TWCR |= (1<<TWINT);
}

void
init_twi(void)
{
 /* INIT fuer den TWI i2c */
  TWCR = 0;
 /* hier wird die Addresse des µC festgelegt
  * (in den oberen 7 Bit, das LSB(niederwertigstes Bit)
  * steht dafür ob der µC auf einen general callreagiert
  */
  TWAR = I2C_ADDR << 1;

 /* TWI Control Register, hier wird der TWI aktiviert,
  *der Interrupt aktiviert und solche Sachen
  */
  TWCR = (1<<TWIE) | (1<<TWEN) | (1<<TWEA); 

  /* TWI Status Register init */
  TWSR &= 0xFC;

#if defined( PRR )
    PRR &= ~( 1 << PRTWI );
      STATUSLED(1);//PORTB |= _BV(PB0);
#endif

  /* hier werden Interrupts global aktiviert */
  sei();

  /* hier wird das TWI-Modul aktiv geschalten */
  TWCR |= (1<<TWINT);

}

int main(void)
{
  void (*applptr)( void ) = 0x0000;
  MCUCR = _BV(IVCE);  // enable wechsel der Interrupt Vectoren
  MCUCR = _BV(IVSEL); // Interrupts auf Boot Section umschalten
#if MITLED
  LEDDDR |= _BV(LEDPIN);  //LED pin auf ausgang
  blink(2, 20, 20);
#endif
  init_twi();
 

  while (1) { /* Endlosschleife weil 1 immer WAHR*/

    /* Bootloader Quit und sprung zur App */
    if(kommando == 0xFE || timeout == 0xFFFE){

      // Turn off the I2C interrupts
      TWCR = 0;

#if MITLED
      blink(5, 5, 10);
#endif
      cli();
      MCUCR = _BV(IVCE);  // enable wechsel der Interrupt Vectoren
      MCUCR = 0x00; // Interrupts auf Application Section umschalten
      applptr(); // Rücksprung zur Application
    }
    /* Datenblock (64byte) in Flash schreiben */
    if(kommando == 0x04){
#if MITLED
      STATUSLED(1);//PORTB |= _BV(PB0);
#endif
      boot_program_page (pageaddr);
#if MITLED
      STATUSLED(0);//PORTB &= ~_BV(PB0);
#endif
      kommando = 0x00;

      /* TWI wieder aktivieren */
      TWCR |= (1<<TWINT); 
    }
    if(TWSR == 0x00){
      init_twi();
    }
    if(timeout != 0xFFFF){
      timeout++;
      _delay_us(50);
    }

  }

  return 0;
}

void
boot_program_page (uint32_t page)
{
  uint16_t i;
  uint8_t sreg;

  // Disable interrupts.
  sreg = SREG;
  cli();

  eeprom_busy_wait ();

  boot_page_erase (page);
  boot_spm_busy_wait ();      // Wait until the memory is erased.
  uint8_t wbufaddr = 0;

  for (i=0; i<SPM_PAGESIZE; i+=2)
  {
    // Set up little-endian word.
    uint16_t w = buf[wbufaddr++];
    w += buf[wbufaddr++] << 8;

    boot_page_fill (page + i, w);
  }

  boot_page_write (page);     // Store buffer in flash page.
  boot_spm_busy_wait();       // Wait until the memory is written.

  // Reenable RWW-section again. We need this if we want to jump back
  // to the application after bootloading.

  boot_rww_enable ();

  // Re-enable interrupts (if they were ever enabled).

  SREG = sreg;
}
