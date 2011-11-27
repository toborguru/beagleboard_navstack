/*********************************************
* vim: set sw=8 ts=8 si :
* Author: Jochen Roessner Copyright: GPL v.2 
* Bootloader ber TWI (I2C) Interface
* 
* Chip Typ           : Atmega8 
* Clock Frequenz     : 8,000000 MHz
*********************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define F_CPU 8000000UL  // 8 MHz
#include <util/delay.h>
#include <avr/boot.h>
#include <inttypes.h>

volatile uint8_t byteanzahl;
volatile uint8_t txbyte; 
volatile uint8_t runbootload;
volatile uint8_t kommando;
volatile uint32_t pageaddr;
volatile uint8_t buf[64];
volatile uint8_t bufaddr;


//void blink(uint8_t blinki, uint8_t puls, uint8_t pause) __attribute__ ((section (".bootloader")));
//void boot_program_page (uint32_t page, uint8_t *buf) __attribute__ ((section (".bootloader")));
//void bootload(void) __attribute__ ((section (".bootloader")));
//SIGNAL (SIG_2WIRE_SERIAL) __attribute__ ((section (".bootloader")));
//void init_twi(void) __attribute__ ((section (".bootloader")));
//void boot_program_page (uint32_t page, uint8_t *buf);
//void bootload(void);
void boot_program_page (uint32_t page);


void
blink(uint8_t blinki, uint8_t puls, uint8_t pause)
{
  uint8_t pulstemp = puls;
  uint8_t pausetemp = pause;
  while(blinki > 0){
    pulstemp = puls;
    pausetemp = pause;
    PORTB |= _BV(PB0);
    while(pulstemp > 0){
     _delay_ms(25);
      pulstemp--;
    }
    PORTB &= ~_BV(PB0);
    while(pausetemp > 0){
      _delay_ms(25);
      pausetemp--;
    }
    blinki--;
  }
}

/* Interruptroutine des TWI
 */
SIGNAL (SIG_2WIRE_SERIAL)
{
  /* hier wird das TWSR darauf geprüft, ob 0x80 darin steht, dieser Wert bedeutet, dass Daten
         * empfangen wurden, 0x60 würde zB bedeuten, dass er addressiert wurde und dies bestätigt hat
         */
  
  if((TWSR & 0xF8) == 0x80){
    /* Datenbyte wurde empfangen
     * hier Code einfügen, der bearbeitet werden soll, die empfangenen Daten kann man aus
                 * TWDR auslesen
                 */
    uint8_t rxbyte = TWDR; /*hier das empfange byte verarbeiten */
    if (rxbyte == 0xFF && byteanzahl == 0){
      runbootload++;
    }
    if (rxbyte == 0x04 && byteanzahl == 1 && runbootload == 1){
      runbootload++;
    }
    if (byteanzahl == 2 && runbootload == 2){
      pageaddr = rxbyte<<8;
      runbootload++;
    }
    if (byteanzahl == 3 && runbootload == 3){
      pageaddr += rxbyte;
      runbootload++;
      bufaddr = 0;
    }
    if (byteanzahl > 3 && runbootload == 4){
      buf[bufaddr++] = rxbyte;
      if(bufaddr == 64){
        kommando = 0x04;
      }
    }


    byteanzahl++;
  }
  else if((TWSR & 0xF8) == 0x60){
    /* Der Avr wurde mit seiner Adresse angesprochen  */
    runbootload = 0;
    byteanzahl = 0;
  }

  /* hier wird an den Master gesendet */
  else if((TWSR & 0xF8) == 0xA8){
    txbyte = 1;    //demo zaehler auf 1 setzen
    TWDR = txbyte; //zur demo den zaehler txbyte senden
    byteanzahl = 0;
  }
  else if((TWSR & 0xF8) == 0xB8){
    txbyte++;
    TWDR = txbyte;
    byteanzahl++;
  }
  /* fuer einen zukuenftigen general call */
  /* 
  if((TWSR & 0xF8) == 0x70){
      //general call
            motorschalter = 2;
  }
  */
  
  /* wenn der Interrupt ausgelöst wird, wird der TWI des µC blockiert,                      
   * damit man die Daten verarbeiten kann
   * um ihn wieder zu aktivieren, muss man eben folgenden Befehl ausführe
   */ 
  TWCR |= (1<<TWINT); //0x80; //0B10000000;
  

}

void
init_twi(void)
{
  /* INIT fuer den TWI i2c
   * diese 6 Zeilen Code sind für das Initialisieren des TWI-Moduls zuständig, also kommt das Stück
   * Code am besten irgendwo in die Mainmethode vor die Endlosschleife
   */
  
  /* hier wird die Addresse des µC festgelegt(in den oberen 7 Bit, das LSB(niederwertigstes Bit) steht dafür
   * , ob der µC auf einen general callreagiert
   */ 
  TWAR = 0x08;
  
  /* TWI Control Register, hier wird der TWI aktiviert, der Interrupt aktiviert und solche Sachen
   */
  TWCR = (1<<TWIE) | (1<<TWEN) | (1<<TWEA); //  0x45; //0b01000101;
  
  /* TWI Bitrate Register, für die Frequenz des TWI wichtig
   */
  //TWBR = 0x01;
  
  /* TWI Status Register, die 2 niederwertigsten sind für den Prescaler zur Taktberechnung, aus de
   * anderen kann man auslesen, was genau passiert, wenn ein Interrupt ausgelöst wird, es gibt nämli
   * für den TWI nur einen
   */
  TWSR &= 0xFC; //0B11111100;

}

int main(void)
{
  void (*applptr)( void ) = 0x0000;
  GICR = _BV(IVCE);  // enable wechsel der Interrupt Vectoren
  GICR = _BV(IVSEL); // Interrupts auf Boot Section umschalten
  runbootload = 0;
  init_twi();
 
  /* hier werden Interrupts global aktiviert
   */
  sei();
  /* hier wird das TWI-Modul aktiv geschalten, ab hier man den µC per TWI ansteuern, den Befehl aber
   * auf jeden Fall hinter das "sei();", da es sonst nicht geht
   */
  TWCR |= (1<<TWINT); //0x80; //0B10000000;

  while (1) { /* Endlosschleife weil 1 immer WAHR*/
    /* donothingloop ;-) */
    blink(1,40,40);
    if(kommando == 0xFF){
      cli();
      GICR = _BV(IVCE);  // enable wechsel der Interrupt Vectoren
      GICR = 0x00; // Interrupts auf Application Section umschalten
      applptr(); // Rücksprung zur Application
    }
    if(kommando == 0x04){
      //cli();
      boot_program_page (pageaddr);
      blink(1,1,1);
      kommando = 0x00;
      //sei();
    }

  }

  return 0;
}


/*void bootload()
{
  uint8_t buf[64];
  uint8_t bufaddr;
  for(bufaddr = 0;bufaddr < 64;bufaddr++){
    buf[bufaddr] = bufaddr;
  }

  boot_program_page (0x00001200, &buf[0]);

  runbootload = 0;

}
*/

//void boot_program_page (uint32_t page, uint8_t *buf)
void boot_program_page (uint32_t page)
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

    /*uint16_t w = *wbuf++;
    w += (*wbuf++) << 8;
    */
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
