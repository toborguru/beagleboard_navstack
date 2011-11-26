 /* Copyright(C) 2005 Jochen Roessner <jochen@lugrot.de>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/eeprom.h>

volatile uint8_t byteanzahl; //zaehler der bytes beim twi empfang
volatile uint8_t runbootload;//statuszaehler fuer den sprung zum bootloader
volatile uint8_t runbootseq[3] = {0x01,0x02,0xFF};

volatile uint8_t smbuscommand;
volatile uint8_t smbuscount;

void
blink(uint8_t blinki, uint8_t puls, uint8_t pause){
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
  
  if((TWSR & 0xF8) == 0x80){
    /* Datenbyte wurde empfangen
     * TWDR auslesen
     */
    if (byteanzahl == 0){
      smbuscommand = TWDR;
    }
    if (byteanzahl == 1){
      smbuscount = TWDR;
    }
    if(smbuscommand == 0xF0 && smbuscount == 3){
      if(runbootseq[byteanzahl-2] == TWDR){
        runbootload++;
      }
      else{
        runbootload = 0;
      }
    }

    byteanzahl++;
  }
  else if((TWSR & 0xF8) == 0x60){
    /* Der Avr wurde mit seiner Adresse angesprochen  */
    byteanzahl = 0;
    runbootload = 0;
    PORTB |= _BV(PB0);
  }

  /* hier wird an den Master gesendet */
  else if((TWSR & 0xF8) == 0xA8){
    TWDR = 0x10; //zur demo den wert senden
  }
  else if((TWSR & 0xF8) == 0xB8){
    TWDR++;      //zur demo den wert erhoehen und senden
    byteanzahl++;
  }
  else{
    PORTB &= ~_BV(PB0);
  }
  /* fuer einen zukuenftigen general call */
  /* 
  if((TWSR & 0xF8) == 0x70){
      //general call
            motorschalter = 2;
  }
  */

  TWCR |= (1<<TWINT); //TWI wieder aktivieren

}


void
init_twi(void)
{
  /* INIT fuer den TWI i2c
   * hier wird die Addresse des µC festgelegt
   * (in den oberen 7 Bit, das LSB(niederwertigstes Bit)
   * steht dafür ob der µC auf einen general callreagiert
   */ 
  TWAR = 0x10;
  
  /* TWI Control Register, hier wird der TWI aktiviert, der Interrupt aktiviert und solche Sachen
   */
  TWCR = (1<<TWIE) | (1<<TWEN) | (1<<TWEA); 
  
  /* TWI Status Register init */
  TWSR &= 0xFC; 

#if defined( PRR )
    PRR &= ~( 1 << PRTWI );
#endif
 
}

int
main(void)
{
  void (*bootptr)( void ) = (void *) 0x0C00;
  DDRB |= _BV(PB0); //LED pin auf ausgang
  init_twi();
  runbootload = 0;
  blink(3, 20, 20);
  sei();
  TWCR |= (1<<TWINT); //TWI-Modul aktiv

  while (1) { /* Endlosschleife weil 1 immer WAHR*/
    /* donothingloop ;-) */
    if (runbootload == 3){
      //blink(4, 2, 10);
      runbootload = 0;
      bootptr();
    }
  }

  return 0;
}
  
