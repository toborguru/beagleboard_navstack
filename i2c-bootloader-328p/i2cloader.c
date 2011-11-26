/**********************************************************
 Copyright(C) 2006 Jochen Roessner <jochen@lugrot.de>

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#define MAX_LINE_SIZE 1024
#define MEMORY_SIZE 1024*32
#include "i2c-host-routines.h"

typedef unsigned char byte;

void usage(void){
  fprintf(stderr,
    "Benutzung: i2cloader [-d devnr] [-r Addresse] [-bi2c addr] [-f file.hex] [-ai2c addr] [-s F00102FF ] [-e FF] [-smbusdelay]\n");
  fprintf(stderr,"Flashdownload: -r Startaddresse\n");
  fprintf(stderr,"Bootloader i2c Addresse: -bi2c Addresse (default 0x04)\n");
  fprintf(stderr,"Uploadhexfile: -f upload.hex\n");
  fprintf(stderr,"Application i2c Addresse: -ai2c Addresse (default 0x04)\n");
  fprintf(stderr,"Startcode: -s HEXCODE die den AVR in Bootloadmodus schaltet\n");
  fprintf(stderr,"Exitcode: -e HEXCODE Der Befehl um den Bootloader zu verlassen\n");
  fprintf(stderr,"i2c-Devicenummer: -d Nummer\n");
  fprintf(stderr,"Delay (50ms) zwischen den Schreibvorgaengen: -smbusdelay\n");
  fprintf(stderr,"Ping Funktion um den Timeout abzubrechen: -ping\n");
  
}

int 
main(int argc, char **argv)
{
  if(argc < 2){
    usage();
    return 1;
  }

  char Line[MAX_LINE_SIZE];
  char *Data;
  byte *Memory_Block;
  unsigned int Byte_Addr = 0;
  unsigned int Phys_AddrBegin = 0;
  unsigned int Phys_AddrEnde = 0;
  unsigned int readflash = 0;
  unsigned int i;
  unsigned int begin = 0;
  int PadByte = 0xFF;
  unsigned int  Anzahl_Bytes;
  unsigned int  Erstes_Wort;
  unsigned int  Type;
  byte  Checksum = 0;
  byte  Data_Str[MAX_LINE_SIZE];
  char *Filename = NULL;
  char *startcode = NULL;
  char *exitcode = NULL;
  int devicenr = 0;
  int appli2caddr = 4;
  int booti2caddr = 4;
  //Filename = argv[1];
  int temp2;
  int delay = 0;
  int timeoutping = 0;
  byte recv[35];
  unsigned int recvlen;
  
  argv ++; /* skip first arg, which is our program name ... */
  while(*argv) {

    if(! strcmp(*argv, "-smbusdelay")){
      delay = 50;
      argv++;
      continue;
    }

    if(! strcmp(*argv, "-ping")){
      timeoutping = 100;
      argv++;
      continue;
    }

    if(! argv[1]) {
      usage();
      return 2;
    }


    if(! strcmp(*argv, "-f"))
      Filename = argv[1];

    else if(! strcmp(*argv, "-s"))
      startcode = argv[1];

    else if(! strcmp(*argv, "-r")){
      Phys_AddrBegin = (unsigned int) strtoul(argv[1], NULL, 0);
      readflash = 1;
    }

    else if(! strcmp(*argv, "-e"))
      exitcode = argv[1];
    
    else if(! strcmp(*argv, "-d"))
      devicenr = (int) strtoul(argv[1], NULL, 0);

    else if(! strcmp(*argv, "-ai2c"))
      appli2caddr = (int) strtoul(argv[1], NULL, 0);

    else if(! strcmp(*argv, "-bi2c"))
      booti2caddr = (int) strtoul(argv[1], NULL, 0);


    argv += 2;
  }

  Memory_Block = (byte *) malloc(MEMORY_SIZE);
  if (Memory_Block == NULL){
    fprintf(stderr,"Malloc schlug fehl\n");
    return 1;
  }
  memset (Memory_Block,PadByte,MEMORY_SIZE);

  /* open the i2c port */
  int file = i2c_open(devicenr, appli2caddr);
  if(file < 0){ /* i2c_open already emitted an error message */
    fprintf(stderr,"Konnte Device nicht oeffnen\n");
    //return 1;
  }
  else{
    if(startcode != NULL){
      int startcodelen = strlen(startcode);
      if((startcodelen % 2) > 0){
        fprintf(stderr,"Startcode hat keine Bytelaenge\n");
      return 1;
      }
    
      byte* startcodebin = (byte *) malloc(startcodelen/2);
      if (startcodebin == NULL){
        fprintf(stderr,"Malloc schlug fehl\n");
        return 1;
      }
      
      for(i= 0;i < startcodelen/2; i++){
        sscanf (startcode, "%2x",&temp2);
        startcode += 2;
        startcodebin[i] = temp2;
      }
      fprintf(stderr, "schalte auf Bootloader\n");
      if(i2c_send_to_avr(file, startcodebin[0], (char *) &startcodebin[1], (startcodelen-1)/2)) {
        fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
        //return 1;
      }
      sleep(3);
  
    }
    close(file);
  }
  /* open the i2c port */
  file = i2c_open(devicenr, booti2caddr);
  if(file < 0){ /* i2c_open already emitted an error message */
    fprintf(stderr,"Konnte Device nicht oeffnen\n");
    return 1;
  }

  /* test ob der bootloader laeuft */
  do{

    if(i2c_recv_from_avr(file, 0xF0, (char *) &recv[0], &recvlen)) {
        fprintf(stderr, "couldn't successfully recv data from avr. sorry.\n");
        usleep(300000);
        //return 1;
    }else{
      if(recvlen == 1 && recv[0] == 0xFF){
        printf("Bootloader Antwort: %02x = aktiv :-)\n", recv[0]);
        timeoutping = 0;
      }else{
        printf("Bootloader Antwort: %02x, Bootloader nicht aktiv :-(\n", recv[0]);
        if(timeoutping < 2)
          return 1;
      }
    }
  }while(timeoutping-- > 0);

  /* flash auslesen und als intel hex format ausgeben */
  if(readflash == 1){
    fprintf(stderr, "setze Zeiger auf Adresse %04x\n", Phys_AddrBegin);
    byte FlashAddr[2];

    // Verify we can write to all locations?
    while(Phys_AddrBegin < 0x6FFF){
      //memcpy(&FlashAddr[4], Memory_Block, 64);
      //fwrite(FlashAddr, 2, 2, stdout);
      //fwrite(Memory_Block, 1, 64, stdout);
      FlashAddr[0]=Phys_AddrBegin & 0xFF;
      FlashAddr[1]=(Phys_AddrBegin>>8) & 0xFF;
      //fprintf(stderr, "setze zeiger auf Adresse %04x\n", Phys_AddrBegin);
      if(i2c_send_to_avr(file, 0xF1, (char *) &FlashAddr[0], 2)) {
        fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
        return 1;
      }
  
      if(i2c_recv_from_avr(file, 0xF4, (char *) &recv[0], &recvlen)) {
        fprintf(stderr, "couldn't successfully recv data from avr. sorry.\n");
        return 1;
      }
      if(recvlen == 0x10){
        Checksum = recvlen + (Phys_AddrBegin >> 8) + (Phys_AddrBegin & 0xFF);
        printf(":10%04X00",Phys_AddrBegin);
        for(i=0;i<0x10;i++){
        printf("%02X", recv[i]);
        Checksum = (Checksum + recv[i]) & 0xFF;
        }
        printf("%02X\n", (0x100-Checksum) & 0xFF);
      }
      Phys_AddrBegin+=16;
    }

  }

  byte* exitcodebin;
  int exitcodelen;
  if(exitcode != NULL){
    exitcodelen = strlen(exitcode);
    if((exitcodelen % 2) > 0){
      fprintf(stderr,"Exitcode hat keine Bytelaenge\n");
     return 1;
    }
  
    exitcodebin = (byte *) malloc(exitcodelen/2);
    if (exitcodebin == NULL){
      fprintf(stderr,"Malloc schlug fehl\n");
      return 1;
    }
    
    for(i= 0;i < exitcodelen/2; i++){
      sscanf (exitcode, "%2x",&temp2);
      exitcode += 2;
      exitcodebin[i] = temp2;
    }

  }

if(Filename != NULL){
  FILE  *Hexfile;
  Hexfile = fopen(Filename,"r");
  if (Hexfile == NULL){
    fprintf(stderr,"Konnte Hexfile nicht oeffnen\n");
    return 1;
  }

  do /* repeat until EOF(Filin) */
      {
      /* Read a line from input file. */
      fgets(Line,MAX_LINE_SIZE,Hexfile);
  
      /* Remove carriage return/line feed at the end of line. */
      i = strlen(Line)-1;
  
      if (Line[i] == '\n') Line[i] = '\0';
  
      /* Scan the first two bytes and nb of bytes.
      The two bytes are read in First_Word since it's use depend on the
      record type: if it's an extended address record or a data record.
      */
      sscanf (Line, ":%2x%4x%2x%s",&Anzahl_Bytes,&Erstes_Wort,&Type,Data_Str);
      if(begin == 0){
        Phys_AddrBegin = Erstes_Wort;
        begin = 1;
      }
      if(Type == 0){
        Checksum = Anzahl_Bytes + (Erstes_Wort >> 8) + (Erstes_Wort & 0xFF) + Type;
    
        Data = (char *) Data_Str;
/*erst testen        if(Erstes_Wort != (Byte_Addr + Phys_AddrBegin){
          Byte_Addr = Erstes_Wort - Phys_AddrBegin;
      }*/
        for (i= Anzahl_Bytes; i > 0; i--){
          sscanf (Data, "%2x",&temp2);
          Data += 2;
          Memory_Block[Byte_Addr++] = temp2;
          Checksum = (Checksum + temp2) & 0xFF;
        };
        
        /* Read the Checksum value. */
        sscanf (Data, "%2x",&temp2);
        
        /* Verify Checksum value. */
        Checksum = (Checksum + temp2) & 0xFF;
        
        /*if ((Checksum != 0) && Enable_Checksum_Error){
          Status_Checksum_Error = TRUE;
        }*/
    
        //fprintf(stderr,"%i, %i, %i, %0x\n", Anzahl_Bytes, Erstes_Wort, Type, Checksum);
        //fprintf(stderr,"%s\n", Data_Str);
      }
  
    } while (!feof (Hexfile));
      Phys_AddrEnde = Byte_Addr + Phys_AddrBegin;
      if((Phys_AddrBegin % 128) == 0)
      { 
        fprintf(stderr,"Adressbegin: 0x%x,Anzahl 0x%x,Ende 0x%x\n", Phys_AddrBegin,Byte_Addr,Phys_AddrEnde);
      }
  
      //fwrite(Memory_Block, 1, Byte_Addr, stdout);
      byte FlashAddr[2];
      while(Phys_AddrBegin < Phys_AddrEnde){
        //memcpy(&FlashAddr[4], Memory_Block, 64);
        //fwrite(FlashAddr, 2, 2, stdout);
        //fwrite(Memory_Block, 1, 64, stdout);
        FlashAddr[0]=Phys_AddrBegin & 0xFF;
        FlashAddr[1]=(Phys_AddrBegin>>8) & 0xFF;
        fprintf(stderr, "schreibe an Adresse %04x\n", Phys_AddrBegin);
        if(i2c_send_to_avr(file, 0xF1, (char *) &FlashAddr[0], 2)) {
          fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
          return 1;
        }
        i=0;
        while(i < 128){
          if(i2c_send_to_avr(file, 0xF2, (char *) &Memory_Block[i], 16)) {
            fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
            return 1;
          }

          i+=16;
          if(delay > 0)
            usleep(delay*1000);
        }
        //usleep(50000);
        //sleep(3);
        Checksum = 0;
        for(i=0;i<128;i++)
          Checksum = (Checksum + Memory_Block[i]) & 0xFF; 
  
        if(i2c_recv_from_avr(file, 0xF3, (char *) &recv[0], &recvlen)) {
          fprintf(stderr, "couldn't successfully recv data from avr. sorry.\n");
          return 1;
        }
        if(Checksum != recv[0])
          fprintf(stderr,"Checksum Error bei %04x: %02x != %02x\n",Phys_AddrBegin, Checksum, recv[0]);

        Phys_AddrBegin+=128;
        Memory_Block+=128;
      }
    }
    if(exitcode != NULL){
      fprintf(stderr, "schalte auf Applikation\n");
      sleep(3);
      if(i2c_send_to_avr(file, 0xFE, (char *) &exitcodebin[0], exitcodelen/2)) {
        fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
        return 1;
      }
      
    }


  return 0;
}

