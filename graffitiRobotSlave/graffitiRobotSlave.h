#include <16F886.h>
#device adc=8

#FUSES NOWDT                 	//No Watch Dog Timer
#FUSES INTRC                    	//High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES NOPUT                 	//No Power Up Timer
#FUSES MCLR                  	//Master Clear pin enabled
#FUSES NOPROTECT             	//Code not protected from reading
#FUSES NOCPD                 	//No EE protection
#FUSES NOBROWNOUT            	//No brownout reset
#FUSES IESO                  	//Internal External Switch Over mode enabled
#FUSES FCMEN                 	//Fail-safe clock monitor enabled
#FUSES NOLVP                 	//No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NODEBUG               	//No Debug mode for ICD
#FUSES NOWRT                 	//Program memory not write protected
#FUSES BORV40                	//Brownout reset at 4.0V
#FUSES RESERVED              	//Used to set the reserved FUSE bits

#use delay(clock=8000000)
#use i2c(Slave,Fast,sda=PIN_C4,scl=PIN_C3,address=0x1F)
#use fast_io(A)

#BYTE PORTA = 0x05
#BYTE TRISA = 0x85

#BYTE PORTB = 0x06
#BYTE TRISB = 0x86

#BYTE PORTC = 0x07
#BYTE TRISC = 0x87

#BYTE SSPADD = 0x93

