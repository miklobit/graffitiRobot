#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotSlave\2SlaveUpRight\graffitiRobotSlave2.h"

unsigned int8 estado;               //variable de estado del master
unsigned int8 estadoAnterior;       //variable que lleva el registro del estado anterior del master
const int8 STEP[] = {0x01,0x03,0x02,0x06,0x04,0x0B,0x07,0x08};
byte fstate;                     //Guardara el estado del bus I2C
int8 numero;
char orden;

/*****************************************************************************/
/********************** FUNCION INTERPRETA COMANDOS I2C *********************/
/**        interpreta los comandos contenidos en la cadena comando[]        **/

void interpreta()
{
   //unsigned int16 aux16=0;
   switch(orden)
   {
      case 'p':
      case 'P':
         switch(numero)
         {
            case 0x00:
               estado = 0x00;
               break;
            case 0x01:
               estado = 0x01;
               break;
         }
         break;
      case 't':
      case 'T':
         //aux16=get16(1);
         //if(aux16<60000) tpaso=aux16;
         break;
   }
}
/*
#int_TIMER1
void  TIMER1_isr(void) 
{

}*/

/******************************************************************************/
/***************** FUNCIÓN INTERRUPCIÓN POR RECEPCION I2C *********************/
/**  Saltará a esta función cada vez que se detecte actividad en el bus I2C  **/

#int_SSP
void  SSP_isr(void) 
{
   int incoming;                    //Variable donde se recibe el byte que manda el maestro
   fstate = i2c_isr_state();           //Lectura del estado del bus I2c la interrupción
   /* Solicitud de lectura del esclavo por el master */
   if(fstate == 0x80) {         
      i2c_write(estado);               //Manda al maestro el estado del esclavo
   }
   /* Sino está solicitando lectura es que está enviando algo */
   else {                           //Sino es que hay dato en el bus I2C...
      incoming = i2c_read();           //... lo lee
      if (fstate == 1) {               //Información recibida corresponde a la posicion
         orden = incoming;          //Se guarda posición
      }
      else if (fstate == 2) {          //Información recibida corresponde al dato
         numero = incoming;         //Se guarda dato
         interpreta();
      }
   }
}



void main()
{
   TRISA = 0x00;
   TRISB = 0x00;
   TRISC0 = 1;
   TRISC1 = 1;
   TRISC2 = 0;

   estado = 0x00;                //Representa el estado del master
   estadoAnterior = 0x01;        //Definido con un valor diferente a estado solo a fines de que pase por el primer bucle de desenergizado
   
   unsigned int stepIndex = 0;

   setup_adc_ports(NO_ANALOGS|VSS_VDD);    setup_adc(ADC_CLOCK_DIV_2);
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);
   setup_timer_2(T2_DISABLED,0,1);
   setup_comparator(NC_NC_NC_NC);// This device COMP currently not supported by the PICWizard
// enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_SSP);
   enable_interrupts(GLOBAL);
//Setup_Oscillator parameter not selected from Intr Oscillator Config tab

   for (;;)
   {
      switch(estado)
      {
         case 0x00:              //Solicitar desenergizar motores a los esclavos
            if(estadoAnterior & 0x01)
            {
               estadoAnterior = estado;
               PORTB = 0x00;
               PORTA = estado;
            }
            break;
         case 0x01:              //Solicitar energizar motores a los esclavos
            if (estadoAnterior == 0x00)
            {
               estadoAnterior = estado;
               PORTB = STEP[stepIndex];
               PORTA = estado;
            }
            break;
         case 0x03:              //Solicitar hacer homing a los esclavos
            break;
         case 0x05:              //Esperando ordenes de dibujo (Punto de Inicio y Punto de Final)
            break;
         case 0x0D:              //Enviar coordenadas de inicio a los esclavos
            break;
         case 0x15:              //Enviar valor de interrupcion de timer a los esclavos
            break;
         case 0x3D:              //Enviar coordenadas de final a los esclavos
            break;
      }
   }
}
