#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotMaster\graffitiRobotMaster.h"

char comando[LONGI_BUF];            // array para recibir las cadenas de caracteres de los comandos (buffer de recepci贸n)
unsigned int8 i;                     // -indice para apuntar a los elementos dentro del array comando[]
unsigned int8 cmd;                  // -Flag para indicar que se hay recepci贸n de mensaje en curso (para evitar interpretar
unsigned int8 estado;               //variable de estado del master
unsigned int8 estadoAnterior;        //variable que lleva el registro del estado anterior del master
const unsigned int8 dirEsclavo1 = 0x10;
const unsigned int8 dirEsclavo2 = 0x20;

/* get16(k) - Funci髇 auxiliar que devuelve el valor num閞ico (int16) de una cadena decimal 
a partir del elemento k hasta encontrar el caracter null (similar a atoi o atol)*/
unsigned int16 get16(int k)
{
	unsigned int16 aux16=0;
	while(comando[k]!=0)  // despu閟 de la cadena decimal hay un caracter null (ASCII 0)
	{
		aux16=aux16*10+comando[k]-'0'; //'0' es 48 en decimal o 0x30 en hexa 
		k++;   
	}  
	return aux16;
}
   
/*****************************************************************************/
/********************** FUNCION INTERPRETA COMANDOS UART *********************/
/**        interpreta los comandos contenidos en la cadena comando[]        **/

void interpreta()
{
	//unsigned int16 aux16=0;
	switch(comando[0])
	{
		case 'p':
		case 'P':
			switch(comando[1])
			{
				case '0':
					estado = 0x00;
					break;
				case '1':
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

/******************************************************************************/
/******************* FUNCI脫N ESCRITURA EN PICS ESCLAVOS ***********************/
/** Guarda un dato (n煤mero aleatorio) en la posici贸n de memoria indicada y en el 
          PIC correspondiente a la direcci贸n I2C proporcionada               **/

void Envio_I2C(direccion, caracter, numero)
{
	i2c_start();               // Comienzo comunicaci贸n
	i2c_write(direccion);      // Direcci贸n del esclavo en el bus I2C
	i2c_write(caracter);      // Caracter representativo del comando solicitado
	i2c_write(numero);      // Numero complemento del comando
	i2c_stop();                // Fin comunicaci贸n
	delay_ms(50);
}

#int_RDA
void  RDA_isr(void) 
{
	char dato;
	dato = getc();
	switch(dato)
	{
		case ':':						//si es delimitador de inicio 
			i = 0;						//inicializa contador i
			cmd = 1;					//y activa bandera que indica que hay comando en curso
			break;      
		case 13:						//si es delimitador de final 
			if (cmd == 1)				// si hab铆a comando en curso
			{
				comando[i] = 0;			//termina cadena de comando con caracter null
				interpreta();			//va a interpretar el comando
				cmd = 0;				//desactiva bandera de comando en curso
			}
			break;
		default:
			if (i < LONGI_BUF)			//si contador menor que longitud del buffer
			{
				comando[i] = dato;		//pone caracter en cadena
				i++;					//incrementa contador i
			}
			break;
	}
}

#int_SSP
void  SSP_isr(void) 
{

}



void main()
{
	/* Definicion de variables */
	estado = 0x00;						//Representa el estado del master
	estadoAnterior = 0x01;				//Definido con un valor diferente a estado solo a fines de que pase por el primer bucle de desenergizado
	TRISA = 0x00;
	TRISC = 0x06;
   
	//char orden;

	setup_adc_ports(NO_ANALOGS|VSS_VDD);
	setup_adc(ADC_CLOCK_DIV_2);
	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
	setup_timer_1(T1_DISABLED);
	setup_timer_2(T2_DISABLED,0,1);
	setup_comparator(NC_NC_NC_NC);		// This device COMP currently not supported by the PICWizard
	enable_interrupts(INT_RDA);
	enable_interrupts(INT_SSP);
	enable_interrupts(GLOBAL);
//Setup_Oscillator parameter not selected from Intr Oscillator Config tab

	printf("ok\r");
	for (;;)
	{
		switch(estado)
		{
			case 0x00:					//Solicitar desenergizar motores a los esclavos
				if(estadoAnterior & 0x01)
				{
					estadoAnterior = estado;
					printf("Desenergizando...\r");
					char orden = 'P';
					int8 opcion = 0x00;
					Envio_I2C(dirEsclavo1 , orden, opcion);
					Envio_I2C(dirEsclavo2 , orden, opcion);
					PORTA = estado;
				}
				break;
			case 0x01:					//Solicitar energizar motores a los esclavos
				if (estadoAnterior == 0x00)
				{
					estadoAnterior = estado;
					printf("Energizando...\r");
					char orden = 'P';
					int8 opcion = 0x01;
					Envio_I2C(dirEsclavo1 , orden, opcion);
					Envio_I2C(dirEsclavo2 , orden, opcion);
					PORTA = estado;
				}
				break;
			case 0x03:					//Solicitar hacer homing a los esclavos
				break;
			case 0x05:					//Esperando ordenes de dibujo (Punto de Inicio y Punto de Final)
				break;
			case 0x0D:					//Enviar coordenadas de inicio a los esclavos
				break;
			case 0x15:					//Enviar valor de interrupcion de timer a los esclavos
				break;
			case 0x3D:					//Enviar coordenadas de final a los esclavos
				break;
			}
	}
}
