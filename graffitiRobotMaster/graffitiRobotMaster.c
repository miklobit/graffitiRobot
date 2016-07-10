#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotMaster\graffitiRobotMaster.h"

char comando[LONGI_BUF];            // array para recibir las cadenas de caracteres de los comandos (buffer de recepciÃ³n)
unsigned int8 i;                     // -indice para apuntar a los elementos dentro del array comando[]
unsigned int8 cmd;                  // -Flag para indicar que se hay recepciÃ³n de mensaje en curso (para evitar interpretar
unsigned int8 estado;               //variable de estado del master
unsigned int8 estadoAnterior;        //variable que lleva el registro del estado anterior del master
const unsigned int8 dirEsclavo1 = 0x10;
const unsigned int8 dirEsclavo2 = 0x20;
char orden;				//variable para el envio de ordenes por I2C
int8 opcion1, opcion2;	//variables para el envio de ordenes por I2C

/* get16(k) - Función auxiliar que devuelve el valor numérico (int16) de una cadena decimal 
a partir del elemento k hasta encontrar el caracter null (similar a atoi o atol)*/
unsigned int16 get16(int k)
{
	unsigned int16 aux16=0;
	while(comando[k]!=0)  // después de la cadena decimal hay un caracter null (ASCII 0)
	{
		aux16=aux16*10+comando[k]-'0'; //'0' es 48 en decimal o 0x30 en hexa 
		k++;   
	}  
	return aux16;
}

/******************************************************************************/
/*********************  FUNCION ENVIO A PICS ESCLAVOS *************************/
/** Envia el caracter de la orden y dos numeros que complementan la orden    **/

void Envio_I2C( int8 direccion, char caracter, int8 numero1, int8 numero2)
{
	i2c_start();               // Comienzo comunicaciÃ³n
	i2c_write(direccion);      // DirecciÃ³n del esclavo en el bus I2C
	i2c_write(caracter);      // Caracter representativo del comando solicitado
	i2c_write(numero1);      // Numero complemento del comando
	i2c_write(numero2);      // Numero complemento del comando
	i2c_stop();                // Fin comunicacion
	delay_ms(50);
}

/*****************************************************************************/
/******************** FUNCION LECTURA DE PICS ESCLAVOS ***********************/
/**     Solicita la variable de estados del PIC de la dirección I2C         **/

void Lectura_I2C( int8 direccion, char caracter, int8 &estadoEsclavo) {

   i2c_start();            // Comienzo de la comunicación
   i2c_write(direccion);   // Dirección del esclavo en el bus I2C
   i2c_write(caracter);    // Posición de donde se leerá el dato en el esclavo
   i2c_start();            // Reinicio
   i2c_write(direccion+1); // Dirección del esclavo en modo lectura
   estadoEsclavo=i2c_read(0);       // Lectura del dato
   i2c_stop();             // Fin comunicación
   delay_ms(50);           // Espera finalización del envio
}

/*****************************************************************************/
/********************** FUNCION INTERPRETA COMANDOS UART *********************/
/**        interpreta los comandos contenidos en la cadena comando[]        **/

void interpreta(void)
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
		case 's':
		case 'S':
			orden = 'S';
			int8 estadoEsclavo;
			switch(comando[1])
			{
				case '0':					//envio del estado del master
					printf("%x\r", estado);
					break;
				case '1':					//envio del estado del esclavo 1
					Lectura_I2C( 0x10, orden, estadoEsclavo);
					printf("%x\r", estadoEsclavo);
					break;
				case '2':					//envio del estado del esclavo 2
					Lectura_I2C( 0x10, orden, estadoEsclavo);
					printf("%x\r", estadoEsclavo);
					break;
			}
			break;
		case 'h':
		case 'H':
			estado = 0x03;
			break;	
		case 't':
		case 'T':
			//aux16=get16(1);
			//if(aux16<60000) tpaso=aux16;
			break;
	}
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
			if (cmd == 1)				// si habÃ­a comando en curso
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

void main(void)
{
	/* Definicion de variables */
	estado = 0x00;						//Representa el estado del master
	estadoAnterior = 0x01;				//Definido con un valor diferente a estado solo a fines de que pase por el primer bucle de desenergizado
	set_tris_A(0x00);	
	bit_clear(TRISC,0);	//RC0 salida
	bit_set(TRISC,1);	//RC1 entrada
	bit_set(TRISC,2);	//RC2 entrada
	
	output_low(56);		//RC0=0
	
	opcion2=0x00;

	setup_adc_ports(NO_ANALOGS|VSS_VDD);
	setup_adc(ADC_CLOCK_DIV_2);
	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
	setup_timer_1(T1_DISABLED);
	setup_timer_2(T2_DISABLED,0,1);
	setup_comparator(NC_NC_NC_NC);		// This device COMP currently not supported by the PICWizard
	enable_interrupts(INT_RDA);
	enable_interrupts(GLOBAL);
//Setup_Oscillator parameter not selected from Intr Oscillator Config tab

	printf("ok\r");
	for (;;)
	{
		switch(estado)
		{
			case 0x00:					//Solicitar desenergizar motores a los esclavos
				orden = 'P';
				if(estadoAnterior & 0x01)
				{
					estadoAnterior = estado;
					printf("Desenergizando...\r");
					opcion1 = 0x00;
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
					output_A(estado);
				}
				break;
			case 0x01:					//Solicitar energizar motores a los esclavos
				if (estadoAnterior == 0x00)
				{
					estadoAnterior = estado;
					printf("Energizando...\r");
					opcion1 = 0x01;
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
					output_A(estado);
				}
				break;
			case 0x03:						//Solicitar hacer homing a los esclavos
				if(estadoAnterior & 0x01)
				{
					estadoAnterior = estado;
					output_A(estado);
					orden = 'H';										

					printf("Homing esclavo 1...\r");
					opcion1 = 0x00;										//orden H, opcion1 0x00 avanzar para permitir al otro esclavo hacer homing
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);	//pedir avanzar a esclavo 2
					opcion1 = 0x01;										//orden H, opcion1 0x01 retroceder para hacer homing
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);	//pedir homing esclavo 1 	
					//el esclavo1 pone portc1 en 1 hasta que termina de hacer homing
					while(input(PIN_C1) == 0){								//esperar que portc1 este en 1, indicando que empezo a hacer el homing
					}
					while(input(PIN_C1) == 1){								//esperar que portc1 este en 0, indicando que termino el homing
					}
					orden = 'X';
					Envio_I2C(dirEsclavo2, orden, opcion1, opcion2);	//Orden de detener el avance al esclavo 2

					orden = 'H';
					printf("Homing esclavo 2...\r");
					opcion1 = 0x00;										//orden H, opcion1 0x00 avanzar para permitir al otro esclavo hacer homing
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);	//pedir avanzar a esclavo 1
					opcion1 = 0x01;										//orden H, opcion1 0x01 retroceder para hacer homing
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);	//pedir homing esclavo 1 	
					//el esclavo2 pone portc2 en 1 hasta que termina de hacer homing
					while(input(PIN_C2) == 0){								//esperar que portc2 este en 1, indicando que arranco el homing
					}     
					while(input(PIN_C2) == 1){								//esperar que portc2 este en 0, indicando que termino el homing
					}     
					orden = 'X';
					Envio_I2C(dirEsclavo1, orden, opcion1, opcion2);	//Orden de detener el avance al esclavo 1
					estado = 0x05;
				}
				break;
			case 0x05:					//Esperando ordenes de dibujo (Punto de Inicio y Punto de Final)
				if (estadoAnterior != estado)
				{
					estadoAnterior = estado;
					printf("Esperando Ordenes...\r");
				}
				output_A(estado);
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
