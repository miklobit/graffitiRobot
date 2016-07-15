#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotMaster\graffitiRobotMaster.h"
//#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotMaster\reducedmath.h"
#include <math.h>
/*****************************************************************************/
/**********************        Variables Globales        *********************/

/** Variables de estado **/
unsigned int8 estado;               //variable de estado del master
unsigned int8 estadoAnterior;        //variable que lleva el registro del estado anterior del master

/** Variables para comunicación I2C**/
const unsigned int8 dirEsclavo1 = 0x10;
const unsigned int8 dirEsclavo2 = 0x20;
char orden;				//variable para el envio de ordenes por I2C
int8 opcion1, opcion2;	//variables para el envio de ordenes por I2C

/** Variables para comunicación UART**/
char comando[LONGI_BUF];            // array para recibir las cadenas de caracteres de los comandos (buffer de recepciÃ³n)
unsigned int8 ii;                     // -indice para apuntar a los elementos dentro del array comando[]
unsigned int8 cmd;                  // -Flag para indicar que se hay recepciÃ³n de mensaje en curso (para evitar interpretar

/** Variables para el calculo de coordenadas**/
int16 xi, yi, xf, yf;	//variables de almacenamiento de las coordenadas cartesianas de la linea a dibujar
short finalRecibido;
int16 objIniEscIzq, objIniEscDer;
int16 objFinEscIzq, objFinEscDer;
const int16 anchoDibujo = 2500;		//MAX 2573.6-->> cantidad de pasos max 65535 y resolución de 0.9 grados
/*-variables constructivas-*/
const float32 pasoDist = 25.4648; //1paso/0.9grados*360grados/(pi*5)mm
/** Variables para el calculo de tiempo de interrupcion de timers**/
const int16 Vmax = 0xF03C;
const int16 IntTimMin = 0x0FC3; //intervalo de interrupcion de la velocidad maxima 4036=65536-61500 (0x0FC4=0xFFFF-0xF03C)

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

/******************************************************************************/
/*************   FUNCION DE SETEO DE LOS TIMERS DE ESCLAVOS   *****************/
/*- La funcion realiza el calculo y ordena a los esclavos actualizar el tiempo
interrupcion de los timers para lograr un desplazamiento coordenado-*/
void timersSetting(void)
{
	float32 distEscIzq = 0.00;
	float32 distEscDer = 0.00;
	int16 vel2 =0x00;
	if (objIniEscIzq > objFinEscIzq)
		distEscIzq = objIniEscIzq - objFinEscIzq;
	else
		distEscIzq = objFinEscIzq - objIniEscIzq;
	if (objIniEscDer > objFinEscDer)
		distEscDer = objIniEscDer - objFinEscDer;
	else
		distEscDer = objFinEscDer - objIniEscDer;

	if (distEscDer < distEscIzq)
	{
		orden = 'T';
		opcion1 = 0xF0;		//la velocidad máxima es a 0xF03C
		opcion2 = 0x3C;
		printf("Esclavo 1 -> Vmax (%lx)\r", Vmax);
		Envio_I2C(dirEsclavo1, orden, opcion1, opcion2);
		vel2 = 0xFFFF-(int16)((distEscIzq/distEscDer)*(float32)IntTimMin);	//Como el movimiento es coordinado (en el mismo tiempo), las velocidades son directamente proporcionales a las distancias
		opcion1 = vel2 >> 8;
		opcion2 = vel2;
		printf("Esclavo 2 -> Vel = %lx\r", vel2);
		Envio_I2C(dirEsclavo2, orden, opcion1, opcion2);
	}
	else if (distEscDer > distEscIzq)
	{
		orden = 'T';
		vel2 = 0xFFFF-(int16)((distEscDer/distEscIzq)*(float32)IntTimMin);	//Como el movimiento es coordinado (en el mismo tiempo), las velocidades son directamente proporcionales a las distancias
		opcion1 = vel2 >> 8;
		opcion2 = vel2;
		printf("Esclavo 1 -> Vel = %lx\r", vel2);
		Envio_I2C(dirEsclavo1, orden, opcion1, opcion2);
		opcion1 = 0xF0;		//la velocidad máxima es a 0xF03C
		opcion2 = 0x3C;
		printf("Esclavo 2 -> Vmax (%lx)\r", Vmax);
		Envio_I2C(dirEsclavo2, orden, opcion1, opcion2);
	}
	else	//distEscDer==distEscIzq
	{
		orden = 'T';
		opcion1 = 0xF0;		//la velocidad máxima es a 0xF03C
		opcion2 = 0x3C;
		printf("Esclavo 1 -> Vmax (%lx)\r", Vmax);
		Envio_I2C(dirEsclavo1, orden, opcion1, opcion2);
		printf("Esclavo 2 -> Vmax (%lx)\r", Vmax);
		Envio_I2C(dirEsclavo2, orden, opcion1, opcion2);
		//ordenar la misma veloc
	}
}

/******************************************************************************/
/*************   FUNCION PASO DE DISTANCIA A PASOS DE MOTOR   *****************/
unsigned int16 distToPasos(float32 d)
{
	unsigned int16 pasos = d*pasoDist;
	return pasos;
}

/******************************************************************************/
/************   FUNCION PARA CALCULO DE OBJETIVOS DE ESCLAVOS   ***************/
/**calculan la cantidad de pasos que deberia realizar cada motor para cumplir 
con las coordenadas ordenadas**/
unsigned int16 objetivoEsclavoIzq(float32 x, float32 y)
{
	float32 aux = sqrt( x*x + y*y );	//calculo la distancia del origen
	return distToPasos(aux);			//la devuelvo transformada a pasos
}

unsigned int16 objetivoEsclavoDer(float32 x, float32 y)
{
	float32 xaux = anchoDibujo-x;			//para la otra esquina, la coord y es el complemento
	float32 aux = sqrt( xaux*xaux + y*y);	//calculo la distancia del origen
	return distToPasos(aux);				//la devuelvo transformada a pasos
}

/* get16(k) - Función auxiliar que devuelve el valor numérico (int16) de una cadena decimal 
a partir del elemento k hasta encontrar el caracter null (similar a atoi o atol)*/
unsigned int16 get16(int k)
{
	unsigned int16 aux16=0;
	int i;
	for (i = 0; i < 4; ++i)
	{
		aux16=aux16*10+comando[k+i]-'0'; //'0' es 48 en decimal o 0x30 en hexa 
	}
	return aux16;
}

/*****************************************************************************/
/************** FUNCION PARA GUIAR A UN ESCLAVO A HACER EL HOMING ************/
/**        ordena hacer homing a uno mientras ordena avanzar al otro        **/

void doHoming(int8 dirEsclavoHoming, int8 dirEsclavoAvanza, int8 inputPinEsclavoHoming)
{
	orden = 'H';	
	opcion1 = 0x00;											//orden H, opcion1 0x00 avanzar para permitir al otro esclavo hacer homing
	Envio_I2C(dirEsclavoAvanza , orden, opcion1, opcion2);	//pedir avanzar a un esclavo
	opcion1 = 0x01;											//orden H, opcion1 0x01 retroceder para hacer homing
	Envio_I2C(dirEsclavoHoming , orden, opcion1, opcion2);	//pedir hacer homing al otro esclavo 	
	//el esclavo1 pone un puerto en 1 hasta que termina de hacer homing
	while(input(inputPinEsclavoHoming) == 0){								//esperar que el puerto este en 1, indicando que empezo a hacer el homing
	}
	while(input(inputPinEsclavoHoming) == 1){								//esperar que el puerto este en 0, indicando que termino el homing
	}
	orden = 'X';
	Envio_I2C(dirEsclavoAvanza, orden, opcion1, opcion2);	//Orden de detener al esclavo que avanzaba
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
					estadoAnterior = estado;
					estado = 0x00;
					break;
				case '1':
					estadoAnterior = estado;
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
			estadoAnterior = estado;
			estado = 0x03;
			break;	
		case 'i':
		case 'I':
			if ((estado & 0x05) == 0x05)	//ha hecho homing y esta energizado
			{
				if (ii == 9)		//se recibio un 'I', cuatro digitos correspondientes a la coordenada x y cuatro correspondientes a la coordenada y
				{
					xi=get16(1);	//almaceno el numero ubicado en los caracteres 1 al 4 en xi
					yi=get16(5);	//almaceno el numero ubicado en los caracteres 5 al 8 en xi
					printf("Punto Inicial recibido (%ld , %ld)\r", xi, yi);
					estadoAnterior = estado;
					estado = 0x0D;
				}
				else
				{
					printf("Faltan datos.\rSintaxis del comando -> :Ixxxxyyyy\r");	//en caso de un mal ingreso de la informacion
				}
			}
			else
				printf("Motor sin referenciar. Hacer homing.\r");
			break;
		case 'f':
		case 'F':
			if (estado & 0x05)
			{
				if (ii == 9)		//se recibio un 'F', cuatro digitos correspondientes a la coordenada x y cuatro correspondientes a la coordenada y
				{
					xf=get16(1);	//almaceno el numero ubicado en los caracteres 1 al 4 en xf
					yf=get16(5);	//almaceno el numero ubicado en los caracteres 5 al 8 en yf
					printf("Punto Final recibido (%ld , %ld)\r", xf, yf);
					finalRecibido = 1;
				}
				else
				{
					printf("Faltan datos.\rSintaxis del comando -> :Fxxxxyyyy\r");	//en caso de un mal ingreso de la informacion
				}
			}
			else
				printf("Motor sin referenciar. Hacer homing.\r");		
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
			ii = 0;						//inicializa contador i
			cmd = 1;					//y activa bandera que indica que hay comando en curso
			break;      
		case 13:						//si es delimitador de final 
			if (cmd == 1)				// si habÃ­a comando en curso
			{
				comando[ii] = 0;			//termina cadena de comando con caracter null
				interpreta();			//va a interpretar el comando
				cmd = 0;				//desactiva bandera de comando en curso
			}
			break;
		default:
			if (ii < LONGI_BUF)			//si contador menor que longitud del buffer
			{
				comando[ii] = dato;		//pone caracter en cadena
				ii++;					//incrementa contador i
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
	
	output_low(PIN_C0);		//RC0=0
	
	opcion1 = 0x00;
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

	printf("OK\r");
	for (;;)
	{
		switch(estado)
		{
			case 0x00:					//Solicitar desenergizar motores a los esclavos
				if(estadoAnterior & 0x01)
				{
					estadoAnterior = 0x00;
					printf("Desenergizando...\r");
					orden = 'P';
					opcion1 = 0x00;
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
					output_A(estado);
				}
				break;
			case 0x01:					//Solicitar energizar motores a los esclavos
				if (estadoAnterior == 0x00)
				{
					estadoAnterior = 0x01;
					printf("Energizando...\r");
					orden = 'P';
					opcion1 = 0x01;
					Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
					Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
					output_A(estado);
				}
				break;
			case 0x03:						//Solicitar hacer homing a los esclavos
				output_A(estado);
				printf("Homing esclavo 1...\r");
				doHoming(dirEsclavo1, dirEsclavo2, PIN_C1);
				printf("Homing esclavo 2...\r");
				doHoming(dirEsclavo2, dirEsclavo1, PIN_C2);
				estadoAnterior = estado;
				estado = 0x05;
				break;
			case 0x05:					//Esperando ordenes de dibujo (Punto de Inicio y Punto de Final)
				if (estadoAnterior == 0x03)
				{
					estadoAnterior = estado;
					output_A(estado);
					printf("Homing terminado. Esperando ordenes...\r");
					finalRecibido = 0;
				}
				if (estadoAnterior == 0x2D)
				{
					estadoAnterior = estado;
					output_A(estado);
					printf("Segmento terminado. Esperando ordenes...\r");
					finalRecibido = 0;
				}
				break;
			case 0x0D:					//Enviar coordenadas de inicio a los esclavos
				estadoAnterior = estado;
				output_A(estado);
				printf("Desplazando al inicio...\r");
				objIniEscIzq = objetivoEsclavoIzq(xi,yi);
				objIniEscDer = objetivoEsclavoDer(xi,yi);
				//los esclavos se desplazan al punto inicial
				orden = 'M';
				opcion1 = objIniEscIzq >> 8;
				opcion2 = objIniEscIzq ;
				printf("ObjI: %lu = %x %x\r", objIniEscIzq, opcion1, opcion2);
				Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
				opcion1 = objIniEscDer >> 8;
				opcion2 = objIniEscDer;
				printf("ObjD: %lu = %x %x\r", objIniEscDer, opcion1, opcion2);
				Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
				while((input(PIN_C1) == 0) | (input(PIN_C2)== 0)){	//espera a que los dos lleguen					
				}
				while((input(PIN_C1) == 1) | (input(PIN_C2)== 1)){	//espera a que los dos lleguen					
				}
				if (finalRecibido == 1)
					estado = 0x2D;
				else
					estado = 0x15;
				break;
			case 0x15:					//Enviar valor de interrupcion de timer a los esclavos
				if (estadoAnterior != estado)
				{
					estadoAnterior = estado;
					output_A(estado);
					printf("Posicion inicial alcanzada. Esperando coordenada final...\r");
				}
				if (finalRecibido == 1)
				{
					estado = 0x2D;
				}
				break;
			case 0x2D:					//Enviar coordenadas de final a los esclavos
				estadoAnterior = estado;
				output_A(estado);
				printf("Preparando para dibujar...\r");
				objFinEscIzq = objetivoEsclavoIzq(xf,yf);
				objFinEscDer = objetivoEsclavoDer(xf,yf);
				while((input(PIN_C1) == 1) | (input(PIN_C2)== 1)){	//si es que aun están desplazandose los motores, espera a que terminen
				}
				timersSetting();				//calcular los timers de ambos
				//los esclavos se desplazan al punto final
				orden = 'D';
				opcion1 = objFinEscIzq >> 8;
				opcion2 = objFinEscIzq ;
				printf("ObjI: %lu = %x %x\r", objFinEscIzq, opcion1, opcion2);
				Envio_I2C(dirEsclavo1 , orden, opcion1, opcion2);
				opcion1 = objFinEscDer >> 8;
				opcion2 = objFinEscDer;
				printf("ObjD: %lu = %x %x\r", objFinEscDer, opcion1, opcion2);
				Envio_I2C(dirEsclavo2 , orden, opcion1, opcion2);
				output_high(PIN_C0);
				while((input(PIN_C1) == 0) | (input(PIN_C2)== 1)){	//espera a que los dos lleguen					
				}
				while((input(PIN_C1) == 1) | (input(PIN_C2)== 1)){	//espera a que los dos lleguen					
				}
				output_low(PIN_C0);
				estadoAnterior = 0x2D;
				estado = 0x05;
				break;
			}
	}
}
