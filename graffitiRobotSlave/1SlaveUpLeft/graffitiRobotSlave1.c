#include "C:\Users\Nahuel\Documents\GitHub\microcontroladores\graffitiRobot\graffitiRobotSlave\1SlaveUpLeft\graffitiRobotSlave1.h"

/*****************************************************************************/
/**********************        Variables Globales        *********************/

/** Variables de estado **/
unsigned int8 estado;											//variable de estado del master
unsigned int8 estadoAnterior;									//variable que lleva el registro del estado anterior del master

/** Variables para comunicación I2C**/
byte fstate;													//Guardara el estado del bus I2C
char orden;
int8 numero1;
int8 numero2;

/** Variables para cotrol del robot**/
/*-Variables de control de la interrupcion del timer-*/
unsigned int16 timerCarga;
const int16 pasoRapido = 0xFF6A; 							//velocidad máxima de avance del motor (1 paso cada 30 us -> 65386)
const int16 homingVel = 0xC568; 							//velocidad para realización del homing (1 paso cada 150 us -> 64786)
const int16 finHomingVel = 0x15A0; 							//velocidad para terminar el homing (1 paso cada 600 us -> 64786)
/*-Variables de control de pasos-*/
const int8 STEP[] = {0x01,0x03,0x02,0x06,0x04,0x0C,0x08,0x09};	//valores para el puerto de salida de los pasos
unsigned int stepIndex;											//apuntador al paso actual
int16 posicionActual;									//variable que cuenta los pasos - representa posicion actual
int16 posicionObjetivo;									//variable que representa la posiçion objetivo


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
			switch(numero1)
			{
				case 0x00:
					estado = 0x00;
					break;
				case 0x01:
					if (estado == 0x00)
					{
						estado = 0x01;
					}
					break;
			}
			break;
		case 'h':
		case 'H':
			switch(numero1)
			{
				case 0x00:
					if (estado == 0x01)
					{
						estado = 0x09;
					}
					else if (estado == 0x05)
					{
						estado = 0x0D;
					}
					break;
				case 0x01:
					estado = 0x0B;
					break;
			}
			break;
		case 'x':
		case 'X':
			disable_interrupts(INT_TIMER1);
			if (estado == 0x0D)
			{
				estado = 0x05;
			}
			break;
		case 't':
		case 'T':
			//aux16=get16(1);
			//if(aux16<60000) tpaso=aux16;
			break;
	}
}

#int_TIMER1
void  TIMER1_isr(void) 
{
	set_timer1(timerCarga);							//recarga del timer
	if (posicionObjetivo < posicionActual)			//si la posicion actual es mayor a la objetivo, decremento el indice y la posicion actual
	{
		posicionActual--;
		if(stepIndex == 0)							//verifico no exceder el limite inferior de valores posibles del indice
			stepIndex = 7;
		else
			stepIndex--;							
	}
	else if (posicionObjetivo > posicionActual)		//si la posicion actual es inferior a la objetivo, incremento el indice y la posicion actual
	{
		posicionActual++;
		if(stepIndex == 7)							//verifico no exceder el limite superor de los valores posibles del indice
			stepIndex = 0;
		else
			stepIndex++;
	}
	output_B(STEP[stepIndex]);						//actualizo el puerto que actua sobre el motor
}

/******************************************************************************/
/***************** FUNCIÓN INTERRUPCIÓN POR RECEPCION I2C *********************/
/**  Saltará a esta función cada vez que se detecte actividad en el bus I2C  **/

#int_SSP
void  SSP_isr(void) 
{
	int incoming;							//Variable donde se recibe el byte que manda el maestro
	fstate = i2c_isr_state();				//Lectura del estado del bus I2c la interrupción
	/* Solicitud de lectura del esclavo por el master */
	if(fstate == 0x80) {         
		i2c_write(estado);					//Manda al maestro el estado del esclavo
	}
	/* Sino está solicitando lectura es que está enviando algo */
	else {									//Sino es que hay dato en el bus I2C...
		incoming = i2c_read();				//... lo lee
		if (fstate == 1) {					//Información recibida corresponde a la orden
			orden = incoming;				//Se guarda posición
		}
		else if (fstate == 2) {				//Información recibida corresponde a la primera opcion de la orden
			numero1 = incoming;				//Se guarda dato
		}
		else if (fstate == 3) {				//Información recibida corresponde a la segunda opcion de la orden
			numero2 = incoming;				//Se guarda dato
			interpreta();
		}
	}
}



void main()
{
	set_tris_A(0x00);
	set_tris_B(0x00);
	bit_set(TRISC,0);	//RC0 entrada
	bit_set(TRISC,1);	//RC1 entrada
	bit_clear(TRISC,2);	//RC2 salida

	bit_clear(PORTC,2);	//RC2=0

	estado = 0x00;                //Representa el estado del master
	estadoAnterior = 0x01;        //Definido con un valor diferente a estado solo a fines de que pase por el primer bucle de desenergizado
	
	stepIndex = 0;

	setup_adc_ports(NO_ANALOGS|VSS_VDD);    
	setup_adc(ADC_CLOCK_DIV_2);
	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
	setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);
	setup_timer_2(T2_DISABLED,0,1);
	setup_comparator(NC_NC_NC_NC);// This device COMP currently not supported by the PICWizard
	enable_interrupts(INT_TIMER1);
	enable_interrupts(INT_SSP);
	enable_interrupts(GLOBAL);
//Setup_Oscillator parameter not selected from Intr Oscillator Config tab

	for (;;)
	{
		switch(estado)
		{
			case 0x00:              //Desenergizar motor
				if(estadoAnterior & 0x01)
				{
					estadoAnterior = estado;
					disable_interrupts(INT_TIMER1);
					output_B(0x00);
					output_A(estado);
				}
				break;
			case 0x01:              //Energizar motor
				if (estadoAnterior == 0x00)
				{
					estadoAnterior = estado;
					disable_interrupts(INT_TIMER1);
					output_B(STEP[stepIndex]);
					output_A(estado);
				}
				break;
			case 0x0B:              //Hacer homing
				estadoAnterior = estado;
				output_A(estado);
				output_high(PIN_C2);								//informo que estoy ocupado
				//BUSCANDO FIN DE CARRERA
				timerCarga = homingVel;
				set_timer1(timerCarga);					//Seteo la interrupcion del timer en una velocidad moderada
				posicionObjetivo = 10;
				enable_interrupts(INT_TIMER1);			//habilito la interrupcion del timer
				while(input(PIN_C0) == 0){						//mientras no reciba una señal del fin de carrera para homing
					posicionActual = 100;				//actualizo la posicion actual siempre superior a la posicion objetivo para que en la interrupcion se retroceda
				}
				//SEPARANDOSE DEL FIN DE CARRERA
				timerCarga = finHomingVel;					//seteo la interrupcion del timer en una velocidad lenta
				posicionObjetivo = 100;
				while(input(PIN_C0) == 1){						//mientras el fin de carrera hace contacto
					posicionActual = 10;					//actualiza la posicion actual a un valor inferior a la posición objetivo para que avance lentamente
				}
				//FIN DEL HOMING
				disable_interrupts(INT_TIMER1);
				output_low(PIN_C2);								//informo que estoy desocupado, que terminé de hacer el homing
				posicionActual = 0;						//defino el origen
				posicionObjetivo = 0;					//no tengo un objetivo nuevo
				estado = 0x05;							//paso a estado de reposo, ya referenciado
				break;
			case 0x09:              //avanzando ciego para permitir al otro robot hacer el homing
				estadoAnterior = estado;
				output_A(estado);
				//AVANZANDO
				timerCarga = homingVel;
				set_timer1(timerCarga);					//Seteo la interrupcion del timer en una velocidad moderada
				posicionObjetivo = 100;
				enable_interrupts(INT_TIMER1);			//habilito la interrupcion del timer
				while(estado == 0x09){					//mientras no reciba una mensaje del master de cambiar de estado, dejar de avanzar
					posicionActual = 10;				//actualizo la posicion actual siempre superior a la posicion objetivo para que en la interrupcion avance
				}
				break;
			case 0x0D:              //Enviar coordenadas de inicio a los esclavos
				estadoAnterior = estado;
				output_A(estado);
				//AVANZANDO
				timerCarga = homingVel;
				set_timer1(timerCarga);					//Seteo la interrupcion del timer en una velocidad moderada
				posicionObjetivo = posicionActual + 50;	//pongo un objetivo 'adelante' de la posicion actual
				enable_interrupts(INT_TIMER1);			//habilito la interrupcion del timer
				while(estado == 0x0D){					//mientras no reciba una mensaje del master de cambiar de estado, dejar de avanzar
					posicionObjetivo = posicionActual + 50;				//actualizo la posicion actual siempre superior a la posicion objetivo para que en la interrupcion avance
				}
				break;
			case 0x05:								//Estado de reposo, energizado y referenciado
				if (estadoAnterior != estado)
				{
					estadoAnterior = estado;
					posicionObjetivo = posicionActual;	//No hay objetivo, la posicion objetivo es la actual
				}	
				output_A(estado);		
				break;
			case 0x3D:              //Enviar coordenadas de final a los esclavos
				break;
		}
	}
}
