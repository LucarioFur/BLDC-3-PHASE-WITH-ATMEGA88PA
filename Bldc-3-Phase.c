/*
 * BLDC 3 Phase.c
 *
 * Created: 25/04/2024 16:15:00
 * Author : Lorhan
 */ 
//=================================
//---Clock MCU---
#define F_CPU 16000000UL

//=================================
//---Bibliotecas---
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//=================================
//---Mapeamento do Hardware---
#define LED  (1<<PB0)
#define pwm1 (1<<PB1)
#define pwm2 (1<<PB2)
#define pwm3 (1<<PB3)
#define PWM4 (1<<PD2)
#define PWM5 (1<<PD3)
#define PWM6 (1<<PD4)
#define BT1  (1<<PB4)
#define BT2  (1<<PB5)

//===========================
//---Constante de Amostragem---
#define TABLE_SIZE 180
#define MAX_ANGLE (2 * M_PI)
volatile uint16_t sin_table[TABLE_SIZE];

//===========================
//---Variaveis Globais---
volatile uint8_t AWP_MAX = 255;
volatile uint8_t Sine_index;
volatile uint8_t Sine_index_V;
volatile uint8_t Sine_index_U;
volatile uint8_t boot;
volatile uint8_t period;

//===========================
//---Cacula o Seno---
void seno_calc()
{
  Sine_index_V = TABLE_SIZE / 3;
  Sine_index_U = Sine_index_V * 2;
  	
  for(int i = 0; i < TABLE_SIZE; i++)
  {
	float angle = i * (MAX_ANGLE / TABLE_SIZE);
	float sin_value = sin(angle);
	sin_table[i] = (uint8_t)((sin_value + 1.0) * 127.5); 
  }	
  
  static int Vt;
  while(Vt < 10)
  {
   Vt++;
  }
  boot = 1;
}

//===========================
//---Configuraçao SVPWM---
void SVPWM()
{ 	
  PORTB |= LED;
  PORTD |= (PWM4 | PWM5 | PWM6);
  
  TCCR1A = 0xA1;
  TCCR2A = 0x83;

  float angle_W = Sine_index * (MAX_ANGLE / TABLE_SIZE);
  float angle_V = Sine_index_V * (MAX_ANGLE / TABLE_SIZE);
  float angle_U = Sine_index_U * (MAX_ANGLE / TABLE_SIZE);
  
  uint8_t sine_W_lower = sin_table[Sine_index];
  uint8_t sine_V_lower = sin_table[Sine_index_V];
  uint8_t sine_U_lower = sin_table[Sine_index_U];
  
  uint8_t sine_W_upper = sin_table[(Sine_index + 1) % TABLE_SIZE];
  uint8_t sine_V_upper = sin_table[(Sine_index_V + 1) % TABLE_SIZE];
  uint8_t sine_U_upper = sin_table[(Sine_index_U + 1) % TABLE_SIZE];
  
  float fraction_W = angle_W - (Sine_index * (MAX_ANGLE / TABLE_SIZE));
  float fraction_V = angle_V - (Sine_index_V * (MAX_ANGLE / TABLE_SIZE));
  float fraction_U = angle_U - (Sine_index_U * (MAX_ANGLE / TABLE_SIZE));
  
  uint8_t sine_W = (uint8_t)(sine_W_lower + fraction_W * (sine_W_upper - sine_W_lower)) * AWP_MAX / 255;
  uint8_t sine_V = (uint8_t)(sine_V_lower + fraction_V * (sine_V_upper - sine_V_lower)) * AWP_MAX / 255;
  uint8_t sine_U = (uint8_t)(sine_U_lower + fraction_U * (sine_U_upper - sine_U_lower)) * AWP_MAX / 255;
  
  OCR1A = sine_W;
  OCR1B = sine_V;
  OCR2A = sine_U;
}

//===========================
//---Configuraçao Setores---
void sector()
{
	static int step;
	step++;
	step %= 6;
	
	switch(step)
	{
		case 0:
		Sine_index++;
		Sine_index %= TABLE_SIZE;
		break;
		case 1:
		Sine_index++;
		Sine_index %= TABLE_SIZE;
		Sine_index_V++;
		Sine_index_V %= TABLE_SIZE;
		break;
		case 2:
		Sine_index_V++;
		Sine_index_V %= TABLE_SIZE;
		break;
		case 3:
		Sine_index_V++;
		Sine_index_V %= TABLE_SIZE;
		Sine_index_U++;
		Sine_index_U %= TABLE_SIZE;
		break;
		case 4:
		Sine_index_U++;
		Sine_index_U %= TABLE_SIZE;
		break;
		case 5:
		Sine_index++;
		Sine_index %= TABLE_SIZE;
		Sine_index_U++;
		Sine_index_U %= TABLE_SIZE;
		break;
	}
	SVPWM();
}

//===========================
//---Conversor ADC---
uint16_t read_ADC(uint8_t channel)
{
	// Limpa o registro ADMUX para garantir a seleção correta do canal
	ADMUX &= 0xF0;
	
	// Seleciona o canal desejado
	ADMUX |= channel;
	
	// Inicia a conversão
	ADCSRA |= (1 << ADSC);
	
	// Aguarda a conclusão da conversão
	while (ADCSRA & (1 << ADSC));
	
	// Retorna o resultado da conversão (ADC)
	return ADC;
}


//=================================
//---Interrupções---
ISR(TIMER0_COMPA_vect)
{
  static int counter;
  counter++;
  if(boot && counter > period)
  {
    sector();
	counter = 0;	 
  }	
}

//=================================
//---Interrupções botões---
ISR(PCINT0_vect)
{
 
}

//=================================
//---Função Principal---
int main(void)
{
	//Configura as I/Os
    DDRB |= pwm1 | pwm2 | pwm3 | LED;
	
	DDRD |= PWM4 | PWM5 | PWM6;
   
    PORTB |= BT1 | BT2;
   
    PORTB &= ~pwm1 & ~pwm2 & ~pwm3 & ~LED;
	
	PORTD &= ~(PWM4 | PWM5 | PWM6);
   
	// Configura Interrupçao Interna Timer0
	cli();
	TCCR0A = 0x02; //Modo CTC
	TCCR0B = 0x01; //No Prescale
	TIMSK0 = 0x02; //Inicia OCIEA
	OCR0A  = 0x00; //Inicia em 1
    
    PCICR  = 0x01; //Habilitar interrupções PCINT para o grupo de pinos PCINT0-7
    PCMSK0 = 0x30; //Habilitar interrupção para o pinos específicos (PCINT4) e (PCINT5) 
	sei();
	
	// Configuração do ADC
	ADMUX |= ~(1<<REFS1) | (1<<REFS0); // Referencia
	ADCSRA = 0x87;                     // Habilita o conversor A/D e prescale de divisao
	ADCSRB = 0x00;                     // Modo de conversão livre (Free Running mode)
	
	// Configura Timer 1 e 2 (8-bit)
	TCCR1A = 0x01;
	TCCR1B = 0x09;
	TCCR2A = 0x03;
	TCCR2B = 0x01;
	seno_calc();
	
	while (1)
    {
	 uint8_t adc_read = read_ADC(0) >> 8;
	 period = adc_read;
	}
	return 0;
}

