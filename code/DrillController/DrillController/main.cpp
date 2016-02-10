#include "Arduino.h"
//#define ARDUINO 101 //this is defined in project properties
//#define F_CPU 16000000UL //it is defined in Project properties

#include "../../uartlib.h"
#include <util/delay.h>
#include "GPIO.h"
#include "myStack.h"
#include "i2c.h"

#define LED_YELLOW	5
#define LED_GREEN	4
#define LED_BLUE	3
#define LED_RED		2

#define SW_1		8
#define SW_2		9

#define POT			A0
#define SHUNT_RES	A1

#define MOTOR_PWM_1		10
#define MOTOR_PWM_2		11
#define MOTOR_ENABLE	12

	GPIO ledYellow(LED_YELLOW);
	GPIO ledGreen(LED_GREEN);
	GPIO ledBlue(LED_BLUE);
	GPIO ledRed(LED_RED);
	
	GPIO switch_1(SW_1);
	GPIO switch_2(SW_2);
	
	GPIO motorPWM_1(MOTOR_PWM_1);
	GPIO motorPWM_2(MOTOR_PWM_2);
	GPIO motorEnable(MOTOR_ENABLE);

void loop();
void adc_init();
unsigned int adc_read(unsigned char channel);
void setPwmFrequency(int pin, int divisor);
uint16_t readVoltage(uint8_t channel);
void powerToggle();
void directionToggle();
void clockwise();
void anti_clockwise();

#define	BUTTON_DEBOUNCE_INTERVAL	200
#define INCREMENTER_INIT_VALUE		50
#define STARTUP_INTERVAL_DELAY		100
uint32_t sw1_last_click_time=0;
uint32_t sw2_last_click_time=0;
uint8_t powerFlag=0;
uint8_t directionFlag=0;	//0 stands for clockwise
uint8_t soft_start_flag=1; 
uint8_t pwmValue = 0;
uint8_t incrementer = INCREMENTER_INIT_VALUE;
uint32_t softStartInterval = 0;

int main(void)
{
	init();		//Initialize arduino core library functions/peripherals.

	Serial.begin(57600);
	
	//analogReference(INTERNAL);  //set to 1.1VBG set by my adc_init()
	//adc_init();	//we have a problem now that if we use internal 1.1v then how will we read POT value properly
					//since it will swing across VCC-GND
	
	pinMode(POT,INPUT);
	pinMode(SHUNT_RES,INPUT);
	
	motorEnable.Low();
	motorPWM_1.Low();
	motorPWM_2.Low();
	
	ledBlue.Low();
	ledRed.Low();
	ledGreen.Low();
	ledYellow.Low();
	
	switch_1.setInputPullUp();
	switch_2.setInputPullUp();
	
	Serial.println("Program Initialized");
	
	while(1)
	{
		
	loop();
	//if (serialEventRun) serialEventRun(); //some arduino stuff
 	
	}

	return 0;
}

void loop()
{
	
	if(!switch_1.getState() && millis()-sw1_last_click_time > BUTTON_DEBOUNCE_INTERVAL)
	{	
		powerToggle();
		while(!switch_1.getState());	//wait until switch release.
		sw1_last_click_time = millis();
	}
	
	
	if(powerFlag)	//main body code
	{
		if(!switch_2.getState() && millis()-sw2_last_click_time > BUTTON_DEBOUNCE_INTERVAL)
		{
			directionToggle();
			while(!switch_2.getState());	//wait until switch released.
			sw2_last_click_time = millis();
		}
		
		motorEnable.High();
		
		
		
		if(soft_start_flag)
		{
			uint8_t currentPwmVal = map(analogRead(POT),0,1023,0,255);
			
			if(incrementer < currentPwmVal && millis() - softStartInterval > STARTUP_INTERVAL_DELAY)
			{
				incrementer+=10;
				pwmValue = incrementer;
				softStartInterval = millis();
			}
			
			else if(incrementer >= currentPwmVal)
			{	
				soft_start_flag = 0;
				incrementer = INCREMENTER_INIT_VALUE;
			}
		}
		
		else
		{
			pwmValue = map(analogRead(POT),0,1023,0,255);
		}
		
		
		if(!directionFlag)
			clockwise();
		else
			anti_clockwise();
		
		Serial.println(analogRead(SHUNT_RES));
		
		if(analogRead(SHUNT_RES)>=14)
		{
		//	uint32_t startCheck = millis();
			Serial.println("OverCurrent_detected");
		/*	while(millis() - startCheck < 500 && analogRead(SHUNT_RES) >= 10);
			
			if (millis()-startCheck >= 300)
			{
				Serial.println("stop");
				motorEnable.Low();
				delay(1000);
			}
		*/
			
			delay(400);
			if(analogRead(SHUNT_RES)>=14)
			{
				ledRed.High();
				Serial.println("stop");
				motorEnable.Low();
				delay(2000);
				soft_start_flag=1;
			}
			
		}
		else
		{
			//soft_start_flag = 1;
			motorEnable.High();
			ledRed.Low();
		}
	}
	else
	{
		//power down all , blue led is controlled in powerToggle().
		motorEnable.Low();
		ledRed.Low();
		ledGreen.Low();
		ledYellow.Low();
		digitalWrite(MOTOR_PWM_1,LOW);
		digitalWrite(MOTOR_PWM_2,LOW);
	}
	
}

void powerToggle()
{
	ledBlue.toggle();
	powerFlag = !powerFlag;
	if(powerFlag)				//statements to be executed only once on power-on
	{
		ledYellow.toggle();		//initial direction indication led.
		soft_start_flag = 1;	//set the soft-start-flag
	}
	
}

void clockwise()
{
	ledYellow.High();
	ledGreen.Low();
	
	digitalWrite(MOTOR_PWM_2,LOW);		//if I use my own GPIO pin control functions then it goes mad, because my function doesn't take care
										//if any PWM is present on the pin but digitalWrite() takes that into account and disables the PWM 
										//accordingly.
	analogWrite(MOTOR_PWM_1,pwmValue);
}

void anti_clockwise()
{
	ledYellow.Low();
	ledGreen.High();
	
	digitalWrite(MOTOR_PWM_1,LOW);
	analogWrite(MOTOR_PWM_2,pwmValue);
}

void directionToggle()
{
	directionFlag = !directionFlag;
	soft_start_flag = 1;
}

void adc_init()
{
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //clk divided by 128 for highest accuracy.
	ADMUX = (1<<REFS0)|(1<<REFS1) ; //Internal Vbg 1.1V as Reference
}

unsigned int adc_read(unsigned char channel)
{
	ADMUX |= (0b00001111 & channel); //there are some other uses of channel values beyond 0-7, read datasheet of 328p for more clarification.
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADIF)) == 0);
	ADCSRA |= (1<<ADIF);
	return ADC;
}

uint16_t readVoltage(uint8_t channel)
{
	return ((adc_read(channel)*1100UL)/1024UL);
}


void setPwmFrequency(int pin, int divisor) {
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 64: mode = 0x03; break;
			case 256: mode = 0x04; break;
			case 1024: mode = 0x05; break;
			default: return;
		}
		
		if(pin == 5 || pin == 6)
		{
			TCCR0B = (TCCR0B & 0b11111000) | mode;//dont use pin5 and 6 because they will modify timer0 which will affect millis()/micros()
			// and in turn it will affect many other things
		}
		
		else
		{
			TCCR1B = (TCCR1B & 0b11111000) | mode;
		}
		
	}
	
	else if(pin == 3 || pin == 11)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 32: mode = 0x03; break;
			case 64: mode = 0x04; break;
			case 128: mode = 0x05; break;
			case 256: mode = 0x06; break;
			case 1024: mode = 0x7; break;
			default: return;
		}
		TCCR2B = (TCCR2B & 0b11111000) | mode;
	}
}