to decrease startup inrush after reset or poweron switch we will slowly increase the speed irrespective of the pot state or upto
 the pot state, that way we will decrease inrush and there will be very small chance that we loose control or mcu gets reset
 during startup dueto emi.
 
 We will first turn the pwm on and keep slowly rising it then turn on enable.


 To decrease noise we can increase PWM freq and increase stability in current detection
 we should decrease ADC clock rate to approx 1KHz.

 We should propbably remove the soft_start system because when the drill is stuck and
 I want to restart or change direction I am not getting sufficient impact, and by the 
 time it reaches full speed it gets locked down because of over-current detection.

 And I want that the overcurrent detection should not use delay() it should use millis()
 and let the reverse of power button to function so that if drill is stuck I can use
 reverse to get it out.

 And one more thing this could which was commented out in void loop()

 	if(analogRead(SHUNT_RES)>=14)
		{
		//	uint32_t startCheck = millis();
			Serial.println("OverCurrent_detected");
		/*	while(millis() - startCheck < 500 && analogRead(SHUNT_RES) >= 8);
			
			if (millis()-startCheck >= 300)
			{
				Serial.println("stop");
				motorEnable.Low();
				delay(1000);
			}
		*/

		can be made to work be giving the function a little hysterisis along with
	decreasing the frequency of ADC.

But again becarefull don't decrease the ADC freq to so low that the whole program will be
slowed down because of it.

Anyway I still feel it was not worth the effort of building it because it can immitate the 
effect of raw power with all this control, speed, direction and protection. If it is completely
on raw power then only it can be effective in serving its purpose well which is to DRILL.