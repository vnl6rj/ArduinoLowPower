#if defined(ARDUINO_ARCH_SAMD)

#include "ArduinoLowPower.h"
#include "WInterrupts.h"

#if (SAMD21)
static void configGCLK6()
{
	// enable EIC clock
	GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	/* Errata: Make sure that the Flash does not power all the way down
     	* when in sleep mode. */

	NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
}
#endif

void ArduinoLowPowerClass::idle() {
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	#if (SAMD21)
	PM->SLEEP.reg = 2;
	#elif (SAMR34)
    PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_IDLE;
	#endif
	__DSB();
	__WFI();
	
}

void ArduinoLowPowerClass::idle(uint32_t millis) {
	setAlarmIn(millis);
	idle();
}

void ArduinoLowPowerClass::sleep() {
	#if (SAMD21)
	// Disable systick interrupt:  See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	// Enable systick interrupt
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;	
	#elif (SAMR34)
    GCLK_GENCTRL_Type gclkConfig;
	gclkConfig.reg = 0;
	gclkConfig.reg = GCLK->GENCTRL[0].reg;
	gclkConfig.bit.SRC = GCLK_GENCTRL_SRC_OSC16M_Val;// GCLK_GENCTRL_SRC_OSCULP32K_Val ;//GCLK_GENCTRL_SRC_OSC16M_Val
	GCLK->GENCTRL[0].reg = gclkConfig.reg;
	
	while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(0)) {
		/* Wait for synchronization */
	};
	OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;

	/* Clear performance level status */
	PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
	/* Switch performance level to PL0 - best power saving */
	PM->PLCFG.reg = PM_PLCFG_PLSEL_PL0_Val;
	while (!PM->INTFLAG.reg) {
		;
	}


	OSCCTRL_DFLLCTRL_Type dfllCtrlSlp;
	dfllCtrlSlp.reg = OSCCTRL->DFLLCTRL.reg;
	dfllCtrlSlp.bit.ENABLE = 0;
	OSCCTRL->DFLLCTRL.reg = dfllCtrlSlp.reg;

	// disable DFLL GCLK
	/* Disable the peripheral channel 0 ( DFLL )*/
	GCLK->PCHCTRL[0].reg &= ~GCLK_PCHCTRL_CHEN;

	while (GCLK->PCHCTRL[0].reg & GCLK_PCHCTRL_CHEN) {
		/* Wait for clock synchronization */
	}
	

	// disable xosc32k clock
	OSC32KCTRL->XOSC32K.reg &= ~OSC32KCTRL_XOSC32K_ENABLE;

	// disable generator 1
	GCLK->GENCTRL[1].bit.GENEN = 0;
	
	// Disable systick interrupt:  See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	// set core voltage regulator to "runstandby" - erratta Main Voltage Regulator Reference:15264
	//SUPC->VREG.bit.RUNSTDBY = 1;
	//SUPC->VREG.bit.STDBYPL0 = 1;
	
	/* CPU and BUS clocks slow down - slow down busses BEFORE cpu.. */
	MCLK->BUPDIV.reg = MCLK_BUPDIV_BUPDIV_DIV128;/** Divide Main clock ( 4MHz OSC ) by 64,ie run at 31.768kHz */
	MCLK->LPDIV.reg = MCLK_BUPDIV_BUPDIV_DIV128; /** Divide low power clock ( 4MHz OSC ) by 64, ie run at 31.768kHz*/
	MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV_DIV64; /**(MCLK_CPUDIV) Divide by 64 ,ie run at 62.5kHz */
	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	// sleeping here, will wake from here ( except from OFF or Backup modes, those look like POR )
	
	/* CPU and BUS clocks back to "regular ratio"*/
	MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV_DIV1; /**(MCLK_CPUDIV) Divide by 1 ,ie run at 4MHz.. until we start the DFLL again */
	MCLK->BUPDIV.reg = MCLK_BUPDIV_BUPDIV_DIV1;/** Div 1, so run these at main clock rate */
	MCLK->LPDIV.reg = MCLK_BUPDIV_BUPDIV_DIV1; /**low power domain back to CPU clock speed */
	
	// enable xosc32k clock
	OSC32KCTRL->XOSC32K.reg |= OSC32KCTRL_XOSC32K_ENABLE;
	// wait for clock to become ready
	while ((OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0);

	GCLK->GENCTRL[1].bit.GENEN = 1; // re-enable generator 1 ( xosc32k )
	while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(1)) {
		/* Wait for synchronization */
	};

	/* Enable DFLL peripheral channel */
	GCLK->PCHCTRL[0].reg |= GCLK_PCHCTRL_CHEN;

	while (GCLK->PCHCTRL[0].reg & GCLK_PCHCTRL_CHEN) {
		/* Wait for clock synchronization */
	}

	// re-enable DFLL
	dfllCtrlSlp.bit.ENABLE = 1;
	OSCCTRL->DFLLCTRL.reg = dfllCtrlSlp.reg;

	/* Clear performance level status */
	PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
	/* Switch performance level to PL2 - Highest performance */
	PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2_Val;
	/* Waiting performance level ready */
	while (!PM->INTFLAG.reg) {
		;
	}
	
	OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;

	gclkConfig.reg = 0;
	gclkConfig.reg = GCLK->GENCTRL[0].reg;
	gclkConfig.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
	GCLK->GENCTRL[0].reg = gclkConfig.reg;
	while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(0)) {
		/* Wait for synchronization */
	};
	//	GCLK->GENCTRL[0].reg |= GCLK_GENCTRL_GENEN;
	/*  Switch to PL2 to be sure configuration of GCLK0 is safe */
	// Enable systick interrupt
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	#endif
}

void ArduinoLowPowerClass::sleep(uint32_t millis) {
	setAlarmIn(millis);
	sleep();
}

void ArduinoLowPowerClass::deepSleep() {
	sleep();
}

void ArduinoLowPowerClass::deepSleep(uint32_t millis) {
	sleep(millis);
}

void ArduinoLowPowerClass::setAlarmIn(uint32_t millis) {

	if (!rtc.isConfigured()) {
		attachInterruptWakeup(RTC_ALARM_WAKEUP, NULL, 0);
	}

	uint32_t now = rtc.getEpoch();
	rtc.setAlarmEpoch(now + millis/1000);
	rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode) {

	if (pin > PINS_COUNT) {
		// check for external wakeup sources
		// RTC library should call this API to enable the alarm subsystem
		switch (pin) {
			case RTC_ALARM_WAKEUP:
				rtc.begin(false);
				rtc.attachInterrupt(callback);
			/*case UART_WAKEUP:*/
		}
		return;
	}

	uint8_t in = g_APinDescription[pin].ulExtInt; 
	if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
    		return;

	//pinMode(pin, INPUT_PULLUP);
	attachInterrupt(pin, callback, mode);
    #if (SAMD21)
	configGCLK6();
	#endif

    #if (SAMD21)
	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);
    #elif (SAMR34)
    // Enable wakeup capability on pin in case being used during sleep
	EIC->CTRLA.bit.CKSEL = 1; // use ULP32k as source ( SAML21 is different to D21 series, EIC can be set to use ULP32k without GCLK )
    // Enable EIC
	EIC->CTRLA.bit.ENABLE = 1;
    while (EIC->SYNCBUSY.bit.ENABLE == 1) { /*wait for sync*/ }

	/* Errata: Make sure that the Flash does not power all the way down
     	* when in sleep mode. */
 	// NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
	 NVMCTRL->CTRLB.bit.SLEEPPRM =  NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val;
	 #endif
}

#if (SAMD21)
void ArduinoLowPowerClass::attachAdcInterrupt(uint32_t pin, voidFuncPtr callback, adc_interrupt mode, uint16_t lo, uint16_t hi)
{
	uint8_t winmode = 0;

	switch (mode) {
		case ADC_INT_BETWEEN:   winmode = ADC_WINCTRL_WINMODE_MODE3; break;
		case ADC_INT_OUTSIDE:   winmode = ADC_WINCTRL_WINMODE_MODE4; break;
		case ADC_INT_ABOVE_MIN: winmode = ADC_WINCTRL_WINMODE_MODE1; break;
		case ADC_INT_BELOW_MAX: winmode = ADC_WINCTRL_WINMODE_MODE2; break;
		default: return;
	}

	adc_cb = callback;

	configGCLK6();

	// Configure ADC to use GCLK6 (OSCULP32K)
	while (GCLK->STATUS.bit.SYNCBUSY) {}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
						| GCLK_CLKCTRL_GEN_GCLK6
						| GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {}

	// Set ADC prescaler as low as possible
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Configure window mode
	ADC->WINLT.reg = lo;
	ADC->WINUT.reg = hi;
	ADC->WINCTRL.reg = winmode;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable window interrupt
	ADC->INTENSET.bit.WINMON = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable ADC in standby mode
	ADC->CTRLA.bit.RUNSTDBY = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable continuous conversions
	ADC->CTRLB.bit.FREERUN = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Configure input mux
	ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable the ADC
	ADC->CTRLA.bit.ENABLE = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Start continuous conversions
	ADC->SWTRIG.bit.START = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable the ADC interrupt
	NVIC_EnableIRQ(ADC_IRQn);
}

void ArduinoLowPowerClass::detachAdcInterrupt()
{
	// Disable the ADC interrupt
	NVIC_DisableIRQ(ADC_IRQn);

	// Disable the ADC
	ADC->CTRLA.bit.ENABLE = 0;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable continuous conversions
	ADC->CTRLB.bit.FREERUN = 0;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable ADC in standby mode
	ADC->CTRLA.bit.RUNSTDBY = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable window interrupt
	ADC->INTENCLR.bit.WINMON = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable window mode
	ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Restore ADC prescaler
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Restore ADC clock
	while (GCLK->STATUS.bit.SYNCBUSY) {}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
						| GCLK_CLKCTRL_GEN_GCLK0
						| GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {}

	adc_cb = nullptr;
}

void ADC_Handler()
{
	// Clear the interrupt flag
	ADC->INTFLAG.bit.WINMON = 1;
	LowPower.adc_cb();
}
#endif

void ArduinoLowPowerClass::wakeOnWire(TwoWire * wire, bool intEnable) {
	wire->sercom->disableWIRE();
	wire->sercom->sercom->I2CS.CTRLA.bit.RUNSTDBY = intEnable ;
	wire->sercom->enableWIRE();
}

void ArduinoLowPowerClass::wakeOnSPI(SPIClass * spi, bool intEnable) {
	spi->_p_sercom->disableSPI();
	spi->_p_sercom->sercom->SPI.CTRLA.bit.RUNSTDBY = intEnable ;
	spi->_p_sercom->enableSPI();
}

void ArduinoLowPowerClass::wakeOnSerial(Uart * uart, bool intEnable) {
	uart->sercom->disableUART();
	uart->sercom->sercom->USART.CTRLA.bit.RUNSTDBY = intEnable ;
	uart->sercom->enableUART();
}

ArduinoLowPowerClass LowPower;

#endif // ARDUINO_ARCH_SAMD
