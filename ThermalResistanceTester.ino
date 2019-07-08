// The Thermal Resistance Tester
// (c) Rainer Urlacher 2019
//
// for more details and usage see videos in my Youtube video channels:
// Elekrobits         (German)  https://youtu.be/yBIe8qAUBh4
//
// hard- and software published under the MIT License
// - on github:      https://github.com/RainerU/ThermalResistanceTester
// - and on EasyEDA: https://easyeda.com/BitsAndElectrons/Thermal-Resistance-Meter


// smartdebug library
// https://github.com/hobbyelektroniker/SmartDebug/blob/master/smartdebug.h
// https://youtu.be/MOJGBwsPD7I
//#define _SMARTDEBUG  // activate this line for debug printing
#include <smartdebug.h>


// display library u8g2 by olikraus
// https://github.com/olikraus/u8g2
// there go to the wiki
#include <U8g2lib.h>
// Nokia 5110 display
#define displayCE  4
#define displayDIN 5
#define displayDC  4
#define displayCLK 6
#define displayRST 7
// display constructor, PCD8544 84X48, full framebuffer, size = 528 bytes
U8G2_PCD8544_84X48_F_4W_SW_SPI display(U8G2_R0, displayCLK, displayDIN, displayCE, displayDC, displayRST);
// display lines
#define line1 7
#define line2 15
#define line3 23
#define line4 31
#define line5 39
#define line6 47


// UI pins
#define LED 2
#define SWITCH 11
#define BUTTON 12


// ADC
#define adcReso 1023.0 // ADC resolution
#define adcRef 3.256 // ADC voltage reference (measured)
int readADC(int pin) {
	analogRead(pin); // first read is ignored
	delay(10); int value0 = analogRead(pin);
	delay(10); int value1 = analogRead(pin);
	delay(10); int value2 = analogRead(pin);
	// find the median, which is the middle value in a sorted list, no value is modified
	if (value0 < value1) {
		if (value1 < value2) {
			return value1;
		} else {
			if (value0 < value2) {
				return value2;
			}
		}
	} else {
		if (value2 < value1) {
			return value1;
		} else {
			if (value2 < value0) {
				return value2;
			}
		}
	}
	return value0;
};


// temperature measurement
#define SENSOR A7
#define tempMin 40
#define tempStep 10
#define tempMax 80
float ambientTemp = 22; // ambient temperature
float currentTemp; // last measured T
float targetTemp = 50; // user selected T for measurement
// NTC parameters
#define sensorRHigh 10000.0 // high side resistor connected to NTC
#define sensorAmp 2 // amplification factor
#define sensorB 3988.0 // B factor of NTC B57703M0103G040
#define sensorR25 10000.0 // R at 25dgrC of NTC 
float currentRNTC;
// filter for detection of a stable temperature
#define tempFilterWeight 0.01 // weight of new value for filtering
float filteredTemp = 0;
#define tempStableThreshold 0.1 // max. range for a stable temperature (+ or -)
#define tempStableTimeThreshold 30000 // min. duration of a stable temperature
bool tempIsStable = false;
unsigned long tempStableTime = 0;
// read temperature value
void readTemperature() {
	currentRNTC = adcReso/(float)readADC(SENSOR)*sensorAmp - 1;
	currentRNTC = sensorRHigh/currentRNTC; // this is the NTC resistant
	currentTemp = log(currentRNTC/sensorR25);
	currentTemp = 1/298.15 + 1/sensorB*currentTemp;
	currentTemp = 1/currentTemp - 273.15; // this is the NTC temperature
	// low-pass filter of temperature
	if (filteredTemp == 0) filteredTemp = currentTemp;
	else filteredTemp = (filteredTemp + currentTemp*tempFilterWeight)/(1 + tempFilterWeight);
	// detect stable temperature
	if ((currentTemp - filteredTemp > -tempStableThreshold) &&
	    (currentTemp - filteredTemp < tempStableThreshold)) {
		if (millis() >= tempStableTime + tempStableTimeThreshold) tempIsStable = true;
	} else {
		tempIsStable = false;
		tempStableTime = millis();
	};
};


// voltage measurement
#define VOLTAGE A0
#define voltageDivider 0.0909091 // 1k/(10k + 1k)
#define heaterCurrent 1.0 // real (measured) constant current
#define shuntVoltage 0.25 // voltage consumed by shunt resistor
// filter
#define voltageFilterWeight 0.2
float voltage = 0;
float filteredVoltage = 0;
float maxPower = 0;
void readVoltage() {
	voltage = readADC(VOLTAGE)*adcRef/adcReso/voltageDivider;
	if (filteredVoltage == 0) filteredVoltage = voltage;
	else filteredVoltage = (filteredVoltage + voltage*voltageFilterWeight)/(1 + voltageFilterWeight);
	// limit power if Junction Temperature becomes higher than 120dgrC
	maxPower = (120.0 - filteredTemp)/2; // (Tjmax - Tsense)/Rth_transistor
	// limit power to Input Voltage * 1A, more is not available
	if ((filteredVoltage - shuntVoltage)*heaterCurrent < maxPower) maxPower = (filteredVoltage - shuntVoltage)*heaterCurrent;
};


// heater control
#define HEATER 10 // output pin for heater transistor
float heaterPower = 0; // heater power in Watt
void increaseHeaterPower() { // this is used only before a power can be calculated from Rth
	if (heaterPower == 0.0) heaterPower = 0.3;
	else if (heaterPower == 0.3) heaterPower = 0.6;
	else if (heaterPower == 0.6) heaterPower = 1.0;
	else if (heaterPower == 1.0) heaterPower = 1.5;
	else if (heaterPower == 1.5) heaterPower = 2.0;
	else heaterPower += 1.0;
	if (heaterPower > maxPower) heaterPower = maxPower;
	tempIsStable = false;
	tempStableTime = millis();
};
int heaterPowerLevel = 0; // value between 0=0% and 255=100%
float heaterPowerRemain = 0; // rounding error remained for the next cycle
float rampTargetTemp = 0; // temporary target temperature
float resistant = 0; // measured thermal resistant
float resistantMin = 0;
float resistantMax = 0;
unsigned long heaterConstantTime = 0;
// heater state machine
#define heaterOff        1 // heater is off, do nothing (not used)
#define heaterCooling    2 // cool down below target before start
#define heaterRunning80  3 // heater is running for 80% temp delta
#define heaterRunning100 4 // heater is running for 100% temp delta
#define heaterConstant   5 // heater is running with constant power
char heaterState = heaterOff; // state of heater state machine
// heater regulator
void calculatePower() {
	switch (heaterState) {
		case heaterOff: // heater was off, start now
			heaterState = heaterCooling;
			DEBUG_PRINTLN("heaterOff done");
			break;
		case heaterCooling: // cool down below target before heating
			if (filteredTemp < targetTemp) {
				rampTargetTemp = 0.8*targetTemp + 0.2*ambientTemp; // 80% of delta
				increaseHeaterPower();
				heaterState = heaterRunning80;
				DEBUG_PRINTLN("heaterCooling done");
			};
			break;
		case heaterRunning80: // heater is running and heating for 80% of temperature delta
			if (tempIsStable) {
				if (filteredTemp > rampTargetTemp - 2) {
					rampTargetTemp = targetTemp; // 100% of delta
					heaterState = heaterRunning100;
					DEBUG_PRINTLN("heater80 done");
				};
				if (filteredTemp > ambientTemp + 5) {
					resistant = (filteredTemp - ambientTemp)/heaterPower;
					heaterPower = (rampTargetTemp - ambientTemp)/resistant;
					if (heaterPower > maxPower) heaterPower = maxPower;
					tempIsStable = false;
					tempStableTime = millis();
				} else {
					increaseHeaterPower();
				};
			};
			break;
		case heaterRunning100: // heater is running and heating for 100% of temperature delta
			if (tempIsStable) {
				resistant = (filteredTemp - ambientTemp)/heaterPower;
				heaterPower = (rampTargetTemp - ambientTemp)/resistant;
				if (heaterPower > maxPower) heaterPower = maxPower;
				tempIsStable = false;
				tempStableTime = millis();
				if ((filteredTemp > targetTemp-1) && (filteredTemp < targetTemp+1)) {
					heaterState = heaterConstant;
					resistantMin = resistant;
					resistantMax = resistant;
					heaterConstantTime = tempStableTime;
					DEBUG_PRINTLN("heater100 done");
				};
			};
			break;
		case heaterConstant: // heater is running at constant power
			resistant = (filteredTemp - ambientTemp)/heaterPower;
			if (resistantMin > resistant) resistantMin = resistant;
			if (resistantMax < resistant) resistantMax = resistant;
			break;
	};
};


// UI state machine, comments are in loop function
#define stateTempSet           1
#define stateTempIncr          2
#define stateAmbTempSet        3
#define stateAmbTempIncr       4
#define stateHeatingWait       5
#define stateHeating           6
#define stateRunWait           7
#define stateSensorConfirm     8
#define stateVoltageError      9
#define stateTempError        10
char state = stateRunWait;
unsigned long modeStartTime; // time of mode pressed, for detecting long pressing
bool modeWait = false; // marker for pressed mode button (wait for releasing)

// print functions
void printVoltageError() {
	display.setCursor(0, line1);
	display.print(F("WARNING!!!"));
	display.setCursor(0, line3);
	display.print(F("voltage "));
	display.print(filteredVoltage, 1);
	display.print("V");
	display.setCursor(0, line4);
	display.print(F("out of range"));
};
void printTempError() {
	display.setCursor(0, line1);
	display.print(F("WARNING!!!"));
	display.setCursor(0, line2);
	display.print(F("temperature"));
	display.setCursor(0, line3);
	display.print(filteredTemp, 1);
	display.print("C");
	display.setCursor(0, line4);
	display.print(F("out of range"));
};
void printSwitchRunOff() {
	display.setCursor(0, line3);
	display.print(F("switch RUN off"));
};
void printSensorConnected() {
	display.setCursor(0, line1);
	display.print(F("Is the sensor"));
	display.setCursor(0, line2);
	display.print(F("connected well"));
	display.setCursor(0, line3);
	display.print(F("to heater?"));
	display.setCursor(0, line5);
	display.print(F("mode: yes"));
};
void printVTT() {
	display.setCursor(0, line1);
	display.print(F("VOLTAGE  "));
	if (filteredVoltage < 10) display.print(" ");
	display.print(filteredVoltage, 1);
	display.print("V");
	
	display.setCursor(0, line2);
	display.print(F("Tset    "));
	if (targetTemp < 100) display.print(" ");
	if (targetTemp < 10) display.print(" ");
	display.print(targetTemp, 1);
	display.print("C");
	
	display.setCursor(0, line3);
	display.print(F("Tsense  "));
	if (filteredTemp < 100) display.print(" ");
	if (filteredTemp < 10) display.print(" ");
	display.print(filteredTemp, 1);
	display.print("C");
};
void printPower() {
	display.setCursor(0, line4);
	display.print(F("POWER   "));
	if (heaterPower < 10) display.print(" ");
	display.print(heaterPower, 2);
	display.print("W");

	display.setCursor(0, line5);
	display.print(F("POWER PWM  "));
	if (heaterPowerLevel < 100) display.print(" ");
	if (heaterPowerLevel < 10) display.print(" ");
	display.print(heaterPowerLevel);
};
void printResistant() {
	display.setCursor(0, line6);
	if (resistant == 0) {
		display.print(F("mode:  power++"));
		return;
	};
	float r;
	unsigned long m = millis() % 14000;
	if ((m > 9000) || (resistantMin == 0)) {
		r = resistant;
		display.print(F("Rth  "));
	} else if (m > 6000) {
		r = resistantMax;
		display.print(F("Rmax "));
	} else if (m > 3000) {
		r = resistantMin;
		display.print(F("Rmin "));
	} else {
		float t = ((float)(millis() - heaterConstantTime))/60000;
		display.print(F("Time    "));
		if (t < 100) display.print(" ");
		if (t < 10) display.print(" ");
		display.print(t, 0);
		display.print(F("Min"));
		return;
	}
	if (r < 100) display.print(" ");
	if (r < 10) display.print(" ");
	display.print(r, 2); 
	display.print(F("K/W"));
};
void printAmbTemp() {
	display.setCursor(0, line4);
	display.print(F("Tambient "));
	int x = (int)(ambientTemp + 0.5);
	if (x < 10) display.print(" ");
	display.print(x);
	display.print(F(".0C"));
};
void printModeTarget() {
	display.setCursor(0, line6);
	display.print(F("MODE: Tset"));
};
void printModeAmbient() {
	display.setCursor(0, line6);
	display.print(F("MODE: Tambient"));
};


// setup ...
void setup(void) {
	analogReference(EXTERNAL); // use 3.3V at AREF pin
	
	analogWrite(HEATER, 255); // heater off, inverting output
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW); // LED off
	pinMode(SWITCH, INPUT_PULLUP);
	pinMode(BUTTON, INPUT_PULLUP);
	
	// set PWM frequency to 30.64 Hz (slowest possible when just changing the prescaler)
	// this reduces the effect of transient errors 
	// https://www.hackerspace-ffm.de/wiki/index.php?title=ArduinoVariablePWMFrequenz
	TCCR1B = TCCR1B & 0b11111000 | 0b00000101; 
	
	DEBUG_INIT(115200);

	// initialize display
	delay(300);
	display.begin();
	display.clearBuffer();
	display.setFont(u8g2_font_6x10_tr);
	display.setCursor(0, line2);
	display.print(F("Bits&Electrons"));
	display.setCursor(0, line4);
	display.print(F("   Thermal"));
	display.setCursor(0, line5);
	display.print(F("  Resistance"));
	display.setCursor(0, line6);
	display.print(F("    Tester"));
	display.sendBuffer();
	
	delay(3000);
};


// ... and go
void loop(void) {
	// read analog signals
	readTemperature();
	readVoltage();
	
	// emergency check for voltages
	if ((filteredVoltage < 9) || (filteredVoltage > 31)) state = stateVoltageError;
	
	// emergency check for temperature
	if (filteredTemp > targetTemp + 10) state = stateTempError;

	// calculate power level and run heater
	if (state == stateHeating) {
		calculatePower();
		if (heaterPower > maxPower) heaterPower = maxPower;
		// calculate 8 Bit PWM value
		heaterPowerLevel = (int)((heaterPower + heaterPowerRemain) / heaterCurrent / (filteredVoltage - shuntVoltage) * 255.0 + 0.5);
		heaterPowerLevel = constrain(heaterPowerLevel, 0, 255);
		analogWrite(HEATER, 255 - heaterPowerLevel); // heat with PWM, inverting output
		// calculate rounding error in order to compensate for it in the next loop
		if (heaterPowerLevel == 255) heaterPowerRemain = 0;
		else heaterPowerRemain += heaterPower - ((float)heaterPowerLevel)*heaterCurrent*(filteredVoltage - shuntVoltage)/255.0;
	}
	else { // heater off
		heaterState = heaterOff;
		heaterPower = 0;
		heaterPowerLevel = 0;
		resistant = 0;
		resistantMin = 0;
		resistantMax = 0;
		rampTargetTemp = targetTemp; // for debug display
		analogWrite(HEATER, 255); // = off, inverted output
	}

	//LED control
	if (state == stateHeating) {
		if (heaterState == heaterConstant) digitalWrite(LED, HIGH);
		else digitalWrite(LED, !digitalRead(LED));
	} else digitalWrite(LED, LOW);
	
	// UI state machine
	bool run = !digitalRead(SWITCH);
	bool mode = !digitalRead(BUTTON);
	display.clearBuffer();
	display.setFont(u8g2_font_6x10_tr);
	switch (state) {
		case stateVoltageError: // voltage out of range
			if ((filteredVoltage >= 9.5) && (filteredVoltage <= 30.5)) state = stateRunWait;
			printVoltageError();
			break;
		case stateTempError: // temperature out of range
			if (filteredTemp < targetTemp + 5) state = stateRunWait;
			printTempError();
			break;
		case stateRunWait: // on start wait for run-switch is off
			if (!run) state = stateAmbTempSet;
			printSwitchRunOff();
			break;
		case stateSensorConfirm: // befor heat, confirm that sensor is mounted to heating transistor
			if (mode) state = stateHeatingWait;
			if (!run) state = stateAmbTempSet;
			printSensorConnected();
			break;
		case stateTempSet: // do nothing, when mode is pressed, increase target temperature
			if (run) state = stateSensorConfirm;
			if (mode) {
				state = stateTempIncr;
				modeStartTime = millis();
			}
			printVTT();
			printAmbTemp();
			printModeTarget();
			break;
		case stateTempIncr: // increment target temperature
			if (!mode) {
				if (millis() - modeStartTime > 1000) {
					state = stateAmbTempSet;
				} else {
					state = stateTempSet;
					targetTemp = targetTemp + tempStep;
					if (targetTemp > tempMax) targetTemp = tempMin;
				};
			};
			printVTT();
			printAmbTemp();
			if (millis() - modeStartTime > 1000) printModeAmbient();
			else printModeTarget();
			break;
		case stateAmbTempSet: // do nothing, when mode is pressed, increase ambient temperature
			if (run) state = stateSensorConfirm;
			if (mode) {
				state = stateAmbTempIncr;
				modeStartTime = millis();
			};
			printVTT();
			printAmbTemp();
			printModeAmbient();
			break;
		case stateAmbTempIncr: // increment ambient temperature
			if (!mode) {
				if (millis() - modeStartTime > 1000) {
					state = stateTempSet;
				} else {
					state = stateAmbTempSet;
					ambientTemp = ambientTemp + 1;
					if (ambientTemp > 30) ambientTemp = 20;
				};
			};
			printVTT();
			printAmbTemp();
			if (millis() - modeStartTime > 1000) printModeTarget();
			else printModeAmbient();
			break;
		case stateHeatingWait: // wait for mode button to be disabled
			if (!mode) state = stateHeating;
			break;
		case stateHeating: // heater running
			if (!run) state = stateAmbTempSet;
			if (!mode) modeWait = false;
			if ((resistant == 0) && mode & !modeWait) {
				increaseHeaterPower();
				modeWait = true;
			}
			printVTT();
			printPower();
			printResistant();
			break;
	};
	display.sendBuffer();
	
	// output for serial monitor or plotter
	DEBUG_PRINT_VALUE(" P", heaterPower);
	DEBUG_PRINT_VALUE(" T target", rampTargetTemp);
	DEBUG_PRINT_VALUE(" T current", currentTemp);
	DEBUG_PRINTLN_VALUE(" T filtered", filteredTemp);
};
