/*
 ===========================================================
   i G R O W 
   H Y D R O P H O N I C  -- P L A N T -- R E G U L A T O R
   CODEBAESE CREATED: 9th Feb 2024 1:27AM
  ==========================================================
*/

//#include <protothreads.h>
#include <LCD_I2C.h>
#include <Servo.h>
#include <ph4502c_sensor.h>
#include <SoftwareSerial.h>
#include <uRTCLib.h>
#include <uEEPROMLib.h>
//#include <Wire.h>

#define RELAY_PUMP_INTAKE 12
#define RELAY_PUMP_SOLUTION 10
#define RELAY_PUMP_AIRSTONE 11
#define RELAY_PUMP_DRAIN 9

#define PIN_PH_LEVEL_SENSOR A2
#define PIN_TEMP_LEVEL_SENSOR A3
#define PIN_SOLUTION_LEVEL_SENSOR A1

#define PIN_LED_GROW_LIGHT 8
#define PIN_PHOTORESISTOR A6
#define PIN_SERVO_MOTOR_CUTTER 7

#define BLUETOOTH_TX 6
#define BLUETOOTH_RX 5

#define PIN_LCD_CLOCK A5
#define PIN_LCD_DATA A4

#define LCD_SIZE_COLUMN 16
#define LCD_SIZE_ROW 2

#define I2C_ADDRESS_LCD 0x27
#define I2C_ADDRESS_RTC 0x68
#define I2C_ADDRESS_EEPROM 0x57
#define I2C_ADDRESS_RTC_TEMPERATURE 0x5F

#define PH_THRESHOLD 4

/* =================================================

  NOTE TO MYSELF: For EEPROM, Storing date 
  starts at address 5

  Address 0 - 4 = iGrow String Signature [CONSTANT] 
  Address 5 = Month of the year
  Address 7 = Day of the Year
  Address 9 = Year


 ================================================= */


LCD_I2C lcd(I2C_ADDRESS_LCD, LCD_SIZE_COLUMN, LCD_SIZE_ROW);
Servo MG996;
PH4502C_Sensor phlevel(PIN_PH_LEVEL_SENSOR, PIN_TEMP_LEVEL_SENSOR);
SoftwareSerial bluetooth(BLUETOOTH_RX, BLUETOOTH_TX);
uRTCLib clock(I2C_ADDRESS_RTC);
uEEPROMLib eeprom(I2C_ADDRESS_EEPROM);

static uint8_t dayOfDrain, monthOfDrain;
static uint8_t dayOfHarvest, monthOfHarvest;
static uint16_t yearOfDrain, yearOfHarvest;
static uint8_t counter = 0;

String cmdBluetooth = "";

volatile bool isPumpingWaterIn = false;
volatile bool isPumpingAir = true;
volatile bool isPumpingSolution = false;
volatile bool isPumpingWaterOut = false;
volatile bool turnOffAllPumps = false;
volatile bool isReadyToCut = false;
volatile bool isDateSynched = false;
volatile bool isDateDisplayedInLCD = false;
volatile bool isNeedToCleanUp = false;


/*volatile bool isMonthToDrain, isDayToDrain;
volatile bool isMonthToHarvest, isDayToHarvest;*/

////////----- PUMP FUNCTIONS -----////////////
//////////////// S T A R T ///////////////////

/* Functions:

    void WaterIntake(){}
        A function that calls in the beginning of Arduino loop(),
        where it checks if the water inside the chassis are on maximum 
        threshold. if it is not, the code will activate the Relay Module
        where the Intake Pump is connected (see RELAY_PUMP_INTAKE pin definition).
    
    void DrainWater(){}
        A function that is called when a Serial data from Bluetooth had been received
        and are requesting to drain the water inside the chassis.
        [IMPLEMENTATION IS CURRENTLY LACKING, MORE CONDITIONS ARE NEEDED].

    void ApplySolution(){}
        A function that is called when injecting a solution to the renewed water.
        At first, the function will check if solution level is currently within the 
        max boundaries.

            If is:       Apply the solution
            If is not:   Notify the user to supply more in the chassis, until it reached the max level.
        
    void AirPump(){}
        A function that is called when activating the pump where Air Stone is installed.
        The function will continue to operate until an interrupt has been raised. 

*/

void turnOffPump() {
  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);
  digitalWrite(RELAY_PUMP_INTAKE, HIGH);
  digitalWrite(RELAY_PUMP_SOLUTION, HIGH);
}

void WaterIntake() {

  /*
    =======================================================================
    First, we check if Water Level reaches maximum threshold.
    If it's false, we'll continously pump in water inside the chassis.
    If it's true, we'll stop supply more water.
    =======================================================================
  */
  //uint16_t waterlevel = analogRead(PIN_WATER_LEVEL_SENSOR);

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Water Intake");
  lcd.setCursor(1, 1);
  lcd.print("Pumping In ...");

  Serial.println("Water Intake: Pumping In...");

  delay(1500);

  // detection with water level sensor

  /*while (waterlevel < 100) {
    
    // Continously pump in water, and display the current status onto the LCD.
    digitalWrite(RELAY_PUMP_INTAKE, LOW);

    // Update the water level reading
    waterlevel = analogRead(PIN_WATER_LEVEL_SENSOR);
  }*/

  turnOffPump();
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);
  digitalWrite(RELAY_PUMP_SOLUTION, HIGH);
  digitalWrite(RELAY_PUMP_INTAKE, LOW);
  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);
  //delay(167000);
  delay(5000);

  /* 
  /  ======================================
  /  After that, the pump will deactivite 
  /  and show some messages in the LCD
  /  that it is done pumping in water. 
  /  ======================================
  */

  digitalWrite(RELAY_PUMP_INTAKE, HIGH);

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Water Intake");
  lcd.setCursor(1, 1);
  lcd.print("Pumping Done!");

  Serial.println("Water Intake: Pumping Done!");

  delay(2000);

  /*
  / ===========================================
  / Sets the current date of Water Drain 
  / schedule plus the date of the harvest time.
  / ===========================================
  */

  /*rtc_module.updateTime();
  dayOfDrain = rtc_module.dayofmonth;
  monthOfDrain = rtc_module.month;

  dayOfHarvest = rtc_module.dayofmonth;
  monthOfHarvest = rtc_module.month + 1;

  if (monthOfHarvest > 12){
    monthOfHarvest = 1;
  }

  /*
  /  ==========================================
  /  And update the pump flags for status 
  /  checking on other pump functions
  /  ==========================================
  */

  isPumpingWaterIn = false;
  isPumpingWaterOut = false;
  isPumpingAir = true;
  isPumpingSolution = false;
  isDateDisplayedInLCD = false;
}

void DrainWater() {

  /*
  / ========================================================
  / Activate the relay conneted to the drain pump
  / to drain the water continously until there's no water
  / inside, and display on the LCD the current status 
  / [A CERTAIN CONDITION NEEDS TO BE DEFINED WHEN TO STOP DRAINING WATER]
  / ========================================================
  */

  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Drain Pump");
  lcd.setCursor(0, 1);
  lcd.print("Draining Out... ");
  lcd.blink();
  Serial.println("Drain Pump: Draining Out...");

  delay(1500);

  digitalWrite(RELAY_PUMP_DRAIN, LOW);
  digitalWrite(RELAY_PUMP_SOLUTION, HIGH);
  digitalWrite(RELAY_PUMP_INTAKE, HIGH);
  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);

  //delay(180000);  // [THE CONDITION HERE ARE NOT FINALIZED. NEED FURTHER ELABORATION]
  delay(5000);
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);

  lcd.clear();
  lcd.noBlink();
  lcd.setCursor(3, 0);
  lcd.print("Drain Pump");
  lcd.setCursor(1, 1);
  lcd.print("Draining Done!");

  Serial.println("Drain Pump: Draining Done!");
  delay(1500);

  if (turnOffAllPumps == true) {
    isPumpingAir = false;
    isPumpingSolution = false;
    isPumpingWaterIn = false;
    isPumpingWaterOut = false;
    isDateDisplayedInLCD = false;
  } else {
    isPumpingAir = false;
    isPumpingSolution = true;
    isPumpingWaterIn = false;
    isPumpingWaterOut = false;
    isDateDisplayedInLCD = false;
  }
}

void AirWater() {

  /*
  / ==========================================================+

  30..
  0
  /H345678 After all the process of draining, supplying in water,
  / and applying solution, the air water should be running
  / continously until there's a change of pump boolean flags 
  / through interrupts or Serial communication via Bluetooth) 
  / during operation.
  / ===========================================================
  */

  if (isDateDisplayedInLCD == false) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("DoH: ");
    lcd.setCursor(5, 1);
    lcd.print(monthOfHarvest);
    lcd.setCursor(7, 1);
    lcd.print("/");
    lcd.setCursor(8, 1);
    lcd.print(dayOfHarvest);
    lcd.setCursor(10, 1);
    lcd.print("/");
    lcd.setCursor(11, 1);
    lcd.print(yearOfHarvest);
    Serial.println("Air Stone");
    isDateDisplayedInLCD = true;
  }

  lcd.setCursor(0, 0);
  lcd.print("pH Level: ");
  lcd.setCursor(11, 0);
  lcd.print(phlevel.read_ph_level());

  if (phlevel.read_ph_level() <= 4) {
    isNeedToCleanUp = true;
  }

  if (isPumpingWaterOut == true) {
    lcd.clear();
    Serial.println("Turn off air stone");
    turnOffPump();
    delay(1000);
  } else {
    digitalWrite(RELAY_PUMP_DRAIN, HIGH);
    digitalWrite(RELAY_PUMP_SOLUTION, HIGH);
    digitalWrite(RELAY_PUMP_INTAKE, HIGH);
    digitalWrite(RELAY_PUMP_AIRSTONE, LOW);
  }

  Serial.println("Air Stone");
}

void ApplySolution() {
  /*
  / ===================================================================
  / After pumping in renewed water from external source to chassis,
  / This function will then run to sujpply a hydrophonic solution
  / to provide nutrients in the water.
  / ===================================================================
  */

  pinMode(PIN_SOLUTION_LEVEL_SENSOR, INPUT);
  uint16_t solutionLevel = analogRead(PIN_SOLUTION_LEVEL_SENSOR);

  if (solutionLevel < 400) {
    digitalWrite(PIN_LED_GROW_LIGHT, HIGH);
    delay(500);
    digitalWrite(PIN_LED_GROW_LIGHT, LOW);
    delay(500);
    digitalWrite(PIN_LED_GROW_LIGHT, HIGH);
    delay(500);
    digitalWrite(PIN_LED_GROW_LIGHT, LOW);
    delay(500);
    Serial.println("No Solution");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Solution!");
    delay(2000);
  }

  digitalWrite(PIN_LED_GROW_LIGHT, LOW);
  pinMode(PIN_SOLUTION_LEVEL_SENSOR, OUTPUT);
  solutionLevel = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Hydrophonic Sltn");
  lcd.setCursor(2, 1);
  lcd.print("Injecting...");
  lcd.blink();

  Serial.println("Hydrophonic Sltn: Injecting...");

  delay(1500);
  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);
  digitalWrite(RELAY_PUMP_INTAKE, HIGH);
  digitalWrite(RELAY_PUMP_SOLUTION, LOW);
  delay(3000);
  digitalWrite(RELAY_PUMP_SOLUTION, HIGH);

  Serial.println("Hydrophonic Sltn: Injecting Done!");

  lcd.clear();
  lcd.noBlink();
  lcd.setCursor(0, 0);
  lcd.println("Hydrophonic Sltn");
  lcd.setCursor(1, 1);
  lcd.print("Injection Done!");

  delay(1500);

  isPumpingAir = false;
  isPumpingSolution = false;
  isPumpingWaterIn = true;
  isPumpingWaterOut = false;
  isDateDisplayedInLCD = false;

  lcd.clear();
}

void CleanUp() {

  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);
  digitalWrite(RELAY_PUMP_INTAKE, HIGH);
  digitalWrite(RELAY_PUMP_SOLUTION, HIGH);

  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cleaning Up...");

  delay(1000);

  digitalWrite(RELAY_PUMP_DRAIN, LOW);
  //delay(1500);
  delay(180000);
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);
  delay(1500);

  digitalWrite(RELAY_PUMP_INTAKE, LOW);
  //delay(1500);
  delay(100000);
  digitalWrite(RELAY_PUMP_INTAKE, HIGH);
  delay(1500);

  digitalWrite(RELAY_PUMP_AIRSTONE, LOW);
  //delay(1500);
  delay(60000);
  digitalWrite(RELAY_PUMP_AIRSTONE, HIGH);
  delay(1500);

  digitalWrite(RELAY_PUMP_DRAIN, LOW);
  //delay(1500);
  delay(120000);
  digitalWrite(RELAY_PUMP_DRAIN, HIGH);

  isPumpingAir = false;
  isPumpingSolution = false;
  isPumpingWaterIn = false;
  isPumpingWaterOut = false;
  isNeedToCleanUp = false;
  isDateDisplayedInLCD = false;
}

////////----- PUMP FUNCTIONS -----////////////
////////////////// E N D /////////////////////

/*bool TimeForHarvestAndDrain(){
  uint8_t monthDifference;

  monthDifference = monthOfHarvest - rtc_module.dayofmonth;

  if (monthDifference == 0){
    return true;
  } else {
    return false;
  }
}*/

/////////----- DEBUGGING FUNCTIONS -----////////////
////////////////// S T A R T ///////////////////////

/*void debugServo(void _syntax){

  MG996.write(180);
  delay(3500);
  MG996.write(0);
  delay(3500);

}*/

/////////----- DEBUGGING FUNCTIONS -----////////////
//////////////////// E N D /////////////////////////



void setup() {

  // Initialize Pinout
  pinMode(RELAY_PUMP_AIRSTONE, OUTPUT);
  pinMode(RELAY_PUMP_DRAIN, OUTPUT);
  pinMode(RELAY_PUMP_INTAKE, OUTPUT);
  pinMode(RELAY_PUMP_SOLUTION, OUTPUT);
  //pinMode(BUTTON_DRAIN, INPUT_PULLUP);
  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(A5, INPUT);
  pinMode(2, INPUT);
  pinMode(PIN_LED_GROW_LIGHT, OUTPUT);
  pinMode(PIN_SOLUTION_LEVEL_SENSOR, INPUT);
  pinMode(PIN_PHOTORESISTOR, INPUT);

  turnOffPump();

  // calibrate ph level sensor (default: 0.0 = ph 7 -- neutral) and initialize
  //phlevel.recalibrate(3000);
  phlevel.init();

  // Initialize LCD Process and LCD Backlight
  lcd.begin();
  lcd.backlight();
  lcd.clear();

  // UNCOMMENT THIS ON DEBUGGING PHASE
  Serial.begin(9600);
  bluetooth.begin(9600);

  MG996.attach(PIN_SERVO_MOTOR_CUTTER);
  MG996.write(0);

  URTCLIB_WIRE.begin();

  clock.set(0, 10, 12, URTCLIB_WEEKDAY_SATURDAY, 18, 5, 24);
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)

  /*clock.refresh();
  dayOfHarvest = clock.day();
  monthOfHarvest = clock.month();
  yearOfHarvest = clock.year();*/

  eeprom.eeprom_write(5, 4);   // set month on eeprom
  eeprom.eeprom_write(7, 9);   // set day on eeprom
  eeprom.eeprom_write(9, 24);  // set year on eeprom
  eeprom.eeprom_read(5, &monthOfHarvest);
  eeprom.eeprom_read(7, &dayOfHarvest);
  eeprom.eeprom_read(9, &yearOfHarvest);

  delay(1000);
}

void loop() {
  // read ph level
  //timer.updateTime();                                             // get current updated time

  if (bluetooth.available()) {
    cmdBluetooth = bluetooth.readString();
    Serial.println(cmdBluetooth);

    if (cmdBluetooth.equalsIgnoreCase("da")) {
      isPumpingWaterOut = true;
      turnOffAllPumps = true;
    } else if (cmdBluetooth.equalsIgnoreCase("dn")) {
      isPumpingWaterOut = true;
      turnOffAllPumps = false;
    } else if (cmdBluetooth.equalsIgnoreCase("in")) {
      isPumpingSolution = true;
    } else if (cmdBluetooth.equalsIgnoreCase("cp")) {
      isReadyToCut = true;
    }
  }

  if (digitalRead(2) == true) {
    if (isDateSynched == false) {
      bluetooth.print(monthOfHarvest);
      bluetooth.print("/");
      bluetooth.print(dayOfHarvest);
      bluetooth.print("/");
      bluetooth.println(yearOfHarvest + 2000);
      isDateSynched = true;
    }
  } else {
    isDateSynched = false;
  }

  if (analogRead(PIN_PHOTORESISTOR) < 200) {  // Activate LED Grow Light on LDR Conditions
    digitalWrite(PIN_LED_GROW_LIGHT, HIGH);
  } else {
    digitalWrite(PIN_LED_GROW_LIGHT, LOW);
  }

  /*if (TimeForHarvestAndDrain() == true){
    turnOffPump();
    isPumpingAir        = false;
    isPumpingWaterIn    = false;
    isPumpingWaterOut   = false;
    isPumpingSolution   = false;
  }*/

  if (isPumpingAir == true) {  // Activates Air Stone
    AirWater();
  }

  if (isPumpingWaterIn == true) {  // Activates Water Intake Pump
    WaterIntake();
  }

  if (isPumpingSolution == true) {  // Activates Solution Injector
    ApplySolution();
  }

  if (isPumpingWaterOut == true) {  // Activates Water Drain Pump
    DrainWater();
  }

  if (isNeedToCleanUp == true) {
    CleanUp();
  }

  if (isReadyToCut == true) {
    turnOffPump();
    delay(1000);
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Cutting...");
    lcd.blink();
    Serial.print("Cutting...\n");
    delay(1500);

    MG996.write(90);

    delay(1500);
    lcd.noBlink();
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Done Cutting! ");
    Serial.print("Done Cutting!\n");
    MG996.write(0);
    delay(1500);
    lcd.clear();

    clock.refresh();
    monthOfHarvest = clock.month();
    dayOfHarvest = clock.day();
    yearOfHarvest = clock.year();

    eeprom.eeprom_write(5, monthOfHarvest);  // set month on eeprom
    eeprom.eeprom_write(7, dayOfHarvest);    // set day on eeprom
    eeprom.eeprom_write(9, yearOfHarvest);   // set year on eeprom*/

    isDateSynched = false;
    isDateDisplayedInLCD = false;
    isPumpingAir = true;
    isPumpingWaterIn = false;
    isPumpingWaterOut = false;
    isPumpingSolution = false;
    isReadyToCut = false;
  }
}