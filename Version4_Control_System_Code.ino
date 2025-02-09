/**************************************************************
*     Rainwater Processing System Control Module Firmware     *
*                        Version 1.6                          *
*                           Rev 0                             *
*                       February 2025                         *
*                                                             *
*                    RainCycle Systems                        *
*             Highline Technical Enterprises LLC              *
*       Sole Intellectual Property of Andrew McMillian        *
*                                                             *
*                  Contact: 406-209-5710                      *
*           highlinetechnicalenterprises@gmail.com            *
***************************************************************/





// -------------------------------------------------- Pin Assignments -------------------------------------------------- 

// User Interface Inputs
#define MANUAL_STOP_RECIRCULATION 2
#define MANUAL_PRIME_DISTRIBUTION_PUMP 4
#define MANUAL_PRIME_TRANSFER_PUMP 12
#define TRANSFER_PUMP_MODE_NORMAL 5
#define TRANSFER_PUMP_MODE_MANUAL 6
#define DISTRIBUTION_PUMP_MODE_NORMAL 7
#define DISTRIBUTION_PUMP_MODE_MANUAL 8
#define RECIRCULATION_FUNCTION_ENABLE 9
#define SEDIMENT_PURGE_FUNCTION_NORMAL 10
#define SEDIMENT_PURGE_FUNCTION_MANUAL 11

// Indicator Light Assignments
#define NORMAL_AUTO_INDICATOR 30
#define TRANSFER_PUMP_ON_INDICATOR 31
#define DISTRIBUTION_PUMP_ON_INDICATOR 32
#define UV_LAMP_ON_INDICATOR 33
#define NEW_WATER_PROCESS_INDICATOR 34
#define ACCUMULATOR_UPPER_FLOAT_INDICATOR 35
#define ACCUMULATOR_LOWER_FLOAT_INDICATOR 36
#define STORAGE_UPPER_FLOAT_INDICATOR 37
#define STORAGE_LOWER_FLOAT_INDICATOR 38
#define DISTRIBUTION_PRESSURE_LOW_INDICATOR 39
#define MASTER_CAUTION_INDICATOR 46

//Warning Buzzer
#define BUZZER 28

// Sensors
#define DISTRIBUTION_PRESSURE_SWITCH 3
#define ACCUMULATOR_UPPER_FLOAT 14
#define ACCUMULATOR_LOWER_FLOAT 15
#define STORAGE_UPPER_FLOAT 16
#define STORAGE_LOWER_FLOAT 17

// Actuators (on 8 unit relay module)
#define TRANSFER_PUMP_RELAY 22
#define PWM_RELAY 23
#define DISTRIBUTION_PUMP_RELAY 24
#define RECIRCULATION_MODE_RELAY 25
#define RECIRCULATION_VALVE_RELAY 26
#define PURGE_VALVE_RELAY 27
#define UV_LIGHT_RELAY 28
#define MASTER_PWR_RELAY 28

// -------------------------------------------------- Constants -------------------------------------------------- 

//Time Constants
const int refresh_rate = 100; //system refresh at 0.1 seconds (need to adjust to max while maintaining stability)
const int RECIRCULATION_MODE_TIMEOUT = 1380000; // 23 minutes
const int valve_operation_time = 6000 //6 seconds
const int pump_stop_delay = 500 // 0.5 second
const int uv_light_delay = 2500 // 2.5 second
const int short_startup_steps_delay = 750 // 0.75 second
const int long_startup_steps_delay = 2500 // 2.5 seconds

//I2C Addresses:
#define OLED_ADDRESS 0x3C //Display Address
#define RTC_ADDR 0x68 //Real Time Clock Address

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// -------------------------------------------------- Global Variables -------------------------------------------------- 

volatile bool UVlightActive = false;
volatile bool recirculationValveOpen = false;
volatile bool purgeValveOpen = false;
volatile bool recirculationActive = false;
volatile bool manualExitTriggered = false;
volatile bool autoExitTriggered = false;
volatile bool transferPumpDryRun = false;
volatile bool distributionPumpDryRun = false;
volatile bool transferPumpActive = false;
volatile bool recirculationModeActive = false;
volatile bool recirculationModePending = false;
volatile bool recirculationModeBlockTransferPump = false;
volatile unsigned long recirculationModeTimeOfRequestedAction = 0;
volatile unsigned long recirculationModeRunTime = 0;

// Global State Variables for Control Modes
volatile bool transferPumpModeNormal = false;
volatile bool transferPumpModeDisabled = false;
volatile bool transferPumpModeManual = false;

volatile bool distributionPumpModeNormal = false;
volatile bool distributionPumpModeDisabled = false;
volatile bool distributionPumpModeManual = false;

volatile bool recirculationFunctionNormal = false;
volatile bool recirculationFunctionDisabled = false;

volatile bool sedimentScreenPurgeNormal = false;
volatile bool sedimentScreenPurgeDisabled = false;
volatile bool sedimentScreenPurgeManual = false;

volatile int currentControlState = 0; //sets the current state to idle
volatile int previousControlState = 0; //sets the previous state to idle
volatile bool currentControlStateIsPreviousState = false;

//System Alarm States (for audible buzzer and the master caution light)
volatile bool alarmState = false;
volatile int alarmMode = 0; //used to set tone

//Misc.
volatile float temperature = 0; //hold control module temperature

//Software lockouts for electronic hardware
volatile bool displayDisabled = false; 
volatile bool RTC_Disabled = false; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! update: auto recirculate function to accomidate this !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
volatile bool AudibleWarningDisabled = false; 

//Display
volatile int TextSize = 1;
volatile int lcdPrintLine = 0;

// -------------------------------------------------- Libraries -------------------------------------------------- 


#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Create objects for RTC and OLED
RTC_DS3231 rtc;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {


    // #################################################### Setup serial com link, OLED display and RTC module #########################################################
    
    Serial.begin(115200);
    Wire.begin();

   // Initialize display
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED display initialization failed!");
        delay(100);
        Serial.println("Trying Again...");
         if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)){
            Serial.println("Second display failure...disabling display");
            displayDisabled = true; //display disabled from here on out
         }
    }

    //Send a startup message
    Serial.println("System Initializing...");
    
    display.clearDisplay();
    display.setTextSize(TextSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, lcdPrintLine);
    display.print("System Initializing...");
    display.display();
    
    delay(short_startup_steps_delay);
    lcdPrintLine = lcdPrintLine + 16;


    Serial.println("Connecting To RTC...");
    
    display.setCursor(0, lcdPrintLine);
    display.print("Connecting To RTC...");
    display.display();
    delay(short_startup_steps_delay);
    lcdPrintLine = lcdPrintLine + 16;
    
    // ---------------------------------------------------------- Initialize RTC -------------------------------------------

    if (!rtc.begin()) {
        Serial.println("RTC initialization failed!");
           
        display.setCursor(0, lcdPrintLine);
        display.print("RTC Failure...");
        display.display();
        delay(short_startup_steps_delay);
        lcdPrintLine = lcdPrintLine + 16;
        delay(100);
        
        Serial.println("Trying Again...");
         if (!rtc.begin()){
            Serial.println("Second RTC failure...disabling display");
            
            display.setCursor(0, lcdPrintLine);
            display.print("Disabling RTC...");
            display.display();
            delay(short_startup_steps_delay);
            lcdPrintLine = lcdPrintLine + 16;

             RTC_Disabled = true; //RTC disabled from here on out (this changes the method for auto recirculation timing)
             }
        } else{
                DateTime now = rtc.now();  // Get current date, time, and temperature
                temperature = rtc.getTemperature();  // Get temperature from DS3231
        

             // Print to OLED Display
             display.clearDisplay();
             display.setTextSize(TextSize);
             display.setTextColor(SSD1306_WHITE);
             display.setCursor(0, 0);
             display.print("System Initializing...");
             display.setCursor(0, 32);
             display.print("RTC Active, Time/Date:");
             display.display();

             delay(short_startup_steps_delay);
        
             display.clearDisplay();
             display.setTextSize(1);
             display.setTextColor(SSD1306_WHITE);
             display.setCursor(0, 0);

             display.print("Date: ");
             display.print(now.year());
             display.print("/");
             display.print(now.month());
             display.print("/");
             display.print(now.day());
    
             display.setCursor(0, 16);
             display.print("Time: ");
             display.print(now.hour());
             display.print(":");
             display.print(now.minute());
             display.print(":");
             display.print(now.second());

             display.setCursor(0, 32);
             display.print("Temp: ");
             display.print(temperature);
             display.print(" C");

             display.setCursor(0, 52);
             display.print("Dryrun: ");
             display.print(dryrun);


             display.display(); // Update OLED display

             delay(long_startup_steps_delay);
        
            
        //************************************************** RTC Accuracy Check ***********************************************************

        
        }

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! check for missing display setup code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Missing RTC Code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    */

    // ######################################################### Pinmodes and Interupts ############################################################

// ------------------------------------------------------------------ UI Output Initialization ---------------------------------------

    
    Serial.println("Initalizing UI Outputs and Running Test.  Verify every light turns on the buzzer sounds...");
    
    // Display Time
    display.clearDisplay();
    display.display();
    lcdPrintLine = 0;
    
    display.setCursor(0, lcdPrintLine);
    display.print("Activating All UI Outputs...");
    display.display();
    delay(short_startup_steps_delay);
    lcdPrintLine = lcdPrintLine + 16;
    
    //Indicactors and buzzer pin mode assignments
    pinMode(TRANSFER_PUMP_ON_INDICATOR, OUTPUT);
    pinMode(DISTRIBUTION_PUMP_ON_INDICATOR, OUTPUT);
    pinMode(UV_LAMP_ON_INDICATOR, OUTPUT);
    pinMode(NEW_WATER_PROCESS_INDICATOR, OUTPUT);
    pinMode(ACCUMULATOR_UPPER_FLOAT_INDICATOR, OUTPUT);
    pinMode(ACCUMULATOR_LOWER_FLOAT_INDICATOR, OUTPUT);
    pinMode(STORAGE_UPPER_FLOAT_INDICATOR, OUTPUT);
    pinMode(STORAGE_LOWER_FLOAT_INDICATOR, OUTPUT);
    pinMode(DISTRIBUTION_PRESSURE_LOW_INDICATOR, OUTPUT);
    pinMode(MASTER_CAUTION_INDICATOR, OUTPUT);

    pinMode(BUZZER, OUTPUT);

    //Check to see if it all works
    digitalWrite(TRANSFER_PUMP_ON_INDICATOR, HIGH);
    digitalWrite(DISTRIBUTION_PUMP_ON_INDICATOR, HIGH);
    digitalWrite(UV_LAMP_ON_INDICATOR, HIGH);
    digitalWrite(NEW_WATER_PROCESS_INDICATOR, HIGH);
    digitalWrite(ACCUMULATOR_UPPER_FLOAT_INDICATOR, HIGH);
    digitalWrite(ACCUMULATOR_LOWER_FLOAT_INDICATOR, HIGH);
    digitalWrite(STORAGE_UPPER_FLOAT_INDICATOR, HIGH);
    digitalWrite(STORAGE_LOWER_FLOAT_INDICATOR, HIGH);
    digitalWrite(DISTRIBUTION_PRESSURE_LOW_INDICATOR, HIGH);
    digitalWrite(MASTER_CAUTION_INDICATOR, HIGH);

    if(AudibleWarningDisabled == false){
        tone(BUZZER, 500);
        Serial.println("AudibleWarningDisabled == false");
        display.setCursor(0, lcdPrintLine);
        display.print("Verify: All Indicators ON?");
        display.display();
      }else{
        Serial.println("AudibleWarningDisabled == true");
        display.setCursor(0, lcdPrintLine);
        display.print("All Lights ON - Buzzer DBLD");
        display.display();
        
      }

    //Provide time to check UI outputs and then reset outputs and display
    delay(long_startup_steps_delay);

    lcdPrintLine = 0;
    display.clearDisplay();

    //turn everything off

    noTone(BUZZER);

    digitalWrite(TRANSFER_PUMP_ON_INDICATOR, LOW);
    digitalWrite(DISTRIBUTION_PUMP_ON_INDICATOR, LOW);
    digitalWrite(UV_LAMP_ON_INDICATOR, LOW);
    digitalWrite(NEW_WATER_PROCESS_INDICATOR, LOW);
    digitalWrite(ACCUMULATOR_UPPER_FLOAT_INDICATOR, LOW);
    digitalWrite(ACCUMULATOR_LOWER_FLOAT_INDICATOR, LOW);
    digitalWrite(STORAGE_UPPER_FLOAT_INDICATOR, LOW);
    digitalWrite(STORAGE_LOWER_FLOAT_INDICATOR, LOW);
    digitalWrite(DISTRIBUTION_PRESSURE_LOW_INDICATOR, LOW);
    digitalWrite(MASTER_CAUTION_INDICATOR, LOW);

    display.setCursor(0, lcdPrintLine);
    display.print("System Initializing...");
    display.display();
    lcdPrintLine = 32;
    Serial.println("Turning all indicators off...");
    display.setCursor(0, lcdPrintLine);
    display.print("Verify: All Indicators OFF?");
    display.display();

    //Provide time to check UI outputs are off
    delay(long_startup_steps_delay);

    // ------------------------------------------------------------------ UI Input and SensorsInitialization ---------------------------------------
    lcdPrintLine = 0;
    display.clearDisplay();

    display.setCursor(0, lcdPrintLine);
    display.print("System Initializing...");
    display.display();
    lcdPrintLine = 16;
    Serial.println("Connecting UI Controls and Sensors...");
    display.setCursor(0, lcdPrintLine);
    display.print("Connecting Sensors and UI Controls...");
    display.display();

    

    //UI Controls

    pinMode(MANUAL_STOP_RECIRCULATION, INPUT_PULLUP);
    pinMode(MANUAL_PRIME_DISTRIBUTION_PUMP, INPUT_PULLUP);
    pinMode(MANUAL_PRIME_TRANSFER_PUMP, INPUT_PULLUP);
    pinMode(TRANSFER_PUMP_MODE_NORMAL, INPUT_PULLUP);
    pinMode(TRANSFER_PUMP_MODE_MANUAL, INPUT_PULLUP);
    pinMode(DISTRIBUTION_PUMP_MODE_NORMAL, INPUT_PULLUP);
    pinMode(DISTRIBUTION_PUMP_MODE_MANUAL, INPUT_PULLUP);
    pinMode(RECIRCULATION_FUNCTION_ENABLE, INPUT_PULLUP);
    pinMode(SEDIMENT_PURGE_FUNCTION_NORMAL, INPUT_PULLUP);
    pinMode(SEDIMENT_PURGE_FUNCTION_MANUAL, INPUT_PULLUP);

    
    //Float Switches

    pinMode(ACCUMULATOR_UPPER_FLOAT, INPUT_PULLUP);
    pinMode(ACCUMULATOR_LOWER_FLOAT, INPUT_PULLUP);
    pinMode(STORAGE_UPPER_FLOAT, INPUT_PULLUP);
    pinMode(STORAGE_LOWER_FLOAT, INPUT_PULLUP);

    //Sensor Interupt
    pinMode(DISTRIBUTION_PRESSURE_SWITCH, INPUT_PULLUP);

    //Attach Interupts (sensor and manual)
    attachInterrupt(digitalPinToInterrupt(MANUAL_STOP_RECIRCULATION), exitRecirculationManual, FALLING);
    attachInterrupt(digitalPinToInterrupt(DISTRIBUTION_PRESSURE_SWITCH), exitRecirculationAuto, FALLING);


    //-------------------------------------------------------- Setup relays and test functionality -----------------------------------------------------
    delay(short_startup_steps_delay);
    
    lcdPrintLine = 32;
    Serial.println("Connecting to relays...");
    display.setTextSize(TextSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, lcdPrintLine);
    display.print("Connecting to relays");
    display.display();

    
     //Outputs
    pinMode(TRANSFER_PUMP_RELAY, OUTPUT);
    pinMode(PWM_RELAY, OUTPUT);
    pinMode(DISTRIBUTION_PUMP_RELAY, OUTPUT);
    pinMode(RECIRCULATION_MODE_RELAY, OUTPUT);
    pinMode(RECIRCULATION_VALVE_RELAY, OUTPUT);
    pinMode(PURGE_VALVE_RELAY, OUTPUT);
    pinMode(UV_LIGHT_RELAY, OUTPUT);
    pinMode(MASTER_PWR_RELAY, OUTPUT);

    
    digitalWrite(MASTER_PWR_RELAY, LOW); //prevents activation of system power durring relay test
    
    pinMode(PWM_RELAY, OUTPUT);
    pinMode(DISTRIBUTION_PUMP_RELAY, OUTPUT);
    pinMode(RECIRCULATION_MODE_RELAY, OUTPUT);
    pinMode(RECIRCULATION_VALVE_RELAY, OUTPUT);
    pinMode(PURGE_VALVE_RELAY, OUTPUT);
    pinMode(UV_LIGHT_RELAY, OUTPUT);
    pinMode(TRANSFER_PUMP_RELAY, OUTPUT);


    

    
    delay(short_startup_steps_delay);

    lcdPrintLine = 0;
    display.clearDisplay();

    display.setTextSize(TextSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, lcdPrintLine);
    display.print("System Initialized");
    display.display();
    lcdPrintLine = 16;
    Serial.println("Connecting UI Controls and Sensors...");
    display.setTextSize(TextSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, lcdPrintLine);
    display.print("All Systems Go...");
    display.display();

    delay(long_startup_steps_delay);

} /* ______________________________________________________________ End of Setup() ___________________________________________________________________ */

// Function to display text on OLED Primary and Serial Monitor (will soon also log to SD card)
// Use the formate: showMessage("message", "display", 1); inwhich display is either primary or secondary and 1 is the text size
void showMessage(const char *message, const char *displayType, int textSize) {
    
    Serial.println(message);  // Print to Serial Monitor

    // Determine which display to update
    Adafruit_SSD1306 *targetDisplay = nullptr;

    if (strcmp(displayType, "primary") == 0) {
        targetDisplay = &primaryDisplay;
    } else if (strcmp(displayType, "secondary") == 0) {
        targetDisplay = &secondaryDisplay;
    } else {
        Serial.println("Error: Invalid display type! Use 'primary' or 'secondary'.");
        return;
    }

    // Clear the selected display and print the message
    targetDisplay->clearDisplay();
    targetDisplay->setCursor(0, 0);
    targetDisplay->setTextSize(textSize);   // Set the custom text size
    targetDisplay->setTextColor(SSD1306_WHITE);
    targetDisplay->print(message);
    targetDisplay->display();
}

void updateIndicators() {
    digitalWrite(TRANSFER_PUMP_ON_INDICATOR, transferPumpActive ? HIGH : LOW);
    digitalWrite(DISTRIBUTION_PUMP_ON_INDICATOR, distributionPumpDryRun ? HIGH : LOW);
    digitalWrite(UV_LAMP_ON_INDICATOR, recirculationModeActive ? HIGH : LOW);
    digitalWrite(NEW_WATER_PROCESS_INDICATOR, recirculationModePending ? HIGH : LOW);
    digitalWrite(ACCUMULATOR_UPPER_FLOAT_INDICATOR, digitalRead(ACCUMULATOR_UPPER_FLOAT));
    digitalWrite(ACCUMULATOR_LOWER_FLOAT_INDICATOR, digitalRead(ACCUMULATOR_LOWER_FLOAT));
    digitalWrite(STORAGE_UPPER_FLOAT_INDICATOR, digitalRead(STORAGE_UPPER_FLOAT));
    digitalWrite(STORAGE_LOWER_FLOAT_INDICATOR, digitalRead(STORAGE_LOWER_FLOAT));
    digitalWrite(DISTRIBUTION_PRESSURE_LOW_INDICATOR, digitalRead(DISTRIBUTION_PRESSURE_SWITCH));
    digitalWrite(MASTER_CAUTION_INDICATOR, (transferPumpDryRun || distributionPumpDryRun) ? HIGH : LOW);

}

void readControlStates() {
    // Read Transfer Pump Mode Switch
    int transferPumpMode = digitalRead(TRANSFER_PUMP_MODE_NORMAL);
    int transferPumpModeManualPin = digitalRead(TRANSFER_PUMP_MODE_MANUAL);
    
    transferPumpModeNormal = (transferPumpMode == HIGH && transferPumpModeManualPin == LOW);
    transferPumpModeDisabled = (transferPumpMode == LOW && transferPumpModeManualPin == LOW);
    transferPumpModeManual = (transferPumpModeManualPin == HIGH);

    // Read Distribution Pump Mode Switch
    int distributionPumpModeNormalPin = digitalRead(DISTRIBUTION_PUMP_MODE_NORMAL);
    int distributionPumpModeManualPin = digitalRead(DISTRIBUTION_PUMP_MODE_MANUAL);
    
    distributionPumpModeNormal = (distributionPumpModeNormalPin == HIGH);
    distributionPumpModeDisabled = (distributionPumpModeNormalPin == LOW && distributionPumpModeManualPin == LOW);
    distributionPumpModeManual = (distributionPumpModeManualPin == HIGH);

    // Read Recirculation Function Switch
    int recirculationFunctionNormalPin = digitalRead(RECIRCULATION_FUNCTION_ENABLE);
    recirculationFunctionNormal = (recirculationFunctionNormalPin == HIGH);
    recirculationFunctionDisabled = (recirculationFunctionNormalPin == LOW);

    // Read Sediment Screen Purge Function Switch
    int sedimentScreenPurgeNormalPin = digitalRead(SEDIMENT_PURGE_FUNCTION_NORMAL);
    sedimentScreenPurgeNormal = (sedimentScreenPurgeNormalPin == HIGH);
    sedimentScreenPurgeDisabled = (sedimentScreenPurgeNormalPin == LOW);

    // Read Manual Control Buttons
    int manualStopRecirculation = digitalRead(MANUAL_STOP_RECIRCULATION);
    int manualPrimeTransferPump = digitalRead(MANUAL_PRIME_TRANSFER_PUMP);
    int manualPrimeDistributionPump = digitalRead(MANUAL_PRIME_DISTRIBUTION_PUMP);

    // Handle Manual Control Buttons
    manualStopRecirculationActive = (manualStopRecirculation == HIGH);
    manualPrimeTransferPumpActive = (manualPrimeTransferPump == HIGH);
    manualPrimeDistributionPumpActive = (manualPrimeDistributionPump == HIGH);
}



void determineControlMode() {
    // Reset the current state
    currentControlState = 0;

    // Check Transfer Pump Mode
    if (transferPumpModeDisabled) {
        currentControlState |= (1 << 0); // Bit 0 for disabled
    } else if (transferPumpModeManual) {
        currentControlState |= (1 << 1); // Bit 1 for manual
    } else {
        currentControlState |= (1 << 2); // Bit 2 for normal
    }

    // Check Distribution Pump Mode
    if (distributionPumpModeDisabled) {
        currentControlState |= (1 << 3); // Bit 3 for disabled
    } else if (distributionPumpModeManual) {
        currentControlState |= (1 << 4); // Bit 4 for manual
    } else {
        currentControlState |= (1 << 5); // Bit 5 for normal
    }

    // Check Recirculation Function
    if (recirculationFunctionDisabled) {
        currentControlState |= (1 << 6); // Bit 6 for disabled
    } else {
        currentControlState |= (1 << 7); // Bit 7 for normal
    }

    // Check Sediment Screen Purge Function
    if (sedimentScreenPurgeDisabled) {
        currentControlState |= (1 << 8); // Bit 8 for disabled
    } else {
        currentControlState |= (1 << 9); // Bit 9 for normal
    }

    // Check Manual Control Buttons
    if (manualStopRecirculationActive) {
        currentControlState |= (1 << 10); // Bit 10 for manual stop recirculation
    }
    if (manualPrimeTransferPumpActive) {
        currentControlState |= (1 << 11); // Bit 11 for manual prime transfer pump
    }
    if (manualPrimeDistributionPumpActive) {
        currentControlState |= (1 << 12); // Bit 12 for manual prime distribution pump
    }


    // ____________________________________________ Check to see if the control state has changed _________________________________________
    if(previousControlState == currentControlState){ 
        currentControlStateIsPreviousState = true;
    }else{
        previousControlState = currentControlState;
        currentControlStateIsPreviousState = false;
    }

}

void dryRunCutoffTransferPump() {
    if (digitalRead(ACCUMULATOR_LOWER_FLOAT) == HIGH) {
        digitalWrite(TRANSFER_PUMP_RELAY, LOW);
         millis(pump_stop_delay);
        transferPumpDryRun = true;
        Serial.println("Transfer Pump Dry Run Cutoff Triggered");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("TP Dry Run Cutoff");
        display.display();
    }
}

void dryRunCutoffDistributionPump() {
    if (digitalRead(STORAGE_LOWER_FLOAT) == HIGH) {
        digitalWrite(DISTRIBUTION_PUMP_RELAY, LOW);
         millis(pump_stop_delay);
        distributionPumpDryRun = true;
        Serial.println("Distribution Pump Dry Run Cutoff Triggered");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("DP Dry Run Cutoff");
        display.display();
    }
}

void stopUVLight() {
    digitalWrite(UV_LIGHT_RELAY, LOW);
    UVlightActive = false;
    Serial.println("UV Light Stopped");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("UV Light Off");
    display.display();
 }

void startUVLight() {
  digitalWrite(UV_LIGHT_RELAY, HIGH);
  Serial.println("Starting UV Light");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Starting UV Light");
  display.display();
  millis(uv_light_delay);
  UVlightActive = true;
  Serial.println("UV Light Started");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("UV Light On");
  display.display();
}


void openRecirculationValve() {
    digitalWrite(RECIRCULATION_VALVE_RELAY, HIGH);
    Serial.println("Opening Recirculation Valve...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Opening Recir. Valve...");
    display.display();
    millis(valve_operation_time);
    recirculationValveOpen = true;
    Serial.println("Recirculation Valve Open");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Recirc Valve Open");
    display.display();
}

void closeRecirculationValve() {
    digitalWrite(RECIRCULATION_VALVE_RELAY, LOW);
    Serial.println("Closing Recirculation Valve...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Closing Recir. Valve...");
    display.display();
    millis(valve_operation_time);
    recirculationValveOpen = false;
    Serial.println("Recirculation Valve Closed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Recirc Valve Closed");
    display.display();
}

void openPurgeValve() {
    digitalWrite(PURGE_VALVE_RELAY, HIGH);
    Serial.println("Opening Purge Valve...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Opening Purge. Valve...");
    display.display();
    millis(valve_operation_time);
    purgeValveOpen = true;
    Serial.println("Purge Valve Open");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Purge Valve Open");
    display.display();
}

void closePurgeValve() {
    digitalWrite(PURGE_VALVE_RELAY, LOW);
    Serial.println("Closing Purge Valve...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Closing Purge. Valve...");
    display.display();
    millis(valve_operation_time);
    purgeValveOpen = false;
    Serial.println("Purge Valve Closed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Purge Valve Closed");
    display.display();
}

void stopTransferPump() {
    digitalWrite(TRANSFER_PUMP_RELAY, LOW);
    transferPumpActive = false;
     millis(pump_stop_delay);
    Serial.println("Transfer Pump Stopped");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Transfer Pump Off");
    display.display();
}

void startTransferPump() {
    if(recirculationValveOpen == true){
      closeRecirculationValve() //IF THE RECIRCULATION VALVE IS OPEN, CLOSE IT
    }
  digitalWrite(TRANSFER_PUMP_RELAY, HIGH);
  transferPumpActive = true;
  Serial.println("Transfer Pump Started");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Transfer Pump On");
  display.display();
}

void recirculationMode() {

//verify that the mode is allowed to be operated and exit if not
if(recirculationFunctionDisabled == false){
    //send a system flag and exit
    return;
}


//turn off the transfer pump, if needed, to prevent exceeding flow rate limit
    if(transferPumpActive == true){
       stopTransferPump();
    } else {

        UVlightOn();
        openRecirculationValve();
    
        recirculationModePending = false;
        recirculationModeBlockTransferPump = true;
        recirculationModeActive = true;
        recirculationModeTimeOfRequestedAction = millis();

        Serial.println("Recirculation Mode Started");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Recirc Mode On");
        display.display();
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("TP Active: ");
    display.println(transferPumpActive ? "Yes" : "No");
    display.print("DP Active: ");
    display.println(distributionPumpActive ? "Yes" : "No");
    display.print("Recirc Valve: ");
    display.println(recirculationValveOpen ? "Open" : "Closed");
    display.print("Purge Valve: ");
    display.println(purgeValveOpen ? "Open" : "Closed");
    display.display();
}

void updateIndicators() {
    digitalWrite(TRANSFER_PUMP_ON_INDICATOR, transferPumpActive ? HIGH : LOW);
    digitalWrite(DISTRIBUTION_PUMP_ON_INDICATOR, distributionPumpDryRun ? HIGH : LOW);
    digitalWrite(UV_LAMP_ON_INDICATOR, recirculationModeActive ? HIGH : LOW);
    digitalWrite(NEW_WATER_PROCESS_INDICATOR, recirculationModePending ? HIGH : LOW);
    digitalWrite(ACCUMULATOR_UPPER_FLOAT_INDICATOR, digitalRead(ACCUMULATOR_UPPER_FLOAT));
    digitalWrite(ACCUMULATOR_LOWER_FLOAT_INDICATOR, digitalRead(ACCUMULATOR_LOWER_FLOAT));
    digitalWrite(STORAGE_UPPER_FLOAT_INDICATOR, digitalRead(STORAGE_UPPER_FLOAT));
    digitalWrite(STORAGE_LOWER_FLOAT_INDICATOR, digitalRead(STORAGE_LOWER_FLOAT));
    digitalWrite(DISTRIBUTION_PRESSURE_LOW_INDICATOR, digitalRead(DISTRIBUTION_PRESSURE_SWITCH));
    digitalWrite(MASTER_CAUTION_INDICATOR, (transferPumpDryRun || distributionPumpDryRun) ? HIGH : LOW);

    Serial.println("Indicators Updated");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Indicators Updated");
    display.display();
}


void loop() {

    // Read control states
    readControlStates();
    determineControlMode(); // Determine and print the current control state mode
    
    // Call to check float sensors
    checkFloatSensors();

    // This needs work
    if (recirculationModeActive && (millis() - recirculationModeTimeOfRequestedAction > RECIRCULATION_MODE_TIMEOUT)) {
        closeRecirculationValve(); // Close valve if timeout reached
        recirculationModeActive = false;
        Serial.println("Recirculation Mode Timeout");
    }

    // Regularly update the user interface output
    updateDisplay();
    updateIndicators();


}
