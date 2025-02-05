// Pin Assignments
#define MANUAL_STOP_RECIRCULATION D2
#define DISTRIBUTION_PRESSURE_SWITCH D3
#define MANUAL_PRIME_DISTRIBUTION_PUMP D4
#define MANUAL_PRIME_TRANSFER_PUMP D12
#define TRANSFER_PUMP_MODE_NORMAL D5
#define TRANSFER_PUMP_MODE_MANUAL D6
#define DISTRIBUTION_PUMP_MODE_NORMAL D7
#define DISTRIBUTION_PUMP_MODE_MANUAL D8
#define RECIRCULATION_FUNCTION_ENABLE D9
#define SEDIMENT_PURGE_FUNCTION_NORMAL D10
#define SEDIMENT_PURGE_FUNCTION_MANUAL D11

#define ACCUMULATOR_UPPER_FLOAT A0
#define ACCUMULATOR_LOWER_FLOAT A1
#define STORAGE_UPPER_FLOAT A2
#define STORAGE_LOWER_FLOAT A3

#define TRANSFER_PUMP_RELAY D22
#define PWM_RELAY D23
#define DISTRIBUTION_PUMP_RELAY D24
#define RECIRCULATION_MODE_RELAY D25
#define RECIRCULATION_VALVE_RELAY D26
#define PURGE_VALVE_RELAY D27
#define UV_LIGHT_RELAY D28

// Constants

#define RECIRCULATION_MODE_TIMEOUT 1380000 // 23 minutes
#define valve_operation_time 6000 //6 seconds
#define pump_stop_delay 500 // 0.5 second
#define uv_light_delay 2500 // 2.5 second

// Global State Variables

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


void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 allocation failed");
        while (1);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("System Initializing...");
    display.display();
    delay(1000);

    pinMode(MANUAL_STOP_RECIRCULATION, INPUT_PULLUP);
    pinMode(DISTRIBUTION_PRESSURE_SWITCH, INPUT_PULLUP);
    pinMode(MANUAL_PRIME_DISTRIBUTION_PUMP, INPUT_PULLUP);
    pinMode(MANUAL_PRIME_TRANSFER_PUMP, INPUT_PULLUP);
    pinMode(ACCUMULATOR_UPPER_FLOAT, INPUT_PULLUP);
    pinMode(ACCUMULATOR_LOWER_FLOAT, INPUT_PULLUP);
    pinMode(STORAGE_UPPER_FLOAT, INPUT_PULLUP);
    pinMode(STORAGE_LOWER_FLOAT, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(MANUAL_STOP_RECIRCULATION), exitRecirculationManual, FALLING);
    attachInterrupt(digitalPinToInterrupt(DISTRIBUTION_PRESSURE_SWITCH), exitRecirculationAuto, FALLING);

    Serial.println("Interrupts Configured:");
    Serial.println("- Manual Stop Recirculation");
    Serial.println("- Distribution Pressure Switch");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Interrupts Config");
    display.display();

    pinMode(TRANSFER_PUMP_RELAY, OUTPUT);
    pinMode(PWM_RELAY, OUTPUT);
    pinMode(DISTRIBUTION_PUMP_RELAY, OUTPUT);
    pinMode(RECIRCULATION_MODE_RELAY, OUTPUT);
    pinMode(RECIRCULATION_VALVE_RELAY, OUTPUT);
    pinMode(PURGE_VALVE_RELAY, OUTPUT);
    pinMode(UV_LIGHT_RELAY, OUTPUT);

    for (int i = TRANSFER_PUMP_RELAY; i <= UV_LIGHT_RELAY; i++) {
        digitalWrite(i, LOW);
    }
    
    Serial.println("System Ready");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("System Ready");
    display.display();

} /* ______________________________________________________________ End of Setup() ___________________________________________________________________ */

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

void dryRunCutoffTransferPump() {
    if (digitalRead(ACCUMULATOR_LOWER_FLOAT) == HIGH) {
        digitalWrite(TRANSFER_PUMP_RELAY, LOW);
         wait(pump_stop_delay);
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
         wait(pump_stop_delay);
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
  wait(uv_light_delay);
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
    wait(valve_operation_time);
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
    wait(valve_operation_time);
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
    wait(valve_operation_time);
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
    wait(valve_operation_time);
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
     wait(pump_stop_delay);
    Serial.println("Transfer Pump Stopped");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Transfer Pump Off");
    display.display();
    wait(pump_stop_delay);
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
    if (transferPumpActive) {
        recirculationModePending = true;
        return;
    }

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

void loop() {

    // Read control states
    readControlStates();
    
    // Call to check float sensors
    checkFloatSensors();

    // Additional functionality
    if (recirculationModeActive && (millis() - recirculationModeTimeOfRequestedAction > RECIRCULATION_MODE_TIMEOUT)) {
        closeRecirculationValve(); // Close valve if timeout reached
        recirculationModeActive = false;
        Serial.println("Recirculation Mode Timeout");
    }

    // Regularly update display with current state
    updateDisplay();
}
