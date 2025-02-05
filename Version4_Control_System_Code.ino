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

// Global State Variables

volatile bool recirculationValveOpen = false;
volatile bool purgeValveOpen = false;
volatile bool recirculationActive = false;
volatile bool manualExitTriggered = false;
volatile bool autoExitTriggered = false;
volatile bool transferPumpDryRun = false;
volatile bool distributionPumpDryRun = false;
volatile bool transferPumpActive = false;

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

}

void dryRunCutoffTransferPump() {
    if (digitalRead(ACCUMULATOR_LOWER_FLOAT) == HIGH) {
        digitalWrite(TRANSFER_PUMP_RELAY, LOW);
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
        distributionPumpDryRun = true;
        Serial.println("Distribution Pump Dry Run Cutoff Triggered");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("DP Dry Run Cutoff");
        display.display();
    }
}

void openRecirculationValve() {
    digitalWrite(RECIRCULATION_VALVE_RELAY, HIGH);
    recirculationValveOpen = true;
    Serial.println("Recirculation Valve Open");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Recirc Valve Open");
    display.display();
}

void closeRecirculationValve() {
    digitalWrite(RECIRCULATION_VALVE_RELAY, LOW);
    recirculationValveOpen = false;
    Serial.println("Recirculation Valve Close");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Recirc Valve Close");
    display.display();
}

void openPurgeValve() {
    digitalWrite(PURGE_VALVE_RELAY, HIGH);
    purgeValveOpen = true;
    Serial.println("Purge Valve Open");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Purge Valve Open");
    display.display();
}

void closePurgeValve() {
    digitalWrite(PURGE_VALVE_RELAY, LOW);
    purgeValveOpen = false;
    Serial.println("Purge Valve Close");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Purge Valve Close");
    display.display();
}

void stopTransferPump() {
    digitalWrite(TRANSFER_PUMP_RELAY, LOW);
    transferPumpActive = false;
    Serial.println("Transfer Pump Stopped");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Transfer Pump Off");
    display.display();
}

void startTransferPump() {
    if(recirculationValveOpen == true){
      closeRecirculationValve() //IF THE RECIRCULATION VALVE IS OPEN, SEND CLOSE SIGNAL AND WAIT 6 SECONDS
      wait(6000);
    }
  digitalWrite(TRANSFER_PUMP_RELAY, HIGH);
  transferPumpActive = true;
  Serial.println("Transfer Pump Started");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Transfer Pump On");
  display.display();
}

