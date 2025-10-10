#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include "DHT.h"

// RGB led
#define LED_RED   9
#define LED_GREEN 10
#define LED_BLUE  11
// DHT
#define DHTPIN 3
#define DHTTYPE DHT11
// Sensors
#define LDR_PIN A3
#define TEMP_PIN A0

// Password for Keypad
const char clave[6] = {'2', '0', '2', '5', '2', 'A'};
char clave_user[6];

// Global variables
float tempA = 0.0;
float humedad = 0.0;
float luz = 0.0;
float PMV = 0.0;
float tempR = 0.0;

// LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// DHT
DHT dht(DHTPIN, DHTTYPTE);

// KEYPAD Definition
const byte ROWS = 4; // cuatro filas
const byte COLS = 4; // cuatro columnas
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {40, 42, 44, 46}; // conecta a los pines de las filas del teclado
byte colPins[COLS] = {48, 50, 52, 53}; // conecta a los pines de las columnas del teclado

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// State Alias
enum State {
  INICIO = 0,
  CONFIG = 1,
  ALARMA = 2,
  MONITOR = 3,
  BLOQUEO = 4,
  PMV_ALTO = 5,
  PMV_BAJO = 6,
};

// Input Alias
enum Input {
  SISTEMA_BLOQUEADO = 0,      // Se activa cuando se detecta condición de bloqueo (por ejemplo, intentos fallidos)
  TECLA_ASTERISCO = 1,        // Usuario presiona tecla '*'
  CLAVE_CORRECTA = 2,         // Clave válida ingresada (autenticación correcta)
  TIEMPO_EXPIRADO = 3,        // Se cumplió el tiempo definido para el estado actual
  PMV_MENOR_QUE_MENOS1 = 4,   // Resultado de cálculo PMV < -1
  PMV_MAYOR_QUE_1 = 5,        // Resultado de cálculo PMV > 1
  TEMP_ALTA_3_INTENTOS = 6,   // Temperatura > 30 después de 3 intentos consecutivos
  SENSOR_INFRARROJO = 7,       // Detección de movimiento (para salir de alarma)
  Unknown = 8
};


// Create new StateMachine
StateMachine stateMachine(7, 11);
// Stores last user input
Input input = Unknown;

// --- Async tasks ---
// Time Tasks
void runTime();
AsyncTask TaskTime(5000, true, runTime); // cada 5 segundos lanza TIEMPO_EXPIRADO
void runTime() {
  input = TIEMPO_EXPIRADO;
}

// LED toggles ---------------------------------------------------------------------------------
// Red ----
void toggleRed();
AsyncTask TaskLedRed(100, true, toggleRed);  // periodo de 100 ms ON, manejaremos el OFF por ciclo

bool ledState = false;
unsigned long lastToggle = 0;

void toggleRed() {
  unsigned long now = millis();
  
  // Si el LED está encendido y han pasado 100 ms → apagar
  if (ledState && (now - lastToggle >= 100)) {
    ledState = false;
    digitalWrite(LED_RED, LOW);
    lastToggle = now;
    TaskLedRed.SetIntervalMillis(300);  // siguiente ciclo: 300 ms apagado
  }
  // Si el LED está apagado y han pasado 300 ms → encender
  else if (!ledState && (now - lastToggle >= 300)) {
    ledState = true;
    digitalWrite(LED_RED, HIGH);
    lastToggle = now;
    TaskLedRed.SetIntervalMillis(100);  // siguiente ciclo: 100 ms encendido
  }
}

// Setup the State Machine
void setupStateMachine() {
  // Add transitions Inicio
  stateMachine.AddTransition(INICIO, CONFIG, []() { return input == CLAVE_CORRECTA; });
  stateMachine.AddTransition(INICIO, BLOQUEO, []() { return input == SISTEMA_BLOQUEADO; });

  // Add transitions Bloqueo
  stateMachine.AddTransition(BLOQUEO, INICIO, []() { return input == TECLA_ASTERISCO; });

  // Add transitions Config
  stateMachine.AddTransition(CONFIG, MONITOR, []() { return input == TIEMPO_EXPIRADO; }); 

  // Add transitions Monitor
  stateMachine.AddTransition(MONITOR, PMV_BAJO, []() { return input == PMV_MENOR_QUE_MENOS1; }); 
  stateMachine.AddTransition(MONITOR, PMV_ALTO, []() { return input == PMV_MAYOR_QUE_1; }); 
  stateMachine.AddTransition(MONITOR, CONFIG, []() { return input == TIEMPO_EXPIRADO; }); 

  // Add transitions PMV
  stateMachine.AddTransition(PMV_BAJO, MONITOR, []() { return input == TIEMPO_EXPIRADO; });
  stateMachine.AddTransition(PMV_ALTO, MONITOR, []() { return input == TIEMPO_EXPIRADO; });
  stateMachine.AddTransition(PMV_ALTO, ALARMA, []() { return input == TEMP_ALTA_3_INTENTOS; });

  // Add transitions Alarma
  stateMachine.AddTransition(ALARMA, INICIO, []() { return input == SENSOR_INFRARROJO; }); 
  

  // Add enterings
  stateMachine.SetOnEntering(INICIO, outputInicio);
  stateMachine.SetOnEntering(BLOQUEO, outputBloqueo);
  stateMachine.SetOnEntering(CONFIG, outputConfig);
  stateMachine.SetOnEntering(MONITOR, outputMonitor);
  stateMachine.SetOnEntering(PMV_BAJO, outputPMV_Bajo);
  stateMachine.SetOnEntering(PMV_ALTO, outputPMV_Alto);
  stateMachine.SetOnEntering(ALARMA, outputAlarma);

  // Add leavings
  stateMachine.SetOnLeaving(BLOQUEO, onLeavingBloqueo);
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();

  lcd.setCursor(0, 0);

  setupStateMachine();
  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM Iniciada");
  lcd.println("Iniciando...");
}

void loop() {
  // Leer input usuario (por serial)
  if (input == Unknown) {
    input = static_cast<Input>(readInput());
  }

  // Actualizar tareas (timers)
  TaskTime.Update();
  // Actualizar leds (toggles)
  TaskLedRed.Update();

  // Read "*" when we are in BLOQUEO
  checkBloqueo();
  // Actualizar máquina de estados
  stateMachine.Update();
}

// K E Y P A D
// Read password from KEYPAD
void readPassword() {
  for (int i = 0; i < 6; i++) {
    char key = NO_KEY;
    lcd.clear();
    lcd.print("Caracter ");
    lcd.print(i + 1);
    lcd.setCursor(0, 1);
    lcd.print("Ingrese clave...");
    while (key == NO_KEY) {
      key = keypad.getKey();
    }
    clave_user[i] = key;
    TaskTime.SetIntervalMillis(500);
    TaskTime.Start(); 
  }
}

// Check password
bool checkPassword() {
  bool correcta = true;
  for (int i = 0; i < 6; i++) {
    if (clave_user[i] != clave[i]) {
      lcd.print("Clave Incorrecta");
      bool correcta = false;
      return correcta;
    }
  }
  return correcta;
}

// Auxiliar function that reads the user input
int readInput() {
  Input currentInput = Input::Unknown;
  if (Serial.available()) {
    char incomingChar = Serial.read();
    switch (incomingChar) {
      case '0': currentInput = Input::SISTEMA_BLOQUEADO; break;
      case '1': currentInput = Input::TECLA_ASTERISCO;  break;
      case '2': currentInput = Input::CLAVE_CORRECTA; break;
      case '3': currentInput = Input::TIEMPO_EXPIRADO; break;
      case '4': currentInput = Input::PMV_MENOR_QUE_MENOS1; break;
      case '5': currentInput = Input::PMV_MAYOR_QUE_1; break;
      case '6': currentInput = Input::TEMP_ALTA_3_INTENTOS; break;
      case '7': currentInput = Input::SENSOR_INFRARROJO; break;
      default: break;
    }
  }
  return currentInput;
}

// Auxiliar output functions that show the state debug-----------------------------------------
void outputInicio() {
  input = Unknown;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Seguridad");
  readPassword();
  if (checkPassword()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Clave correcta");
    input = CLAVE_CORRECTA;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Clave incorrecta");
    lcd.setCursor(0, 1);
    lcd.print("Sistema Bloqueado");
    input = SISTEMA_BLOQUEADO;
  }
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("  X                                                               ");
  Serial.println();
}

void outputConfig() {
  TaskTime.SetIntervalMillis(5000); // 5 Segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("            X                                                     ");
  Serial.println();
}

void outputAlarma() {
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}

void outputBloqueo() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SISTEMA BLOQUEADO");
  lcd.setCursor(0, 1);
  lcd.print("Presione '*'");


  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                              X   ");
  Serial.println();

  ledState = false;
  digitalWrite(LED_RED, LOW);
  lastToggle = millis();
  TaskLedRed.Start();
}

void outputMonitor() {
  
  TaskTime.SetIntervalMillis(7000); // 7 segs hasta config si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                     X                                            ");
  Serial.println();
}

void outputPMV_Bajo() {
  TaskTime.SetIntervalMillis(3000); // 3 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                        X                         ");
  Serial.println();
}

void outputPMV_Alto() {
  TaskTime.SetIntervalMillis(4000); // 4 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                    X             ");
  Serial.println();
}

// Functions para leaving -----------------------------------------------
void onLeavingBloqueo() {
  TaskLedRed.Stop();
  digitalWrite(LED_RED, LOW);
  Serial.println("Saliendo de BLOQUEO, LED apagado");
}

// AUXILIAR FUNCTIONS ----------------------------------------------------------
// Check * to go back to INICIO
void checkBloqueo() {
  if (stateMachine.GetState() == BLOQUEO) {
      char key = keypad.getKey();
      if (key == '*') {
        input = TECLA_ASTERISCO;
      }
    }
}

// Read sensors
void leerSensores() {
  tempA = dht.readTemperature(); // °C
  humedad = dht.readHumidity(); // %
  int rawLuz = analogRead(LDR_PIN);
  luz = map(rawLuz, 0, 1032, 0, 100); // % aproximado de iluminación
  tempR = analogRead(TEMP_PIN); // °C

  if (isnan(humidity) || isnan(temperture)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("Tem Amb:");
  lcd.print(tempA, 1);  //print the temperature on lcd
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Hum:");
  lcd.print(humedad, 1);  //print the humidity on lcd
  lcd.print(" %");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Luz: ");
  lcd.print(luz, 1);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Tem Rad:");
  lcd.print(tempR, 1);  //print the temperature on lcd
  lcd.print(" C");
}

// Calculating PMV
