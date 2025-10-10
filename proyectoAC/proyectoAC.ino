#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include "DHT.h"
#include <math.h>

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

// Global variables
float M = 0.0;
float clo = 0.5;
float vel_ar = 0.1;
float tempA = 0.0;
float tempR = 0.0;
float humedad = 0.0;
float luz = 0.0;

// Password for Keypad
const char clave[6] = {'2', '0', '2', '5', '2', 'A'};
char clave_user[6];

// LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// DHT
DHT dht(DHTPIN, DHTTYPE);

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
// Read password from KEYPAD
void readPassword() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ingrese clave:");
  
  for (int i = 0; i < 6; i++) {
    lcd.setCursor(i, 1);  // Mostrar posición actual
    lcd.print("_");
    
    char key = NO_KEY;
    while (key == NO_KEY) {
      key = keypad.getKey();
    }
    
    clave_user[i] = key;
    lcd.setCursor(i, 1);
    lcd.print("*");  // Mostrar asterisco en lugar del carácter real
    
    TaskTime.SetIntervalMillis(300); // Pequeña pausa entre teclas
    TaskTime.Start();
  }
}

// Check password
bool checkPassword() {
  for (int i = 0; i < 6; i++) {
    if (clave_user[i] != clave[i]) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Clave Incorrecta");
      return false;
    }
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Clave Correcta!");
  return true;
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
  lcd.print("Estado: INICIO");
  lcd.setCursor(0, 1);
  lcd.print("Ingrese clave...");
  
  // Leer la clave
  readPassword();
  
  // Verificar la clave
  if (checkPassword()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Clave correcta");
    lcd.setCursor(0, 1);
    lcd.print("-> CONFIG");
    TaskTime.SetIntervalMillis(5000); // 5 Segs hasta monitor si no hay entrada
    TaskTime.Start(); 
    input = CLAVE_CORRECTA;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Clave incorrecta");
    lcd.setCursor(0, 1);
    lcd.print("-> BLOQUEO");
    TaskTime.SetIntervalMillis(5000); // 5 Segs hasta monitor si no hay entrada
    TaskTime.Start(); 
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
  leerSensores();
  // Actividad ligera (persona de pie o caminando lentamente)
  float M = 100;
  // Ropa ligera (camisa, pantalón)
  float clo = 0.5;
  // Velocidad de aire típica en interior
  float vel_ar = 0.1;
  // Calcular PMV
  float pmv = calcularPMV_Fanger(tempA, tempR, humedad, vel_ar, M, clo);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PMV: ");
  lcd.print(pmv, 2);
  Serial.print("PMV calculado: ");
  Serial.println(pmv, 2);

  if (pmv < -1) 
    input = PMV_MENOR_QUE_MENOS1;
  else if 
    (pmv > 1) input = PMV_MAYOR_QUE_1;
  else 
    input = TIEMPO_EXPIRADO;

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

  if (isnan(humedad) || isnan(tempA)) {
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

// Calculating PMV (Modelo Fanger adaptado a Arduino)
// Función auxiliar: calcular presión parcial de vapor
float calcularPresionVapor(float ta, float rh) {
  // pa = rh/100 * exp(16.6536 - 4030.183/(ta + 235))
  float pa = (rh / 100.0) * exp(16.6536 - 4030.183 / (ta + 235.0));
  return pa;
}

// Función auxiliar: calcular hcl (coeficiente de convección)
// Parámetros:
//   tcl: temperatura de superficie de la ropa [°C]
//   ta: temperatura del aire [°C]
//   var: velocidad relativa del aire [m/s]
// Retorna: hcl en W/(m²·K)
float calcularHcl(float tcl, float ta, float vel_ar) {
  // Cálculo de convección natural: depende de la diferencia de temperatura
  // hc_nat = 2.38 * |tcl - ta|^0.25
  float hc_natural = 2.38 * pow(fabs(tcl - ta), 0.25);
  
  // Cálculo de convección forzada: depende de la velocidad del aire
  // hc_for = 12.1 * √(var)
  float hc_forzada = 12.1 * sqrt(vel_ar);
  
  // Seleccionar el MAYOR de los dos (más dominante)
  // El aire en movimiento rápido supera la convección natural
  float hcl;
  if (hc_natural > hc_forzada) {
    hcl = hc_natural;  // Domina convección natural (aire quieto)
  } else {
    hcl = hc_forzada;  // Domina convección forzada (hay movimiento de aire)
  }
  
  return hcl;
}

// Función auxiliar: calcular fcl (factor de superficie)
float calcularFcl(float icl) {
  float fcl;
  
  if (icl <= 0.078) {
    fcl = 1.00 + 1.290 * icl;
  } else {
    fcl = 1.05 + 0.645 * icl;
  }
  return fcl;
}

// ============================================================
// CALCULAR PMV (Modelo Fanger adaptado a Arduino)
// ============================================================
float calcularPMV_Fanger(float ta, float tr, float rh, float vel, float met, float clo) {
  // Conversión de unidades y constantes base
  float pa, icl, m, w, fcl, hc, tcl, hl1, hl2, hl3, hl4, hl5, hl6, ts, pmv;
  float eps = 0.00015;

  // Constantes
  m = met * 58.15;  // Met -> W/m²
  w = 0.0;          // Trabajo mecánico (reposo)
  icl = clo * 0.155; // Aislamiento térmico (m²K/W)
  pa = calcularPresionVapor(tempA, humedad); // Presión de vapor (Pa)

  // Factor de área de la ropa
  fcl = calcularFcl(icl);

  // Temperatura inicial de la superficie de la ropa
  float tcla = ta + (35.5 - ta) / (3.5 * icl + 0.1);

  // Iteración para hallar tcl
  int iter = 0;
  float p1 = icl * fcl;
  float p2 = icl * 3.96 * pow(10, -8) * fcl;
  float p3 = icl * fcl * 100.0;
  float p4 = icl * fcl * 3.96 * pow(10, -8);
  float tcl1 = tcla;
  float tcl2 = tcla;
  do {
    tcl1 = tcl2;
    hc = 2.38 * pow(fabs(tcl1 - ta), 0.25);
    if (hc < 12.1 * sqrt(vel)) hc = 12.1 * sqrt(vel);
    tcl2 = (35.7 - 0.028 * (m - w) - icl * (fcl * (3.96 * pow(10, -8) * (pow(tcl1 + 273.0, 4) - pow(tr + 273.0, 4)) + fcl * hc * (tcl1 - ta)))) / (1 + icl * fcl * (3.96 * pow(10, -8) * 4 * pow(tcl1 + 273.0, 3) + fcl * hc));
    iter++;
  } while (fabs(tcl1 - tcl2) > eps && iter < 150);
  tcl = tcl2;

  // Cálculo de las pérdidas de calor
  hl1 = 3.05 * 0.001 * (5733 - 6.99 * (m - w) - pa);
  hl2 = 0.42 * (m - w - 58.15);
  hl3 = 1.7 * 0.00001 * m * (5867 - pa);
  hl4 = 0.0014 * m * (34 - ta);
  hl5 = 3.96 * 0.00000001 * fcl * (pow(tcl + 273.0, 4) - pow(tr + 273.0, 4));
  hl6 = fcl * hc * (tcl - ta);

  // Ecuación principal del PMV
  pmv = (0.303 * exp(-0.036 * m) + 0.028) * ((m - w) - hl1 - hl2 - hl3 - hl4 - hl5 - hl6);

  return constrain(pmv, -3.0, 3.0); // limitar rango típico del índice PMV
}