#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include "DHT.h"
#include <math.h>
#include <Servo.h>

// RGB led
#define LED_RED 9
#define LED_GREEN 10
#define LED_BLUE 11
// DHT
#define DHTPIN 3
#define DHTTYPE DHT11
// Sensors
#define LDR_PIN A3
#define TEMP_PIN A0
const float BETA = 3950;
// Servomotor
#define SERVO_PIN 13
// Ventilador


// Global variables to calculate PMV and to get through the sensors
float M = 0.0;
float clo = 0.5;
float vel_ar = 0.1;
float tempA = 0.0;
float tempR = 0.0;
float humedad = 0.0;
float luz = 0.0;

// LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// DHT
DHT dht(DHTPIN, DHTTYPE);

// KEYPAD Definition
// Password for Keypad
const char clave[6] = { '2', '0', '2', '5', '2', 'A' };
char clave_user[6];

const byte ROWS = 4;  // cuatro filas
const byte COLS = 4;  // cuatro columnas
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 40, 42, 44, 46 };  // conecta a los pines de las filas del teclado
byte colPins[COLS] = { 48, 50, 52, 53 };  // conecta a los pines de las columnas del teclado

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Servo
Servo servo;
int pos = 0;

// STATE MACHINE
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
  SISTEMA_BLOQUEADO = 0,     // Se activa cuando se detecta condición de bloqueo (por ejemplo, intentos fallidos)
  TECLA_ASTERISCO = 1,       // Usuario presiona tecla '*'
  CLAVE_CORRECTA = 2,        // Clave válida ingresada (autenticación correcta)
  TIEMPO_EXPIRADO = 3,       // Se cumplió el tiempo definido para el estado actual
  PMV_MENOR_QUE_MENOS1 = 4,  // Resultado de cálculo PMV < -1
  PMV_MAYOR_QUE_1 = 5,       // Resultado de cálculo PMV > 1
  TEMP_ALTA_3_INTENTOS = 6,  // Temperatura > 30 después de 3 intentos consecutivos
  SENSOR_INFRARROJO = 7,     // Detección de movimiento (para salir de alarma)
  Unknown = 8
};


// Create new StateMachine
StateMachine stateMachine(7, 11);
// Stores last user input
Input input = Unknown;

// --- Async tasks ---
// Time Tasks
void runTime();
AsyncTask TaskTime(5000, true, runTime);  // cada 5 segundos lanza TIEMPO_EXPIRADO
void runTime() {
  input = TIEMPO_EXPIRADO;
}

// LED toggles ---------------------------------------------------------------------------------
// Red ----
void toggleRed();
AsyncTask TaskLedRed(100, true, toggleRed);  // periodo de 100 ms ON, manejaremos el OFF por ciclo

bool ledStateRed = false;
unsigned long lastToggleRed = 0;

// Green ----
void toggleGreen();
AsyncTask TaskLedGreen(200, true, toggleGreen);  // periodo de 200 ms ON, manejaremos el OFF por ciclo

bool ledStateGreen = false;
unsigned long lastToggleGreen = 0;

// Blue ----
void toggleBlue();
AsyncTask TaskLedBlue(300, true, toggleBlue);  // periodo de 300 ms ON, manejaremos el OFF por ciclo

bool ledStateBlue = false;
unsigned long lastToggleBlue = 0;

// Setup the State Machine
void setupStateMachine() {
  // Add transitions Inicio
  stateMachine.AddTransition(INICIO, CONFIG, []() { return input == CLAVE_CORRECTA;});
  stateMachine.AddTransition(INICIO, BLOQUEO, []() { return input == SISTEMA_BLOQUEADO;});

  // Add transitions Bloqueo
  stateMachine.AddTransition(BLOQUEO, INICIO, []() { return input == TECLA_ASTERISCO;});

  // Add transitions Config
  stateMachine.AddTransition(CONFIG, MONITOR, []() { return input == TIEMPO_EXPIRADO;});

  // Add transitions Monitor
  stateMachine.AddTransition(MONITOR, PMV_BAJO, []() { return input == PMV_MENOR_QUE_MENOS1;});
  stateMachine.AddTransition(MONITOR, PMV_ALTO, []() { return input == PMV_MAYOR_QUE_1;});
  stateMachine.AddTransition(MONITOR, CONFIG, []() { return input == TIEMPO_EXPIRADO;});

  // Add transitions PMV
  stateMachine.AddTransition(PMV_BAJO, MONITOR, []() { return input == TIEMPO_EXPIRADO;});
  stateMachine.AddTransition(PMV_ALTO, MONITOR, []() { return input == TIEMPO_EXPIRADO;});
  stateMachine.AddTransition(PMV_ALTO, ALARMA, []() { return input == TEMP_ALTA_3_INTENTOS;});

  // Add transitions Alarma
  stateMachine.AddTransition(ALARMA, INICIO, []() { return input == SENSOR_INFRARROJO;});


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
  servo.attach(SERVO_PIN);

  lcd.setCursor(0, 0);

  setupStateMachine();
  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM Iniciada");
  lcd.println("Iniciando...");
}

void loop() {
  // Leer input usuario (por serial)
  if (input == Unknown) {
    input = static_cast<Input>(readInputSerial());
  }

  // Actualizar tareas (timers)
  TaskTime.Update();
  // Actualizar leds (toggles)
  TaskLedRed.Update();
  TaskLedGreen.Update();
  TaskLedBlue.Update();

  // Read "*" when we are in BLOQUEO
  checkBloqueo();
  // Actualizar máquina de estados
  stateMachine.Update();
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
    TaskTime.SetIntervalMillis(5000);
    TaskTime.Start();
    input = CLAVE_CORRECTA;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Clave incorrecta");
    lcd.setCursor(0, 1);
    lcd.print("-> BLOQUEO");
    TaskTime.SetIntervalMillis(5000);
    TaskTime.Start();
    input = SISTEMA_BLOQUEADO;
  }

  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("  X                                                               ");
  Serial.println();
}

void outputConfig() {
  TaskTime.SetIntervalMillis(5000);  // 5 Segs hasta monitor si no hay entrada
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

  ledStateRed = false;
  digitalWrite(LED_RED, LOW);
  lastToggleRed = millis();
  TaskLedRed.Start();
}

void outputMonitor() {
  leerSensores();
  // Actividad ligera (persona de pie o caminando lentamente)
  M = 100;
  // Ropa ligera (camisa, pantalón)
  clo = 0.5;
  // Velocidad de aire típica en interior
  vel_ar = 0.1;
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
  else if (pmv > 1) 
    input = PMV_MAYOR_QUE_1;
  else
    input = TIEMPO_EXPIRADO;

  TaskTime.SetIntervalMillis(7000);  // 7 segs hasta config si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                     X                                            ");
  Serial.println();
}

void outputPMV_Bajo() {
  TaskTime.SetIntervalMillis(3000);  // 3 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                        X                         ");
  Serial.println();

  ledStateGreen = false;
  digitalWrite(LED_GREEN, LOW);
  lastToggleGreen = millis();
  TaskLedGreen.Start();

  girarServo(pos);
}

void outputPMV_Alto() {
  TaskTime.SetIntervalMillis(4000);  // 4 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                    X             ");
  Serial.println();

  ledStateBlue = false;
  digitalWrite(LED_BLUE, LOW);
  lastToggleBlue = millis();
  TaskLedBlue.Start();
}

// Functions para leaving -----------------------------------------------
void onLeavingBloqueo() {
  TaskLedRed.Stop();
  digitalWrite(LED_RED, LOW);
  Serial.println("Saliendo de BLOQUEO, LED apagado");
}

// AUXILIAR FUNCTIONS ----------------------------------------------------------
// Auxiliar function that reads the user input from Serial
int readInputSerial() {
  Input currentInput = Input::Unknown;
  if (Serial.available()) {
    char incomingChar = Serial.read();
    switch (incomingChar) {
      case '0': currentInput = Input::SISTEMA_BLOQUEADO; break;
      case '1': currentInput = Input::TECLA_ASTERISCO; break;
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

// LED toggles ---------------------------------------------------------------------------------
// Red ----
void toggleRed() {
  unsigned long now = millis();

  // Si el LED está encendido y han pasado 100 ms → apagar
  if (ledStateRed && (now - lastToggleRed >= 100)) {
    ledStateRed = false;
    digitalWrite(LED_RED, LOW);
    lastToggleRed = now;
    TaskLedRed.SetIntervalMillis(300);  // siguiente ciclo: 300 ms apagado
  }
  // Si el LED está apagado y han pasado 300 ms → encender
  else if (!ledStateRed && (now - lastToggleRed >= 300)) {
    ledStateRed = true;
    digitalWrite(LED_RED, HIGH);
    lastToggleRed = now;
    TaskLedRed.SetIntervalMillis(100);  // siguiente ciclo: 100 ms encendido
  }
}
// Green ----
void toggleGreen() {
  unsigned long now = millis();

  // Si el LED está encendido y han pasado 200 ms → apagar
  if (ledStateGreen && (now - lastToggleGreen >= 200)) {
    ledStateGreen = false;
    digitalWrite(LED_GREEN, LOW);
    lastToggleGreen = now;
    TaskLedGreen.SetIntervalMillis(300);  // siguiente ciclo: 300 ms apagado
  }
  // Si el LED está apagado y han pasado 300 ms → encender
  else if (!ledStateGreen && (now - lastToggleGreen >= 300)) {
    ledStateGreen = true;
    digitalWrite(LED_GREEN, HIGH);
    lastToggleGreen = now;
    TaskLedGreen.SetIntervalMillis(200);  // siguiente ciclo: 200 ms encendido
  }
}
// Blue ----
void toggleBlue() {
  unsigned long now = millis();

  // Si el LED está encendido y han pasado 300 ms → apagar
  if (ledStateBlue && (now - lastToggleBlue >= 300)) {
    ledStateBlue = false;
    digitalWrite(LED_BLUE, LOW);
    lastToggleBlue = now;
    TaskLedBlue.SetIntervalMillis(400);  // siguiente ciclo: 400 ms apagado
  }
  // Si el LED está apagado y han pasado 400 ms → encender
  else if (!ledStateBlue&& (now - lastToggleBlue >= 400)) {
    ledStateBlue = true;
    digitalWrite(LED_BLUE, HIGH);
    lastToggleBlue = now;
    TaskLedBlue.SetIntervalMillis(300);  // siguiente ciclo: 300 ms encendido
  }
}

// K E Y P A D
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

    TaskTime.SetIntervalMillis(300);  // Pequeña pausa entre teclas
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
  tempA = dht.readTemperature();  // °C
  humedad = dht.readHumidity();   // %
  int rawLuz = analogRead(LDR_PIN);
  luz = map(rawLuz, 0, 1032, 0, 100);  // % aproximado de iluminación
  int analogValue = analogRead(TEMP_PIN);
  tempR = 1 / (log(1 / (1023. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15; // °C        

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

// Función auxiliar: calcular hc (coeficiente de convección)
// Parámetros:
//   tcl: temperatura de superficie de la ropa [°C]
//   ta: temperatura del aire [°C]
//   var: velocidad relativa del aire [m/s]
// Retorna: hc en W/(m²·K)
float calcularHc(float tcl, float ta, float vel_ar) {
  // Cálculo de convección natural: depende de la diferencia de temperatura
  // hc_nat = 2.38 * |tcl - ta|^0.25
  float hc_natural = 2.38 * pow(fabs(tcl - ta), 0.25);

  // Cálculo de convección forzada: depende de la velocidad del aire
  // hc_for = 12.1 * √(var)
  float hc_forzada = 12.1 * sqrt(vel_ar);

  // Seleccionar el MAYOR de los dos (más dominante)
  // El aire en movimiento rápido supera la convección natural
  float hc;
  if (hc_natural > hc_forzada) {
    hc = hc_natural;  // Domina convección natural (aire quieto)
  } else {
    hc = hc_forzada;  // Domina convección forzada (hay movimiento de aire)
  }

  return hc;
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

// CALCULAR PMV (Modelo Fanger adaptado a Arduino)
float calcularPMV_Fanger(float ta, float tr, float rh, float vel_ar, float M, float clo) {
	// Conversión de unidades y constantes base
	float pa, icl, V, fcl, hc, pmv;
	float tol = 0.0001;  // Tolerancia para iteraciones
	float tcl = ta;      // Temperatura inicial de la superficie de la ropa
	float tcl_prev;
	int iteracion = 0;
	int max_iters = 100;
	
	// Constantes
	V = 0.0;                            // Trabajo mecánico (reposo)
	icl = clo * 0.155;                  // Aislamiento térmico (m²K/W)
	pa = calcularPresionVapor(ta, rh);  // Presión de vapor (Pa)
	
	// Factor de superficie de la ropa
	fcl = calcularFcl(icl);
	
	// ------------Iteración para hallar tcl---------------
	do {
		tcl_prev = tcl;
		
		// Calcular hc
		hc = calcularHc(tcl, ta, vel_ar);
		
		// Convertidr a grados Kelvin para la fórmula
		float tr_rad = tr + 273.0;
		float tcl_rad = tcl + 273.0;
		
		// Calcular la radiación térmica de onda larga (transferencia de calor radiante)
		float radiation = (3.96 * pow(10, -8)) * fcl * (pow(tcl_rad, 4) - pow(tr_rad, 4));
		// Calcular la transferencia de calor por convección (aire calentando/enfriando la ropa)
		float convection = fcl * hc * (tcl - ta);
		
		tcl = 35.7 - 0.028 * (M - V) - icl * (radiation - convection);
		
		iteracion++;
	} while (fabs(tcl - tcl_prev) > tol && iteracion < max_iters);
	
	// Cálculo de hcl final
	float hc_final;
	hc_final = calcularHc(tcl, ta, vel_ar);
	
	// --------------CÁLCULO DEL PMV------------------
	// Convertidr a grados Kelvin para la fórmula
	float tr_rad = tr + 273.0;
	float tcl_rad = tcl + 273.0;
	
	// - - -Calcular las formas de perder calor- - -
	// Calcular la radiación térmica de onda larga (transferencia de calor radiante)
	float radiation = (3.96e-8) * fcl * (pow(tcl_rad, 4) - pow(tr_rad, 4));
	// Calcular la transferencia de calor por convección (aire calentando/enfriando la ropa)
	float convection = fcl * hc_final * (tcl - ta);
	// Calcular la pérdida de calor por respiración
	float respiration = 0.0014 * M * (34.0 - ta);
	// Calcular la pérdida de calor por transpiración
	float latent = (1.7e-5) * M * (5867.0 - pa);
	// Pérdida de calor total
	float heat_loss = latent + respiration + radiation + convection;
	
	pmv = 	(0.303 * exp(-0.036 * M) + 0.028) * (
			(M - V)
			- (3.05e-3) * (5733.0 - 6.99 * (M - V) - pa)
			- 0.42 * ((M - V) - 58.15)
			- heat_loss
			);
	
  return constrain(pmv, -3.0, 3.0); // limitar rango típico del índice PMV
}

// Move Servo
void girarServo(int pos) {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  // in steps of 1 degree
  servo.write(pos); // tell servo to go to position in variable 'pos'
  TaskTime.SetIntervalMillis(15); // waits 15ms for the servo to reach the position
  TaskTime.Start();
  }

  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  servo.write(pos); // tell servo to go to position in variable 'pos'
  TaskTime.SetIntervalMillis(15); // waits 15ms for the servo to reach the position
  TaskTime.Start();
  }
}