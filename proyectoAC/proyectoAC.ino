#include "StateMachineLib.h"
#include "AsyncTaskLib.h"

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
  

  // Add leavings
}

void setup() {
  Serial.begin(9600);

  setupStateMachine();
  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM Iniciada");
}

void loop() {
  
}

// Auxiliar output functions that show the state debug
void outputInicio() {
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("  X                                                               ");
  Serial.println();
}

void outputConfig() {
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
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                              X   ");
  Serial.println();
}

void outputMonitor() {
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                     X                                            ");
  Serial.println();
}


void outputPMV_Bajo() {
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                        X                         ");
  Serial.println();
}

void outputPMV_Alto() {
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                        X                         ");
  Serial.println();
}