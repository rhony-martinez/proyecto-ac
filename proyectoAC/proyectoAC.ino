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

// --- Async tasks ---
void runTime();
AsyncTask TaskTime(5000, true, runTime); // cada 5 segundos lanza TIEMPO_EXPIRADO

void runTime() {
  input = TIEMPO_EXPIRADO;
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
}

void setup() {
  Serial.begin(9600);

  setupStateMachine();
  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM Iniciada");
}

void loop() {
  // Leer input usuario (por serial)
  if (input == Unknown) {
    input = static_cast<Input>(readInput());
  }

  // Actualizar tareas (timers)
  TaskTime.Update();

  // Actualizar máquina de estados
  stateMachine.Update();

  // Reset input para no disparar de nuevo automáticamente
  input = Unknown;
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

// Auxiliar output functions that show the state debug
void outputInicio() {
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
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                              X   ");
  Serial.println();
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
  Serial.println("                                        X                         ");
  Serial.println();
}