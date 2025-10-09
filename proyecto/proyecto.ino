#include "StateMachineLib.h"
#include "AsyncTaskLib.h"

#define LED_RED 9
#define LED_BLUE 11
#define LED_GREEN 10

const int sensorMov = A0;
const int buzzerPin = 7;//the buzzer pin attach to
int fre;//set the variable to store the frequence value

// State Alias
enum State {
  PosicionInicio = 0,
  PosicionConfig = 1,
  PosicionAlarma = 2,
  PosicionMonitor = 3,
  PosicionBloqueo = 4,
  PosicionPMV_Alto = 5,
  PosicionPMV_Bajo = 6,
};

// Input Alias
enum Input {
  SensorInfrarojo = 0,
  Tecla = 1,
  Tiempo = 2,
  DHT11 = 3,
  RFID = 4,
  SensorTemperatura = 5,
  Unknown = 6
};

// Create new StateMachine
StateMachine stateMachine(7, 11);

// Stores last user input
Input input = Unknown;

void runTime(void);
void turnLed(void);
AsyncTask TaskTime(2000, false, runTime);
AsyncTask TaskLed(100, false, turnLed);

void runTime(void) {
  input = Input::Tiempo;
}

void turnLed(void) {
  digitalWrite()
}

// Setup the State Machine
void setupStateMachine() {
  // Add transitions Inicio
  stateMachine.AddTransition(PosicionInicio, PosicionConfig, []() { return input == RFID; });
  stateMachine.AddTransition(PosicionInicio, PosicionBloqueo, []() { return input == RFID; });

  // Add transitions Bloqueo
  stateMachine.AddTransition(PosicionBloqueo, PosicionInicio, []() { return input == Tecla; });

  // Add transitions Config
  stateMachine.AddTransition(PosicionConfig, PosicionMonitor, []() { return input == Tiempo; }); 

  // Add transitions Monitor
  stateMachine.AddTransition(PosicionMonitor, PosicionPMV_Bajo, []() { return input == DHT11; }); 
  stateMachine.AddTransition(PosicionMonitor, PosicionPMV_Alto, []() { return input == DHT11; }); 
  stateMachine.AddTransition(PosicionMonitor, PosicionConfig, []() { return input == Tiempo; }); 

  // Add transitions PMV
  stateMachine.AddTransition(PosicionPMV_Bajo, PosicionMonitor, []() { return input == Tiempo; });
  stateMachine.AddTransition(PosicionPMV_Alto, PosicionMonitor, []() { return input == Tiempo; });
  stateMachine.AddTransition(PosicionPMV_Alto, PosicionAlarma, []() { return input == SensorTemperatura; });

  // Add transitions Alarma
  stateMachine.AddTransition(PosicionBloqueo, PosicionInicio, []() { return input == SensorInfrarojo; }); 
  

  // Add actions
  stateMachine.SetOnEntering(PosicionInicio, outputInicio);
  stateMachine.SetOnEntering(PosicionConfig, outputConfig);
  stateMachine.SetOnEntering(PosicionAlarma, outputAlarma);
  stateMachine.SetOnEntering(PosicionBloqueo, outputBloqueo);
  stateMachine.SetOnEntering(PosicionMonitor, outputMonitor);
}

void setup() {
  
}

void loop() {
  
}

// Auxiliar output functions that show the state debug
void outputInicio() {
  TaskTime.SetIntervalMillis(7000); // 7s hasta Config si no hay input
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}

void outputConfig() {
  TaskTime.SetIntervalMillis(5000); // 5s hasta Monitor
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}

void outputAlarma() {
  TaskTime.SetIntervalMillis(4000); // 4s hasta Monitor
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}

void outputBloqueo() {
  TaskTime.SetIntervalMillis(7000); // 7s hasta Inicio
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}

void outputMonitor() {
  TaskTime.SetIntervalMillis(3000); // 3s hasta Config
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}


void outputPMV_Bajo() {
  TaskTime.SetIntervalMillis(3000); // 3s hasta Monitor
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();
}