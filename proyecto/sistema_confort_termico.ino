/**
 * @file sistema_confort_termico.ino
 * @brief Sistema de Confort Térmico Inteligente con Actuadores Complejos y RFID
 * @author Jonatan David Bolaños, Rhony Daniel Martinez, Juan Esteban Moscoso, 
 *         Andres Felipe Obando, Daniel Alejandro Vidal
 * @date 2025
 * @version 3.1
 * 
 * @details
 * Sistema avanzado de confort térmico que implementa máquina de estados finitos,
 * cálculo PMV según modelo de Fanger, sistema RFID para identificación de usuarios
 * y sistema completo de actuadores (servomotor, ventilador, buzzer, LEDs RGB) 
 * para regulación automática del ambiente.
 * 
 * @section features_sec Características Principales
 * - Control de acceso mediante keypad matricial 4x4 y sistema RFID MFRC522
 * - Monitoreo en tiempo real con DHT11, LDR, termistor NTC y sensor PIR
 * - Cálculo automático del índice PMV para evaluación científica de confort térmico
 * - Sistema de actuadores: servomotor (persianas), ventilador DC, buzzer (alarma), LEDs RGB
 * - Interfaz visual con LCD 16x2 para feedback al usuario
 * - Máquina de estados robusta con 7 estados y transiciones asíncronas
 * - Sistema de identificación por tarjetas RFID con almacenamiento en EEPROM
 * - Detección de condiciones críticas y sistema de alarma con sensor de movimiento
 * 
 * @section states_sec Estados del Sistema FSM
 * - INICIO: Autenticación de usuario mediante clave de seguridad
 * - CONFIG: Configuración inicial y registro de tarjetas RFID
 * - MONITOR: Monitoreo continuo de sensores y cálculo PMV
 * - PMV_ALTO: Activación de sistemas de enfriamiento (PMV > 1)
 * - PMV_BAJO: Activación de sistemas de calefacción (PMV < -1)
 * - ALARMA: Estado de emergencia por temperaturas críticas persistentes
 * - BLOQUEO: Seguridad por intentos fallidos de autenticación
 * 
 * @section hardware_sec Hardware Integrado
 * - Arduino Mega 2560 como unidad de control principal
 * - LCD 16x2 para interfaz de usuario
 * - Keypad 4x4 para entrada de credenciales
 * - Módulo RFID MFRC522 para identificación de usuarios
 * - Sensor DHT11 (temperatura y humedad ambiente)
 * - Sensor LDR (luminosidad ambiental)
 * - Termistor NTC (temperatura radiante)
 * - Sensor PIR (detección de movimiento)
 * - LED RGB para indicación visual de estados
 * - Buzzer pasivo para alertas audibles
 * - Servomotor SG90 para control de persianas/ventilación
 * - Módulo de ventilador DC para control de flujo de aire
 * - Memoria EEPROM para almacenamiento de datos de tarjetas RFID
 * 
 * @section pmv_sec Cálculo PMV (Predicted Mean Vote)
 * Implementa el modelo termofisiológico de Fanger para evaluación científica
 * del confort térmico según normas ISO 7730:2005
 * - PMV < -1: Condiciones frías - Activa calefacción y cierra persianas
 * - PMV > +1: Condiciones cálidas - Activa ventilación y abre persianas
 * - PMV entre -1 y +1: Condiciones óptimas - Mantiene estado neutral
 * 
 * @section rfid_sec Sistema de Identificación RFID
 * - Registro y reconocimiento de tarjetas RFID
 * - Almacenamiento seguro en EEPROM con nombres de usuario
 * - Gestión de hasta 10 tarjetas diferentes
 * - Interfaz serial para administración de tarjetas
 * 
 * @dot
 * digraph FSM {
 *   rankdir=TB;
 *   node [shape=ellipse, style=filled, fillcolor=lightblue];
 *   edge [color=darkgreen, fontsize=10];
 *   
 *   INICIO [fillcolor=gold];
 *   BLOQUEO [fillcolor=red];
 *   ALARMA [fillcolor=orange];
 *   PMV_ALTO [fillcolor=lightcoral];
 *   PMV_BAJO [fillcolor=lightcyan];
 *   CONFIG [fillcolor=lightgreen];
 *   MONITOR [fillcolor=lightyellow];
 *   
 *   INICIO -> CONFIG [label="CLAVE_CORRECTA"];
 *   INICIO -> BLOQUEO [label="SISTEMA_BLOQUEADO"];
 *   BLOQUEO -> INICIO [label="TECLA_ASTERISCO"];
 *   CONFIG -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   MONITOR -> PMV_BAJO [label="PMV_MENOR_QUE_MENOS1"];
 *   MONITOR -> PMV_ALTO [label="PMV_MAYOR_QUE_1"];
 *   MONITOR -> CONFIG [label="TIEMPO_EXPIRADO"];
 *   PMV_BAJO -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   PMV_ALTO -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   PMV_ALTO -> ALARMA [label="TEMP_ALTA_3_INTENTOS"];
 *   ALARMA -> INICIO [label="SENSOR_INFRARROJO"];
 * }
 * @enddot
 * 
 * @references
 * - Fanger, P. O. (1970). Thermal Comfort. Danish Technical Press.
 * - ISO 7730:2005 - Ergonomics of the thermal environment
 * - ASHRAE Standard 55-2020 - Thermal Environmental Conditions for Human Occupancy
 * - MFRC522 Datasheet - NFC/RFID Reader Module
 */

#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include "DHT.h"
#include <math.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>

// ============================================================================
// DEFINICIÓN DE PINES Y CONSTANTES DEL SISTEMA
// ============================================================================

/** @defgroup pines_grupo Configuración de Pines Hardware */
///@{

#define LED_RED     8   /**< Pin digital para canal rojo del LED RGB - Indicador de BLOQUEO/ALARMA */
#define LED_GREEN   9   /**< Pin digital para canal verde del LED RGB - Indicador de PMV_BAJO */
#define LED_BLUE    10  /**< Pin digital para canal azul del LED RGB - Indicador de PMV_ALTO */
#define DHTPIN      41  /**< Pin digital para sensor DHT11 - Temperatura y humedad ambiente */
#define DHTTYPE     DHT11 /**< Tipo de sensor DHT configurado */
#define LDR_PIN     A3  /**< Pin analógico para sensor LDR - Medición de luminosidad ambiental */
#define TEMP_PIN    A0  /**< Pin analógico para termistor NTC - Temperatura radiante */
#define PIR_SENSOR  A5  /**< Pin analógico para sensor PIR - Detección de movimiento */
#define SERVO_PIN   13  /**< Pin PWM para servomotor SG90 - Control de persianas/ventilación */
#define FAN_PIN     38  /**< Pin digital para control de ventilador DC - Sistema de enfriamiento */
#define BUZZER_PIN  7   /**< Pin digital para buzzer pasivo - Sistema de alertas audibles */

// Configuración RFID MFRC522
#define SS_PIN      53  /**< Pin Slave Select para módulo RFID MFRC522 */
#define RST_PIN     6   /**< Pin Reset para módulo RFID MFRC522 */

/**
 * @brief Coeficiente beta del termistor NTC
 * @details Valor característico del termistor para cálculo de temperatura
 * @reference Datasheet NTC Thermistor 3950K
 */
const float BETA = 3950;

///@}

// ============================================================================
// CONFIGURACIÓN RFID Y EEPROM
// ============================================================================

/** @defgroup rfid_grupo Configuración Sistema RFID */
///@{

#define MAX_UID_LENGTH   10     /**< Longitud máxima UID RFID (4, 7 o 10 bytes) */
#define NAME_MAX_LENGTH  16     /**< Longitud máxima nombre por tarjeta */
#define RECORD_SIZE      (1 + MAX_UID_LENGTH + NAME_MAX_LENGTH)  /**< Tamaño registro EEPROM: [len][UID][nombre] */
#define MAX_RECORDS      10     /**< Número máximo de tarjetas registrables */

/**
 * @brief Buffer para almacenar nombre de usuario de tarjeta RFID
 */
char nombre[NAME_MAX_LENGTH];

/**
 * @brief Flag para controlar procesamiento único de tarjeta en estado CONFIG
 */
bool tarjetaProcesada = false;

/**
 * @brief Instancia del lector RFID MFRC522
 */
MFRC522 mfrc522(SS_PIN, RST_PIN);

///@}

// ============================================================================
// VARIABLES GLOBALES DEL SISTEMA
// ============================================================================

/** @defgroup variables_grupo Variables Globales de Estado y Cálculo */
///@{

/**
 * @brief Tasa metabólica de referencia [W/m²]
 * @details Valor base para cálculo PMV - Actividad ligera (oficina)
 * @range 0-200 W/m²
 */
float M = 0.0;

/**
 * @brief Aislamiento térmico de la ropa [clo]
 * @details Resistencia térmica de la vestimenta - Valor típico para ropa ligera
 * @range 0-2 clo
 */
float clo = 0.0;

float vel_ar = 0.0;     /**< Velocidad del aire en el ambiente [m/s] */
float tempA = 0.0;      /**< Temperatura del aire ambiente [°C] - Sensor DHT11 */
float tempR = 0.0;      /**< Temperatura radiante media [°C] - Sensor NTC */
float humedad = 0.0;    /**< Humedad relativa del ambiente [%] - Sensor DHT11 */
float luz = 0.0;        /**< Nivel de luminosidad ambiental [%] - Sensor LDR */

/**
 * @brief Contador de temperaturas altas consecutivas
 * @details Utilizado para detección de condiciones críticas en estado PMV_ALTO
 * @security Al alcanzar 3 activa transición a estado ALARMA
 */
int conteoTempAlta = 0;

/**
 * @brief Flag de timeout del estado MONITOR
 * @details Indica cuando ha finalizado el período de monitoreo para cálculo PMV
 */
bool tiempoMonitorVencido = false;

///@}

// ============================================================================
// CONFIGURACIÓN DE DISPOSITIVOS HARDWARE
// ============================================================================

/** @defgroup hardware_grupo Instancias de Dispositivos Hardware */
///@{

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); /**< Objeto LCD para display 16x2 con configuración de pines */
DHT dht(DHTPIN, DHTTYPE); /**< Objeto sensor DHT11 para medición de temperatura y humedad */
Servo servo; /**< Objeto servomotor para control de actuadores mecánicos */

///@}

// ============================================================================
// SISTEMA DE ALERTAS - BUZZER
// ============================================================================

/**
 * @brief Estado actual del buzzer
 * @details Controla el estado de activación/desactivación del buzzer
 */
bool buzzerState = false;

/**
 * @brief Tarea asíncrona para control del buzzer
 * @details Genera patrón de sonido intermitente en estado ALARMA
 */
void toggleBuzzer();
AsyncTask TaskBuzzer(200, true, toggleBuzzer);

// ============================================================================
// SISTEMA DE SEGURIDAD - KEYPAD
// ============================================================================

/**
 * @brief Clave de acceso predefinida para autenticación
 * @details Secuencia de 6 caracteres requerida para desbloquear el sistema
 * @security Almacenada en memoria flash - No modificable en tiempo de ejecución
 */
const char clave[6] = {'2', '0', '2', '5', '2', 'A'};
char clave_user[6];  /**< Buffer para almacenar clave ingresada por usuario */

/**
 * @brief Configuración del teclado matricial 4x4
 * @details Mapeo completo de teclas y configuración de pines de conexión
 */
const byte ROWS = 4; /**< Número de filas del keypad */
const byte COLS = 4; /**< Número de columnas del keypad */

// Matriz de caracteres del keypad
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {40, 42, 44, 46}; /**< Pines de conexión para filas del keypad */
byte colPins[COLS] = {48, 24, 22, 43}; /**< Pines de conexión para columnas del keypad */

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ============================================================================
// SISTEMA DE ACTUADORES - SERVOMOTOR
// ============================================================================

/**
 * @brief Posición actual del servomotor [grados]
 * @details Rango de 20° (mínimo) a 90° (máximo) para control de persianas
 */
int servoPos = 20;

/**
 * @brief Dirección actual del movimiento del servomotor
 * @details true = incrementando (hacia 90°), false = decrementando (hacia 20°)
 */
bool servoUp = true;

/**
 * @brief Tarea asíncrona para control del servomotor
 * @details Genera movimiento suave y continuo del servomotor
 */
void moverServo();
AsyncTask TaskServo(15, true, moverServo);

// ============================================================================
// MÁQUINA DE ESTADOS FINITOS (FSM)
// ============================================================================

/** @defgroup fsm_grupo Sistema de Máquina de Estados Finitos */
///@{

/**
 * @enum State
 * @brief Enumeración de estados del sistema FSM
 * @details Define los 7 estados principales del sistema de control con sus transiciones
 */
enum State {
  INICIO = 0,    /**< Estado inicial - Autenticación de usuario mediante keypad */
  CONFIG = 1,    /**< Estado de configuración - Inicialización y registro RFID */
  ALARMA = 2,    /**< Estado de alarma - Condiciones críticas detectadas */
  MONITOR = 3,   /**< Estado de monitoreo - Lectura de sensores y cálculo PMV */
  BLOQUEO = 4,   /**< Estado de bloqueo - Seguridad por autenticación fallida */
  PMV_ALTO = 5,  /**< Estado PMV alto - Activación de sistemas de enfriamiento */
  PMV_BAJO = 6   /**< Estado PMV bajo - Activación de sistemas de calefacción */
};

/**
 * @enum Input
 * @brief Enumeración de entradas que disparan transiciones de estado
 * @details Eventos del sistema que provocan cambios entre estados de la FSM
 */
enum Input {
  SISTEMA_BLOQUEADO = 0,     /**< Activación por condición de bloqueo de seguridad */
  TECLA_ASTERISCO = 1,       /**< Usuario presiona tecla '*' para desbloquear sistema */
  CLAVE_CORRECTA = 2,        /**< Clave válida ingresada - Autenticación exitosa */
  TIEMPO_EXPIRADO = 3,       /**< Temporizador expirado - Transiciones automáticas */
  PMV_MENOR_QUE_MENOS1 = 4,  /**< Condición PMV < -1 detectada - Requiere calefacción */
  PMV_MAYOR_QUE_1 = 5,       /**< Condición PMV > 1 detectada - Requiere ventilación */
  TEMP_ALTA_3_INTENTOS = 6,  /**< Temperatura crítica persistente - Activa alarma */
  SENSOR_INFRARROJO = 7,     /**< Detección de movimiento - Desactiva alarma */
  Unknown = 8                /**< Entrada desconocida o no definida */
};

StateMachine stateMachine(7, 11); /**< Instancia de máquina de estados con 7 estados y 11 transiciones */
Input input = Unknown; /**< Almacena la última entrada del usuario para procesamiento FSM */

///@}

// ============================================================================
// SISTEMA DE TEMPORIZACIÓN Y TAREAS ASÍNCRONAS
// ============================================================================

/** @defgroup tareas_grupo Sistema de Tareas Asíncronas y Temporizadores */
///@{

/**
 * @brief Tarea de temporizador principal del sistema
 * @details Gestiona timeouts y transiciones automáticas entre estados
 */
void runTime();
AsyncTask TaskTime(5000, true, runTime);

/**
 * @brief Función callback para temporizador principal
 * @details Maneja comportamientos específicos según el estado actual
 */
void runTime() {
  if (stateMachine.GetState() == MONITOR) {
    tiempoMonitorVencido = true;
  } else {
    input = TIEMPO_EXPIRADO;
  }
}

// ============================================================================
// SISTEMA DE INDICACIÓN VISUAL - LEDs RGB
// ============================================================================

/**
 * @brief Tarea para parpadeo asíncrono del LED rojo
 * @details Indicador visual para estados de BLOQUEO y ALARMA
 */
void toggleRed();
AsyncTask TaskLedRed(100, true, toggleRed);

bool ledStateRed = false; /**< Estado actual del LED rojo */
unsigned long lastToggleRed = 0; /**< Último tiempo de cambio del LED rojo */

/**
 * @brief Tarea para parpadeo asíncrono del LED verde  
 * @details Indicador visual para estado PMV_BAJO (condiciones frías)
 */
void toggleGreen();
AsyncTask TaskLedGreen(200, true, toggleGreen);

bool ledStateGreen = false; /**< Estado actual del LED verde */
unsigned long lastToggleGreen = 0; /**< Último tiempo de cambio del LED verde */

/**
 * @brief Tarea para parpadeo asíncrono del LED azul
 * @details Indicador visual para estado PMV_ALTO (condiciones cálidas)
 */
void toggleBlue();
AsyncTask TaskLedBlue(300, true, toggleBlue);

bool ledStateBlue = false; /**< Estado actual del LED azul */
unsigned long lastToggleBlue = 0; /**< Último tiempo de cambio del LED azul */

///@}

// ============================================================================
// CONFIGURACIÓN DE LA MÁQUINA DE ESTADOS
// ============================================================================

/**
 * @brief Configura todas las transiciones y callbacks de la FSM
 * 
 * @details
 * Define la lógica completa del sistema mediante:
 * - Transiciones entre estados basadas en condiciones de entrada
 * - Callbacks de entrada (OnEntering) para inicialización de estados
 * - Callbacks de salida (OnLeaving) para limpieza de recursos
 * - Funciones lambda para evaluación de condiciones en tiempo de ejecución
 * 
 * @dot
 * digraph FSM {
 *   rankdir=TB;
 *   node [shape=ellipse, style=filled, fillcolor=lightblue];
 *   edge [color=darkgreen, fontsize=10];
 *   
 *   INICIO [fillcolor=gold];
 *   BLOQUEO [fillcolor=red];
 *   ALARMA [fillcolor=orange];
 *   PMV_ALTO [fillcolor=lightcoral];
 *   PMV_BAJO [fillcolor=lightcyan];
 *   CONFIG [fillcolor=lightgreen];
 *   MONITOR [fillcolor=lightyellow];
 *   
 *   INICIO -> CONFIG [label="CLAVE_CORRECTA"];
 *   INICIO -> BLOQUEO [label="SISTEMA_BLOQUEADO"];
 *   BLOQUEO -> INICIO [label="TECLA_ASTERISCO"];
 *   CONFIG -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   MONITOR -> PMV_BAJO [label="PMV_MENOR_QUE_MENOS1"];
 *   MONITOR -> PMV_ALTO [label="PMV_MAYOR_QUE_1"];
 *   MONITOR -> CONFIG [label="TIEMPO_EXPIRADO"];
 *   PMV_BAJO -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   PMV_ALTO -> MONITOR [label="TIEMPO_EXPIRADO"];
 *   PMV_ALTO -> ALARMA [label="TEMP_ALTA_3_INTENTOS"];
 *   ALARMA -> INICIO [label="SENSOR_INFRARROJO"];
 * }
 * @enddot
 * 
 * @note Cada transición es evaluada en cada ciclo del loop principal
 * @see State
 * @see Input
 */
void setupStateMachine() {
  // ==================== TRANSICIONES DESDE INICIO ====================
  stateMachine.AddTransition(INICIO, CONFIG, []() {
    return input == CLAVE_CORRECTA;
  });
  stateMachine.AddTransition(INICIO, BLOQUEO, []() {
    return input == SISTEMA_BLOQUEADO;
  });

  // ==================== TRANSICIONES DESDE BLOQUEO ====================
  stateMachine.AddTransition(BLOQUEO, INICIO, []() {
    return input == TECLA_ASTERISCO;
  });

  // ==================== TRANSICIONES DESDE CONFIG ====================
  stateMachine.AddTransition(CONFIG, MONITOR, []() {
    return input == TIEMPO_EXPIRADO;
  });

  // ==================== TRANSICIONES DESDE MONITOR ====================
  stateMachine.AddTransition(MONITOR, PMV_BAJO, []() {
    return input == PMV_MENOR_QUE_MENOS1;
  });
  stateMachine.AddTransition(MONITOR, PMV_ALTO, []() {
    return input == PMV_MAYOR_QUE_1;
  });
  stateMachine.AddTransition(MONITOR, CONFIG, []() {
    return input == TIEMPO_EXPIRADO;
  });

  // ==================== TRANSICIONES DESDE ESTADOS PMV ====================
  stateMachine.AddTransition(PMV_BAJO, MONITOR, []() {
    return input == TIEMPO_EXPIRADO;
  });
  stateMachine.AddTransition(PMV_ALTO, MONITOR, []() {
    return input == TIEMPO_EXPIRADO;
  });
  stateMachine.AddTransition(PMV_ALTO, ALARMA, []() {
    return input == TEMP_ALTA_3_INTENTOS;
  });

  // ==================== TRANSICIONES DESDE ALARMA ====================
  stateMachine.AddTransition(ALARMA, INICIO, []() {
    return input == SENSOR_INFRARROJO;
  });

  // ==================== CONFIGURACIÓN DE CALLBACKS DE ENTRADA ====================
  stateMachine.SetOnEntering(INICIO, outputInicio);
  stateMachine.SetOnEntering(BLOQUEO, outputBloqueo);
  stateMachine.SetOnEntering(CONFIG, outputConfig);
  stateMachine.SetOnEntering(MONITOR, outputMonitor);
  stateMachine.SetOnEntering(PMV_BAJO, outputPMV_Bajo);
  stateMachine.SetOnEntering(PMV_ALTO, outputPMV_Alto);
  stateMachine.SetOnEntering(ALARMA, outputAlarma);

  // ==================== CONFIGURACIÓN DE CALLBACKS DE SALIDA ====================
  stateMachine.SetOnLeaving(BLOQUEO, onLeavingBloqueo);
  stateMachine.SetOnLeaving(ALARMA, []() {
    TaskBuzzer.Stop(); 
    digitalWrite(BUZZER_PIN, LOW);
  });
  stateMachine.SetOnLeaving(PMV_BAJO, onLeavingPMV_Bajo);
  stateMachine.SetOnLeaving(PMV_ALTO, onLeavingPMV_Alto);
}

// ============================================================================
// CONFIGURACIÓN INICIAL DEL SISTEMA
// ============================================================================

/**
 * @brief Función de inicialización del sistema Arduino
 * 
 * @details
 * Configuración inicial que se ejecuta una vez al iniciar el sistema:
 * - Inicializa comunicación serial a 9600 baudios para monitoreo y debugging
 * - Configura display LCD 16x2 con pines específicos
 * - Inicializa sensor DHT11 para lectura de temperatura y humedad
 * - Configura servomotor en pin PWM específico
 * - Establece estado inicial de la FSM en INICIO
 * - Configura pines de LEDs RGB, buzzer, ventilador y sensor PIR como salidas
 * - Inicializa sistema SPI y módulo RFID MFRC522
 * - Configura sistema de tareas asíncronas
 * 
 * @pre Todos los componentes deben estar correctamente conectados según diagrama
 * @post Sistema listo para ejecutar loop principal y procesamiento FSM
 * 
 * @see setupStateMachine()
 * @see loop()
 */
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
  servo.attach(SERVO_PIN);

  // Configuración de pines de salida
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(PIR_SENSOR, INPUT);

  // Inicialización segura de actuadores
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);

  // Inicialización del sistema RFID
  SPI.begin();           // Inicializa bus SPI
  mfrc522.PCD_Init();    // Inicializa el lector RFID

  lcd.setCursor(0, 0);

  setupStateMachine();
  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM Iniciada - Sistema de Confort Térmico v3.1 con RFID");
  lcd.println("Iniciando...");
}

// ============================================================================
// BUCLE PRINCIPAL DEL SISTEMA
// ============================================================================

/**
 * @brief Bucle principal del sistema - Ejecución continua
 * 
 * @details
 * Ejecuta continuamente las siguientes tareas en cada iteración:
 * - Lectura de entradas del usuario (serial y keypad)
 * - Actualización de todas las tareas asíncronas (temporizadores, LEDs, buzzer, servo)
 * - Verificación de condiciones de bloqueo y temperatura
 * - Evaluación de cálculo PMV en estado MONITOR
 * - Detección de movimiento para desactivación de alarma
 * - Gestión de configuración RFID en estado CONFIG
 * - Actualización de la máquina de estados
 * 
 * @remark Esta función se ejecuta indefinidamente después de setup()
 * @frequency Aproximadamente 50-100 iteraciones por segundo dependiendo de carga
 * 
 * @see setup()
 */
void loop() {
  // Leer input usuario (por serial)
  if (input == Unknown) {
    input = static_cast<Input>(readInputSerial());
  }

  // Actualizar todas las tareas asíncronas
  TaskTime.Update();
  TaskLedRed.Update();
  TaskLedGreen.Update();
  TaskLedBlue.Update();
  TaskBuzzer.Update();
  TaskServo.Update();

  // Verificaciones de estado específicas
  checkBloqueo();
  checkPMV();
  checkMovimiento();
  
  // Actualizar máquina de estados
  stateMachine.Update();

  // Gestión de configuración RFID (solo en estado CONFIG)
  handleConfigRFID();

  // Comandos por serial para gestión de tags RFID
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    // Aquí se pueden implementar comandos adicionales para gestión RFID
  }
}

// ============================================================================
// FUNCIONES DE SALIDA PARA CADA ESTADO FSM
// ============================================================================

/** @defgroup estados_grupo Funciones de Salida de Estados FSM */
///@{

/**
 * @brief Maneja la entrada al estado INICIO - autenticación de usuario
 * 
 * @details
 * - Limpia entradas previas y prepara el sistema para autenticación
 * - Muestra mensaje de bienvenida en LCD
 * - Solicita y valida contraseña mediante keypad con feedback visual
 * - Transiciona a CONFIG (éxito) o BLOQUEO (fallo)
 * - Configura temporizador para transiciones automáticas
 * 
 * @post Sistema autenticado y listo para configuración o bloqueado por seguridad
 * @timeout 5000 ms para transición automática
 * 
 * @see readPassword()
 * @see checkPassword()
 */
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

/**
 * @brief Maneja la entrada al estado CONFIG - configuración del sistema y RFID
 * 
 * @details
 * Estado de configuración que espera 5 segundos antes de
 * transicionar automáticamente a MONITOR. Permite la
 * inicialización de parámetros del sistema y preparación
 * de periféricos. Además, habilita el registro de nuevas
 * tarjetas RFID.
 * 
 * @timeout 5000 ms para transición automática a MONITOR
 * @see handleConfigRFID()
 */
void outputConfig() {
  input = Unknown;
  TaskTime.SetIntervalMillis(5000);  // 5 Segs hasta monitor si no hay entrada
  TaskTime.Start();
  tarjetaProcesada = false;  // Reset flag para nuevo procesamiento RFID
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("            X                                                     ");
  Serial.println();
}

/**
 * @brief Maneja la entrada al estado ALARMA - condiciones críticas
 * 
 * @details
 * Estado de emergencia activado por temperaturas críticas persistentes:
 * - Muestra mensaje de alarma en LCD
 * - Activa buzzer con patrón intermitente (400ms)
 * - Requiere detección de movimiento (sensor PIR) para desactivarse
 * - Indica al usuario la acción requerida para resetear el sistema
 * 
 * @warning Estado que requiere intervención manual para salir
 * @security Protección contra condiciones térmicas peligrosas
 * @see checkMovimiento()
 */
void outputAlarma() {
  input = Unknown;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("!!! ALARMA !!!");

  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                               X                                  ");
  Serial.println();

  buzzerState = false;
  digitalWrite(BUZZER_PIN, LOW);
  TaskBuzzer.SetIntervalMillis(400);  // Patrón de sonido más lento para alarma
  TaskBuzzer.Start();
}

/**
 * @brief Maneja la entrada al estado BLOQUEO - seguridad por fallos
 * 
 * @details
 * - Muestra mensaje de sistema bloqueado en LCD
 * - Indica al usuario presionar '*' para desbloquear
 * - Activa parpadeo del LED rojo como indicador visual
 * - Espera entrada específica para regresar a INICIO
 * 
 * @security Estado de protección contra intentos fallidos de autenticación
 * @see toggleRed()
 * @see checkBloqueo()
 */
void outputBloqueo() {
  input = Unknown;
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

/**
 * @brief Maneja la entrada al estado MONITOR - monitoreo principal
 * 
 * @details
 * Estado principal de monitoreo que prepara el sistema para:
 * - Lectura de todos los sensores del sistema
 * - Cálculo del índice PMV en tiempo real
 * - Evaluación de condiciones de confort térmico
 * - Toma de decisiones de transición automática
 * - Permanece activo por 7 segundos antes de regresar a CONFIG
 * 
 * @timeout 7000 ms para transición automática
 * @see leerSensores()
 * @see calcularPMV_Fanger()
 * @see checkPMV()
 */
void outputMonitor() {
  input = Unknown;
  leerSensores();

  // Mostrar datos de sensores en tiempo real en LCD
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(tempA, 1);
  lcd.print("C  H:");
  lcd.print(humedad, 0);
  lcd.print("%   ");
  lcd.setCursor(0, 1);
  lcd.print("Luz:");
  lcd.print(luz, 0);
  lcd.print("% Tr:");
  lcd.print(tempR, 1);
  lcd.print("  ");

  // Iniciar temporizador para cálculo PMV
  TaskTime.SetIntervalMillis(7000);  // 7 segs hasta cambio de estado
  TaskTime.Start();

  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                     X                                            ");
  Serial.println();
}

/**
 * @brief Maneja la entrada al estado PMV_BAJO - condiciones frías
 * 
 * @details
 * Estado activo cuando PMV < -1 (condiciones de frío):
 * - Activa LED verde con patrón de parpadeo específico
 * - Inicializa servomotor para control de persianas (cierre)
 * - Configura movimiento continuo del servomotor (20°-90°)
 * - Permanece activo por 3 segundos antes de regresar a MONITOR
 * 
 * @timeout 3000 ms para transición automática
 * @see toggleGreen()
 * @see moverServo()
 */
void outputPMV_Bajo() {
  input = Unknown;
  TaskTime.SetIntervalMillis(3000);  // 3 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                        X                         ");
  Serial.println();

  ledStateGreen = false;
  digitalWrite(LED_GREEN, LOW);
  lastToggleGreen = millis();
  TaskLedGreen.Start();

  // Reiniciar posición y dirección del servo
  servo.attach(SERVO_PIN);
  servoPos = 0;
  servoUp = true;
  servo.write(servoPos);

  // Iniciar movimiento asíncrono
  TaskServo.Start();
}

/**
 * @brief Maneja la entrada al estado PMV_ALTO - condiciones cálidas
 * 
 * @details
 * Estado activo cuando PMV > 1 (condiciones de calor):
 * - Activa LED azul con patrón de parpadeo específico
 * - Activa ventilador DC para enfriamiento
 * - Monitorea temperatura para detección de condiciones críticas
 * - Permanece activo por 4 segundos antes de regresar a MONITOR
 * - Transiciona a ALARMA si detecta 3 lecturas consecutivas >30°C
 * 
 * @timeout 4000 ms para transición automática
 * @see toggleBlue()
 * @see checkTemperaturaAlta()
 */
void outputPMV_Alto() {
  input = Unknown;
  
  // Monitoreo de temperatura para detección de condiciones críticas
  tempA = dht.readTemperature();
  if (tempA > 30.0) {
    conteoTempAlta++;
    Serial.print("Temperatura alta detectada (");
    Serial.print(conteoTempAlta);
    Serial.println("/3)");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp > 30, Intentos: ");
    lcd.setCursor(0, 1);
    lcd.print(conteoTempAlta);
  } else {
    conteoTempAlta = 0;  // reset si baja la temperatura
  }
  
  if (conteoTempAlta >= 3) {
    input = TEMP_ALTA_3_INTENTOS;
    conteoTempAlta = 0;  // reinicia para futuras detecciones
  }
  
  TaskTime.SetIntervalMillis(4000);  // 4 segs hasta monitor si no hay entrada
  TaskTime.Start();
  Serial.println("Inicio   Config   Monitor   Alarma   PMV_Bajo   PMV_Alto   Bloqueo");
  Serial.println("                                                    X             ");
  Serial.println();

  ledStateBlue = false;
  digitalWrite(LED_BLUE, LOW);
  lastToggleBlue = millis();
  TaskLedBlue.Start();

  // Activar ventilador para enfriamiento
  digitalWrite(FAN_PIN, HIGH);
}

///@}

// ============================================================================
// FUNCIONES DE SALIDA Y LIMPIEZA DE ESTADOS
// ============================================================================

/**
 * @brief Maneja la salida del estado BLOQUEO - limpieza de recursos
 * 
 * @details
 * - Detiene el parpadeo del LED rojo
 * - Apaga completamente el LED rojo
 * - Limpia recursos del estado de bloqueo
 * - Prepara sistema para retornar a estado normal
 * 
 * @see outputBloqueo()
 */
void onLeavingBloqueo() {
  TaskLedRed.Stop();
  digitalWrite(LED_RED, LOW);
  Serial.println("Saliendo de BLOQUEO, LED apagado");
}

/**
 * @brief Maneja la salida del estado PMV_BAJO - limpieza de recursos
 * 
 * @details
 * - Detiene el parpadeo del LED verde
 * - Apaga completamente el LED verde
 * - Detiene el movimiento del servomotor
 * - Desconecta el servomotor para ahorro energético
 * 
 * @see outputPMV_Bajo()
 */
void onLeavingPMV_Bajo() {
  TaskLedGreen.Stop();
  digitalWrite(LED_GREEN, LOW);
  TaskServo.Stop();
  servo.detach();
  Serial.println("Saliendo de PMV_Bajo, LED apagado");
}

/**
 * @brief Maneja la salida del estado PMV_ALTO - limpieza de recursos
 * 
 * @details
 * - Detiene el parpadeo del LED azul
 * - Apaga completamente el LED azul
 * - Desactiva el ventilador DC
 * - Prepara sistema para retorno a monitoreo
 * 
 * @see outputPMV_Alto()
 */
void onLeavingPMV_Alto() {
  TaskLedBlue.Stop();
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(FAN_PIN, LOW);
  Serial.println("Saliendo de PMV_Alto, LED apagado");
}

// ============================================================================
// FUNCIONES AUXILIARES DEL SISTEMA
// ============================================================================

/** @defgroup auxiliares_grupo Funciones Auxiliares del Sistema */
///@{

/**
 * @brief Lee entrada del usuario desde el puerto serial
 * 
 * @return Input Entrada del usuario mapeada a enumeración Input
 * 
 * @details
 * Convierte caracteres numéricos del puerto serial a las
 * entradas definidas en la enumeración Input para la FSM.
 * Utilizado principalmente para debugging y control manual
 * durante desarrollo.
 * 
 * @mapping
 * - '0' → SISTEMA_BLOQUEADO
 * - '1' → TECLA_ASTERISCO  
 * - '2' → CLAVE_CORRECTA
 * - '3' → TIEMPO_EXPIRADO
 * - '4' → PMV_MENOR_QUE_MENOS1
 * - '5' → PMV_MAYOR_QUE_1
 * - '6' → TEMP_ALTA_3_INTENTOS
 * - '7' → SENSOR_INFRARROJO
 * 
 * @see Input
 */
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

// ============================================================================
// SISTEMA DE INDICACIÓN VISUAL - LEDs RGB
// ============================================================================

/**
 * @brief Controla el parpadeo del LED rojo con patrón asíncrono
 * @details Implementa patrón: 100ms encendido, 300ms apagado
 * @pattern ON: 100ms, OFF: 300ms
 * @usage Estados de BLOQUEO y ALARMA
 */
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

/**
 * @brief Controla el parpadeo del LED verde con patrón asíncrono
 * @details Implementa patrón: 200ms encendido, 300ms apagado
 * @pattern ON: 200ms, OFF: 300ms
 * @usage Estado PMV_BAJO (condiciones frías)
 */
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

/**
 * @brief Controla el parpadeo del LED azul con patrón asíncrono
 * @details Implementa patrón: 300ms encendido, 400ms apagado
 * @pattern ON: 300ms, OFF: 400ms
 * @usage Estado PMV_ALTO (condiciones cálidas)
 */
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
  else if (!ledStateBlue && (now - lastToggleBlue >= 400)) {
    ledStateBlue = true;
    digitalWrite(LED_BLUE, HIGH);
    lastToggleBlue = now;
    TaskLedBlue.SetIntervalMillis(300);  // siguiente ciclo: 300 ms encendido
  }
}

// ============================================================================
// SISTEMA RFID - GESTIÓN DE TARJETAS
// ============================================================================

/** @defgroup rfid_funciones_grupo Funciones de Gestión RFID */
///@{

/**
 * @brief Compara dos arrays de bytes para verificar igualdad
 * 
 * @param arrayA Primer array a comparar
 * @param arrayB Segundo array a comparar  
 * @param length Longitud de los arrays a comparar
 * @return true si los arrays son idénticos
 * @return false si los arrays son diferentes
 * 
 * @details
 * Utilizada para comparar UIDs de tarjetas RFID durante
 * el proceso de identificación y registro.
 */
bool isEqualArray(byte* arrayA, byte* arrayB, int length) {
  for (int index = 0; index < length; index++) {
    if (arrayA[index] != arrayB[index]) return false;
  }
  return true;
}

/**
 * @brief Busca un UID en la memoria EEPROM
 * 
 * @param uid Array de bytes con el UID a buscar
 * @param uidLength Longitud del UID
 * @return int Dirección en EEPROM si se encuentra, -1 si no existe
 * 
 * @details
 * Recorre todos los registros en EEPROM comparando el UID
 * proporcionado con los almacenados. Considera tanto la
 * longitud como cada byte del UID.
 * 
 * @see MAX_RECORDS
 * @see RECORD_SIZE
 */
int findUIDInEEPROM(byte* uid, byte uidLength) {
  for (int recordIndex = 0; recordIndex < MAX_RECORDS; recordIndex++) {
    int address = recordIndex * RECORD_SIZE;
    byte storedLen = EEPROM.read(address);

    // Registro vacío → saltar
    if (storedLen == 0xFF) continue;

    // Si la longitud no coincide → no es la misma tarjeta
    if (storedLen != uidLength) continue;

    // Comparar byte a byte
    bool match = true;
    for (int i = 0; i < uidLength; i++) {
      byte storedByte = EEPROM.read(address + 1 + i);
      if (storedByte != uid[i]) {
        match = false;
        break;
      }
    }

    if (match) {
      return address; // encontrado
    }
  }
  return -1; // no encontrado
}

/**
 * @brief Lee el nombre asociado a un registro desde EEPROM
 * 
 * @param recordAddress Dirección base del registro en EEPROM
 * @param buffer Buffer donde almacenar el nombre leído
 * @param maxLen Longitud máxima del buffer
 * 
 * @details
 * Extrae el campo de nombre de un registro RFID almacenado
 * en EEPROM. El nombre se almacena después del UID en la
 * estructura del registro.
 */
void readNameFromEEPROM(int recordAddress, char* buffer, int maxLen) {
  int base = recordAddress;
  for (int i = 0; i < maxLen; i++) {
    buffer[i] = EEPROM.read(base + 1 + MAX_UID_LENGTH + i);
    if (buffer[i] == '\0') break;
  }
}

/**
 * @brief Guarda un nuevo UID y nombre en EEPROM
 * 
 * @param uid Array de bytes con el UID de la tarjeta
 * @param uidLength Longitud del UID
 * @param name Nombre asociado a la tarjeta
 * 
 * @details
 * Busca la primera posición libre en EEPROM y almacena:
 * - Longitud del UID (1 byte)
 * - UID completo (hasta MAX_UID_LENGTH bytes)
 * - Nombre (hasta NAME_MAX_LENGTH caracteres)
 * 
 * @security Los registros vacíos se identifican por 0xFF
 * @see MAX_RECORDS
 * @see RECORD_SIZE
 */
void saveUIDAndName(byte* uid, byte uidLength, const char* name) {
  for (int recordIndex = 0; recordIndex < MAX_RECORDS; recordIndex++) {
    int address = recordIndex * RECORD_SIZE;
    byte firstByte = EEPROM.read(address);

    if (firstByte == 0xFF) { // posición libre (EEPROM limpia es 0xFF)
      // Guardar longitud UID
      EEPROM.write(address, uidLength);

      // Guardar UID
      for (int i = 0; i < uidLength; i++) {
        EEPROM.write(address + 1 + i, uid[i]);
      }

      // Guardar nombre
      for (int i = 0; i < NAME_MAX_LENGTH; i++) {
        if (i < strlen(name)) {
          EEPROM.write(address + 1 + MAX_UID_LENGTH + i, name[i]);
        } else {
          EEPROM.write(address + 1 + MAX_UID_LENGTH + i, '\0');
        }
      }

      Serial.print(" Guardado en registro ");
      Serial.println(recordIndex);
      break;
    }
  }
}

/**
 * @brief Imprime por serial todos los registros RFID almacenados en EEPROM
 * 
 * @details
 * Función de diagnóstico que muestra el contenido completo
 * de la EEPROM relacionado con registros RFID. Útil para
 * debugging y verificación del estado del sistema.
 * 
 * @usage Llamar desde monitor serial para inspeccionar registros
 */
void printEEPROMRecords() {
  Serial.println("========================================");
  Serial.println("=== Contenido EEPROM ===");

  for (int recordIndex = 0; recordIndex < MAX_RECORDS; recordIndex++) {
    int address = recordIndex * RECORD_SIZE;
    byte uidLen = EEPROM.read(address);

    // Registro vacío (EEPROM limpia → 0xFF)
    if (uidLen == 0xFF) {
      Serial.print("Registro "); Serial.print(recordIndex);
      Serial.println(" [VACÍO]");
      continue;
    }

    // Leer UID
    Serial.print("Registro "); Serial.print(recordIndex);
    Serial.print(" (addr "); Serial.print(address); Serial.print("): UID = ");
    for (int i = 0; i < uidLen; i++) {
      byte b = EEPROM.read(address + 1 + i);
      Serial.print(b < 0x10 ? " 0" : " ");
      Serial.print(b, HEX);
    }

    // Leer nombre
    Serial.print(" | Nombre = ");
    for (int i = 0; i < NAME_MAX_LENGTH; i++) {
      char c = EEPROM.read(address + 1 + MAX_UID_LENGTH + i);
      if (c == '\0') break;
      Serial.print(c);
    }
    Serial.println();
  }
  Serial.println("========================================");
}

/**
 * @brief Maneja la configuración de tarjetas RFID en estado CONFIG
 * 
 * @details
 * Función principal de gestión RFID que se ejecuta en estado CONFIG:
 * - Detecta presencia de nuevas tarjetas RFID
 * - Verifica si la tarjeta ya está registrada
 * - Si es nueva, solicita nombre y guarda en EEPROM
 * - Si existe, muestra mensaje de bienvenida personalizado
 * - Controla el flujo de registro con timeouts
 * 
 * @condition Solo se ejecuta en estado CONFIG
 * @condition Evita reprocesamiento con flag tarjetaProcesada
 * 
 * @see outputConfig()
 * @see findUIDInEEPROM()
 * @see saveUIDAndName()
 */
void handleConfigRFID() {
  // Asegurar que estamos en el estado CONFIG
  if (stateMachine.GetState() != CONFIG) return;

  // Evitar reprocesar la misma tarjeta en este estado
  if (tarjetaProcesada) return;

  // Detectar nueva tarjeta
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;  // No hay tarjeta todavía → seguir esperando
  }

  byte* uid = mfrc522.uid.uidByte;
  byte uidLength = mfrc522.uid.size;

  // Buscar si la tarjeta ya existe en EEPROM
  int recordAddress = findUIDInEEPROM(uid, uidLength);
  if (recordAddress != -1) {
    // Tarjeta ya registrada → leer nombre desde EEPROM
    readNameFromEEPROM(recordAddress, nombre, NAME_MAX_LENGTH);
    lcd.clear();
    lcd.print("Bienvenido:");
    lcd.setCursor(0, 1);
    lcd.print(nombre);
    Serial.print("Nombre leído: ");
    Serial.println(nombre);
  } else {
    // Nueva tarjeta → solicitar y registrar nombre
    lcd.clear();
    lcd.print("Nueva tarjeta");
    Serial.println("Ingrese nombre para esta tarjeta:");

    // Esperar entrada del nombre por Serial (máx. 15 s)
    unsigned long start = millis();
    while (!Serial.available()) {
      if (millis() - start > 15000) {  // tiempo máximo de espera
        Serial.println("Tiempo de espera agotado.");
        lcd.clear();
        lcd.print("Sin nombre");
        TaskTime.SetIntervalMillis(1500);
        TaskTime.Start();
        mfrc522.PICC_HaltA();
        return; // cancelar registro
      }
    }

    // Leer nombre
    String nombreStr = Serial.readStringUntil('\n');
    nombreStr.trim();

    if (nombreStr.length() > 0) {
      // Convertir a char[] y guardar en EEPROM
      nombreStr.toCharArray(nombre, NAME_MAX_LENGTH);
      saveUIDAndName(uid, uidLength, nombre);

      lcd.clear();
      lcd.print("Nombre guardado");
      Serial.print("Nombre guardado: ");
      Serial.println(nombreStr);
      TaskTime.SetIntervalMillis(2000);
      TaskTime.Start();
    } else {
      lcd.clear();
      lcd.print("Nombre invalido");
      Serial.println("Nombre inválido, no se guardó.");
      TaskTime.SetIntervalMillis(1500);
      TaskTime.Start();
      mfrc522.PICC_HaltA();
      return;  // No continuar si el nombre es inválido
    }
  }

  // Marcar tarjeta como procesada y activar temporizador para volver a MONITOR
  tarjetaProcesada = true;

  // Finalizar comunicación con la tarjeta
  mfrc522.PICC_HaltA();
}

///@}

// ============================================================================
// SISTEMA DE SEGURIDAD - AUTENTICACIÓN
// ============================================================================

/**
 * @brief Lee la contraseña del keypad con feedback visual en LCD
 * 
 * @details
 * Captura secuencialmente 6 caracteres del keypad mostrando:
 * - Guión bajo (_) en posición actual antes de ingresar carácter
 * - Asterisco (*) después de ingresar carácter (ocultación por seguridad)
 * - Pequeño delay entre teclas para mejor experiencia de usuario
 * - Timeout automático por seguridad
 * 
 * @post clave_user contiene los 6 caracteres ingresados por el usuario
 * @security Los caracteres se muestran como '*' para evitar visualización
 * 
 * @see checkPassword()
 */
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

/**
 * @brief Valida la contraseña ingresada contra la clave predefinida
 * 
 * @return true si la clave es correcta (autenticación exitosa)
 * @return false si la clave es incorrecta (autenticación fallida)
 * 
 * @details
 * Compara carácter por carácter el buffer clave_user con la
 * clave predefinida almacenada en memoria. Muestra mensaje
 * apropiado en LCD según resultado de la validación.
 * 
 * @security Función sensible a timing attacks - considerar mejora
 * 
 * @see readPassword()
 * @see clave
 */
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

/**
 * @brief Verifica entrada de desbloqueo en estado BLOQUEO
 * 
 * @details
 * Monitorea continuamente el keypad cuando el sistema está
 * en estado BLOQUEO, buscando específicamente la tecla '*'
 * para desbloquear el sistema y retornar a INICIO.
 * 
 * @security Función crítica para recuperación del sistema
 * 
 * @see outputBloqueo()
 * @see onLeavingBloqueo()
 */
void checkBloqueo() {
  if (stateMachine.GetState() == BLOQUEO) {
    char key = keypad.getKey();
    if (key == '*') {
      input = TECLA_ASTERISCO;
    }
  }
}

// ============================================================================
// SISTEMA DE MONITOREO - SENSORES
// ============================================================================

/**
 * @brief Lee y procesa datos de todos los sensores del sistema
 * 
 * @details
 * Realiza lecturas de:
 * - Temperatura ambiente (DHT11) [°C] - Precisión ±1°C
 * - Humedad relativa (DHT11) [%] - Precisión ±2%
 * - Luminosidad ambiente (LDR) [%] - Rango 0-100%
 * - Temperatura radiante (NTC) [°C] - Usando ecuación Steinhart-Hart
 * 
 * @post Actualiza variables globales tempA, humedad, luz, tempR
 * @validation Verifica lecturas válidas del sensor DHT11
 * 
 * @see tempA
 * @see humedad
 * @see luz
 * @see tempR
 */
void leerSensores() {
  // Lectura de sensor DHT11 (temperatura y humedad)
  tempA = dht.readTemperature();  // °C
  humedad = dht.readHumidity();   // %
  
  // Lectura de sensor LDR (luminosidad) con mapeo a porcentaje
  int rawLuz = analogRead(LDR_PIN);
  luz = map(rawLuz, 0, 1032, 0, 100);  // % aproximado de iluminación
  
  // Lectura de sensor NTC (temperatura radiante) usando ecuación Steinhart-Hart
  int analogValue = analogRead(TEMP_PIN);
  tempR = 1 / (log(1 / (1023. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;  // °C

  // Validación de lecturas del sensor DHT11
  if (isnan(humedad) || isnan(tempA)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
}

// ============================================================================
// MÓDULO DE CÁLCULO PMV - MODELO FANGER
// ============================================================================

/** @defgroup pmv_grupo Cálculo PMV según Modelo de Fanger */
///@{

/**
 * @brief Calcula la presión parcial de vapor de agua en el aire
 * 
 * @param ta Temperatura del aire [°C]
 * @param rh Humedad relativa [%]
 * @return float Presión parcial de vapor [Pa]
 * 
 * @details
 * Calcula usando la fórmula de Magnus-Tetens:
 * pa = (rh/100) * exp(16.6536 - 4030.183/(ta + 235))
 * 
 * @equation $$ p_a = \\frac{rh}{100} \\times e^{\\left(16.6536 - \\frac{4030.183}{t_a + 235}\\right)} $$
 * 
 * @reference ASHRAE Fundamentals 2001
 */
float calcularPresionVapor(float ta, float rh) {
  float pa = (rh / 100.0) * exp(16.6536 - 4030.183 / (ta + 235.0));
  return pa;
}

/**
 * @brief Calcula el coeficiente de transferencia de calor por convección
 * 
 * @param tcl Temperatura de superficie de la ropa [°C]
 * @param ta Temperatura del aire [°C]
 * @param vel_ar Velocidad relativa del aire [m/s]
 * @return float Coeficiente de convección hc [W/(m²·K)]
 * 
 * @details
 * Calcula ambos mecanismos y selecciona el mayor:
 * - Convección natural: hc_nat = 2.38 * |tcl - ta|^0.25
 * - Convección forzada: hc_for = 12.1 * √(vel_ar)
 * 
 * @equation $$ h_c = \\max\\left(2.38 \\times |t_{cl} - t_a|^{0.25},\\ 12.1 \\times \\sqrt{v_{ar}}\\right) $$
 * 
 * @reference Fanger (1970) - Thermal Comfort
 */
float calcularHc(float tcl, float ta, float vel_ar) {
  float hc_natural = 2.38 * pow(fabs(tcl - ta), 0.25);
  float hc_forzada = 12.1 * sqrt(vel_ar);
  
  float hc = (hc_natural > hc_forzada) ? hc_natural : hc_forzada;
  return hc;
}

/**
 * @brief Calcula el factor de superficie de la ropa
 * 
 * @param icl Aislamiento térmico de la ropa [m²K/W]
 * @return float Factor de superficie fcl [adimensional]
 * 
 * @details
 * - Para icl ≤ 0.078: fcl = 1.00 + 1.290 * icl
 * - Para icl > 0.078: fcl = 1.05 + 0.645 * icl
 * 
 * @reference ISO 7730:2005 - Annex D
 */
float calcularFcl(float icl) {
  float fcl;
  if (icl <= 0.078) {
    fcl = 1.00 + 1.290 * icl;
  } else {
    fcl = 1.05 + 0.645 * icl;
  }
  return fcl;
}

/**
 * @brief Calcula el índice PMV (Predicted Mean Vote) usando modelo de Fanger
 * 
 * @param ta Temperatura del aire [°C]
 * @param tr Temperatura radiante media [°C]
 * @param rh Humedad relativa [%]
 * @param vel_ar Velocidad del aire [m/s]
 * @param M Tasa metabólica [W/m²]
 * @param clo Aislamiento térmico de la ropa [clo]
 * @return float Índice PMV en escala de -3 a +3
 * 
 * @details
 * Implementa el modelo completo de Fanger para cálculo de confort térmico:
 * - -3: Muy frío | -2: Frío | -1: Ligeramente frío
 * - 0: Neutral (confort óptimo)
 * - +1: Ligeramente cálido | +2: Cálido | +3: Muy cálido
 * 
 * @algorithm
 * 1. Conversión de unidades y constantes base
 * 2. Iteración para hallar temperatura de superficie de ropa (tcl)
 * 3. Cálculo de pérdidas de calor (radiación, convección, respiración)
 * 4. Cálculo final del índice PMV
 * 
 * @equation PMV = (0.303 × e^{-0.036M} + 0.028) × (M - W - H)
 * 
 * @warning Resultado limitado al rango [-3, +3] según estándar ISO 7730
 * 
 * @reference Fanger, P. O. (1970). Thermal Comfort. Danish Technical Press.
 * @reference ISO 7730:2005 - Ergonomics of the thermal environment
 */
float calcularPMV_Fanger(float ta, float tr, float rh, float vel_ar, float M, float clo) {
  // Conversión de unidades y constantes base
  float pa, icl, V, fcl, hc, pmv;
  float tol = 0.0001;      // Tolerancia para iteraciones
  float tcl = ta;          // Temperatura inicial de la superficie de la ropa
  float tcl_prev;
  int iteracion = 0;
  int max_iters = 100;

  // Constantes del modelo
  V = 0.0;                            // Trabajo mecánico (reposo) [W/m²]
  icl = clo * 0.155;                  // Conversión clo a m²K/W
  pa = calcularPresionVapor(ta, rh);  // Presión de vapor [Pa]

  // Factor de superficie de la ropa
  fcl = calcularFcl(icl);

  // ------------ Iteración para hallar tcl ------------
  do {
    tcl_prev = tcl;

    // Calcular coeficiente de convección
    hc = calcularHc(tcl, ta, vel_ar);

    // Conversión a Kelvin para fórmulas radiativas
    float tr_rad = tr + 273.0;
    float tcl_rad = tcl + 273.0;

    // Cálculo de transferencia de calor radiante
    float radiation = (3.96 * pow(10, -8)) * fcl * (pow(tcl_rad, 4) - pow(tr_rad, 4));
    // Cálculo de transferencia de calor por convección
    float convection = fcl * hc * (tcl - ta);

    // Ecuación de balance térmico para tcl
    tcl = 35.7 - 0.028 * (M - V) - icl * (radiation - convection);

    iteracion++;
  } while (fabs(tcl - tcl_prev) > tol && iteracion < max_iters);

  // Cálculo final de coeficiente de convección
  float hc_final = calcularHc(tcl, ta, vel_ar);

  // -------------- CÁLCULO FINAL DEL PMV --------------
  // Conversión a Kelvin
  float tr_rad = tr + 273.0;
  float tcl_rad = tcl + 273.0;

  // Cálculo de todas las pérdidas de calor
  float radiation = (3.96e-8) * fcl * (pow(tcl_rad, 4) - pow(tr_rad, 4));
  float convection = fcl * hc_final * (tcl - ta);
  float respiration = 0.0014 * M * (34.0 - ta);
  float latent = (1.7e-5) * M * (5867.0 - pa);
  float heat_loss = latent + respiration + radiation + convection;

  // Ecuación final del PMV
  pmv = (0.303 * exp(-0.036 * M) + 0.028) * (
    (M - V)
    - (3.05e-3) * (5733.0 - 6.99 * (M - V) - pa)
    - 0.42 * ((M - V) - 58.15)
    - heat_loss
  );

  // Limitar resultado al rango estándar [-3, +3]
  return constrain(pmv, -3.0, 3.0);
}

///@}

// ============================================================================
// SISTEMA DE ACTUADORES - CONTROL
// ============================================================================

/** @defgroup actuadores_grupo Sistema de Control de Actuadores */
///@{

/**
 * @brief Controla el movimiento continuo del servomotor
 * 
 * @details
 * Genera movimiento oscilatorio suave del servomotor entre 20° y 90°:
 * - Movimiento incremental de 20° a 90° (apertura)
 * - Movimiento decremental de 90° a 20° (cierre)
 * - Control asíncrono con intervalo de 15ms para movimiento fluido
 * - Utilizado para simular control de persianas o ventilación
 * 
 * @usage Estado PMV_BAJO para cerrar persianas y conservar calor
 */
void moverServo() {
  if (servoUp) {
    servoPos++;
    if (servoPos >= 90) {
      servoPos = 90;
      servoUp = false;
    }
  } else {
    servoPos--;
    if (servoPos <= 20) {
      servoPos = 20;
      servoUp = true;
    }
  }
  servo.write(servoPos);
}

/**
 * @brief Genera patrón de sonido intermitente para el buzzer
 * 
 * @details
 * Alterna el estado del buzzer cada 200ms creando un patrón
 * de sonido de alerta. Utilizado en estado ALARMA para
 * notificación audible de condiciones críticas.
 * 
 * @usage Estado ALARMA para alertas sonoras
 */
void toggleBuzzer() {
  buzzerState = !buzzerState;
  digitalWrite(BUZZER_PIN, buzzerState);
}

///@}

// ============================================================================
// FUNCIONES DE VERIFICACIÓN EN BUCLE PRINCIPAL
// ============================================================================

/** @defgroup verificacion_grupo Funciones de Verificación en Loop */
///@{

/**
 * @brief Verifica cálculo PMV y determina transiciones desde estado MONITOR
 * 
 * @details
 * Ejecutado en cada iteración del loop cuando el sistema está en estado MONITOR
 * y el tiempo de monitoreo ha vencido:
 * - Lee sensores y calcula índice PMV
 * - Muestra resultado PMV en LCD y Serial
 * - Determina transición a PMV_BAJO, PMV_ALTO o CONFIG según resultado
 * - Reinicia temporizador para próximo ciclo de monitoreo
 * 
 * @condition stateMachine.GetState() == MONITOR && tiempoMonitorVencido == true
 * 
 * @see outputMonitor()
 * @see leerSensores()
 * @see calcularPMV_Fanger()
 */
void checkPMV() {
  if (stateMachine.GetState() == MONITOR && tiempoMonitorVencido) {
    tiempoMonitorVencido = false;
    leerSensores();

    // Parámetros para cálculo PMV
    float M = 100;    // metabolismo - actividad ligera [W/m²]
    float clo = 0.5;  // vestimenta - ropa ligera [clo]
    float vel_ar = 0.1; // velocidad del aire [m/s]
    
    // Cálculo del índice PMV
    float pmv = calcularPMV_Fanger(tempA, tempR, humedad, vel_ar, M, clo);
    
    Serial.print("PMV calculado: ");
    Serial.println(pmv, 2);
    
    // Mostrar resultado en LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PMV: ");
    lcd.print(pmv, 2);
    
    // Determinar transición según valor PMV
    if (pmv < -1)
      input = PMV_MENOR_QUE_MENOS1;
    else if (pmv > 1)
      input = PMV_MAYOR_QUE_1;
    else
      input = TIEMPO_EXPIRADO;
      
    // Reinicia el timer para el siguiente cálculo
    TaskTime.Start();
  }
}

/**
 * @brief Detecta movimiento con sensor PIR para desactivar alarma
 * 
 * @details
 * Ejecutado en cada iteración del loop cuando el sistema está en estado ALARMA:
 * - Lee estado del sensor PIR (movimiento detectado = LOW)
 * - Al detectar movimiento, activa transición a INICIO
 * - Permite reset manual del sistema mediante detección de presencia
 * 
 * @condition stateMachine.GetState() == ALARMA
 * 
 * @see outputAlarma()
 */
void checkMovimiento() {
  if (stateMachine.GetState() == ALARMA) {
    int pirValue = digitalRead(PIR_SENSOR);
    if (pirValue == LOW) {
      Serial.println("Movimiento detectado -> Regresando a INICIO");
      input = SENSOR_INFRARROJO;
    }
  }
}

///@}