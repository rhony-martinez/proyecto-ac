# 🧠 Proyecto de Arquitectura Computacional

## 📋 Descripción general

**Diseñar e implementar un sistema automatizado de control ambiental y seguridad basado en una máquina de estados finitos (FSM)**, utilizando **Arduino Mega** y diversos sensores y actuadores.  
El sistema permite monitorear variables físicas del entorno (temperatura, humedad, luminosidad y presencia), calcular el índice de confort térmico (PMV) y ejecutar acciones de control —como activación de ventilación, servomecanismos, alarmas y señales luminosas— garantizando una operación segura y eficiente.

---

## 🧩 Descripción del sistema

Este proyecto implementa un **sistema de monitoreo y control térmico-ambiental** basado en **Arduino Mega 2560**, diseñado bajo una **máquina de estados finitos (FSM)** para gestionar el comportamiento del sistema de forma estructurada y modular.

El sistema supervisa variables ambientales como:
- Temperatura del aire  
- Humedad relativa  
- Temperatura radiante  
- Nivel de iluminación  

A partir de estas mediciones calcula el **índice PMV (Predicted Mean Vote)** según el modelo de Fanger, estimando el nivel de confort térmico en un entorno determinado.

En función de las condiciones detectadas, el sistema activa actuadores (ventilador, servomotor, LEDs y buzzer) y **transita automáticamente entre estados** según reglas definidas en la FSM.

---

## ⚙️ Estados principales

| Estado | Descripción |
|--------|--------------|
| **INICIO** | Estado base del sistema. Permite el ingreso de una clave mediante el teclado para habilitar el funcionamiento. |
| **CONFIG** | Define parámetros iniciales y tiempos de monitoreo. |
| **MONITOR** | Lee los sensores, calcula el PMV, y muestra los valores en el LCD. Según el PMV, transita a PMV_BAJO o PMV_ALTO. |
| **PMV_BAJO** | Representa condiciones frías. Activa el servomotor (calefactor) y LED verde. |
| **PMV_ALTO** | Representa condiciones calurosas. Activa el ventilador (relay) y LED azul. Si T° > 30 °C tres veces seguidas, genera una alarma. |
| **ALARMA** | Estado de advertencia. Activa el buzzer intermitente. Retorna a INICIO al detectar movimiento (PIR). |
| **BLOQUEO** | Estado de seguridad. LED rojo parpadeante. Se desbloquea al presionar `*`. |

---

## 🔌 Componentes principales

### 🧭 Hardware
- Arduino Mega 2560  
- Lector y Tag RFID (MFRC522)  
- LCD 16x2  
- Potenciómetro  
- Protoboard y cableado  
- DHT11 – Sensor de temperatura y humedad  
- Sensor de temperatura analógico (termistor NTC)  
- LDR – Sensor de luz  
- Sensor infrarrojo PIR HC-SR501  
- Keypad matricial  
- LED RGB (rojo, verde, azul)  
- Buzzer activo  
- Ventilador (relay)  
- Servomotor  
- Fuente de poder 5 V  

### 📚 Librerías utilizadas
```cpp
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
```

---

## 🧠 Características técnicas

- Control estructurado mediante **StateMachineLib** (FSM).  
- Ejecución **asíncrona y no bloqueante** con AsyncTaskLib.  
- Cálculo dinámico del **índice PMV**.  
- **Visualización en LCD** y registro en monitor serial.  
- Arquitectura **modular y escalable**.  
- Seguridad integrada con **bloqueo automático** por intentos fallidos.

---

## 🗂️ Estructura del repositorio
```
├── main/
│ ├── proyecto/
│ │ └── proyecto.ino
│ ├── FSM_ARQ_A.png
│ ├── Definicion.docx
│ └── README.md
│
├── develop/
│ ├── proyecto/
│ │ └── proyecto.ino
│ ├── proyectoAC/
│ │ └── proyectoAC.ino
│ ├── FSM_ARQ_A.png
│ ├── Definicion.docx
│ └── README.md
│
├── feature/RM/
│ ├── proyecto/
│ │ └── proyecto.ino
│ ├── proyectoAC/
│ │ └── proyectoAC.ino
│ ├── FSM_ARQ_A.png
│ ├── Definicion.docx
│ └── README.md
│
├── feature/PIPE/
│ ├── proyecto/
│ │ └── proyecto.ino
│ ├── proyectoAC/
│ │ └── proyectoAC.ino
│ ├── FSM_ARQ_A.png
│ ├── Definicion.docx
│ └── README.md
```

---

## 👥 Colaboradores

**Elaborado por:**
- Jonatan David Bolaños Jojoa  
- Rhony Daniel Martínez Benavides  
- Juan Esteban Moscoso Salazar  
- Andrés Felipe Obando Quintero  
- Daniel Alejandro Vidal Guevara  

**Asignatura:** Arquitectura Computacional  
**Profesor:** Fulvio Yesid Vivas  

---

## 🔄 Flujo de trabajo con Git

El flujo de trabajo se basó en la metodología **Feature Branch Workflow**:

1. Cada integrante desarrolló su versión en una rama `feature/` individual.  
2. Las ramas se fusionaron en la rama `develop` para pruebas e integración.  
3. Finalmente, `develop` se fusionó con `main` como rama estable del proyecto.  

---

## ⚙️ Ejecución y pruebas

### 🧰 Requisitos de hardware

| Componente | Descripción | Pin principal |
|-------------|--------------|----------------|
| DHT11 | Sensor de temperatura y humedad | 41 |
| LDR | Sensor de iluminación | A3 |
| Termistor (NTC) | Temperatura radiante | A0 |
| Sensor PIR | Detección de movimiento | A5 |
| Relay + Ventilador | Control de ventilación | 38 |
| Servomotor | Simulación de calefactor | 13 |
| Buzzer activo | Alarma sonora | 7 |
| LED rojo | Estado BLOQUEO | 8 |
| LED verde | Estado PMV_BAJO | 9 |
| LED azul | Estado PMV_ALTO | 10 |
| LCD 16x2 | Interfaz de visualización | Según módulo |
| Keypad | Ingreso de clave | Según sketch |

### 💻 Entorno de desarrollo

- **Plataforma:** Arduino IDE 2.0 o superior  
- **Placa:** Arduino Mega 2560  
- **Velocidad serial:** 9600 bps  
- **Librerías:** instalar desde el Administrador de Librerías

### ▶️ Ejecución

1. Conecta los sensores y actuadores según la tabla de pines.  
2. Carga `proyecto.ino` desde el IDE de Arduino.  
3. Abre el **Monitor Serial** a 9600 bps.  
4. Alimenta el sistema (USB o fuente externa 5 V).  
5. Observa el estado del sistema y lecturas en el LCD.

---

## 🔍 Pruebas y comportamiento esperado

Durante la ejecución, el sistema transita automáticamente entre los estados:

- **INICIO:** mensaje de bienvenida, ingreso de clave.  
- **CONFIG:** tiempo de inicialización antes del monitoreo y lectura de RFIDs.  
- **MONITOR:** muestra variables y calcula PMV.  
- **PMV_BAJO / PMV_ALTO:** activa actuadores según confort.  
- **ALARMA:** buzzer intermitente ante sobrecalentamiento.  
- **BLOQUEO:** Mensaje hasta presionar `*`.

Los cambios de estado se reflejan tanto en el **LCD** como en el **Monitor Serial**, donde se marca con una `X` el estado activo.  

El comportamiento del PMV puede verificarse modificando temperatura o humedad y observando las transiciones automáticas.

---

## 🧾 Créditos

Proyecto académico presentado en el curso **Arquitectura Computacional**.  
**Profesor:** Fulvio Yesid Vivas  

---

## 📜 Licencia

Este proyecto se distribuye con fines educativos y de libre acceso.  
© 2025 — Todos los derechos reservados a los autores.

---

## 🔗 Enlaces adicionales
*(Se agregarán próximamente los enlaces a documentación, diagramas y videos de demostración.)*
