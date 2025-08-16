## StepperCAN

StepperCAN es una librería portable en C que permite controlar de forma independiente hasta 5 motores de pasos,
mediante señales STEP/DIR a través de comandos CAN/UART. Es posible integrarla en entornos de desarrollo profesionales,
como STM32cubeIDE, MPLAB, MCUXpresso IDE, CCS, Keil, VS Code+SDKs, entre otros.
 
### Características generales:

* Hasta 5 motores  (ampliable)
* Modos de funcionamiento independientes por motor:

	* Velocidad constante con rampa de aceleración
	* Velocidad constante con rampa de aceleración por tiempo
	* Posición con perfil trapezoidal a velocidad límite
	* Posición con perfil trapezoidal durante tiempo (punto a punto únicamente)
	* Posición sincronizada para dos o más motores a velocidad límite (punto a punto únicamente)
	
* Aceleración y desaceleración independientes
* Protocolo de comandos por CAN/UART con respuestas y telemetría en tiempo real en CAN.
* Comunicación UART unidireccional para configuración básica de drivers TMC2209
* Portable, optimizado para redimiento, no usa punto flotante
##

La generación de las señales STEP está basada en una ISR maestro de período fijo con la técnica de acumulación de fase.
Se puede integrar en cualquier microcontrolador de 32 bits y asignar cualquier GPIO como señal.

Es importante tener en cuenta que este método tiene limitaciones, tales como:
* No permite sincronización exacta entre múltiples motores
* Genera cierto grado de *jitter*  a velocidades altas
* Limitado a frencuencias medias, por lo general no más de 100KHz en un ARM M0.
* Ocasiona una carga de CPU elevada por cada motor adicional

Esta librería está diseñada como una solución de bajo nivel en entornos donde el rendimiento y la personalización son críticos, por lo que si necesitas mover motores de pasos
sin complicaciones por favor considera Arduino con [AccelStepper](https://github.com/swissbyte/AccelStepper). Encontrarás muchos tutoriales en blogs y YouTube.

Puedes consultar más información y ejemplos de uso en este [blog](https://www.printabot.es/2025/08/15/steppercan/)