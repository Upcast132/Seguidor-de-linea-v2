# Seguidor de Línea Autónomo Diferencial con Control PID y Auto-Calibración

<p align="center">
  <img src="https://img.shields.io/badge/Arduino-Nano-00979D?style=flat-square&logo=arduino&logoColor=white" alt="Arduino Nano" />
  <img src="https://img.shields.io/badge/C%2B%2B-AVR-00599C?style=flat-square&logo=cplusplus&logoColor=white" alt="C++" />
  <img src="https://img.shields.io/badge/Control-PID-orange?style=flat-square" alt="Control PID" />
  <img src="https://img.shields.io/badge/Sensores-TCRT5000-blueviolet?style=flat-square" alt="Sensores TCRT5000" />
  <img src="https://img.shields.io/badge/Driver-TB6612FNG-success?style=flat-square" alt="Driver TB6612FNG" />
  <img src="https://img.shields.io/badge/Licencia-MIT-blue?style=flat-square" alt="Licencia MIT" />
</p>
<p align="center">
  
  <em>Prototipo de robot seguidor de línea con control PID y calibración automática.</em>
</p>

> Robot móvil de tracción diferencial basado en **Arduino Nano**, con **4 sensores infrarrojos TCRT5000**, driver de motores **TB6612FNG** y control **PID**. Incluye una rutina de **auto-calibración dinámica** para adaptarse a cambios de iluminación, distancia al suelo y tolerancias de los sensores.

---

## Tabla de contenido

- [Resumen](#resumen)
- [Características principales](#características-principales)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Hardware](#hardware)
  - [Lista de materiales](#lista-de-materiales)
  - [Alimentación recomendada](#alimentación-recomendada)
- [Esquema de conexiones](#esquema-de-conexiones)
- [Diseño mecánico](#diseño-mecánico)
- [Software](#software)
  - [Estructura sugerida del repositorio](#estructura-sugerida-del-repositorio)
  - [Instalación y carga del firmware](#instalación-y-carga-del-firmware)
  - [Parámetros configurables](#parámetros-configurables)
- [Funcionamiento](#funcionamiento)
  - [Calibración dinámica Auto-swipe](#calibración-dinámica-auto-swipe)
  - [Normalización de lecturas](#normalización-de-lecturas)
  - [Cálculo de posición](#cálculo-de-posición)
  - [Control PID](#control-pid)
  - [Recuperación de línea perdida](#recuperación-de-línea-perdida)
- [Guía de puesta en marcha](#guía-de-puesta-en-marcha)
- [Ajuste del control PID](#ajuste-del-control-pid)
- [Diagnóstico rápido](#diagnóstico-rápido)
- [Roadmap](#roadmap)
- [Contribuciones](#contribuciones)
- [Licencia](#licencia)

---

## Resumen

Este proyecto consiste en el diseño, ensamblaje y programación de un robot móvil de **tracción diferencial** para seguimiento de trayectorias sobre una pista, típicamente una **línea negra sobre fondo blanco** o una configuración equivalente si se adapta la lógica de reflectancia.

El núcleo del sistema es un **Arduino Nano**, que lee las señales analógicas de un arreglo de **4 sensores infrarrojos TCRT5000**, calcula la posición estimada de la línea mediante un **promedio ponderado** y aplica una ley de control **PID** para gobernar dos motorreductores DC a través de un puente H **TB6612FNG**.

Una de las características principales del firmware es la rutina de **calibración dinámica**, también llamada **Auto-swipe**, que realiza un barrido físico al inicio para obtener los valores mínimos y máximos de reflectancia de cada sensor. Esto permite que el robot se adapte automáticamente a variaciones de iluminación ambiental, altura de los sensores, superficie de la pista y tolerancias entre componentes.

---

## Características principales

- **Control PID** para seguimiento suave y estable de la línea.
- **4 sensores TCRT5000** leídos por entradas analógicas, no solo como sensores digitales.
- **Auto-calibración al arranque** mediante barrido físico sobre la línea.
- **Normalización de lecturas** para compensar iluminación y diferencias entre sensores.
- **Estimación de posición continua** mediante promedio ponderado.
- **Recuperación de línea perdida** usando memoria del último error conocido.
- **Driver TB6612FNG**, más eficiente que soluciones clásicas como el L298N.
- **Diagnóstico por monitor serie** para depurar calibración y contraste de sensores.
- **Arquitectura simple**, sin dependencias externas complejas.

---

## Arquitectura del sistema

```text
[4x TCRT5000]
     |
     v
[Arduino Nano - ADC]
     |
     v
[Normalización de lecturas]
     |
     v
[Promedio ponderado / Posición de línea]
     |
     v
[Cálculo de error]
     |
     v
[Controlador PID]
     |
     v
[Mezcla diferencial de motores]
     |
     v
[TB6612FNG]
     |
     v
[Motorreductores DC]
```

El sistema funciona en lazo cerrado: los sensores detectan la posición de la línea, el firmware calcula el error respecto al punto deseado y el controlador PID ajusta la velocidad de cada motor para corregir la trayectoria.

---

## Hardware

### Lista de materiales

| Cantidad | Componente | Función | Notas |
|---:|---|---|---|
| 1 | Arduino Nano | Microcontrolador principal | ATmega328P, suficiente cantidad de pines ADC/PWM. |
| 1 | Driver TB6612FNG | Control de motores DC | Tecnología MOSFET, menor caída de tensión y mejor eficiencia. |
| 4 | Sensor TCRT5000 | Detección de reflectancia | Se recomienda usar la salida analógica. |
| 2 | Motorreductor DC tipo TT | Tracción del robot | Incluyen ruedas acopladas. |
| 2 | Ruedas para motor TT | Tracción | Deben quedar bien ajustadas. |
| 1 | Batería LiPo 2S 7.4 V | Alimentación de motores | Recomendada para el pin `VMOT` del driver. |
| 1 | Regulador 5 V / alimentación lógica | Alimentación del Arduino | Puede ser USB durante pruebas o un regulador/BEC en uso autónomo. |
| 1 | Chasis | Soporte estructural | Debe permitir montar la barra de sensores con voladizo frontal. |
| - | Cables, conectores, tornillería | Integración eléctrica/mecánica | Se recomienda cableado flexible y conexiones firmes. |

> El **TB6612FNG** es preferible frente a drivers como el **L298N** porque utiliza transistores MOSFET, reduce la caída de tensión, genera menos calor y aprovecha mejor la energía de la batería.

---

### Alimentación recomendada

| Riel / Pin | Voltaje recomendado | Uso |
|---|---:|---|
| `VCC` del TB6612 | 5 V | Lógica del driver. |
| `VMOT` del TB6612 | 6 V a 9 V | Alimentación de motores. |
| Arduino Nano | 5 V por USB o entrada adecuada | Microcontrolador. |
| Batería sugerida | LiPo 2S, 7.4 V nominal | Motores. |

> ⚠️ **Importante:** mantener separadas las fuentes de potencia y lógica siempre que sea posible. La tierra de potencia de la batería y la tierra de la lógica deben estar unidas en **un solo punto común de GND**.

Recomendaciones adicionales:

- Colocar un capacitor electrolítico, por ejemplo de `220 µF` a `470 µF`, cerca de `VMOT` y `GND` del driver.
- Añadir capacitores cerámicos de desacoplo si hay ruido eléctrico.
- No alimentar los motores directamente desde los pines de 5 V del Arduino.
- Usar una batería LiPo con cargador adecuado y precauciones de seguridad.

---

## Esquema de conexiones

> ⚠️ **Advertencia de potencia:** la lógica a 5 V y la potencia de los motores deben compartir únicamente un punto de tierra común. No conectar la batería de motores directamente a la alimentación lógica de 5 V.

### Sensores TCRT5000

| Sensor | Pin del módulo | Pin Arduino | Tipo | Descripción |
|---|---|---:|---|---|
| Sensor izquierdo exterior | `AO` | `A0` | Entrada analógica | Lectura ADC de reflectancia. |
| Sensor izquierdo central | `AO` | `A1` | Entrada analógica | Lectura ADC de reflectancia. |
| Sensor derecho central | `AO` | `A2` | Entrada analógica | Lectura ADC de reflectancia. |
| Sensor derecho exterior | `AO` | `A3` | Entrada analógica | Lectura ADC de reflectancia. |

Cada módulo TCRT5000 debe alimentarse con `VCC` y `GND` según su hoja de datos o marcado del módulo.

---

### Driver TB6612FNG

#### Alimentación y habilitación

| Pin TB6612 | Conexión | Descripción |
|---|---|---|
| `VMOT` | Batería + | Alimentación de motores. |
| `GND` | GND común | Tierra de potencia y lógica. |
| `VCC` | 5 V | Alimentación lógica del driver. |
| `STBY` | `D9` o `5V` | `HIGH` habilita el driver. Si se usa un pin, debe ponerse en `HIGH`. |

#### Control de motores

| Motor | Pin TB6612 | Pin Arduino | Tipo | Descripción |
|---|---|---:|---|---|
| Motor A, izquierdo | `PWMA` | `D5` | PWM | Control de velocidad. |
| Motor A, izquierdo | `AIN1` | `D4` | Digital | Dirección del motor A. |
| Motor A, izquierdo | `AIN2` | `D3` | Digital | Dirección del motor A. |
| Motor B, derecho | `PWMB` | `D6` | PWM | Control de velocidad. |
| Motor B, derecho | `BIN1` | `D8` | Digital | Dirección del motor B. |
| Motor B, derecho | `BIN2` | `D7` | Digital | Dirección del motor B. |

> Si el robot gira en sentido contrario al esperado, puede intercambiarse la polaridad de uno de los motores o invertir los pines de dirección correspondientes en el firmware.

---

## Diseño mecánico

Para que el firmware funcione correctamente, el chasis debe cumplir algunas condiciones mecánicas importantes.

### 1. Altura de los sensores

Los sensores **TCRT5000** deben trabajar dentro de su rango óptimo de detección.

- Altura recomendada: **3 mm a 8 mm** sobre la pista.
- Si están demasiado cerca: puede haber ruido, rozamiento o sensibilidad al polvo.
- Si están demasiado lejos: disminuye el contraste entre línea y fondo.

### 2. Voladizo frontal de la barra de sensores

La rutina de calibración **Auto-swipe** hace que el robot gire sobre su propio centro para barrer la línea con todos los sensores.

Por ello, la barra de sensores **debe estar ligeramente adelantada respecto al eje de las ruedas**.

Si los sensores quedan exactamente sobre el eje de giro:

- El barrido puede no cruzar la línea correctamente.
- Los sensores centrales pueden no detectar contraste suficiente.
- La calibración puede fallar o ser incompleta.

### 3. Montaje de motores TT

Los motorreductores tipo TT tienen el eje descentrado y suelen requerir fijación mediante tornillos en la cara plana del cuerpo.

Esto difiere de otros motores, como algunos N20, que pueden usar clips o soportes diferentes. El chasis debe adaptarse a la geometría de estos motores para evitar vibraciones, desalineaciones o pérdida de tracción.

---

## Software

El firmware está escrito en **C++ para Arduino** y se organiza alrededor de tres subsistemas principales:

1. **Calibración dinámica de sensores.**
2. **Procesamiento de señal y cálculo de posición.**
3. **Control PID y gestión de motores.**

---

### Estructura sugerida del repositorio

```text
.
├── docs/
│   ├── images/
│   │   └── robot.png
│   └── diagrams/
├── firmware/
│   └── line_follower/
│       └── line_follower.ino
├── hardware/
│   ├── bom.md
│   └── cad/
├── .gitignore
├── LICENSE
└── README.md
```

---

### Instalación y carga del firmware

#### Requisitos

- Arduino IDE o PlatformIO.
- Arduino Nano compatible.
- Cable USB compatible con Arduino Nano.
- Drivers USB si el convertidor serial lo requiere.

#### Pasos

1. Clonar el repositorio:

```bash
git clone https://github.com/usuario/seguidor-linea-pid.git
cd seguidor-linea-pid
```

2. Abrir el firmware en Arduino IDE:

```text
firmware/line_follower/line_follower.ino
```

3. Conectar el Arduino Nano al ordenador por USB.

4. Seleccionar la placa correcta:

```text
Herramientas > Placa > Arduino Nano
```

5. Seleccionar el puerto serie correspondiente.

6. Subir el firmware.

7. Abrir el monitor serie a:

```text
115200 baudios
```

---

### Parámetros configurables

Estos son algunos de los parámetros típicos del firmware. Los nombres pueden variar según la implementación final.

| Parámetro | Descripción | Recomendación |
|---|---|---|
| `VEL_CALIBRACION` | Velocidad PWM durante el barrido de calibración. | Debe ser suficiente para pivotar sin que el robot salte de la línea. |
| `PERIODO_PIVOTE` | Tiempo entre cambios de giro durante la calibración. | Un valor típico puede estar alrededor de `300 ms` a `500 ms`. |
| `SETPOINT` | Posición deseada de la línea. | `1500` si el rango de posición es `0` a `3000`. |
| `UMBRAL_CONTRASTE` | Diferencia mínima entre `minVal` y `maxVal`. | `30` como referencia inicial. |
| `Kp` | Ganancia proporcional. | Ajustar experimentalmente. |
| `Ki` | Ganancia integral. | Usar valores pequeños. |
| `Kd` | Ganancia derivativa. | Útil para reducir oscilaciones. |

---

## Funcionamiento

### Calibración dinámica Auto-swipe

Al encender el robot, el sistema no utiliza umbrales fijos predefinidos. En su lugar ejecuta una rutina de calibración física:

1. El robot gira sobre su propio eje.
2. Alterna el sentido de giro cada cierto tiempo, por ejemplo cada `400 ms`.
3. Durante el barrido, cada sensor registra:
   - `minVal`: valor asociado a la superficie más clara.
   - `maxVal`: valor asociado a la superficie más oscura.
4. Con esos límites, cada sensor queda normalizado individualmente.

Esto permite compensar:

- Diferencias de fabricación entre sensores.
- Cambios de iluminación ambiental.
- Pequeñas variaciones de altura.
- Diferencias de color o reflectancia de la pista.

---

### Normalización de lecturas

Cada lectura cruda del ADC se normaliza usando los valores obtenidos en calibración.

Conceptualmente:

```text
valor_normalizado = map(lectura_cruda, minVal, maxVal, 0, 1000)
```

De esta forma, cada sensor entrega una escala aproximada de `0` a `1000`, independientemente de su nivel absoluto de reflectancia.

---

### Cálculo de posición

Para estimar la posición de la línea se utiliza un **promedio ponderado**.

Cada sensor se asocia a un peso posicional:

| Sensor | Peso |
|---|---:|
| Sensor 0, izquierdo exterior | `0` |
| Sensor 1, izquierdo central | `1000` |
| Sensor 2, derecho central | `2000` |
| Sensor 3, derecho exterior | `3000` |

La posición resultante varía entre:

```text
0     -> línea muy a la izquierda
1500  -> línea centrada
3000  -> línea muy a la derecha
```

El valor objetivo o **setpoint** es:

```text
SETPOINT = 1500
```

---

### Control PID

El error se calcula como:

```text
error = SETPOINT - posicion_actual
```

El controlador PID genera una señal correctiva a partir de tres componentes:

| Componente | Acción |
|---|---|
| **P** | Corrige el error actual. Cuanto mayor es el error, mayor es la corrección. |
| **I** | Corrige errores acumulados en el tiempo. Útil para sesgos mecánicos o motores ligeramente distintos. |
| **D** | Amortigua cambios bruscos y ayuda a reducir oscilaciones. |

La salida del controlador se mezcla sobre las señales PWM de los motores para producir una acción diferencial:

- Si la línea está desplazada a la izquierda, el robot gira hacia la izquierda.
- Si la línea está desplazada a la derecha, el robot gira hacia la derecha.
- Si la línea está centrada, el robot avanza lo más recto posible.

---

### Recuperación de línea perdida

Si los cuatro sensores detectan una condición equivalente a pérdida de línea, el robot no se queda detenido indefinidamente.

El firmware utiliza el signo del último error conocido para decidir hacia qué lado buscar la línea:

- Si la línea se perdió hacia la izquierda, gira hacia la izquierda.
- Si la línea se perdió hacia la derecha, gira hacia la derecha.

Esto permite recuperar la trayectoria sin intervención externa en muchos casos.

---

## Guía de puesta en marcha

### 1. Colocar el robot sobre la línea

Antes de encender el robot, la barra de sensores debe estar **centrada transversalmente sobre la línea**.

Esto es crítico para que la rutina de calibración pueda cruzar correctamente los bordes de la línea en ambos sentidos.

### 2. Alimentar el sistema

Encender el robot con la batería o fuente de alimentación adecuada.

El firmware iniciará automáticamente la rutina de calibración.

### 3. Calibración

El robot comenzará a pivotar alternadamente.

Durante esta fase:

- Los sensores deben cruzar la línea.
- El robot debe permanecer sobre la pista.
- No se debe mover manualmente.

### 4. Diagnóstico por monitor serie

Si se conecta el cable USB y se abre el monitor serie a `115200` baudios, pueden visualizarse mensajes de diagnóstico.

Ejemplo de aviso:

```text
Aviso: sensor X con poco contraste
```

Esto suele significar que la diferencia entre `minVal` y `maxVal` fue menor al umbral configurado, por ejemplo `30`.

Posibles causas:

- El sensor no cruzó la línea durante la calibración.
- La altura del sensor es excesiva.
- La pista tiene poco contraste.
- La iluminación ambiental cambió durante la calibración.
- El sensor está sucio o mal alineado.

### 5. Inicio del seguimiento

Cuando finaliza la calibración, el robot comienza inmediatamente el seguimiento de línea mediante control PID.

---

## Ajuste del control PID

El ajuste del PID puede variar según:

- Peso del robot.
- Velocidad de los motores.
- Relación de reducción de los motorreductores.
- Nivel de batería.
- Tipo de pista.
- Altura y alineación de los sensores.

Recomendación básica de ajuste:

1. Comenzar con un valor moderado de `Kp`.
2. Mantener `Ki` bajo o cercano a cero durante las primeras pruebas.
3. Añadir `Kd` si aparecen oscilaciones o sobresaltos.
4. Introducir `Ki` lentamente si existe un error persistente.
5. Probar siempre con la batería a un nivel de carga similar al de competición o uso real.

Síntomas comunes:

| Comportamiento | Posible causa | Acción |
|---|---|---|
| Oscila mucho alrededor de la línea. | `Kp` demasiado alto o `Kd` mal ajustado. | Reducir `Kp`, revisar `Kd`. |
| Reacciona lento en curvas. | `Kp` bajo o velocidad insuficiente. | Aumentar `Kp` gradualmente. |
| Deriva lentamente hacia un lado. | Motores desiguales o sesgo mecánico. | Revisar mecánica, usar `Ki` pequeño. |
| Se pasa de la línea en curvas. | Exceso de velocidad o `Kd` insuficiente. | Reducir velocidad o ajustar `Kd`. |
| Comportamiento errático con batería baja. | Caída de tensión o ruido. | Revisar alimentación y filtrado. |

---

## Diagnóstico rápido

| Problema | Causa probable | Solución sugerida |
|---|---|---|
| El robot no calibra correctamente. | No empezó centrado sobre la línea. | Colocar la barra de sensores centrada sobre la línea. |
| Aparece `poco contraste` en un sensor. | Sensor demasiado alto o pista con bajo contraste. | Bajar sensor a 3-8 mm, revisar iluminación y superficie. |
| Los sensores centrales no calibran. | Barra de sensores sin voladizo frontal. | Adelantar ligeramente la barra respecto al eje de ruedas. |
| El robot no se mueve tras calibrar. | `STBY` en nivel bajo o motores mal conectados. | Poner `STBY` en `HIGH`, revisar cableado de motores. |
| Gira hacia el lado equivocado. | Polaridad de motor invertida o lógica de error incorrecta. | Intercambiar polaridad del motor o invertir dirección en firmware. |
| Oscila excesivamente. | Ganancia proporcional demasiado alta. | Reducir `Kp`, ajustar `Kd`. |
| Se detiene al perder la línea. | Recuperación deshabilitada o último error inválido. | Revisar lógica de memoria de dirección. |
| Comportamiento inestable al mover motores. | Ruido eléctrico o alimentación insuficiente. | Añadir capacitores, unir GND, revisar batería. |
| La velocidad cambia con batería baja. | Caída de tensión. | Usar batería adecuada, regulación estable y monitoreo de tensión. |

---

## Roadmap

- [ ] Implementar **anti-windup integral** para evitar saturación del término integral en curvas prolongadas.
- [ ] Guardar calibración en **EEPROM** para reutilizarla entre encendidos.
- [ ] Añadir configuración por **monitor serie**.
- [ ] Soporte mejorado para línea blanca sobre fondo negro.
- [ ] Control de velocidad con encoders.
- [ ] Perfil de velocidad adaptativo según curvatura.
- [ ] Soporte para más de 4 sensores.
- [ ] Dashboard de diagnóstico en tiempo real.

---

## Contribuciones

Las contribuciones son bienvenidas.

Para contribuir:

1. Hacer un **fork** del repositorio.
2. Crear una rama descriptiva:

```bash
git checkout -b feature/nueva-mejora
```

3. Realizar commits claros:

```bash
git commit -m "Agregar ajuste de ganancia desde monitor serie"
```

4. Subir la rama:

```bash
git push origin feature/nueva-mejora
```

5. Abrir un **Pull Request** describiendo los cambios.

Para errores o sugerencias, se recomienda abrir un **issue** con:

- Descripción del problema.
- Pasos para reproducirlo.
- Configuración mecánica y eléctrica.
- Valores de PID utilizados.
- Mensajes del monitor serie, si aplica.

---

## Licencia

Este proyecto se distribuye bajo la [Licencia MIT](LICENSE).

<!-- Si el proyecto usa otra licencia, reemplaza MIT por la licencia correspondiente y agrega el archivo LICENSE. -->
