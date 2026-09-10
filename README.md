# Seguidor de línea con Arduino Nano

Firmware para un robot diferencial con cuatro sensores analógicos TCRT5000,
Arduino Nano y driver de motores TB6612FNG. Calcula la posición de la línea
mediante promedio ponderado y corrige la trayectoria con un controlador PID.

## Funciones

- Calibración automática de los cuatro sensores al encender.
- Validación de contraste antes de permitir el movimiento.
- Lecturas normalizadas y filtro exponencial.
- Detección de línea con histéresis para evitar cambios por ruido.
- PID ejecutado cada 5 ms, con límite integral.
- Velocidad adaptativa: rápida en rectas y reducida en curvas.
- Recuperación hacia el último lado donde se detectó la línea.
- Parada segura si la línea no reaparece.
- Compatibilidad configurable con línea oscura o clara.
- Diagnóstico por Serial limitado a 20 actualizaciones por segundo.

El proyecto no necesita librerías externas.

## Hardware

- 1 Arduino Nano.
- 1 TB6612FNG.
- 4 sensores TCRT5000 con salida analógica.
- 2 motores DC con sus ruedas.
- Batería y regulación apropiadas para el montaje.

La alimentación de los motores se conecta a VMOT, no al pin de 5 V del
Arduino. La batería, el Arduino, el driver y los sensores deben compartir GND.

## Conexiones

### Sensores

| Posición | Arduino |
|---|---:|
| Exterior izquierdo, S0 | A0 |
| Interior izquierdo, S1 | A1 |
| Interior derecho, S2 | A2 |
| Exterior derecho, S3 | A3 |

### TB6612FNG

| Señal | Arduino |
|---|---:|
| AIN1, motor izquierdo | D7 |
| AIN2, motor izquierdo | D8 |
| PWMA, motor izquierdo | D9 |
| BIN1, motor derecho | D4 |
| BIN2, motor derecho | D5 |
| PWMB, motor derecho | D10 |
| STBY | D6 |

Los pines D9 y D10 son las salidas PWM. El firmware activa STBY
automáticamente.

Si un motor gira al revés, intercambia sus dos cables o invierte la dirección
correspondiente en moverMotor().

## Puesta en marcha

1. Coloca la barra de sensores sobre la línea. Debe estar adelantada respecto
   al eje de las ruedas para que el giro cruce la línea.
2. Enciende el robot.
3. Durante cuatro segundos el robot alternará el sentido de giro y el LED
   parpadeará rápidamente.
4. Al terminar, el monitor serie mostrará el mínimo, máximo y rango de cada
   sensor.
5. Si los cuatro sensores son válidos, el LED queda encendido y existe una
   pausa de un segundo antes de comenzar.

Si algún rango es menor que RANGO_MIN_VALIDO, los motores permanecen
detenidos y el LED parpadea lentamente. Corrige la altura, alineación o punto
de inicio y reinicia para repetir la calibración.

Para calibrar moviendo el robot manualmente, cambia AUTO_SWIPE a false.

## Estados del LED

| LED | Estado |
|---|---|
| Parpadeo rápido | Calibración o búsqueda de la línea |
| Encendido | Preparado o siguiendo la línea |
| Apagado | Detenido |
| Parpadeo lento | Error de calibración |

## Parámetros principales

Todos se encuentran al inicio de
[seguidor_nano.ino](seguidor_nano.ino).

| Parámetro | Valor inicial | Función |
|---|---:|---|
| LINEA_OSCURA | true | Usa línea negra; cambia a false para línea clara |
| PERIODO_CONTROL_MS | 5 ms | Periodo del controlador |
| KP | 0.06 | Ganancia proporcional |
| KI | 0.0 | Ganancia integral |
| KD | 0.0025 | Ganancia derivativa expresada respecto al tiempo |
| VEL_RECTA | 170 | PWM cuando la línea está centrada |
| VEL_CURVA | 110 | PWM base ante el error máximo |
| VEL_BUSQUEDA | 120 | PWM del pivote de recuperación |
| TIMEOUT_BUSQUEDA_MS | 1000 ms | Tiempo máximo buscando la línea |
| AJUSTE_MOTOR_IZQ | 0 | Compensación del motor izquierdo |
| AJUSTE_MOTOR_DER | 0 | Compensación del motor derecho |

El error se define como posición menos 1500. La posición aumenta hacia la
derecha. Por ello, un error positivo acelera el motor izquierdo y reduce la
velocidad del derecho.

## Ajuste inicial

1. Prueba primero con las ruedas levantadas y confirma que ambas avanzan al
   recibir una velocidad positiva.
2. Comprueba en el monitor serie que la línea produzca valores normalizados
   altos. Si ocurre lo contrario, cambia LINEA_OSCURA.
3. Ajusta KP hasta que responda bien a las curvas.
4. Ajusta KD si oscila alrededor de la línea.
5. Mantén KI en cero salvo que exista un sesgo constante.
6. Usa AJUSTE_MOTOR_IZQ o AJUSTE_MOTOR_DER si avanza desviado aun con la
   línea centrada.

Activa DEBUG solo durante el diagnóstico. La salida funciona a 115200
baudios e informa lectura cruda/normalizada, posición, error, velocidades y
estado.

## Comportamiento al perder la línea

El robot solo inicia una búsqueda si previamente detectó la línea. Pivota
hacia el signo del último error durante un máximo de un segundo. Después se
detiene, pero continúa leyendo los sensores y reanuda el seguimiento si la
línea vuelve a colocarse debajo de ellos.

## Licencia

Este proyecto se distribuye bajo la [Licencia MIT](LICENSE).
