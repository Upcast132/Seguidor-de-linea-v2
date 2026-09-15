/*
  Seguidor de linea - Arduino Nano - 4x TCRT5000 - TB6612FNG

  Sensores: S0=A0, S1=A1, S2=A2, S3=A3 (izquierda a derecha).
  Motor izquierdo: AIN1=D7, AIN2=D8, PWMA=D9.
  Motor derecho: BIN1=D4, BIN2=D5, PWMB=D10. STBY=D6.

  La bateria de motores va a VMOT, nunca al pin 5V del Arduino.
  Arduino, driver, sensores y bateria deben compartir GND.
*/

// ---------- Configuracion general ----------
const bool DEBUG = false;
const bool AUTO_SWIPE = true;
const bool LINEA_OSCURA = true;  // false para linea clara sobre fondo oscuro

const uint8_t LED_ESTADO = 13;
const unsigned long PERIODO_CONTROL_MS = 5;
const unsigned long PERIODO_DEBUG_MS = 50;
const unsigned long PAUSA_INICIO_MS = 1000;

// ---------- Driver TB6612FNG ----------
const uint8_t STBY = 6;
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;
const uint8_t ENA = 9;
const uint8_t IN3 = 4;
const uint8_t IN4 = 5;
const uint8_t ENB = 10;

const int VELOCIDAD_MAX = 255;
const int AJUSTE_MOTOR_IZQ = 0;
const int AJUSTE_MOTOR_DER = 0;

// ---------- Sensores y calibracion ----------
const uint8_t NUM_SENSORES = 4;
const uint8_t SENSOR_PINS[NUM_SENSORES] = {A0, A1, A2, A3};
const int SETPOINT = 1500;
const int RANGO_MIN_VALIDO = 30;
const int UMBRAL_ENTRADA_LINEA = 250;
const int UMBRAL_SALIDA_LINEA = 150;
const float ALPHA_FILTRO = 0.5f;

const unsigned long TIEMPO_CALIBRACION_MS = 4000;
const unsigned long PERIODO_PIVOTE_MS = 400;
const int VEL_CALIBRACION = 120;

int minVal[NUM_SENSORES];
int maxVal[NUM_SENSORES];
float emaVal[NUM_SENSORES] = {0, 0, 0, 0};
bool filtroInicializado = false;
bool lineaDetectada = false;

// ---------- PID y movimiento ----------
const float KP = 0.06f;
const float KI = 0.0f;
const float KD = 0.0025f;
const float LIMITE_INTEGRAL = 3000.0f;

const int VEL_RECTA = 170;
const int VEL_CURVA = 110;
const int VEL_BUSQUEDA = 120;
const unsigned long TIMEOUT_BUSQUEDA_MS = 1000;

float errorAnterior = 0.0f;
float integral = 0.0f;
int ultimoError = 0;
bool pidInicializado = false;
bool haVistoLinea = false;

enum EstadoRobot {
  CALIBRANDO,
  ESPERANDO_INICIO,
  SIGUIENDO,
  BUSCANDO,
  DETENIDO,
  ERROR_CALIBRACION
};

EstadoRobot estado = CALIBRANDO;
unsigned long inicioEspera = 0;
unsigned long ultimoControl = 0;
unsigned long ultimaVezConLinea = 0;
unsigned long ultimoDebug = 0;

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_ESTADO, OUTPUT);

  digitalWrite(STBY, HIGH);
  detener();

  if (!calibrar()) {
    estado = ERROR_CALIBRACION;
    return;
  }

  estado = ESPERANDO_INICIO;
  inicioEspera = millis();
  digitalWrite(LED_ESTADO, HIGH);
}

void loop() {
  unsigned long ahora = millis();

  if (estado == ERROR_CALIBRACION) {
    detener();
    digitalWrite(LED_ESTADO, (ahora / 500UL) % 2UL);
    return;
  }

  if (estado == ESPERANDO_INICIO) {
    detener();
    digitalWrite(LED_ESTADO, HIGH);
    if (ahora - inicioEspera < PAUSA_INICIO_MS) return;

    estado = DETENIDO;
    ultimoControl = ahora - PERIODO_CONTROL_MS;
  }

  actualizarLed(ahora);
  if (ahora - ultimoControl < PERIODO_CONTROL_MS) return;

  unsigned long tiempoTranscurridoMs = ahora - ultimoControl;
  ultimoControl = ahora;
  float dt = tiempoTranscurridoMs / 1000.0f;

  int valoresRaw[NUM_SENSORES];
  int valoresNorm[NUM_SENSORES];
  int posicion = SETPOINT;
  bool hayLinea = leerLinea(valoresRaw, valoresNorm, posicion);
  int velIzq = 0;
  int velDer = 0;

  if (hayLinea) {
    ultimaVezConLinea = ahora;
    haVistoLinea = true;

    int error = posicion - SETPOINT;
    float derivada = 0.0f;

    if (pidInicializado) {
      integral += error * dt;
      integral = constrain(integral, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
      derivada = (error - errorAnterior) / dt;
    } else {
      integral = 0.0f;
      pidInicializado = true;
    }

    float correccion = KP * error + KI * integral + KD * derivada;
    errorAnterior = error;
    ultimoError = error;

    int errorAbsoluto = abs(error);
    int velocidadBase = map(constrain(errorAbsoluto, 0, 1500),
                            0, 1500, VEL_RECTA, VEL_CURVA);

    velIzq = velocidadBase + (int)correccion;
    velDer = velocidadBase - (int)correccion;
    velIzq = ajustarMotor(velIzq, AJUSTE_MOTOR_IZQ);
    velDer = ajustarMotor(velDer, AJUSTE_MOTOR_DER);
    velIzq = constrain(velIzq, -VELOCIDAD_MAX, VELOCIDAD_MAX);
    velDer = constrain(velDer, -VELOCIDAD_MAX, VELOCIDAD_MAX);

    moverMotor(IN1, IN2, ENA, velIzq);
    moverMotor(IN3, IN4, ENB, velDer);
    estado = SIGUIENDO;
  } else {
    reiniciarPid();

    if (!haVistoLinea) {
      detener();
      estado = DETENIDO;
    } else if (ahora - ultimaVezConLinea <= TIMEOUT_BUSQUEDA_MS) {
      buscar();
      estado = BUSCANDO;
      velIzq = ultimoError >= 0 ? VEL_BUSQUEDA : -VEL_BUSQUEDA;
      velDer = -velIzq;
    } else {
      detener();
      estado = DETENIDO;
    }
  }

  if (DEBUG && ahora - ultimoDebug >= PERIODO_DEBUG_MS) {
    ultimoDebug = ahora;
    imprimirDebug(valoresRaw, valoresNorm, posicion, velIzq, velDer);
  }
}

// Un valor normalizado alto siempre representa linea.
bool leerLinea(int valoresRaw[], int valoresNorm[], int &posicion) {
  long sumaPonderada = 0;
  long sumaValores = 0;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    int norm = map(raw, minVal[i], maxVal[i], 0, 1000);
    norm = constrain(norm, 0, 1000);

    if (!LINEA_OSCURA) norm = 1000 - norm;

    if (!filtroInicializado) {
      emaVal[i] = norm;
    } else {
      emaVal[i] = ALPHA_FILTRO * norm +
                  (1.0f - ALPHA_FILTRO) * emaVal[i];
    }

    norm = (int)emaVal[i];
    valoresRaw[i] = raw;
    valoresNorm[i] = norm;
    sumaPonderada += (long)norm * (i * 1000L);
    sumaValores += norm;
  }

  filtroInicializado = true;

  if (lineaDetectada) {
    if (sumaValores < UMBRAL_SALIDA_LINEA) lineaDetectada = false;
  } else if (sumaValores >= UMBRAL_ENTRADA_LINEA) {
    lineaDetectada = true;
  }

  if (!lineaDetectada || sumaValores == 0) return false;

  posicion = sumaPonderada / sumaValores;
  return true;
}

void moverMotor(uint8_t inA, uint8_t inB, uint8_t pwmPin, int velocidad) {
  velocidad = constrain(velocidad, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  if (velocidad == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  } else if (velocidad > 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(pwmPin, velocidad);
  } else {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(pwmPin, -velocidad);
  }
}

int ajustarMotor(int velocidad, int ajuste) {
  if (velocidad > 0) return velocidad + ajuste;
  if (velocidad < 0) return velocidad - ajuste;
  return 0;
}

void buscar() {
  if (ultimoError >= 0) {
    moverMotor(IN1, IN2, ENA, VEL_BUSQUEDA);
    moverMotor(IN3, IN4, ENB, -VEL_BUSQUEDA);
  } else {
    moverMotor(IN1, IN2, ENA, -VEL_BUSQUEDA);
    moverMotor(IN3, IN4, ENB, VEL_BUSQUEDA);
  }
}

void detener() {
  moverMotor(IN1, IN2, ENA, 0);
  moverMotor(IN3, IN4, ENB, 0);
}

void reiniciarPid() {
  integral = 0.0f;
  errorAnterior = 0.0f;
  pidInicializado = false;
}

// Registra minimos y maximos mientras el robot alterna el pivote.
bool calibrar() {
  estado = CALIBRANDO;
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  Serial.println(F("Calibrando sensores..."));
  unsigned long inicio = millis();

  while (millis() - inicio < TIEMPO_CALIBRACION_MS) {
    unsigned long ahora = millis();
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      int lectura = analogRead(SENSOR_PINS[i]);
      if (lectura < minVal[i]) minVal[i] = lectura;
      if (lectura > maxVal[i]) maxVal[i] = lectura;
    }

    digitalWrite(LED_ESTADO, (ahora / 100UL) % 2UL);

    if (AUTO_SWIPE) {
      bool derecha = ((ahora - inicio) / PERIODO_PIVOTE_MS) % 2UL == 0;
      if (derecha) {
        moverMotor(IN1, IN2, ENA, VEL_CALIBRACION);
        moverMotor(IN3, IN4, ENB, -VEL_CALIBRACION);
      } else {
        moverMotor(IN1, IN2, ENA, -VEL_CALIBRACION);
        moverMotor(IN3, IN4, ENB, VEL_CALIBRACION);
      }
    }
  }

  detener();
  bool calibracionValida = true;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int rango = maxVal[i] - minVal[i];
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(" min="));
    Serial.print(minVal[i]);
    Serial.print(F(" max="));
    Serial.print(maxVal[i]);
    Serial.print(F(" rango="));
    Serial.println(rango);

    if (rango < RANGO_MIN_VALIDO) {
      calibracionValida = false;
      Serial.print(F("ERROR: poco contraste en sensor S"));
      Serial.println(i);
    }
  }

  if (calibracionValida) {
    Serial.println(F("Calibracion correcta"));
  } else {
    Serial.println(F("ERROR: calibracion invalida; motores detenidos"));
  }
  return calibracionValida;
}

void actualizarLed(unsigned long ahora) {
  if (estado == BUSCANDO) {
    digitalWrite(LED_ESTADO, (ahora / 100UL) % 2UL);
  } else if (estado == DETENIDO) {
    digitalWrite(LED_ESTADO, LOW);
  } else {
    digitalWrite(LED_ESTADO, HIGH);
  }
}

void imprimirEstado() {
  switch (estado) {
    case CALIBRANDO:         Serial.print(F("CALIBRANDO")); break;
    case ESPERANDO_INICIO:   Serial.print(F("ESPERANDO")); break;
    case SIGUIENDO:          Serial.print(F("SIGUIENDO")); break;
    case BUSCANDO:           Serial.print(F("BUSCANDO")); break;
    case DETENIDO:           Serial.print(F("DETENIDO")); break;
    case ERROR_CALIBRACION:  Serial.print(F("ERROR_CALIBRACION")); break;
  }
}

void imprimirDebug(int valoresRaw[], int valoresNorm[], int posicion,
                   int velIzq, int velDer) {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F("="));
    Serial.print(valoresRaw[i]);
    Serial.print(F("/"));
    Serial.print(valoresNorm[i]);
    Serial.print('\t');
  }

  Serial.print(F("pos="));
  Serial.print(posicion);
  Serial.print(F("\terror="));
  Serial.print(ultimoError);
  Serial.print(F("\tmotores="));
  Serial.print(velIzq);
  Serial.print(F(","));
  Serial.print(velDer);
  Serial.print(F("\testado="));
  imprimirEstado();
  Serial.println();
}
