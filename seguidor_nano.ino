/*
  Seguidor de linea - Arduino Nano - 4x TCRT5000 - Control proporcional (PID)
  Version de respaldo por si el reglamento exige Arduino en vez de ESP32+QTR.

  Parches respecto a tu version anterior (2 sensores, bang-bang):
  - 4 sensores analogicos -> posicion continua de la linea, no solo izq/der.
  - PID en vez de modos de giro fijos (SOFT/STOP/REVERSE). Ya no hacen falta,
    la correccion proporcional cubre los tres casos automaticamente: giro
    suave con error chico, pivote con error grande (motor entra en reversa).
  - Calibracion automatica al encender. Ya no hace falta correr Calibracion.ino
    aparte y copiar umbrales a mano: aqui se calibra min/max por sensor solo.
  - Deteccion de perdida de linea con timeout: si no la encuentra en
    TIMEOUT_LINEA ms, se detiene en vez de girar para siempre.
  - Debug por Serial detras de un #define, para no meter latencia en pista.

  Conexion sensores (TCRT5000, salida analogica):
    S0 = A0  (extremo izquierdo)
    S1 = A1  (interior izquierdo)
    S2 = A2  (interior derecho)
    S3 = A3  (extremo derecho)

  Conexion driver (mismos nombres de pin para L298N o TB6612FNG:
  AIN1/AIN2/PWMA = IN1/IN2/ENA, BIN1/BIN2/PWMB = IN3/IN4/ENB):
    Motor izquierdo: IN1 = D7, IN2 = D8, ENA = D9  (PWM)
    Motor derecho:   IN3 = D4, IN4 = D5, ENB = D10 (PWM)

  El TB6612FNG, a diferencia del L298N, necesita el pin STBY en HIGH para
  funcionar. Selecciona el driver que uses en DRIVER_TIPO mas abajo: si
  eliges TB6612FNG el codigo activa STBY solo, si eliges L298N ese pin
  ni se toca.

  Alimentacion: el driver de motores (VMOT / logica de potencia) va a
  bateria externa, NO al 5V del Arduino. La logica del driver (VCC) si
  puede ir al 5V del Arduino. GND de la bateria y del Arduino deben
  quedar unidos.

  Si un motor gira al reves de lo esperado, invierte HIGH/LOW de ese motor
  en moverMotor() (o cambia el cableado, es mas facil).

  CALIBRACION: al encender, el robot pivota solo sobre su eje durante
  TIEMPO_CALIBRACION ms (AUTO_SWIPE true) mientras el LED 13 parpadea rapido.
  Para que sirva, tenes que arrancarlo con la linea ya debajo de la barra de
  sensores: si arranca apuntando para cualquier lado, el pivote no la cruza y
  calibra mal, igual que un swipe manual mal hecho. Si tu punto de salida
  varia o no hay espacio para pivotar, pone AUTO_SWIPE en false: ahi vuelve
  al modo manual, pasas el robot a mano sobre la linea durante esos mismos
  segundos. Cuando el LED queda fijo, la calibracion termino.
*/

#define DEBUG false   // pon en true solo para ajustar en banco, apaga en pista

// ---------- Driver ----------
#define DRIVER_L298N     0
#define DRIVER_TB6612FNG 1
#define DRIVER_TIPO DRIVER_L298N   // cambia a DRIVER_TB6612FNG si usas ese driver

#if DRIVER_TIPO == DRIVER_TB6612FNG
const uint8_t STBY = 6; // TB6612FNG no mueve nada si esto no esta en HIGH
#endif

// ---------- Sensores ----------
const uint8_t SENSOR_PINS[4] = {A0, A1, A2, A3};
const uint8_t NUM_SENSORES = 4;
const int UMBRAL_DETECCION = 150; // sobre escala 0-1000, minimo para "hay linea"
const int RANGO_MIN_VALIDO = 30;  // si un sensor calibro con menos rango que esto, se ignora

int minVal[NUM_SENSORES];
int maxVal[NUM_SENSORES];
float emaVal[NUM_SENSORES] = {0, 0, 0, 0}; // suavizado de lecturas
const float ALPHA_FILTRO = 0.5; // 1.0 = sin filtro, mas bajo = mas suave y mas lag
const unsigned long TIEMPO_CALIBRACION = 4000; // ms

// true = el robot pivota solo con los motores durante la calibracion (requiere
// arrancar con la linea ya debajo de la barra de sensores).
// false = calibracion manual: pasas el robot a mano sobre la linea (como antes).
#define AUTO_SWIPE true
const int VEL_CALIBRACION = 120;          // velocidad del pivote automatico
const unsigned long PERIODO_PIVOTE = 400; // ms por lado antes de cambiar de sentido

// ---------- Motor izquierdo ----------
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;
const uint8_t ENA = 9;

// ---------- Motor derecho ----------
const uint8_t IN3 = 4;
const uint8_t IN4 = 5;
const uint8_t ENB = 10;

const uint8_t LED_ESTADO = 13;

// ---------- PID ----------
// Empieza solo con KP subiendo de a poco hasta que oscile, luego agrega KD
// para amortiguar esa oscilacion. KI casi nunca hace falta en esto, se deja
// en 0 salvo que el robot se quede "corriendo" el error en rectas largas.
float KP = 0.06;
float KD = 0.5;
float KI = 0.0;

int velocidadBase = 150; // 0-255, velocidad en recta
int velocidadMax  = 255;

float errorAnterior = 0;
float integral = 0;
int ultimoError = 0; // arranca en 0 -> buscar() pivotea a la derecha por defecto (no es aleatorio, ver buscar())

// ---------- Perdida de linea ----------
const unsigned long TIMEOUT_LINEA = 800; // ms sin ver linea -> detenerse
unsigned long ultimaVezConLinea = 0;

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) pinMode(SENSOR_PINS[i], INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(LED_ESTADO, OUTPUT);

#if DRIVER_TIPO == DRIVER_TB6612FNG
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
#endif

  calibrar();
  ultimaVezConLinea = millis();
}

void loop() {
  int valores[NUM_SENSORES];
  int posicion = 1500;
  bool hayLinea = leerLinea(valores, posicion);

  if (hayLinea) {
    ultimaVezConLinea = millis();

    int error = posicion - 1500; // centro para 4 sensores (rango 0-3000)
    integral += error;
    float derivada = error - errorAnterior;
    float correccion = KP * error + KI * integral + KD * derivada;
    errorAnterior = error;
    ultimoError = error;

    int velIzq = velocidadBase + correccion;
    int velDer = velocidadBase - correccion;

    moverMotor(IN1, IN2, ENA, velIzq);
    moverMotor(IN3, IN4, ENB, velDer);

  } else {
    integral = 0;
    errorAnterior = 0;
    if (millis() - ultimaVezConLinea > TIMEOUT_LINEA) {
      detener();
    } else {
      buscar();
    }
  }

  if (DEBUG) imprimirDebug(valores, posicion);
}

// Lee los 4 sensores, normaliza con la calibracion (0-1000) y calcula la
// posicion ponderada de la linea (0 = todo a la izquierda, 3000 = todo a la
// derecha). Devuelve false si ningun sensor ve la linea.
bool leerLinea(int valoresNorm[], int &posicion) {
  long sumaPonderada = 0;
  long sumaValores = 0;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    int norm;

    if (maxVal[i] - minVal[i] < RANGO_MIN_VALIDO) {
      // este sensor no calibro bien (o min==max): no lo dejamos meter
      // ruido/basura en la posicion en vez de dividir por casi cero
      norm = 0;
    } else {
      norm = map(raw, minVal[i], maxVal[i], 0, 1000);
      norm = constrain(norm, 0, 1000);
    }

    emaVal[i] = ALPHA_FILTRO * norm + (1.0 - ALPHA_FILTRO) * emaVal[i];
    norm = (int)emaVal[i];
    valoresNorm[i] = norm;

    sumaPonderada += (long)norm * (i * 1000L);
    sumaValores += norm;
  }

  if (sumaValores < UMBRAL_DETECCION) return false;

  posicion = sumaPonderada / sumaValores;
  return true;
}

// Controla un motor con direccion + PWM. Velocidad negativa = reversa.
// Esto es lo que reemplaza los modos SOFT/STOP/REVERSE: con error grande,
// la correccion supera velocidadBase y el motor interior entra solo en
// reversa (pivote), sin necesitar un modo aparte.
void moverMotor(uint8_t inA, uint8_t inB, uint8_t pwmPin, int velocidad) {
  velocidad = constrain(velocidad, -velocidadMax, velocidadMax);
  if (velocidad >= 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(pwmPin, velocidad);
  } else {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(pwmPin, -velocidad);
  }
}

// Linea perdida: pivota hacia el lado donde se vio por ultima vez.
void buscar() {
  const int velBusqueda = 150;
  if (ultimoError >= 0) {
    moverMotor(IN1, IN2, ENA, velBusqueda);
    moverMotor(IN3, IN4, ENB, -velBusqueda);
  } else {
    moverMotor(IN1, IN2, ENA, -velBusqueda);
    moverMotor(IN3, IN4, ENB, velBusqueda);
  }
}

void detener() {
  moverMotor(IN1, IN2, ENA, 0);
  moverMotor(IN3, IN4, ENB, 0);
}

// Calibracion automatica: durante TIEMPO_CALIBRACION ms guarda el minimo y
// maximo de cada sensor mientras el LED parpadea rapido. Pasa el robot a
// mano sobre blanco y negro varias veces en esa ventana.
void calibrar() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  Serial.println(F("Calibrando: pasa el sensor sobre la linea y el fondo"));

  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_CALIBRACION) {
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      int lectura = analogRead(SENSOR_PINS[i]);
      if (lectura < minVal[i]) minVal[i] = lectura;
      if (lectura > maxVal[i]) maxVal[i] = lectura;
    }
    digitalWrite(LED_ESTADO, (millis() / 100) % 2);

#if AUTO_SWIPE
    // alterna el sentido del pivote cada PERIODO_PIVOTE ms para que la
    // barra de sensores cruce la linea varias veces sin desplazarse del sitio
    bool haciaLaDerecha = ((millis() - inicio) / PERIODO_PIVOTE) % 2 == 0;
    if (haciaLaDerecha) {
      moverMotor(IN1, IN2, ENA, VEL_CALIBRACION);
      moverMotor(IN3, IN4, ENB, -VEL_CALIBRACION);
    } else {
      moverMotor(IN1, IN2, ENA, -VEL_CALIBRACION);
      moverMotor(IN3, IN4, ENB, VEL_CALIBRACION);
    }
#endif
  }

#if AUTO_SWIPE
  detener(); // frena antes de seguir con el resto de la calibracion
#endif
  digitalWrite(LED_ESTADO, HIGH);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (maxVal[i] - minVal[i] < 30) {
      Serial.print(F("Aviso: sensor "));
      Serial.print(i);
      Serial.println(F(" con poco contraste, revisa altura o alineacion"));
    }
  }
}

void imprimirDebug(int valores[], int posicion) {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(valores[i]);
    Serial.print('\t');
  }
  Serial.print("pos:"); Serial.print(posicion);
  Serial.print(" err:"); Serial.println(ultimoError);
}
