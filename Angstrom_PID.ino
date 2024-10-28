#include <SPI.h>
#include <ADS1220_WE.h>
#include <PID_v1.h>
#include <math.h> // Incluir math.h para usar la función sin()

#define TERMISTOR_PIN A0
#define TERMISTOR_PIN2 A1
#define HEAT_OUTPUT 10
#define COOL_OUTPUT 9
#define MIN_OUTPUT -255
#define MAX_OUTPUT 255
#define ADS1220_CS_PIN    7 // chip select pin para el ADS1220
#define ADS1220_DRDY_PIN  6 // data ready pin para el ADS1220

const float R_DIV = 10000.0;  // Resistencia del divisor de voltaje
const float V_REF = 5.0;      // Voltaje de referencia

// Coeficientes de Steinhart-Hart
const float A = 1.016156e-03;
const float B = 2.545381e-04;
const float C = -9.116603e-09;

// Variables para el PID
double Setpoint, Input, Output;
double Kp=150., Ki=10., Kd=25.;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// String para recibir nuevo valor del setpoint proporcionado por el usuario
String receivedSetpoint; 

const int N = 200; // Número de muestras para promediar
const int NM = 5000; // Número de milisegundos a los cuales se muestra la temperatura

// Variables para el promediado de voltajes
float sumaVoltajesInput = 0.0;
float sumaVoltajesMonitoreo = 0.0;
int count = 0;

unsigned long ultimoTiempoDisplay = 0;

float tempMonitoreo = 0.0;

// Constantes para la ecuación del setpoint
const float TEMP_0 = 25.0;    // Temperatura base en °C
const float AMP = 5.0;        // Amplitud de la función seno en °C
const float FREQ = 0.001;       // Frecuencia de modulación en Hz

//Objeto para controlar el ADS1220
ADS1220_WE ads = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
//variable global para guardar el voltaje del termistor
float voltajeInput;

void setup() {
  // Inicializamos el puerto serial
  Serial.begin(9600);
  if(!ads.init()){
    Serial.println("Fallos de comunicación con el ADS1220");
    while(1);
  }
  //En esta sección se configura el ADS1220 para la lectura del termistor
  ads.setCompareChannels(ADS1220_MUX_0_3);
  ads.setGain(ADS1220_GAIN_1);
  ads.bypassPGA(true); //Se desactiva el PGA
  ads.setIdacCurrent(ADS1220_IDAC_10_MU_A); //Se configura el IDAC1 para una corriente de 10uA (cambiar según sea necesario)
  ads.setIdac1Routing(ADS1220_IDAC_AIN0_REFP1);//Seleccionar el pin al cual se va a conectar el IDAC
  // Configuramos los puertos
  // pinMode(TERMISTOR_PIN, INPUT); // No es necesario para analogRead()
  pinMode(HEAT_OUTPUT, OUTPUT);
  pinMode(COOL_OUTPUT, OUTPUT);
  // Inicializar variables para la entrada del sensor
  //int lecturaInput = analogRead(TERMISTOR_PIN);
  voltajeInput = ads.getVoltage_mV();//leemos el voltaje del ADS1220 directamente en milivoltios
  Input = calcularTemperatura(voltajeInput, 10);
  // Setpoint inicial
  Setpoint = TEMP_0;
  // Definimos los límites para la salida del controlador
  myPID.SetOutputLimits(MIN_OUTPUT, MAX_OUTPUT);
  // Activamos el PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  unsigned long tiempoActual = millis();
  // Calcular el tiempo en segundos
  float t = tiempoActual / 1000.0;

  // Actualizar el setpoint según la ecuación dada
  Setpoint = TEMP_0 + AMP * sin(2 * PI * FREQ * t);

  if (Serial.available() > 0){
    Setpoint = Serial.parseFloat();
    Serial.readStringUntil('\n'); // Limpiar el buffer
  }

  // Leer voltajes de los termistores y acumular sumas
  //int lecturaInput = analogRead(TERMISTOR_PIN);
  voltajeInput = ads.getVoltage_mV();
  sumaVoltajesInput += voltajeInput;

  int lecturaMonitoreo = analogRead(TERMISTOR_PIN2);
  float voltajeMonitoreo = lecturaMonitoreo * (V_REF / 1023.0);
  sumaVoltajesMonitoreo += voltajeMonitoreo;

  count++;

  if (count >= N) {
    // Calcular voltajes promediados
    float promedioVoltajeInput = sumaVoltajesInput / count;
    float promedioVoltajeMonitoreo = sumaVoltajesMonitoreo / count;

    // Calcular temperaturas a partir de los voltajes promediados
    Input = calcularTemperatura(promedioVoltajeInput);
    tempMonitoreo = calcularTemperatura(promedioVoltajeMonitoreo);

    // Reiniciar sumas y contador
    sumaVoltajesInput = 0.0;
    sumaVoltajesMonitoreo = 0.0;
    count = 0;
  }

  // Calcular PID y controlar salida continuamente
  myPID.Compute();
  controlOutput(HEAT_OUTPUT, COOL_OUTPUT, Output);

  // Mostrar temperaturas cada NM milisegundos
  if (tiempoActual - ultimoTiempoDisplay >= NM) {
    ultimoTiempoDisplay = tiempoActual;

    Serial.print(TEMP_0 + AMP);
    Serial.print(", ");
    Serial.print(TEMP_0 - AMP);
    Serial.print(", ");
    Serial.print(Input);
    Serial.print(",   ");
    Serial.println(tempMonitoreo);
  }
}

void controlOutput(uint8_t heatPin, uint8_t coolPin, double outputValue){
  int pwmValue = constrain(abs((int)outputValue), 0, 255);
  if (outputValue <= 0){
    analogWrite(heatPin, 0);
    analogWrite(coolPin, pwmValue);
  }
  else {
    analogWrite(coolPin, 0);
    analogWrite(heatPin, pwmValue);
  }
}

float calcularTemperatura(float voltaje_mV, float corriente_uA) {
  float resistencia = voltaje_mV*1000/corriente_uA; //Aquí calculamos la resistencia con la corriente de excitación pasada como argumento (el x 1000 es para dejarlo en unidades de ohms)
  float logR = log(resistencia);
  float temperaturaKelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
  return temperaturaKelvin - 273.15;  // Convertir a Celsius
}