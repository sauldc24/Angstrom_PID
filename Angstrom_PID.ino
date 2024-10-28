#include <SPI.h>
#include <ADS1220_WE.h>
#include <PID_v1.h>
#include <math.h> // Incluir math.h para usar la función sin()

#define HEAT_OUTPUT 10
#define COOL_OUTPUT 9
#define MIN_OUTPUT -255
#define MAX_OUTPUT 255
#define ADS1220_CS_PIN    7 // chip select pin para el ADS1220
#define ADS1220_DRDY_PIN  6 // data ready pin para el ADS1220
#define IDAC_CURRENT 50.0 //corriente del IDAC en uA

// Coeficientes de Steinhart-Hart
const float A = 1.016156e-03;
const float B = 2.545381e-04;
const float C = -9.116603e-09;

/*variables para la medición de voltajes, resistencias y temperaturass. Las variables con el número 1 corresponden a las del termistor
usado como referencia para el PID, las variables con el número 2 corresponden a las del termistor de monitoreo*/
float voltage1 = 0.0;
float voltage2 = 0.0;
float resistance1 = 0.0;
float resistance2 = 0.0;
double temperature1 = 0.0;
float temperature2 = 0.0;

// Variables para el PID
double Setpoint, Input, Output;
double Kp=150., Ki=10., Kd=25.;
PID myPID(&temperature1, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// String para recibir nuevo valor del setpoint proporcionado por el usuario
String receivedSetpoint; 

// Constantes para la ecuación del setpoint
const float TEMP_0 = 25.0;    // Temperatura base en °C
const float AMP = 5.0;        // Amplitud de la función seno en °C
const float FREQ = 0.001;       // Frecuencia de modulación en Hz

//Objeto para controlar el ADS1220
ADS1220_WE ads = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);

//variables para determinar el momento en el que se manda la información al puerto serial
unsigned long ultimoTiempoDisplay = 0;//guarda el último tiempo en el que se mandó la información al puerto serial
float t = 0;//variable para guardar el tiempo en cada ejecución del bucle loop
const int NM = 5000; // Número de milisegundos a los cuales se muestra la temperatura

void setup() {
  // Inicializamos el puerto serial
  Serial.begin(9600);
  if(!ads.init()){
    Serial.println("Fallos de comunicación con el ADS1220");
    while(1);
  }
  //En esta sección se configura el ADS1220 para la lectura del termistor
  ads.setGain(ADS1220_GAIN_1);
  ads.bypassPGA(true); // true disables PGA, false enables PGA
  ads.setIdacCurrent(ADS1220_IDAC_50_MU_A);//Este hay que ponerlo manualmente por que las definiciones dependen de los registros
  ads.setIdac1Routing(ADS1220_IDAC_AIN0_REFP1);//IDAC 1 en el pin AIN0
  ads.setIdac2Routing(ADS1220_IDAC_AIN1);//IDAC 2 en el pin AIN1

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
  /*El bucle if revisa si hay datos en el puerto serial para actualizar el set point*/
  if (Serial.available() > 0){
    Setpoint = Serial.parseFloat();
    Serial.readStringUntil('\n'); // Limpiar el buffer
  }

  // En esta sección se leen los voltajes y temperaturas de los sensores
  ads.setCompareChannels(ADS1220_MUX_0_3);//configurar el multiplexor en los pines del termistor 1
  voltage1 = ads.getVoltage_mV(); // voltaje del termistor 1
  temperature1 = calcularTemperatura(voltage1, IDAC_CURRENT);
  ads.setCompareChannels(ADS1220_MUX_1_2);//configurar el multiplexor en los pines del termistor 2
  voltage2 = ads.getVoltage_mV(); // voltaje del termistor 2
  temperature2 = calcularTemperatura(voltage1, IDAC_CURRENT);

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
    Serial.print(temperature1);
    Serial.print(",   ");
    Serial.println(temperature2);
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