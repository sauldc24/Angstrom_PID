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
#define ADS1220_CS_PIN_2  5     // Chip select pin for second ADC (control + flux)
#define ADS1220_DRDY_PIN_2 4    // Data ready pin for second ADC
#define IDAC_CURRENT 50.0 //corriente del IDAC en uA
#define VREF 2.048             // Reference voltage (V)
#define SPI_CLOCK 4000000    

// Coeficientes de Steinhart-Hart
const float A = 1.016156e-03;
const float B = 2.545381e-04;
const float C = -9.116603e-09;

// Heat flux sensor calibration constants
const float So = 3.91;    // Base sensitivity μV/(W/m²)
const float Sc = 0.0047;  // Temperature correction factor (μV/(W/m²))/°C
const float To = 22.5;    // Calibration temperature °C


/*variables para la medición de voltajes, resistencias y temperaturass. Las variables con el número 1 corresponden a las del termistor
usado como referencia para el PID, las variables con el número 2 corresponden a las del termistor de monitoreo*/
float controlVoltage = 0.0;
float hotVoltage = 0.0;
float coldVoltage = 0.0;
float heatFluxVoltage = 0.0;
float heatFlux = 0.0;
float controlResistance = 0.0;
float hotResistance = 0.0;
float coldResistance = 0.0;
double controlTemp = 0.0;
float hotTemp = 0.0;
float coldTemp = 0.0;

// Variables para el PID
double Setpoint, Input, Output;
double Kp=150., Ki=10., Kd=25.;
PID myPID(&controlTemp, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// String para recibir nuevo valor del setpoint proporcionado por el usuario
String receivedSetpoint; 

// Constantes para la ecuación del setpoint
const float TEMP_0 = 25.0;    // Temperatura base en °C
const float AMP = 5.0;        // Amplitud de la función seno en °C
const float FREQ = 0.1;       // Frecuencia de modulación en Hz

//Objeto para controlar el ADS1220
ADS1220_WE ads1 = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
ADS1220_WE ads2 = ADS1220_WE(ADS1220_CS_PIN_2, ADS1220_DRDY_PIN_2);

// Timing variables
unsigned long lastDisplayTime = 0;
const int DISPLAY_INTERVAL = 100;  // Display interval in ms

//Variables relacionadas con la comunicación del instrumento
String inputBuffer = "";  // Buffer to store incoming characters
bool commandReady = false; // Flag to indicate command is ready to be processed


float calcularTemperatura(float voltaje_mV, float corriente_uA) {
  float resistencia = voltaje_mV*1000/corriente_uA; //Aquí calculamos la resistencia con la corriente de excitación pasada como argumento (el x 1000 es para dejarlo en unidades de ohms)
  float logR = log(resistencia);
  float temperaturaKelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
  return temperaturaKelvin - 273.15;  // Convertir a Celsius
}

// Function to calculate heat flux from voltage and temperature
float calcHeatFlux(float voltage_uV, float temp) {
  float S = So + (temp - To) * Sc;  // Temperature-corrected sensitivity
  return voltage_uV / S;            // Heat flux in W/m²
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

void setup() {
  // Inicializamos el puerto serial
  Serial.begin(115200);
  // Initialize both ADCs
  if(!ads1.init() || !ads2.init()){
    Serial.println("ADC communication failure");
    while(1);
  }
  //En esta sección se configura el ADS1220 para la lectura del termistor
  // Configure ADC1 for hot/cold temperature measurements
  ads1.setGain(ADS1220_GAIN_1);
  ads1.bypassPGA(true);
  ads1.setIdacCurrent(ADS1220_IDAC_50_MU_A);
  ads1.setIdac1Routing(ADS1220_IDAC_AIN0_REFP1);
  ads1.setIdac2Routing(ADS1220_IDAC_AIN1);
  ads1.setDataRate(ADS1220_DR_LVL_0);
  ads1.setFIRFilter(ADS1220_60HZ);
  ads1.setConversionMode(ADS1220_CONTINUOUS);
  ads1.setOperatingMode(ADS1220_TURBO_MODE);
  ads1.setSPIClockSpeed(SPI_CLOCK);
  ads1.setVRefValue_V(VREF);
  ads1.start();

  // Configure ADC2 for control temperature and heat flux
  ads2.setGain(ADS1220_GAIN_1);
  ads2.bypassPGA(true);
  ads2.setIdacCurrent(ADS1220_IDAC_50_MU_A);
  ads2.setIdac1Routing(ADS1220_IDAC_AIN1);
  ads2.setDataRate(ADS1220_DR_LVL_0);
  ads2.setFIRFilter(ADS1220_60HZ);
  ads2.setConversionMode(ADS1220_CONTINUOUS);
  ads2.setOperatingMode(ADS1220_TURBO_MODE);
  ads2.setSPIClockSpeed(SPI_CLOCK);
  ads2.setVRefValue_V(VREF);
  ads2.start();

  // Setpoint inicial
  Setpoint = TEMP_0;
  // Definimos los límites para la salida del controlador
  myPID.SetOutputLimits(MIN_OUTPUT, MAX_OUTPUT);
  // Activamos el PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  unsigned long currentTime = millis();
  float t = currentTime / 1000.0;
  // Actualizar el setpoint según la ecuación dada
  Setpoint = TEMP_0 + AMP * sin(2 * PI * FREQ * t);
  /*Se leen los datos del puerto serial en caso de estar disponibles*/
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();   // Read one character at a time
    inputBuffer += receivedChar;         // Append character to buffer

    // Check if CRLF (carriage return + line feed) is detected
    if (inputBuffer.endsWith("\r\n")) {
      commandReady = true;
      break; // Exit the loop once CRLF is detected
    }
  }
  // Se procesa el comando si está listo
  if (commandReady) {
    // Remove CRLF from the end of the buffer
    inputBuffer.trim();

    // Parse and handle the command
    handleCommand(inputBuffer);

    // Clear the buffer and reset the flag
    inputBuffer = "";
    commandReady = false;
  }
  // En esta sección se lee el ads2
  ads2.setCompareChannels(ADS1220_MUX_1_2);//configurar el multiplexor en los pines del termistor de control
  controlVoltage = ads2.getVoltage_mV(); // voltaje del termistor de control
  controlTemp = calcularTemperatura(controlVoltage, IDAC_CURRENT);
  ads2.setCompareChannels(ADS1220_MUX_0_3);//configurar el multiplexor en los pines del sensor de flujo de calor
  heatFluxVoltage = ads2.getVoltage_mV(); // voltaje del sensor de flujo de calor
  heatFlux = calcHeatFlux(heatFluxVoltage, controlTemp);

  // En esta sección se lee el ads1
  ads1.setCompareChannels(ADS1220_MUX_1_2);//configurar el multiplexor en los pines del termistor de la cara fría
  coldVoltage = ads1.getVoltage_mV(); // voltaje del termistor de la cara fría
  coldTemp = calcularTemperatura(coldVoltage, IDAC_CURRENT);
  ads1.setCompareChannels(ADS1220_MUX_0_3);//configurar el multiplexor en los pines del termistor de la cara caliente
  hotVoltage = ads1.getVoltage_mV(); // voltaje del termistor de la cara caliente
  hotTemp = calcularTemperatura(hotVoltage, IDAC_CURRENT);

  // Calcular PID y controlar salida continuamente
  myPID.Compute();
  controlOutput(HEAT_OUTPUT, COOL_OUTPUT, Output);
}