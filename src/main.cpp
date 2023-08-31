#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "config.h"
//****************************************************************
// Definición de etiquetas
//****************************************************************
// Selección de parámetros de la señal PWM
#define pwmChannel 0 // 16 canales 0-15
#define ledRChannel 1 //canal de la led R
#define ledGChannel 2 //canal de la led G
#define ledAChannel 3 //canal de la led B
#define freqPWM 50 // Frecuencia para el servo
#define freqLeds 5000 //Frecuencia para las LED
#define resolution 8 // 1-16 bits de resolución
//****************************************************************
//definimos las entradas de nuestras leds
//****************************************************************
#define pinLedR 5 //pin Led Roja
#define pinLedA 18 //pin Led Amarilla
#define pinLedV 19 //pin Led Verde
#define pinPWM 15 // GPIO 2 para tener la salida del PWM
//****************************************************************
//definimos las entradas de nuestros botones
//****************************************************************
#define button1 34
// ****************************************************************************
// Valor para la division del tiempo
// ****************************************************************************
#define prescaler 80
//****************************************************************
//Definimos la entrada del sensor
//****************************************************************
#define sensorTemp 35
// ****************************************************************************
// Variables Globales
// ****************************************************************************
//Variables para el Timmer
int contadorTimer = 0;
hw_timer_t * timer = NULL;
//Variables para antirebote
int Temperatura = 0;
int Cont = 0;
//Variables para el ADC
int adcRaw = 0;
float tempC = 0.0;
float temp = 0.0;
long lastT;
int tMuestreo = 250;
//Variables para colocar los datos en los display
int Valor = 0;
int Decimales = 0;
int Unidades = 0;
int Decenas = 0;
//Variables del Filtro
double adcFiltradoEMA = adcRaw; // S(0) = Y(0)
double alpha = 0.8;//0.05; // Factor de suavizado (0-1)
//Adafruit feed counter
//AdafruitIO_Feed * canalTemp = io.feed("ftemp");
//****************************************************************
// Prototipos de funciones
//****************************************************************
void configurarPWM(void);
void emaADC(void);
void confTimer(void);
float LecVoltaje(int ADC_Raw);
void FuncionLeds(void);
void Antirebote(void);
void unidadTemp(void);
//****************************************************************
// Variables Globales
//****************************************************************
//****************************************************************
// ISR: Interrupciones
//****************************************************************
 void IRAM_ATTR ISRTemp(){
  //Interrupción del Botón
Cont = 1;//Creamos nuestro contador
}
void IRAM_ATTR ISRTimer0 (){
  //Interrupcion del Timer
  contadorTimer++;//Incrementa la variable si es mayor a 2 el contador timer se reiniciara a cero
  if (contadorTimer > 2){
    contadorTimer = 0;
  }
}

//****************************************************************
// Configuración
//****************************************************************
void setup()
{
Serial.begin(115200);

//wait for serial monitor to open
/*while(! Serial);
Serial.print("Conecting to Adafruit IO");

//conect to io.adafruit.com
io.connect();

//wait for a connection
while(io.status() < AIO_CONNECTED){
  Serial.print(".");
  delay(500);
}

//we are connected
Serial.println();
Serial.println(io.statusText());*/

//Configuramos todas nuestras funciones
configurarPWM();
confTimer();
lastT = millis();

//Indicamos las entradas de nuestro botón
pinMode(button1, INPUT_PULLUP);
//ligamos nuestro botón a nuestra interrupción
attachInterrupt(button1, ISRTemp, RISING);

}

//****************************************************************
// Loop Principal
//****************************************************************
void loop()
{
 Antirebote();//funcion antirebote para el botón
 FuncionLeds();//funcion de las led para reconocer la temperatura
 emaADC();//filtrado EMA para el registro del ADC
 unidadTemp();//unidad para los displays

  /*io.run();
  Serial.print("sending -> ");
  Serial.println(temp);
  canalTemp->save(temp);
  delay(3000);*/

//Lectura del sensor de temperatura
 if (millis() - lastT >= tMuestreo){
  lastT = millis();
  adcRaw = analogRead(sensorTemp);
  tempC = LecVoltaje(adcRaw);
  //tempC = (5.0 * adcRaw * 100.0)/1024.0;

  //temp = adcFiltradoEMA / 100.0;
  temp = tempC /10.0;
  Serial.print("Temperatura ADC:");
  Serial.print(tempC/10.0);
  Serial.print(",");
  Serial.print("Temperatura ADC2:");

  Serial.print(analogReadMilliVolts(sensorTemp));
  Serial.print(",");
  Serial.print("Temperatura Filtro EMA:");
  Serial.println(adcFiltradoEMA/10.0);
  delay(1000);

 }


}

//****************************************************************
// Función para configurar módulo PWM
//****************************************************************
void configurarPWM(void)
{
// Paso 1: Configurar el módulo PWM
ledcSetup(pwmChannel, freqPWM, resolution);
ledcSetup(ledRChannel, freqPWM, resolution);
ledcSetup(ledGChannel, freqPWM, resolution);
ledcSetup(ledAChannel, freqPWM, resolution);
// Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
ledcAttachPin(pinPWM, pwmChannel);
ledcAttachPin(pinLedR, ledRChannel);
ledcAttachPin(pinLedV, ledGChannel);
ledcAttachPin(pinLedA, ledAChannel);
}

// ****************************************************************************
// Funcion Calibrar ADC
// ****************************************************************************
float LecVoltaje(int ADC_Raw){
  //calibracion del ADC del ESP32
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

// ****************************************************************************
// Funcion Media Movil
// ****************************************************************************
void emaADC(void){
//inicialzamos nuestro filtro para que reconozca mejor la lectura de temperatura
adcRaw = analogRead(sensorTemp);
//adcFiltradoEMA = (alpha * adcRaw) + ((1.0 - alpha) * (adcFiltradoEMA/10.0));
adcFiltradoEMA = (alpha * adcRaw) + ((1.0 - alpha) * adcFiltradoEMA);
}

// ****************************************************************************
// Funcion paralos leds
// ****************************************************************************
void FuncionLeds(){
int b1 = digitalRead(button1);//leemos nuestro botón
int tempADC = adcFiltradoEMA/10.0;//creamos nuestra variable a detectar

//Aca indicamos que variables debemos de tener para que se lleve a cabo la lectura de la temperatura
if (b1 == 0 && tempADC <= 20){
      ledcWrite(ledRChannel, 0);
      ledcWrite(ledGChannel, 250);
      ledcWrite(ledAChannel, 0);
      ledcWrite(pwmChannel, 7);

    } else if (b1 == 0 && tempADC > 20 && tempADC <= 24){
      ledcWrite(ledRChannel, 0);
      ledcWrite(ledGChannel, 0);
      ledcWrite(ledAChannel, 250);
      ledcWrite(pwmChannel, 19);

    } else if (b1 == 0 && tempADC > 24){
      ledcWrite(ledRChannel, 250);
      ledcWrite(ledGChannel, 0);
      ledcWrite(ledAChannel, 0);
      ledcWrite(pwmChannel, 31);

    }

}

// ****************************************************************************
// Funcion para el Timmer
// ****************************************************************************
void confTimer(void) {
  timer = timerBegin(0, prescaler, true); //seleccionamos el Timer, timer = 0, prescaler = 80 y el flanco de subida
  timerAttachInterrupt(timer, &ISRTimer0, true);//asignamos el handler de la interrupcion
  //Programamos la alarma
  timerAlarmWrite(timer, 4000, true);
  timerAlarmEnable(timer);
}
// ****************************************************************************
// Funcion AntiRebote
// ****************************************************************************
void Antirebote(){
  //si Cont es igual a 1 y menor a 2 a a temperatura se le sumará 1
  if (Cont == 1 && Temperatura < 2){
    Temperatura = Temperatura + 1;
    Cont = 0;
    delay(150);
    if (Cont == 1 && Temperatura == 2){
      Temperatura = 0;
      Cont = 0;
      delay(150);
    }
  }

}
// ****************************************************************************
// Funcion para la Temperatura
// ****************************************************************************
void unidadTemp(){
  Valor = adcFiltradoEMA * 10;//Valor va a ser iguale al Valor del ADC multiplicado por 10
  Decenas = Valor / 100;//Luego dividimos ese valor dentro de 100
  Valor = Valor - (Decenas * 100);//El valor sera igual al valor anterior - (decenas * 100)
  Unidades = Valor / 10;// Ese Valor lo dividiremos entre 10
  Valor = Valor - (Unidades * 10);//Luego el valor sera igual al valor anterior - (decenas * 10)
  Decimales = Valor;//Ese valor sera igual al valor de decimales
}
