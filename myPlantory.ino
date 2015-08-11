#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#define DISPARO 1000
#define SENSOR_HUMEDAD_PIN 0 

enum estados{INACTIVO, ACTIVO};
enum estadosSensor {SECO, HUMEDO};

struct valvula{
    int id;
    int pin;
    enum estados status;
    unsigned long timer;
    unsigned long tiempo;
};

struct sensor{
    int id;
    enum estadosSensor status;
    int valorActual;
    int valorAnterior;
};

struct valvula valvula;
struct sensor sensor;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
unsigned long timer_tsl;
unsigned long tiempo_tsl = 10;

unsigned long timer_riego;
unsigned long tiempo_riego = 10;

void setup()
{
    valvula.id = 0;
    valvula.pin = 8;
    valvula.status = INACTIVO;
    valvula.timer = 0;
    valvula.tiempo = 10;

    sensor.id = 0;
    sensor.status = SECO;
    sensor.valorActual = -1;
    sensor.valorAnterior = -1;

    pinMode(valvula.pin, OUTPUT);
    digitalWrite(valvula.pin, LOW);
    Serial.begin(9600);

    if(!tsl.begin())
        {
        Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    configureLuxSensor();
  
    Serial.println("");
    timer_tsl = millis();
    timer_riego = millis();

}

void loop()
{   
    if(timer(timer_riego, tiempo_riego)){
        ejecutarRiego();
        timer_riego = millis();
    }
    if(timer(timer_tsl, tiempo_tsl)){
        lecturaLuminosidad();
        timer_tsl = millis();
    }

}

int timer(unsigned long inicio,  unsigned long limite){
    unsigned long actual = millis(); 
    if((actual - inicio) > (limite*1000)){
        Serial.println(inicio);
        Serial.println(actual);
        Serial.print("Timer>");
        Serial.println("excedido");
        return 1;
    }
    return 0;
}

void configureLuxSensor(void){
        tsl.setGain(TSL2561_GAIN_1X);
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
        Serial.println("-------------------------------------");
        Serial.print  ("Gain:       "); Serial.println("Auto");
        Serial.print  ("Timing:     "); Serial.println("13 ms");
        Serial.println("-------------------------------------");
        
}

void lecturaLuminosidad(){
    sensors_event_t event;
    tsl.getEvent(&event);

    if (event.light){
        Serial.print(event.light); Serial.println(" lux");
    }else{
        Serial.println("Sensor overload");
    }
    delay(250);
}

void ejecutarRiego(){
    sensor.valorActual = analogRead(SENSOR_HUMEDAD_PIN);
    Serial.print("VALOR>");
    Serial.println(sensor.valorActual);
    delay(1000);

    if(sensor.valorActual < DISPARO && sensor.valorAnterior > DISPARO){
        sensor.status = HUMEDO;
        digitalWrite(valvula.pin, HIGH);
        Serial.print("STATUS>");
        Serial.println("HUMEDO");
        sensor.valorAnterior = sensor.valorActual;
    }else if(sensor.valorActual > DISPARO && sensor.valorAnterior < DISPARO){
        sensor.status = SECO;
        digitalWrite(valvula.pin, LOW);
        Serial.print("STATUS>");
        Serial.println("SECO");
        valvula.timer = millis();
        while(!timer(valvula.timer, valvula.tiempo)){
        //continue
            delay(1000);
            Serial.println("STATUS>");
            Serial.println("REGANDO");
        }
        digitalWrite(valvula.pin, HIGH);
        Serial.println("STATUS>");
        Serial.println("DEJE DE REGAR");
        sensor.valorAnterior = sensor.valorActual;
    }else {
        Serial.print(" STATUS>");
        Serial.println("NADA A EJECUTAR");
    }  
}

