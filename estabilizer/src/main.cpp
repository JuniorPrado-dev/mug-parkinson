#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Servo servo;
Adafruit_MPU6050 comunic;

void setup(void) {
//Inicia o monitor serial
  Serial.begin(115200);
// Setando o PIN do servo
  servo.attach(5);
//Iniciando a biblioteca "Wire"
  Wire.begin();
//Iniciando a MPU6050
  comunic.begin();
// Setando a posição incial do servo para 0
  servo.write(0);

//Setando o range do acelerômetro e giroscópio
  comunic.setAccelerometerRange(MPU6050_RANGE_8_G);//2_G,4_G,8_G,16_G
  comunic.setGyroRange(MPU6050_RANGE_500_DEG);//250,500,1000,2000
  comunic.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {

/*Declara as variáveis "a", "g" e "temp" do tipo "sensors_event_t" que armazenarão 
os valores de aceleração, velocidade angular e temperatura, respectivamente.*/
  sensors_event_t a, g, temp;
  
/* Chama o método "getEvent" do objeto "comunic" e passa os endereços de memória das variáveis 
"a", "g" e "temp" como parâmetros para obter os valores atualizados do sensor.*/  
  comunic.getEvent(&a, &g, &temp);

/*Obtém o valor de aceleração no eixo Y a partir da variável "a" 
 e armazena na variável "valor".*/
  int valor = a.acceleration.y;

/*
 * Mapeia o valor da variável "valor" de uma faixa de -10 a 10 para uma faixa de 180 a 0, 
 * ou seja, converte o valor de aceleração para um ângulo de rotação do servo motor.
 */
  valor = map(valor,  -10, 10, 180, 0);
  
//Seta o movimento do servo
  servo.write(valor);
  
// Printa "valor" no serial 
  Serial.println(valor);
  //delay(10);
}