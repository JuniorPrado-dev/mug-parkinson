// I2Cdev e MPU6050 devem estar instalados como bibliotecas.
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
// O endereço do I2C é: 0x68
// Endereços específicos do I2C pode ser passados da seguinte forma:
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// Definindo os 3 servo motores
Servo servo0;
Servo servo1;
Servo servo2;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 4  // Será o PIN 2 na maioria das placas

bool blinkState = false;

// MPU control/status vars
// int8_t: Inteiro assinado de 1 byte 
// int16_t: Inteiro assinado de 2 bytes
bool dmpReady = false;  // torna-se verdadeiro se DMP for bem sucedido
uint8_t mpuIntStatus;   // mantém o byte de interrupção real do MPU
uint8_t devStatus;      // status de retorno após cada operação do dispositivo (0 = sucesso, !0 = erro)
uint16_t packetSize;    // tamanho esperado do pacote DMP (o padrão é 42 bytes)
uint16_t fifoCount;     // contagem de todos os bytes atuais na fila(FIFO)
uint8_t fifoBuffer[64]; // Buffer de armazenamento da fila(FIFO)

// Variaveis de orientação e movimentação
Quaternion q;           // [w, x, y, z]         "matriz" quaternion
VectorInt16 aa;         // [x, y, z]            medição do sensor de aceleração
VectorInt16 aaReal;     // [x, y, z]            medição do sensor de aceleração, sem gravidade
VectorInt16 aaWorld;    // [x, y, z]            medição do sensor de aceleração, no quadro global
VectorFloat gravity;    // [x, y, z]            vetor gravidade
float euler[3];         // [psi, theta, phi]    Vetor angulo de Euler
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll(Vertical/Transversal/Longit) container e vetor gravidade

// estrutura de pacotes para demonstração do "bule" InvenSense
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ======================================================================
// ===              ROTINA DE DETECÇÃO DE INTERRUPÇÕES                ===
// ======================================================================

volatile bool mpuInterrupt = false;     // indica se o pino de interrupção do MPU está high
void dmpDataReady() {
  mpuInterrupt = true;
}

// =================================================================
// ===                       SETUP INICIAL                       ===
// =================================================================

void setup() {
  // junte-se ao barramento I2C (a biblioteca I2Cdev não faz isso automaticamente)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // clock I2C de 400kHz. Comente esta linha se tiver dificuldades de compilação
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // inicializa a comunicação serial
  // (115200 escolhido porque é necessário para a saída Teapot Demo
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERRO!
    // 1 = falha no carregamento inicial da memória
    // 2 = Falha nas atualizações de configuração DMP
    // (se for quebrar, geralmente o código será 1)
    // Serial.print(F("Falha na inicialização do DMP (código "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // Definindo os pinos aos quais os 3 servo motores estão conectados
  servo0.attach(7);
  servo1.attach(6);
  servo2.attach(5);
}
// =================================================================
// ===                      PRINCIPAL LOOP                       ===
// =================================================================

void loop() {
  // Se a programação falhou, não tente fazer nada.
  if (!dmpReady) return;

  // Aguarde a interrupção do MPU ou pacote(s) extra(s) disponível(is).
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // redefinir o sinalizador de interrupção e obter o byte INT_STATUS
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT 4
// declarando o bit MPU6050_INTERRUPT_DMP_INT_BIT
#define MPU6050_INTERRUPT_DMP_INT_BIT 4
const uint8_t DMP_INT_BIT = MPU6050_INTERRUPT_DMP_INT_BIT;
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // redefinir para que possamos continuar de forma limpa
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Yaw, Pitch, Roll values - Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    
    // Skip 300 readings (self-calibration process)
    if (j <= 300) {
      correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
      j++;
    }
    // After 300 readings
    else {
      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
      // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
      int servo0Value = map(ypr[0], -90, 90, 0, 180);
      int servo1Value = map(ypr[1], -90, 90, 0, 180);
      int servo2Value = map(ypr[2], -90, 90, 180, 0);
      
      // Control the servos according to the MPU6050 orientation
      servo0.write(servo0Value);
      servo1.write(servo1Value);
      servo2.write(servo2Value);
    }
#endif
  }
}
