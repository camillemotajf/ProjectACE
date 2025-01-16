#include <Arduino.h>
#include <HCSR04.h>
#include <Ultrasonic.h>

// Sensor Ultrassonico
#define TRIG 4
#define ECHO 5

// Primeiro Motor
#define IN1_PIN 17
#define IN2_PIN 16
#define ENC1_A 1
#define ENC1_B 0

// Segundo Motor
#define IN3_PIN 8
#define IN4_PIN 9
#define ENC2_A 2
#define ENC2_B 3

// Sensor de Linha
#define A 18
#define B 19
#define C 20
#define ANALOG_PIN 28


int distance;
int sensor_values[5];
Ultrasonic ultrassonic(TRIG, ECHO);

uint32_t last_time;
float motor1_rpm = 0.0;           // Velocidade do motor 1 em RPM
float motor2_rpm = 0.0;           // Velocidade do motor 2 em RPM

// Pulsos por revolução do encoder 
const int PPR = 120;

volatile int encoder1_count = 0;
volatile int encoder2_count = 0;


// Funções de interrupção para encoders
void encoder1_ISR() {
  if (digitalRead(ENC1_B) == HIGH){
    encoder1_count++;
  }
  else{
    encoder1_count--;
}
}

void encoder2_ISR() {
  if (digitalRead(ENC2_B) == HIGH)
    encoder2_count++;
  else
    encoder2_count--;
}

void set_motor_speed(int speed, int IN1, int IN2) {
  if (speed < 0) { 
    // Velocidade negativa: motor gira para trás
    analogWrite(IN1, -speed); // Aplica o PWM no IN1
    analogWrite(IN2, 0);      // Mantém IN2 em LOW
  } else if (speed > 0) {
    // Velocidade positiva: motor gira para frente
    analogWrite(IN1, 0);      // Mantém IN1 em LOW
    analogWrite(IN2, speed);  // Aplica o PWM no IN2
  } else {
    // Velocidade zero: motor parado
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}

int selectChannel(uint8_t chan)
{
  //0b00000101 & 0b00000001 = 0b00000001  = 1
  //(0b00000101 >> 1) & 0x01;
  if (chan > 7) return -1;

  bool bit1 = chan & 0x01;
  bool bit2 = (chan >> 1) & 0x01;
  bool bit3 = (chan >> 2) & 0x01;

  digitalWrite(A, bit1);
  digitalWrite(B, bit2);
  digitalWrite(C, bit3);

  Serial.print("channel: ");
  Serial.print(bit3);
  Serial.print(" ");
  Serial.print(bit2);
  Serial.print(" ");
  Serial.println(bit1);

  delay(10);
  return analogRead(ANALOG_PIN);
}

void reorderSensors(int *values) {
  // Nova ordem dos índices: [1, 0, 2, 3, 4] para sequenciar [IR1, IR2, IR3, IR4, IR5]
  int reordered[5];
  
  reordered[0] = values[1]; // IR1
  reordered[1] = values[0]; // IR2
  reordered[2] = values[2]; // IR3
  reordered[3] = values[3]; // IR4
  reordered[4] = values[4]; // IR5
  
  // Copiar de volta os valores reorganizados para o vetor original
  for (int i = 0; i < 5; i++) {
    values[i] = reordered[i];
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  

  // Configurar interrupções para os encoders
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2_ISR, CHANGE);

  last_time = millis();  // Inicializa o tempo

}

void loop() {

  uint32_t current_time = millis();
  uint32_t delta_time = current_time - last_time;  // Tempo decorrido (ms)
  int motor1_speed = 150; // Velocidade do motor 1 (valores de 0 a 255)
  int motor2_speed = 200; // Velocidade do motor 2 (valores de 0 a 255)

  // Leitura do sensor ultrassonico
  distance = ultrassonic.read(CM);

  // Define a velocidade do motor 1
  set_motor_speed(motor1_speed, IN1_PIN, IN2_PIN);

  // Define a velocidade do motor 2
  set_motor_speed(motor2_speed, IN3_PIN, IN4_PIN);
  

  if (delta_time >= 1000) {  // Atualizar a cada 1 segundo

    // Calcular RPM do Motor 1
    motor1_rpm = (encoder1_count / (float)PPR) * (60000.0 / delta_time);
    encoder1_count = 0;  // Resetar contagem para o próximo intervalo

    // Calcular RPM do Motor 2
    motor2_rpm = (encoder2_count / (float)PPR) * (60000.0 / delta_time);
    encoder2_count = 0;  // Resetar contagem para o próximo intervalo

    last_time = current_time;
  }
  // Imprimir valores no monitor serial
    Serial.print("Motor 1 RPM: ");
    Serial.println(motor1_rpm);
    Serial.print("Motor 2 RPM: ");
    Serial.println(motor2_rpm);
    Serial.print("Distância sensor: ");
    Serial.println(distance);

    Serial.println("Leitura IR: ");
    for (uint8_t channel = 0; channel < 5; channel++) {
      sensor_values[channel] = selectChannel(channel); // Read directly into the array
    }
    reorderSensors(sensor_values);

    // Print all sensor values in one go
    Serial.print("Sensor values: ");
    for (uint8_t i = 0; i < 5; i++) 
    {
      Serial.print(sensor_values[i]);
      if (i < 4) Serial.print(", ");
    }
    Serial.println(); 


}

