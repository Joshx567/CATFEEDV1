#include <ESP32Servo.h>

Servo servo;  // Crear un objeto de tipo Servo

int servoPin = 21;  // El pin al que está conectado el servo

void setup() {
  servo.attach(servoPin);  // Asignar el pin al objeto servo
}

void loop() {
  // Mover el servo a la posición inicial
  servo.write(0);
  delay(1000);  // Esperar 1 segundo

  // Mover el servo a la posición central
  servo.write(90);
  delay(1000);  // Esperar 1 segundo

  // Mover el servo a la posición final
  servo.write(180);
  delay(1000);  // Esperar 1 segundo
}
