#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  // Inicializar la comunicación I2C
  Wire.begin();
  
  // Inicializar la pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error al inicializar la pantalla OLED");
    while (true);
  }

  // Limpiar la pantalla
  display.clearDisplay();
  
  // Configurar tamaño y color del texto
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  
  // Posicionar el cursor y mostrar el mensaje
  display.setCursor(10, 10);
  display.println("Hola, ESP32!");
  
  // Mostrar en la pantalla
  display.display();
}

void loop() {
  // Tu código del bucle principal aquí
}
