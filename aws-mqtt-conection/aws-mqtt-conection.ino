#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Ancho de la pantalla OLED en píxeles
#define SCREEN_HEIGHT 64 // Alto de la pantalla OLED en píxeles

// Inicializar objeto de la pantalla OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Servo servo;
//RTC_DS3231 reloj;

#define echo 16
#define trigger 15
float cm = 0;

// Set x509 CA root (must match server cert
const char AMAZON_ROOT_CA1[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char CERTIFICATE[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUcBW5mnftT6tDOpRtQ2FbgrRacRcwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIzMDUyMjAyMjcz
NVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMgr8L8djug/U732xHJe
GAl1+elUjqYF72FSfCB4IZ4l5TQMy9H08QZG79WZ1qtTm8khyvLO6YzpqFb+jmhA
aMtzGESq04cYNAz4wSOZUfxlOw1+PshQiG1KNBsN42B5BNguyQ1eFxwZMRJTeYFD
9/bTwq5dOxvvFxQ5kdVROQ2QwEInAfa9N89SJd+etA25uMYFW1AEgyCzCLsOXWO9
1jj/VYLotC9HolqrvMWiSOeY/g4lLknchPQQ03iPR6+WzxY8iuJ9LoaptAV0tU+F
skQvdxpigRoqmg+sCFWuF0f8cDdhsizaALss7Dn9Cxr4cAGdYT/cicLNiKWTa/be
m0MCAwEAAaNgMF4wHwYDVR0jBBgwFoAUYy/dIAM4tQaML1pFGiRkNAsLeTUwHQYD
VR0OBBYEFEGuUSvdldxlSWV6wUa4bCdd+jnCMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCQtXtl7Ow8bosvfTq4VV4IWJWa
sohrPzG2G+4X9k9zroi1XnyRK4NIl2o8uO1pPag3YgEKcBEryNBjGJ4lD8JBrlLo
XW9dKyuzrEsWYOtdL41H/n1WwXb5zY7ORTZDFRsCRH9OxYUSL/VuyPhc4BBYY9vl
vwMGapdFcEBt8+7pPp2VYglL68eLoyy1fnzt4TXEmphwB+vH+xGqzXa2OXl753tB
TgrZx9ylmPt+tGh6+UZ6E3gWSJW4DO0rlAglNq6qZpYPWW0bzT/grNXO0InXznov
mbiHA6ezx2C+krVCu1aiNpn4Exn+1kp8MFkTrAc9G0buFnt1QclhtHGnDBl/
-----END CERTIFICATE-----

)KEY";

const char PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAyCvwvx2O6D9TvfbEcl4YCXX56VSOpgXvYVJ8IHghniXlNAzL
0fTxBkbv1ZnWq1ObySHK8s7pjOmoVv6OaEBoy3MYRKrThxg0DPjBI5lR/GU7DX4+
yFCIbUo0Gw3jYHkE2C7JDV4XHBkxElN5gUP39tPCrl07G+8XFDmR1VE5DZDAQicB
9r03z1Il3560Dbm4xgVbUASDILMIuw5dY73WOP9Vgui0L0eiWqu8xaJI55j+DiUu
SdyE9BDTeI9Hr5bPFjyK4n0uhqm0BXS1T4WyRC93GmKBGiqaD6wIVa4XR/xwN2Gy
LNoAuyzsOf0LGvhwAZ1hP9yJws2IpZNr9t6bQwIDAQABAoIBAQCqkz8hLSgFY0mS
PIGkz+f9z+n52pOAiNg3f9nEtXo4USSjGTBqpybo6XQg+mpDLwk2+TP/ezDsYoSQ
NXXXAuQKUvbgNQ3yNj7MuwGiu1lxEjajRexsOKw5LZ1/XHNhV184dY3gdUiOKi11
52pILEpWrJdejJX9NN3e3zwVopRSqIAau+R2G8W/zr2m05ljbIowSBFQrksidIp5
sOMmkSE70Gc81KkghNs/brmSNNI2/4FLIq49wYmD6gKglR26Nmeu1oBCiljEIsev
wW/gVLW4vGoyfT+dq5+w8Qtl5GXT77bkPXi9tPndHjaPbmmZP4ADgQScVFw7gX0a
i5wz/21hAoGBAOnN6/yFtbRDhe+VJ75Izxw2Y9vgUZdcNpn4g7JpvmWlJ1DxNBq1
cvsWrPze4R3+lGlxJroMJwY75kQ+hTpsUCybvhNgFyDgXOZkqccUPVpop1sZrJZi
heQWpamSyA2wESATG0aCOQqW887d04pXEoSPz9AjTWlf+yCN49wWTCpTAoGBANss
pvMUBRbvkwjJt/km7SAW16/BxZQYk1kgj7ve9K6gS6XXOLkU2KWmlj6hMPfh6yub
EHHBKfstHT2Y+V6P8SF1G2XjcoLXOgOxgCnWeDjrHKXpU2ltZo4sGuZuz74OummI
TMKgu7l9pFSJ5qMBG3DUEIBOIU01Yei2p2wNBQ1RAoGAMAszkfl75CX38Mpql0xo
2rhGftkyivLU+YC6tz/JcyOj15spBNACZ5w6QeAezEYTAzPQLGzw2/QVFjLut5mo
Q/Jg9aRKPGQ+Us2eoZ0EGn7k3PCCqYhnUP5iXl0eJMnoSvoCIdQCmq14PisRHB1W
Xc0be/61GpExjnf5ubEPR6cCgYBmPdIzmqc1J9VDSDZWN95uFbnP92ifyUcTUKk8
Tql9vLENT/TaAet+etOjq7YYLH9z/AYTyrQ2w7jcPzw07Jjjrql1QkDs4FHj3y2W
VsIGa0cV7l5G1A3/THvP+ulWE7DKAhcsw6ZOxVmfRiUc7StL2uFHuSq3xatCz9IS
CZDNwQKBgH9LSK2R2XYWcxuXI0aIVV12K9NkOMlUjeR3dqZzlICXXdSe4qEn78Ix
WXWmyyM2ZGOXDW+HTma5CKs7Gc+DSYTtCs59EW/Xub1Bb8RelZ2wa8BhPvDZ+Ifh
d0EFOg4mdxqnvO/vO8n/p94T5Xht+MiAg2ChnggqhIUt8XO2BLGq
-----END RSA PRIVATE KEY-----
)KEY";

//internet
const char *WIFI_SSID = "LOPEZ"; //nombre wifi 
const char *WIFI_PASS = "76486651"; //password

//mqtt
//aue6jf87pppg6-ats.iot.us-east-1.amazonaws.com server mqtt broker 
//arn:aws:iotanalytics:us-east-1:216901685860:channel/gato_channel canal gato_channel

const char *SERVER_ADDRESS = "aue6jf87pppg6-ats.iot.us-east-1.amazonaws.com"; //server a conectar
const int SERVER_PORT = 8883; //1883 si no tiene seguridad sino 8883 si tiene seguridad
const char* USERNAME = "clientarduino"; 
const char* PASSWORD = "12345678";
const char* topic_subcribe = "distanciaGATO";
const char* topic_publish = "distanciaGATO";
const char* MQTT_CLIENT_ID = "clientIDrandom";

//channel
// Parámetros de AWS IoT Analytics
//const char* awsEndpoint = "aws_endpoint";  // Endpoint de AWS IoT Analytics
//const char* awsKeyID = "access_key_id";    // ID de clave de acceso
//const char* awsSecretKey = "secret_access_key";  // Clave de acceso secreta
//const char* awsRegion = "us-east-1";       // Región de AWS IoT Analytics
//const char* awsChannelARN = "arn:aws:iotanalytics:us-east-1:account-id:dataset/dataset-name"; // ARN del Data Store
//

//
//const char* GET_ACCEPTED_TOPIC = "$aws/things/test_thing/shadow/get/accepted";
//const char* UPDATE_ACCEPTED_TOPIC = "$aws/things/test_thing/shadow/update/accepted";
//const char* GET_TOPIC = "$aws/things/test_thing/shadow/get";
//const char* UPDATE_TOPIC = "$aws/things/test_thing/shadow/update";
//
//declaracion de variables
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

//funciones para mqtt
void callback(char* topic, byte* payload, unsigned int length); //funcion que reciba los mensajes
void reconnect();//reconectar por si la conexion se pierde al mqtt
void setup_wifi(); //conexion a internet
//lectura sensor ultrasonico
long readUltrasonicDistance(int triggerPin, int echoPin);

long readUltrasonicDistance(int triggerPin, int echoPin)
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH); //Se devolvera el tiempo entre el envio y la recepcion
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nÏP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  String incoming = "";
  Serial.print("Mensaje recibido desde -> ");
  Serial.print(topic);
  Serial.println("");
  for (int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }
  incoming.trim();
  Serial.println("Mensaje -> " + incoming);
}

void reconnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Intentando conexión Mqtt...");
    // Creamos un cliente ID
    String clientId = "ARDUINO_";
    clientId += String(random(0xffff), HEX);
    // Intentamos conectar
    if (mqttClient.connect(clientId.c_str(), USERNAME, PASSWORD))
    {
      Serial.println("Conectado!");
      // Nos suscribimos
      if (mqttClient.subscribe(topic_subcribe))
      {
        Serial.println("Suscripcion ok");
      }
      else
      {
        Serial.println("fallo Suscripción");
      }
    }
    else
    {
      Serial.print(" falló :( con error -> ");
      Serial.print(mqttClient.state());
      Serial.println(" Intentamos de nuevo en 3 segundo");
      delay(3000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  // Inicializa la comunicación I2C
 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  display.clearDisplay();  // Limpiar la pantalla
  display.setTextSize(2); // Tamaño del texto: 2
  display.setTextColor(SSD1306_WHITE); // Color del texto: blanco
  display.setCursor(10, 10); // Posición del cursor en píxeles
  display.println("Hola, ESP32!"); // Texto a mostrar
  display.display(); // Mostrar en la pantalla  

  ///////////////////////////
  delay(1000);
    
  //conexion a internet
  setup_wifi();
  
  //uso de certificados 
  wifiClient.setCACert(AMAZON_ROOT_CA1);
  wifiClient.setCertificate(CERTIFICATE);
  wifiClient.setPrivateKey(PRIVATE_KEY);

  mqttClient.setBufferSize(4096);
  //iniciar servidor mqtt
  mqttClient.setServer(SERVER_ADDRESS, SERVER_PORT);
  mqttClient.setCallback(callback);
}

void loop()
{
  delay(3500); //envia cada mensaje cada 3,5 seg
  cm = 0.01723 * readUltrasonicDistance(trigger, echo); //se calculara la distancia multiplicando la velocidad en la que el sonido recorre un centimetro por el tiempo de rebote obtenido.
  Serial.println(cm);
  //transforma un json los cms
  DynamicJsonDocument doc(128);
  doc["distancia"] = cm;  

  // Mostrar un mensaje en la pantalla OLED
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("Hola, ESP32!");
  display.display();
  
  if (!mqttClient.connected()) //si el cliente no se conecto, que reconecte con su funcion reconnect
  {
    reconnect();
  }

  if (mqttClient.connected())
  {
    char buffer[128];
    size_t len = serializeJson(doc, buffer);
    mqttClient.publish(topic_publish, buffer, len); //publica si esta conectado al topico asignado en la variable global la distancia leida por el ultrasonido
    delay(30000); //envia cada mensaje cada 30 seg, evita tener que enviar a cada rato el codigo
  }
  
  mqttClient.loop();
}
