
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// Flash
#define LED_BUILTIN 4

//WIFI config
const char* ssid = "HUAWEI P40 Pro";
const char* password = "huawei40";

//MQTT config

const char* topic_PUBLISH = "PICTURE";
const char* topic_PHOTO = "PHOTO";
const char* topic_DISTANCE = "DISTANCE";
const char* topic_DATE = "DATE";
const char* topic_SPEED = "SPEED";
const char* topic_LONGITUDE = "LONGITUDE";
const char* topic_LATITUDE = "LATITUDE";
const int MAX_PAYLOAD = 60000;

const char* mqtt_server = "192.168.43.42";

bool flash;

const int trigPin = 12;
const int echoPin = 13;

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
void startCameraServer();

void setup() {

  // Define Flash as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise the Serial Communication
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  Serial.setDebugOutput(true);
  Serial.println();

  // Config Camera Settings
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  flash = true;

  // Not used in our project
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  // Not used in our project
  #if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Set MQTT Connection
  client.setServer(mqtt_server, 1883);
  client.setBufferSize (MAX_PAYLOAD); //This is the maximum payload length
  client.setCallback(callback);
}

void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  if (topic == topic_PHOTO) {
    take_picture();
  }
  
}

void take_picture() {
  camera_fb_t * fb = NULL;
  if(flash){ digitalWrite(LED_BUILTIN, HIGH);};
  Serial.println("Taking picture");
  fb = esp_camera_fb_get(); // used to get a single picture.
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  Serial.println("Picture taken");
  digitalWrite(LED_BUILTIN, LOW);
  sendMQTT(fb->buf, fb->len);
  esp_camera_fb_return(fb); // must be used to free the memory allocated by esp_camera_fb_get().
  
}

void set_flash() {
    flash = !flash;
    Serial.print("Setting flash to ");
    Serial.println (flash);
    if(!flash){
      for (int i=0; i<6; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    }
    if(flash){
      for (int i=0; i<3; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    }
}

void sendMQTTDistance(const uint8_t * buf, uint32_t len){
  Serial.println("Sending distance...");
  if(len>MAX_PAYLOAD){
    Serial.print("Data too large, increase the MAX_PAYLOAD value");
  }else{
    Serial.print("Distance sent ? : ");
    Serial.println(client.publish(topic_PUBLISH, buf, len, false));
  }
  
  
}


void sendMQTT(const uint8_t * buf, uint32_t len){
  Serial.println("Sending picture...");
  if(len>MAX_PAYLOAD){
    Serial.print("Picture too large, increase the MAX_PAYLOAD value");
  }else{
    Serial.print("Picture sent ? : ");
    Serial.println(client.publish(topic_PUBLISH, buf, len, false));
  }
  
  
}

void reconnect() {
   while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topic_PUBLISH);
      client.subscribe(topic_PHOTO);
      client.subscribe(topic_DISTANCE);
      client.subscribe(topic_DATE);
      client.subscribe(topic_SPEED);
      client.subscribe(topic_LONGITUDE);
      client.subscribe(topic_LATITUDE);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);   
  }
}
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm =  (duration * SOUND_SPEED/2);
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (m): ");
  Serial.println(distanceCm/100);
  char msg_out[10];
  dtostrf(distanceCm/100,2,2,msg_out);

  client.publish(topic_DISTANCE, msg_out);
  delay(200);
  
  client.loop();
}
