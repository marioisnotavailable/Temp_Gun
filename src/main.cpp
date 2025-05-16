#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <LittleFS.h>

void initWiFi();
void connectToWiFi(const String &ssid, const String &password);
void initMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void initLittleFS();

Preferences preferences;

TaskHandle_t otaTaskHandle;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void otaTask(void *parameter) {
    ArduinoOTA.begin();
    for (;;) {
        ArduinoOTA.handle();
        //Serial.println("OTA handle");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}

void setup() {
    Serial.begin(115200);
    initWiFi();
    initLittleFS();

    xTaskCreate(
        otaTask,          // Task function
        "OTA Task",      // Name of the task
        4096,             // Stack size (in words)
        NULL,             // Task input parameter
        1,                // Priority of the task
        &otaTaskHandle    // Task handle
    );

    // Initialize I2C with custom pins
    Wire.begin(15, 2); // SDA on GPIO 23, SCL on GPIO 24

    // Initialize MLX90614 sensor
    if (!mlx.begin()) {
        Serial.println("Error initializing MLX90614 sensor! Restarting...");
    }
    Serial.println("MLX90614 sensor initialized.");

    initMQTT(); // Initialize MQTT connection
}

unsigned long lastTempReadTime = 0;

void loop() {
    // Non-blocking temperature data read
    unsigned long currentMillis = millis();
    if (currentMillis - lastTempReadTime >= 100) {
        lastTempReadTime = currentMillis;

        float ambientTemp = mlx.readAmbientTempC();
        float objectTemp = mlx.readObjectTempC();

        String tempMessage = "Ambient: " + String(ambientTemp) + " C, Object: " + String(objectTemp) + " C";
        Serial.println(tempMessage);

        // Publish sensor values to separate MQTT topics
        if (mqttClient.connected()) {
            String ambientPayload = String(ambientTemp);
            String objectPayload = String(objectTemp);
            mqttClient.publish("mario/tempgun/temperature/ambient", ambientPayload.c_str());
            mqttClient.publish("mario/tempgun/temperature/object", objectPayload.c_str());
            Serial.println("Published to MQTT: ambient=" + ambientPayload + ", object=" + objectPayload);
        }
    }

    mqttClient.loop(); // Ensure MQTT client stays connected and processes incoming messages

    // Removed ArduinoOTA.handle() from loop
}

void connectToWiFi(const String &ssid, const String &password) {
    int connectionTries = 0;
    WiFi.begin(ssid, password);
    Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());

    while (WiFi.status() != WL_CONNECTED && connectionTries < 20) {
        delay(1000);
        Serial.print(".");
        connectionTries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nConnected to WiFi: %s\n", ssid.c_str());
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    }
}

void initWiFi() {
    preferences.begin("wificonfig", false);

    String ssid1 = preferences.getString("SSID1");
    String password1 = preferences.getString("PASSWORD1");
    String ssid2 = preferences.getString("SSID2");
    String password2 = preferences.getString("PASSWORD2");

    preferences.end();

    connectToWiFi(ssid1, password1);

    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi(ssid2, password2);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to any WiFi network. Restarting...");
        ESP.restart();
    }
 // Close Preferences after use
}

void initMQTT() {
    preferences.begin("mqttconfig", false);
    String MqttServer = preferences.getString("MQTT_SERVER");
    int MqttPort = preferences.getInt("MQTT_PORT");
    String MqttUser = preferences.getString("MQTT_USER");
    String MqttPassword = preferences.getString("MQTT_PASSWORD");
    preferences.end();

    mqttClient.setServer(MqttServer.c_str(), MqttPort);
    mqttClient.setCallback(mqttCallback); // Set the callback function for incoming messages

    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT...");
        if (mqttClient.connect("ESP32Client", MqttUser.c_str(), MqttPassword.c_str())) {
            Serial.println("Connected to MQTT broker");
            //mqttClient.subscribe("sensor/commands"); // Example subscription
        } else {
            Serial.print("Failed to connect to MQTT. State: ");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void initLittleFS(){
    if (!LittleFS.begin()) {
        Serial.println("An Error has occurred while mounting LittleFS");
    }
}