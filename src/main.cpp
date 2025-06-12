#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Preferences.h>
#include <PubSubClient.h>

void initWiFi();
void connectToWiFi(const String &ssid, const String &password);
void initMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void otaTask(void *parameter);
void Batterie();
void deepsleep();

Preferences preferences;
TaskHandle_t otaTaskHandle;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup()
{
    Serial.begin(115200);

    deepsleep();

    initWiFi();

    pinMode(36, INPUT);

    xTaskCreate(
        otaTask,       // Task function
        "OTA Task",    // Name of the task
        4096,          // Stack size (in words)
        NULL,          // Task input parameter
        1,             // Priority of the task
        &otaTaskHandle // Task handle
    );

    Wire.begin(15, 2); // Initialize I2C with GPIO 15 (SDA) and GPIO 2 (SCL)

    // Initialize MLX90614 sensor
    if (!mlx.begin())
    {
        Serial.printf("Error initializing MLX90614 sensor!\n");
    }
    Serial.printf("MLX90614 sensor initialized.\n");

    // initMQTT(); // Initialize MQTT connection
}

unsigned long lastTempReadTime = 0;

void loop()
{
    // Non-blocking temperature data read
    unsigned long currentMillis = millis();
    if (currentMillis - lastTempReadTime >= 100)
    {
        lastTempReadTime = currentMillis;

        float ambientTemp = mlx.readAmbientTempC();
        float objectTemp = mlx.readObjectTempC();
        float internalTemp = temperatureRead();
        String ambientTempStr = String(ambientTemp).c_str();
        String objectTempStr = String(objectTemp).c_str();
        String internalTempStr = String(internalTemp, 2);

        Serial.printf("Ambient: %s C, Object: %s C\n", ambientTempStr.c_str(), objectTempStr.c_str());

        // Publish sensor values to separate MQTT topics
        if (mqttClient.connected())
        {
            mqttClient.publish("mario/tempgun/temperature/ambient", ambientTempStr.c_str());
            mqttClient.publish("mario/tempgun/temperature/object", objectTempStr.c_str());
            mqttClient.publish("mario/tempgun/temperature/internal", internalTempStr.c_str());
            Serial.printf("Published to MQTT: ambient=%s, object=%s, internal=%s\n", ambientTempStr.c_str(), objectTempStr.c_str(), internalTempStr.c_str());
        }
    }

    mqttClient.loop(); // Ensure MQTT client stays connected and processes incoming messages

    Batterie(); // Read battery voltage on sensor VP (GPIO 36) with 1:1 voltage divider
}

void connectToWiFi(const String &ssid, const String &password)
{
    int connectionTries = 0;
    WiFi.begin(ssid, password);
    Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());

    while (WiFi.status() != WL_CONNECTED && connectionTries < 10)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Serial.print(".");
        connectionTries++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.printf("Connected to WiFi: %s\n", ssid.c_str());
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    }
}

void initWiFi()
{
    preferences.begin("wificonfig", false);

    String ssid1 = preferences.getString("SSID1");
    String password1 = preferences.getString("PASSWORD1");
    String ssid2 = preferences.getString("SSID2");
    String password2 = preferences.getString("PASSWORD2");

    preferences.end();

    connectToWiFi(ssid1, password1);

    if (WiFi.status() != WL_CONNECTED)
    {
        connectToWiFi(ssid2, password2);
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Failed to connect to any WiFi network. Restarting...");
        ESP.restart();
    }
    // Close Preferences after use
}

void initMQTT()
{
    preferences.begin("mqttconfig", false);

    String MqttServer = preferences.getString("MQTT_SERVER");
    int MqttPort = preferences.getInt("MQTT_PORT");
    String MqttUser = preferences.getString("MQTT_USER");
    String MqttPassword = preferences.getString("MQTT_PASSWORD");

    preferences.end();

    mqttClient.setServer(MqttServer.c_str(), MqttPort);
    mqttClient.setCallback(mqttCallback); // Set the callback function for incoming messages

    while (!mqttClient.connected())
    {
        Serial.println("Connecting to MQTT...");
        if (mqttClient.connect("ESP32Client", MqttUser.c_str(), MqttPassword.c_str()))
        {
            Serial.println("Connected to MQTT broker");
            // mqttClient.subscribe("sensor/commands"); // Example subscription
        }
        else
        {
            Serial.print("Failed to connect to MQTT. State: ");
            Serial.println(mqttClient.state());
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
}

void otaTask(void *parameter)
{
    ArduinoOTA.begin();
    for (;;)
    {
        ArduinoOTA.handle();
        // Serial.println("OTA handle");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}

void Batterie()
{
    static int raw = 0;
    raw += analogRead(36); // Accumulate ADC readings
    static int count = 0;

    if (count++ > 1000) // Average over 1000 readings
    {
        float R1 = 1e6; // 1 MΩ
        float R2 = 1e6; // 1 MΩ
        float voltage = ((raw / 1000.0) / 4095.0) * 3.3 * 2;

        Serial.printf("Batterie  = %.3f V (raw=%d)\n", voltage, raw);
        raw = 0;
        count = 0;

        if (mqttClient.connected())
        {
            mqttClient.publish("mario/tempgun/battery", String(voltage, 3).c_str());
            Serial.printf("Published battery voltage to MQTT: %s V\n", String(voltage, 3).c_str());
        }
    }
}

void deepsleep()
{
    preferences.begin("deepsleep", false);

    if (preferences.getBool("DEEPSLEEP", false))
    {
        preferences.putBool("DEEPSLEEP", false);
        Serial.printf("Entering deep sleep mode with no wakeup...\n");
        preferences.end();
        esp_deep_sleep_start();
    }

    preferences.putBool("DEEPSLEEP", true);
    Serial.printf("Restart now for deepsleep\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    preferences.putBool("DEEPSLEEP", false);

    preferences.end();
}