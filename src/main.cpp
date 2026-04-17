#include <Arduino.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <WiFi.h>

// ================= NETWORK & MQTT CONFIG =================
const char *ssid = "Huynh Hong";
const char *password = "123443215";
const char *mqtt_server = "interchange.proxy.rlwy.net";
const int mqtt_port = 50133;
const char *device_id = "device_001"; // Bắt buộc phải khớp với Main Node
const char *mqtt_user = "long";       // Thay bằng Username thực tế của Broker
const char *mqtt_pass =
    "53zx37kxq3epbexgqt6rjlce1d0e0gwq"; // Thay bằng Password thực tế của Broker

String topic_sensors = String("AGITECH/") + device_id + "/internal/sensors";
String topic_config = String("AGITECH/") + device_id + "/internal/config";

WiFiClient espClient;
PubSubClient client(espClient);

// ================= PIN MAPS =================
#define PIN_DS18B20 2
#define PIN_TRIG 3
#define PIN_ECHO 4
#define PIN_PH_ADC 0
#define PIN_EC_ADC 1

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ================= CALIBRATION =================
#define MAX_WINDOW 50
float ph_v7 = 2170.0, ph_v4 = 2800.0;
float ec_factor = 0.88, ec_offset = 0.0, temp_compensation_beta = 0.0;
int ma_window = 10;

#define V_REF_MV 3300.0
#define ADC_MAX 4095.0

float temp_history[MAX_WINDOW], water_history[MAX_WINDOW];
float ph_history[MAX_WINDOW], ec_history[MAX_WINDOW];
int history_idx = 0;

bool enable_ph = true;
bool enable_ec = true;
bool enable_temp = true;
bool enable_water = true;

// Các hàm đọc & tính toán (giữ nguyên logic gốc của bạn)
int read_adc_filtered(int pin) {
  int buffer[10];
  for (int i = 0; i < 10; i++) {
    buffer[i] = analogRead(pin);
    delay(5);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer[i] > buffer[j]) {
        int temp = buffer[i];
        buffer[i] = buffer[j];
        buffer[j] = temp;
      }
    }
  }
  long sum = 0;
  for (int i = 2; i < 8; i++)
    sum += buffer[i];
  return sum / 6;
}

float calc_average(float history[], float new_val) {
  history[history_idx] = new_val;
  float sum = 0;
  for (int i = 0; i < ma_window; i++)
    sum += history[i];
  return sum / ma_window;
}

float readWaterLevel() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 20000);
  if (duration == 0)
    return -1;
  return (duration / 2.0) * 0.0343;
}

float calculate_ph(float voltage_mv) {
  float diff = ph_v4 - ph_v7;
  float slope = (abs(diff) < 0.1) ? -0.006 : ((4.0 - 7.0) / diff);
  return constrain(7.0 + slope * (voltage_mv - ph_v7), 0.0, 14.0);
}

float calculate_ec(float voltage_mv, float temp) {
  float raw_ec = (voltage_mv / 1000.0) * ec_factor + ec_offset;
  float coef = 1.0 + temp_compensation_beta * (temp - 25.0);
  return max(raw_ec / coef, 0.0f);
}

// ================= MQTT CALLBACK (Nhận Config) =================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++)
    message += (char)payload[i];

  DynamicJsonDocument doc(512);
  if (!deserializeJson(doc, message)) {
    if (doc.containsKey("ph_v7"))
      ph_v7 = doc["ph_v7"].as<float>();
    if (doc.containsKey("ph_v4"))
      ph_v4 = doc["ph_v4"].as<float>();
    if (doc.containsKey("ec_f"))
      ec_factor = doc["ec_f"].as<float>();
    if (doc.containsKey("beta"))
      temp_compensation_beta = doc["beta"].as<float>();
    if (doc.containsKey("ma_window")) {
      ma_window = constrain(doc["ma_window"].as<int>(), 1, MAX_WINDOW);
    }
    if (doc.containsKey("en_ph"))
      enable_ph = doc["en_ph"].as<bool>();
    if (doc.containsKey("en_ec"))
      enable_ec = doc["en_ec"].as<bool>();
    if (doc.containsKey("en_temp"))
      enable_temp = doc["en_temp"].as<bool>();
    if (doc.containsKey("en_water"))
      enable_water = doc["en_water"].as<bool>();

    Serial.println("🔄 Đã nạp cấu hình mới từ Main Node!");
  }
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  sensors.begin();

  for (int i = 0; i < MAX_WINDOW; i++) {
    temp_history[i] = 25;
    water_history[i] = 20;
    ph_history[i] = 7;
    ec_history[i] = 0;
  }

  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT...");

    // 🟢 TẠO CLIENT ID DUY NHẤT (VD: SensorNode_device_001)
    String clientId = "SensorNode_device_001";
    clientId += String(device_id);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Thành công!");
      client.subscribe(topic_config.c_str());
    } else {
      Serial.print("Lỗi, rc=");
      Serial.print(client.state());
      Serial.println(" -> Thử lại sau 5 giây");
      delay(5000);
    }
  }
}

unsigned long last_time = 0;

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.reconnect();
    delay(5000);
    return;
  }
  if (!client.connected())
    reconnect();
  client.loop();

  if (millis() - last_time >= 1000) {
    last_time = millis();
    sensors.requestTemperatures();

    float temp = temp_history[history_idx]; // Giữ nguyên giá trị cũ nếu bị tắt
    if (enable_temp) {
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      if (t >= -50 && t <= 125)
        temp = t;
    }

    float water = water_history[history_idx];
    if (enable_water) {
      float w = readWaterLevel();
      if (w >= 0)
        water = w;
    }

    float ph = ph_history[history_idx];
    if (enable_ph) {
      float ph_mv = (read_adc_filtered(PIN_PH_ADC) / ADC_MAX) * V_REF_MV;
      ph = calculate_ph(ph_mv);
    }

    float ec = ec_history[history_idx];
    if (enable_ec) {
      float ec_mv = (read_adc_filtered(PIN_EC_ADC) / ADC_MAX) * V_REF_MV;
      ec = calculate_ec(ec_mv, temp);
    }

    float ph_mv = (read_adc_filtered(PIN_PH_ADC) / ADC_MAX) * V_REF_MV;
    float ec_mv = (read_adc_filtered(PIN_EC_ADC) / ADC_MAX) * V_REF_MV;

    float avg_temp = calc_average(temp_history, temp);
    float avg_water = calc_average(water_history, water);
    float avg_ph = calc_average(ph_history, calculate_ph(ph_mv));
    float avg_ec = calc_average(ec_history, calculate_ec(ec_mv, temp));

    history_idx = (history_idx + 1) % ma_window;

    // Đóng gói JSON gửi cho Main Node
    DynamicJsonDocument doc(256);
    doc["temp"] = avg_temp;
    doc["water"] = avg_water;
    doc["ph"] = avg_ph;
    doc["ec"] = avg_ec;

    String payload;
    serializeJson(doc, payload);
    client.publish(topic_sensors.c_str(), payload.c_str());

    Serial.println("📡 Đã gửi: " + payload);
  }
}
