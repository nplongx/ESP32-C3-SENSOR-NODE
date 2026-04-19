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
const char *device_id = "device_001";
const char *mqtt_user = "long";
const char *mqtt_pass = "53zx37kxq3epbexgqt6rjlce1d0e0gwq";

String topic_sensors = String("AGITECH/") + device_id + "/internal/sensors";
String topic_config = String("AGITECH/") + device_id + "/internal/config";

WiFiClient espClient;
PubSubClient client(espClient);

// ================= PIN MAPS =================
// Cấu hình chân giữ nguyên - Hoàn toàn hợp lệ trên ESP32-C3
#define PIN_DS18B20 2
#define PIN_TRIG 3
#define PIN_ECHO 4
#define PIN_PH_ADC 0 // ADC1_CH0 trên ESP32-C3
#define PIN_EC_ADC 1 // ADC1_CH1 trên ESP32-C3

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// ================= CALIBRATION & CONFIG =================
#define MAX_WINDOW 50

// 🟢 Đã cập nhật cho pH-4502C (5V qua phân áp)
// ph_v7 = 2500mV (Điểm 0 của mạch khi nối tắt rắc BNC)
float ph_v7 = 2650.0, ph_v4 = 3555.0;
float ec_factor = 0.88, ec_offset = 0.0;
int ma_window = 10;

#define V_REF_MV 3300.0
#define ADC_MAX 4095.0
#define VOLTAGE_DIVIDER_RATIO 1.5 // 🟢 Hệ số phân áp (Điện trở 10k và 20k)

float temp_history[MAX_WINDOW], water_history[MAX_WINDOW];
float ph_history[MAX_WINDOW], ec_history[MAX_WINDOW];
int history_idx = 0;

// Cờ bật/tắt cảm biến
bool enable_ph = true;
bool enable_ec = true;
bool enable_temp = true;
bool enable_water = true;

// CỜ BÙ NHIỆT (Tách riêng biệt)
bool enable_ec_tc = true;            // Bù nhiệt EC
bool enable_ph_tc = true;            // Bù nhiệt pH
float temp_compensation_beta = 0.02; // Hệ số 2% mỗi độ C cho EC (theo tài liệu)

// ================= HÀM TIỆN ÍCH =================

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

// 🟢 TÍNH TOÁN pH (CÓ CƠ CHẾ BÙ NHIỆT THEO PHƯƠNG TRÌNH NERNST)
float calculate_ph(float voltage_mv, float current_temp) {
  float diff = ph_v4 - ph_v7;
  // Module 4502C: pH thấp thì điện áp cao (Slope âm)
  float slope = (abs(diff) < 0.1) ? -0.006 : ((4.0 - 7.0) / diff);

  if (enable_ph_tc) {
    float temp_ratio = (current_temp + 273.15) / (25.0 + 273.15);
    slope = slope / temp_ratio;
  }

  return constrain(7.0 + slope * (voltage_mv - ph_v7), 0.0, 14.0);
}

// 🟢 TÍNH TOÁN EC (CÓ CƠ CHẾ BÙ NHIỆT THEO TÀI LIỆU)
float calculate_ec(float voltage_mv, float current_temp) {
  // EC thô (mS/cm)
  float raw_ec = (voltage_mv / 1000.0) * ec_factor + ec_offset;

  if (enable_ec_tc) {
    // Công thức: EC_compensated = EC_raw / (1 + 0.02 * (Temp - 25))
    float coef = 1.0 + temp_compensation_beta * (current_temp - 25.0);
    return max(raw_ec / coef, 0.0f);
  }
  return max(raw_ec, 0.0f);
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
    if (doc.containsKey("ma_window"))
      ma_window = constrain(doc["ma_window"].as<int>(), 1, MAX_WINDOW);

    if (doc.containsKey("en_ph"))
      enable_ph = doc["en_ph"].as<bool>();
    if (doc.containsKey("en_ec"))
      enable_ec = doc["en_ec"].as<bool>();
    if (doc.containsKey("en_temp"))
      enable_temp = doc["en_temp"].as<bool>();
    if (doc.containsKey("en_water"))
      enable_water = doc["en_water"].as<bool>();

    if (doc.containsKey("en_ec_tc"))
      enable_ec_tc = doc["en_ec_tc"].as<bool>();
    if (doc.containsKey("en_ph_tc"))
      enable_ph_tc = doc["en_ph_tc"].as<bool>();

    Serial.println("🔄 Đã nạp cấu hình mới từ Main Node!");
  }
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Cấu hình ADC cho ESP32-C3
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  sensors.begin();

  for (int i = 0; i < MAX_WINDOW; i++) {
    temp_history[i] = 25.0;
    water_history[i] = 20.0;
    ph_history[i] = 7.0;
    ec_history[i] = 0.0;
  }

  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT...");
    String clientId = "SensorNode_";
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

    // 1. ĐỌC NHIỆT ĐỘ ĐẦU TIÊN
    float current_temp =
        temp_history[(history_idx - 1 + ma_window) % ma_window];
    if (enable_temp) {
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      if (t >= -10 && t <= 80)
        current_temp = t;
    }

    // 2. ĐỌC CÁC CẢM BIẾN KHÁC
    float current_water =
        water_history[(history_idx - 1 + ma_window) % ma_window];
    if (enable_water) {
      float w = readWaterLevel();
      if (w >= 0)
        current_water = w;
    }

    float current_ph = ph_history[(history_idx - 1 + ma_window) % ma_window];
    if (enable_ph) {
      // 🟢 Đã nhân thêm VOLTAGE_DIVIDER_RATIO để khôi phục điện áp 5V
      float ph_mv = (read_adc_filtered(PIN_PH_ADC) / ADC_MAX) * V_REF_MV *
                    VOLTAGE_DIVIDER_RATIO;
      current_ph = calculate_ph(ph_mv, current_temp);

      // 🟢 IN DEBUG mV CỦA pH RA SERIAL MONITOR
      Serial.print("🛠️ DEBUG - pH mV (thực tếmtại cảm biến): ");
      Serial.println(ph_mv);
    }

    float current_ec = ec_history[(history_idx - 1 + ma_window) % ma_window];
    if (enable_ec) {
      // 🟢 Đã nhân thêm VOLTAGE_DIVIDER_RATIO để khôi phục điện áp 5V
      float ec_mv = (read_adc_filtered(PIN_EC_ADC) / ADC_MAX) * V_REF_MV *
                    VOLTAGE_DIVIDER_RATIO;
      current_ec = calculate_ec(ec_mv, current_temp);
    }

    // 3. TÍNH TRUNG BÌNH MẢNG BỘ LỌC
    float avg_temp = calc_average(temp_history, current_temp);
    float avg_water = calc_average(water_history, current_water);
    float avg_ph = calc_average(ph_history, current_ph);
    float avg_ec = calc_average(ec_history, current_ec);

    history_idx = (history_idx + 1) % ma_window;

    // 4. GỬI DỮ LIỆU
    DynamicJsonDocument doc(256);
    doc["temp"] = avg_temp;
    doc["water_level"] = avg_water;
    doc["ph"] = avg_ph;
    doc["ec"] = avg_ec;

    String payload;
    serializeJson(doc, payload);
    client.publish(topic_sensors.c_str(), payload.c_str());

    Serial.println("📡 Đã gửi: " + payload);
    Serial.println(
        "-----------------------------------"); // Đường kẻ vạch để dễ nhìn log
  }
}
