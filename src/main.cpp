#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// --- KONFIGURASI FIREBASE ---
#define API_KEY "AIzaSyCuO-74xl-MsrqEtae9IIbn5bA1pn3BAfY"
#define DATABASE_URL "https://modang-f66ad-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_PROJECT_ID "modang-f66ad"
#define DEFAULT_USER_EMAIL "tes@gmail.com"
#define DEFAULT_USER_PASSWORD "tes123cuy"
#define DEVICE_ID "ESP001"

// --- KONFIGURASI PIN ---
#define TdsSensorPin 36
#define phPin 39
#define ONE_WIRE_BUS 4

// --- ALAMAT & UKURAN LCD ---
LiquidCrystal_I2C lcd(0x27, 20, 4); 

// --- PARAMETER TDS ---
#define VREF 3.3
#define ADC_RES 4096.0

#define TDS_AVG_DURATION 2000
float tdsSum = 0;
int tdsCount = 0;
unsigned long tdsAvgStartMillis = 0;
float tdsAverage = 0;

int analogBuffer[30];
int analogBufferIndex = 0;

// --- PARAMETER pH ---
float phAsli1 = 7.0; float tegangan1 = 1.51; 
float phAsli2 = 4.0; float tegangan2 = 0.27; 

// --- SETUP DS18B20 ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variabel Global Data
float currentTemperature = 25.0;
float currentTDS = 0;
float currentPH = 0;
float currentVoltagePH = 0;

// Variabel Pendukung WiFi & Firebase
Preferences preferences;
AsyncWebServer server(80);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson content;
bool wifiConnected = false;
unsigned long lastFirebaseMillis = 0;
unsigned long lastFirestoreMillis = 0;

// Prototypes
int getMedianNum(int bArray[], int iFilterLen);
void wiFiConnection();
void firebaseConnection();
void sendToFirestore(float PH, int TDS, float Temp);

void setup() {
  Serial.begin(115200);

Serial.println(" OK");
  
  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
    lcd.setCursor(1, 1);
    lcd.print("SYSTEM MONITORING");
    lcd.setCursor(4, 2);
    lcd.print("- MONIKA -");

    delay(1500);
    lcd.clear();
    
    // Inisialisasi Sensor
    pinMode(TdsSensorPin, INPUT);
    pinMode(phPin, INPUT);
    sensors.begin();

    // Inisialisasi WiFi & Firebase
    preferences.begin("wifi-config", false);

    // --- LOGIKA RECONNECT TERUS MENERUS ---
    bool isConnected = false;
    int attempt = 1;

    while (!isConnected) {
      lcd.clear();
      lcd.setCursor(2, 1);
      lcd.print("Connecting WiFi");
      Serial.print("Mencoba koneksi ke-");
      Serial.println(attempt);

      wiFiConnection(); // Memanggil fungsi koneksi WiFi Anda

      if (wifiConnected) {
        // JIKA BERHASIL
        isConnected = true;
        lcd.clear();
        lcd.setCursor(3, 1);
        lcd.print("WiFi: CONNECTED!");
        
        firebaseConnection();
        delay(1500);
      } else {
        // JIKA GAGAL
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("WiFi FAILED!");
        
        // Tetap nyalakan AP Mode jika diperlukan untuk setting ulang
        
        delay(3000); // Jeda sebelum mencoba lagi
        attempt++;
      }
    }

    if (wifiConnected) {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("Sinkron waktu");
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
      Serial.print(".");
      delay(500);
    }
    Serial.println(" OK");
  }
}

void loop() {
  // 1. SAMPLING TDS (Cepat - 40ms) - TIDAK DIUBAH
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex++] = analogRead(TdsSensorPin);
    if (analogBufferIndex == 30) analogBufferIndex = 0;
  }

  // 2. UPDATE DATA & LCD (Lambat - 1 Detik) - TIDAK DIUBAH
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();

    // A. BACA SUHU
    sensors.requestTemperatures();
    float tempReading = sensors.getTempCByIndex(0);
    if (tempReading != 85.00 && tempReading != -127.00) {
      currentTemperature = tempReading;
    }

    // B. HITUNG TDS
    float rawVoltage = getMedianNum(analogBuffer, 30) * VREF / ADC_RES;
    float offset = 0.14;
    float ec = rawVoltage - offset;
    if (ec < 0) ec = 0;
    float ecComp = ec / (1.0 + 0.02 * (currentTemperature - 25.0));


    float tdsInstant =
    (133.42 * pow(ecComp, 3)
    - 255.86 * pow(ecComp, 2)
    + 857.39 * ecComp) * 0.5;

// --- PROSES RATA-RATA 1–2 DETIK ---
  if (tdsAvgStartMillis == 0) {
  tdsAvgStartMillis = millis();
  }

tdsSum += tdsInstant;
tdsCount++;



if (millis() - tdsAvgStartMillis >= TDS_AVG_DURATION) {
  tdsAverage = tdsSum / tdsCount;

  // reset buffer
  tdsSum = 0;
  tdsCount = 0;
  tdsAvgStartMillis = millis();
}

    // C. HITUNG pH
    float phRawSum = 0;
    for (int i = 0; i < 20; i++) { phRawSum += analogRead(phPin); }
    currentVoltagePH = (phRawSum / 20.0) * (3.3 / 4095.0);
    currentPH = phAsli1 + (currentVoltagePH - tegangan1) * (phAsli2 - phAsli1) / (tegangan2 - tegangan1);
    if (currentPH < 0) currentPH = 0;
    if (currentPH > 14) currentPH = 14;

    float phClipped   = round(currentPH * 10.0) / 10.0 + 0.6 ;
    int tdsClipped = (int)(tdsAverage * 10 + 0.5 ) - 660;
    if (tdsClipped < 0) tdsClipped = 0;
    float tempClipped = round(currentTemperature * 10.0) / 10.0;

    // D. TAMPILKAN KE LCD 20x4
    lcd.setCursor(0, 0);
    lcd.print("--- WATER MONITOR ---");
    lcd.setCursor(0, 2);
    lcd.print("PH Air   : "); lcd.print(phClipped, 2); lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("TDS      : "); lcd.print((int)tdsClipped); lcd.print(" ppm   ");
    lcd.setCursor(0, 3);
    lcd.print("Suhu Air : "); lcd.print(tempClipped, 1);
    lcd.print((char)223); lcd.print("C  ");

    // E. OUTPUT KE SERIAL
    Serial.print("pH: "); Serial.print(phClipped);
    Serial.print(" | TDS: "); Serial.print(tdsClipped);
    Serial.print(" | Suhu: "); Serial.println(tempClipped);

    // F. KIRIM KE FIREBASE (Setiap 5 detik)
// F. KIRIM KE FIREBASE (Setiap 5 detik)
    if (wifiConnected && Firebase.ready() && (millis() - lastFirebaseMillis > 5000)) {
        lastFirebaseMillis = millis();
        
        String path = "/" + String(DEVICE_ID); 
        
        // Kirim data dengan menggabungkan path folder dan nama sensor
        Firebase.RTDB.setFloat(&fbdo, (path + "/PH").c_str(), (phClipped));
        Firebase.RTDB.setInt(&fbdo, (path + "/TDS").c_str(), (tdsClipped));
        Firebase.RTDB.setFloat(&fbdo, (path + "/Temp").c_str(), (tempClipped));
        
        Serial.print("Data Terkirim ke folder: ");
        Serial.println(DEVICE_ID);
    } 


    if (wifiConnected && Firebase.ready() &&
    millis() - lastFirestoreMillis > 3600000) { // TEST 1 detik
    
    lastFirestoreMillis = millis();
    sendToFirestore(phClipped, tdsClipped, tempClipped);
}

  }
}

// --- FUNGSI PENDUKUNG (WIFI & FIREBASE) ---

String getMonthKey() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "unknown";
    char buf[8];
    strftime(buf, sizeof(buf), "%Y-%m", &timeinfo);
    return String(buf);
}

String getHourKey() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "00-00";

    char buf[6];  
    strftime(buf, sizeof(buf), "%d-%H", &timeinfo);
    return String(buf);
}


void sendToFirestore(float PH, int TDS, float Temp) {

  String monthKey = getMonthKey();   // "2026-01"
  String hourKey  = getHourKey();    // "18"

  String documentPath =
    "devices/" + String(DEVICE_ID) +
    "/" + monthKey +
    "/" + hourKey;

  FirebaseJson content;
  content.set("fields/values/mapValue/fields/PH/doubleValue", PH);      // PH besar
  content.set("fields/values/mapValue/fields/TDS/integerValue", TDS);   // TDS besar
  content.set("fields/values/mapValue/fields/Temp/doubleValue", Temp);  // Temp (atau TEMP)

  // updateMask harus match
  String updateMask = "values.PH,values.TDS,values.Temp";

  if (Firebase.Firestore.patchDocument(
        &fbdo,
        FIREBASE_PROJECT_ID,
        "",
        documentPath.c_str(),
        content.raw(),
        updateMask.c_str())) {

    Serial.println("✅ Firestore: jam ini ke-update");
  } else {
    Serial.print("❌ Firestore error: ");
    Serial.println(fbdo.errorReason());
  }
}


void wiFiConnection() {
  String ssid = preferences.getString("ssid", "DESA SARIROGO 4G");
  String pass = preferences.getString("password", "sarirogo2025");
  if (ssid == "" || pass == "") return;
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(500); }
  if (WiFi.status() == WL_CONNECTED) wifiConnected = true;
}

void firebaseConnection() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = DEFAULT_USER_EMAIL;
  auth.user.password = DEFAULT_USER_PASSWORD;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i]; bTab[i] = bTab[i + 1]; bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
  else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}