#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "icon.h"
#include <Audio.h>

// 📺 Cấu hình màn hình OLED
#define i2c_Address 0x3C // Địa chỉ I2C của màn hình OLED, thường là 0x3C cho các màn OLED thông dụng
#define SCREEN_WIDTH 128 // Chiều rộng màn hình OLED (pixel)
#define SCREEN_HEIGHT 64 // Chiều cao màn hình OLED (pixel)
#define OLED_RESET -1    // Chân reset cho OLED (không dùng trong QT-PY / XIAO)
#define OLED_SDA 21      // Chân SDA cho giao tiếp I2C
#define OLED_SCL 22      // Chân SCL cho giao tiếp I2C
Adafruit_SH1106G oled = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Đối tượng màn hình OLED
#define WHITE 1          // Màu trắng cho OLED
#define BLACK 0          // Màu đen cho OLED
#define NUMFLAKES 10     // Số bông tuyết cho hiệu ứng hoạt hình (chưa sử dụng trong mã)
#define XPOS 0           // Chỉ số vị trí X cho hoạt hình
#define YPOS 1           // Chỉ số vị trí Y cho hoạt hình
#define DELTAY 2         // Độ lệch Y cho hoạt hình

// 📋 Các trạng thái màn hình
typedef enum {
  SCREEN0,  // Màn hình khởi động
  SCREEN1,  // Màn hình hiển thị số tiền giao dịch
  SCREEN2,  // (Chưa sử dụng)
  SCREEN3,  // (Chưa sử dụng)
  SCREEN4,  // (Chưa sử dụng)
  SCREEN5,  // Màn hình WiFi mất kết nối
  SCREEN6,  // (Chưa sử dụng)
  SCREEN7,  // (Chưa sử dụng)
  SCREEN8,  // (Chưa sử dụng)
  SCREEN9,  // Màn hình cấu hình WiFi (AP mode)
  SCREEN10, // (Chưa sử dụng)
  SCREEN11, // (Chưa sử dụng)
  SCREEN12, // (Chưa sử dụng)
  SCREEN13  // Màn hình khởi động lại
} SCREEN;

// 🔄 Biến quản lý trạng thái màn hình
#define ENABLE    1
#define DISABLE   0
int screenOLED = SCREEN0; // Trạng thái hiện tại của màn hình OLED
int countSCREEN9 = 0;     // Bộ đếm cho màn hình SCREEN9 (chưa sử dụng trong mã)
bool enableShow = DISABLE; // Kích hoạt hiển thị trên OLED (ENABLE/DISABLE)
int Volume = 30;          // Mức âm lượng cho module âm thanh

// 🔊 Cấu hình I2S cho âm thanh
#define I2S_DOUT 25 // Chân dữ liệu đầu ra I2S
#define I2S_BCLK 27 // Chân clock bit I2S
#define I2S_LRC 26  // Chân chọn kênh trái/phải I2S
Audio audio;        // Đối tượng điều khiển âm thanh
bool isPlaying = false; // Trạng thái phát âm thanh (true: đang phát, false: không phát)

// 📣 Cấu hình còi (buzzer)
#define BUZZER 4         // Chân điều khiển còi
#define BUZZER_ON 0      // Trạng thái bật còi
#define BUZZER_OFF 1     // Trạng thái tắt còi

// 🌐 Cấu hình Web Config
#define BUTTON_PIN 0        // Chân nút BOOT (GPIO0)
#define BOOT_HOLD_TIME 1500 // Thời gian giữ nút BOOT để vào chế độ cấu hình (ms)
#define AP_SSID "ESP32 Tingbox" // Tên WiFi AP khi ESP32 ở chế độ điểm truy cập
#define AP_PASS ""          // Mật khẩu WiFi AP (rỗng = không mật khẩu)
WebServer server(80);       // Máy chủ web chạy trên cổng 80
Preferences prefs;          // Đối tượng lưu trữ cấu hình trong bộ nhớ không xóa

// 📡 Biến lưu trữ cấu hình WiFi và Google Script
String wifi_ssid, wifi_pass, script_code; // SSID, mật khẩu WiFi và mã Google Script
String url_ting_ting = "https://tiengdong.com/wp-content/uploads/Tieng-tinh-tinh-www_tiengdong_com.mp3"; // URL âm thanh "ting ting"

// 📊 Cấu hình Google Sheet
String oldMaThamChieu = ""; // Lưu mã tham chiếu giao dịch mới nhất
bool isFirstFetch = true;   // Cờ đánh dấu lần đầu lấy dữ liệu từ Google Sheet

// 🧾 Cấu trúc dữ liệu giao dịch
struct Transaction {
  String nganHang;          // Tên ngân hàng
  String ngayGiaoDich;      // Ngày giao dịch
  String soTaiKhoan;        // Số tài khoản
  String taiKhoanPhu;       // Tài khoản phụ
  String codeTT;            // Mã thanh toán
  String noiDungThanhToan;  // Nội dung thanh toán
  String loai;              // Loại giao dịch
  String maThamChieu;       // Mã tham chiếu
  int soTien;               // Số tiền giao dịch
  int luyKe;                // Tổng lũy kế
};
Transaction t; // Biến lưu thông tin giao dịch hiện tại

// 📈 Trạng thái dữ liệu
#define DATAOK 2   // Dữ liệu lấy thành công
#define DATAERR 0  // Lỗi khi lấy dữ liệu
#define WIFI_DIS 1 // WiFi mất kết nối
int countTaskReadSepay = 0; // Bộ đếm cho tác vụ đọc dữ liệu từ Sepay
bool enableReadSepay = 0;   // Kích hoạt đọc dữ liệu từ Sepay
uint8_t trigAudio = 1;      // Cờ kích hoạt âm thanh (0: không phát, 1: phát "ting ting", 2: phát thông báo)
int checkGetData = DATAERR; // Trạng thái lấy dữ liệu (DATAOK, DATAERR, WIFI_DIS)
int isStart = 0;            // Cờ đánh dấu lần đầu chạy chương trình
int tienNhan[3] = {0, 0, 0}; // Mảng lưu 3 số tiền giao dịch gần nhất

/**
 * 🛠️ Hàm khởi tạo thiết bị
 * Khởi tạo các thành phần phần cứng và kết nối WiFi.
 */
void setup() {
  Serial.begin(115200); // Khởi tạo giao tiếp Serial với tốc độ 115200 baud
  loadConfig();        // Tải cấu hình WiFi và Google Script từ bộ nhớ
  printPrefs();        // In cấu hình đã lưu ra Serial

  // 📣 Khởi tạo còi
  pinMode(BUZZER, OUTPUT);        // Cấu hình chân còi là đầu ra
  digitalWrite(BUZZER, BUZZER_OFF); // Tắt còi ban đầu

  // 🖲️ Khởi tạo nút nhấn
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Cấu hình chân nút BOOT với điện trở kéo lên

  // 📺 Khởi tạo màn hình OLED
  oled.begin(i2c_Address, true); // Khởi động OLED với địa chỉ I2C
  oled.setTextSize(2);          // Đặt kích thước chữ là 2
  oled.setTextColor(SH110X_WHITE); // Đặt màu chữ là trắng
  for (int j = 0; j < 3; j++) { // Hiển thị animation khởi động
    for (int i = 0; i < FRAME_COUNT_loadingOLED; i++) {
      oled.clearDisplay(); // Xóa màn hình
      oled.drawBitmap(32, 0, loadingOLED[i], FRAME_WIDTH_64, FRAME_HEIGHT_64, 1); // Vẽ frame animation
      oled.display();     // Cập nhật hiển thị
      delay(FRAME_DELAY / 4); // Đợi giữa các frame
    }
  }

  // 🔊 Khởi tạo module âm thanh
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT); // Cấu hình chân I2S
  audio.setVolume(Volume);                      // Đặt mức âm lượng

  connectSTA(); // Kết nối WiFi ở chế độ Station
}

/**
 * 🔄 Vòng lặp chính
 * Xử lý các sự kiện: nhấn nút, đọc dữ liệu từ Google Sheet, phát âm thanh, hiển thị trên OLED.
 */
void loop() {
  // 🖲️ Kiểm tra nhấn nút BOOT để vào chế độ cấu hình
  static unsigned long bootPressStart = 0; // Thời điểm bắt đầu nhấn nút
  static bool isBootHeld = false;          // Trạng thái giữ nút
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!isBootHeld) {
      isBootHeld = true;
      bootPressStart = millis();
    } else if (millis() - bootPressStart > BOOT_HOLD_TIME) {
      Serial.println("Đã giữ nút BOOT 3 giây. Vào cấu hình!"); // Thông báo vào chế độ cấu hình
      buzzerBeep(3); // Phát 3 tiếng còi

      // Hiển thị thông tin cấu hình trên OLED
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(40, 5);
      oled.print("Ket noi Wifi:");
      oled.setCursor(40, 17);
      oled.print("TingBox");
      oled.setCursor(40, 38);
      oled.print("Dia chi IP:");
      oled.setCursor(40, 50);
      oled.print("192.168.4.1");
      clearRectangle(0, 0, 32, 64); // Xóa vùng hiển thị
      oled.drawBitmap(0, 16, settingOLED[0], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1); // Vẽ icon cài đặt
      oled.display();
      delay(FRAME_DELAY * 4);

      enterConfigAPMode(); // Chuyển sang chế độ AP
    }
  } else {
    isBootHeld = false;
    bootPressStart = 0;
  }

  // 📊 Kiểm tra và đọc dữ liệu từ Google Sheet
  countTaskReadSepay++;
  if (trigAudio == 0 && countTaskReadSepay >= 6 && enableReadSepay == 1) {
    countTaskReadSepay = 0;
    if (WiFi.status() == WL_CONNECTED) {
      String url = "https://script.google.com/macros/s/" + script_code + "/exec?sts=read"; // URL đọc dữ liệu
      HTTPClient http;
      http.begin(url);
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

      int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK) {
        checkGetData = DATAOK;
        String payload = http.getString();
        Serial.println(payload);
        const size_t capacity = JSON_ARRAY_SIZE(3) + 3 * JSON_ARRAY_SIZE(10) + 500; // Dung lượng JSON
        DynamicJsonDocument doc(capacity);
        auto error = deserializeJson(doc, payload);

        if (!error) {
          int size = doc.size();
          if (isFirstFetch) { // Lần đầu lấy dữ liệu
            for (int i = 0; i < size; i++) {
              Serial.print("Row "); Serial.print(i); Serial.print(": ");
              Serial.println(doc[i][8].as<String>());
            }
            if (size > 0) {
              oldMaThamChieu = doc[size - 1][8].as<String>(); // Lấy mã tham chiếu mới nhất
              Serial.println("[Lần đầu khởi động] Gán oldMaThamChieu = " + oldMaThamChieu);
              tienNhan[0] = doc[0][7].as<int>();
              tienNhan[1] = doc[1][7].as<int>();
              tienNhan[2] = doc[2][7].as<int>();
            }
            isFirstFetch = false;
          } else { // Các lần sau
            tienNhan[0] = doc[0][7].as<int>();
            tienNhan[1] = doc[1][7].as<int>();
            tienNhan[2] = doc[2][7].as<int>();
            int viTriOld = -1;
            for (int i = 0; i < size; i++) {
              if (doc[i][8].as<String>() == oldMaThamChieu) {
                viTriOld = i;
                break;
              }
            }

            bool foundNew = false;
            if (viTriOld >= 0 && viTriOld < (size - 1)) { // Có giao dịch mới
              int idxMoiNhat = viTriOld + 1;
              JsonArray row = doc[idxMoiNhat].as<JsonArray>();
              t.nganHang = row[0].as<String>();
              t.ngayGiaoDich = row[1].as<String>();
              t.soTaiKhoan = row[2].as<String>();
              t.taiKhoanPhu = row[3].as<String>();
              t.codeTT = row[4].as<String>();
              t.noiDungThanhToan = row[5].as<String>();
              t.loai = row[6].as<String>();
              t.soTien = row[7].as<int>();
              t.maThamChieu = row[8].as<String>();
              t.luyKe = row[9].as<int>();

              Serial.println("\n=== Giao dịch mới phát hiện ===");
              Serial.println("Ngân hàng: " + t.nganHang);
              Serial.println("Ngày giao dịch: " + t.ngayGiaoDich);
              Serial.println("Số tài khoản: " + t.soTaiKhoan);
              Serial.println("Tài khoản phụ: " + t.taiKhoanPhu);
              Serial.println("Code TT: " + t.codeTT);
              Serial.println("Nội dung thanh toán: " + t.noiDungThanhToan);
              Serial.println("Loại: " + t.loai);
              Serial.println("Số tiền: " + String(t.soTien));
              Serial.println("Mã tham chiếu: " + t.maThamChieu);
              Serial.println("Lũy kế: " + String(t.luyKe));
              Serial.println("==============================");
              trigAudio = 1;
              oldMaThamChieu = t.maThamChieu;
              foundNew = true;
            } else if (viTriOld == -1 && size > 0) { // Không tìm thấy mã cũ
              JsonArray row = doc[0].as<JsonArray>();
              t.nganHang = row[0].as<String>();
              t.ngayGiaoDich = row[1].as<String>();
              t.soTaiKhoan = row[2].as<String>();
              t.taiKhoanPhu = row[3].as<String>();
              t.codeTT = row[4].as<String>();
              t.noiDungThanhToan = row[5].as<String>();
              t.loai = row[6].as<String>();
              t.soTien = row[7].as<int>();
              t.maThamChieu = row[8].as<String>();
              t.luyKe = row[9].as<int>();

              Serial.println("\n=== Giao dịch mới phát hiện (All new hoặc reset mốc) ===");
              Serial.println("Ngân hàng: " + t.nganHang);
              Serial.println("Ngày giao dịch: " + t.ngayGiaoDich);
              Serial.println("Số tài khoản: " + t.soTaiKhoan);
              Serial.println("Tài khoản phụ: " + t.taiKhoanPhu);
              Serial.println("Code TT: " + t.codeTT);
              Serial.println("Nội dung thanh toán: " + t.noiDungThanhToan);
              Serial.println("Loại: " + t.loai);
              Serial.println("Số tiền: " + String(t.soTien));
              Serial.println("Mã tham chiếu: " + t.maThamChieu);
              Serial.println("Lũy kế: " + String(t.luyKe));
              Serial.println("==============================");
              trigAudio = 1;
              oldMaThamChieu = t.maThamChieu;
              foundNew = true;
            }
            if (!foundNew) Serial.println("Không có giao dịch mới.");
          }
        } else {
          Serial.print("JSON deserialization failed: ");
          Serial.println(error.c_str());
        }
      } else {
        checkGetData = DATAERR;
        Serial.println("Lấy dữ liệu thất bại hoặc mã lỗi HTTP.");
      }
      http.end();
    } else {
      checkGetData = WIFI_DIS;
      Serial.println("WiFi chưa kết nối, đang thử lại...");
    }
  }

  // 🔊 Phát âm thanh khi có giao dịch mới
  if (!isPlaying && trigAudio == 1) {
    Serial.println("Playing audio...");
    isPlaying = 1;
    enableShow = DISABLE;
    audio.connecttohost(url_ting_ting.c_str()); // Phát âm thanh "ting ting"
    audio.loop();
  }
  if (isPlaying) {
    if (audio.isRunning() && trigAudio > 0) {
      audio.loop();
    } else {
      if (trigAudio == 1) {
        Serial.println("Audio finished 1");
        trigAudio = 2;
        String message = "";
        if (isStart == 0) {
          message = "Xin chào";
          isStart = 1;
        } else {
          message = "Thanh toán thành công " + String(t.soTien) + " đồng";
        }
        const char* messageChar = message.c_str();
        audio.connecttospeech(messageChar, "vi"); // Phát thông báo bằng tiếng Việt
      } else if (trigAudio == 2) {
        isPlaying = 0;
        trigAudio = 0;
        enableShow = ENABLE;
        Serial.println("Audio finished 2");
      }
    }
  }

  // 📺 Hiển thị trên OLED theo trạng thái
  switch (screenOLED) {
    case SCREEN1: // Hiển thị 3 số tiền giao dịch gần nhất
      for (int i = 0; i < FRAME_COUNT_Money && enableShow == ENABLE; i++) {
        oled.clearDisplay();
        oled.setTextSize(1.5);
        oled.setCursor(60, 5);
        oled.print(String(tienNhan[0]) + " vnd");
        oled.setCursor(60, 25);
        oled.print(String(tienNhan[1]) + " vnd");
        oled.setCursor(60, 45);
        oled.print(String(tienNhan[2]) + " vnd");
        clearRectangle(0, 0, 32, 64);
        oled.drawBitmap(16, 16, moneyOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1); // Vẽ icon tiền
        if (checkGetData == DATAOK) {
          drawLeftCornerTriangles(WHITE); // Vẽ dấu hiệu kết nối thành công
        } else if (checkGetData == WIFI_DIS) {
          drawTriangleBottomLeft(WHITE); // Vẽ dấu hiệu mất kết nối
        }
        oled.display();
        delay(FRAME_DELAY);
      }
      oled.display();
      break;
    case SCREEN5: // Màn hình mất kết nối WiFi
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(40, 5);
      oled.print("WIFI");
      oled.setTextSize(1.5);
      oled.setCursor(40, 17);
      oled.print("Mat ket noi.");
      oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
      oled.drawLine(31, 0, 0, 31, 1);
      oled.drawLine(32, 0, 0, 32, 1);
      oled.display();
      delay(2000);
      screenOLED = SCREEN9;
      break;
    case SCREEN9: // Màn hình cấu hình AP
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(40, 5);
      oled.print("Ket noi Wifi:");
      oled.setCursor(40, 17);
      oled.print("TingBox");
      oled.setCursor(40, 38);
      oled.print("Dia chi IP:");
      oled.setCursor(40, 50);
      oled.print("192.168.4.1");
      for (int i = 0; i < FRAME_COUNT_settingOLED; i++) {
        clearRectangle(0, 0, 32, 64);
        oled.drawBitmap(0, 16, settingOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
        oled.display();
        delay(FRAME_DELAY * 4);
      }
      if (WiFi.status() == WL_CONNECTED) {
        screenOLED = SCREEN1;
      }
      break;
    case SCREEN13: // Màn hình khởi động lại
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(0, 20);
      oled.print("Khoi dong lai");
      oled.setCursor(0, 32);
      oled.print("Vui long doi ...");
      oled.display();
      break;
    default:
      delay(10);
      break;
  }
}

/**
 * 📡 Kết nối WiFi ở chế độ Station
 * Thử kết nối với WiFi sử dụng SSID và mật khẩu đã lưu. Nếu thất bại, chuyển sang chế độ AP.
 */
void connectSTA() {
  delay(5000);
  enableShow = DISABLE;
  if (wifi_ssid.length() > 1) {
    Serial.println(wifi_ssid);
    Serial.println(wifi_pass);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    int countConnect = 0;
    while (WiFi.status() != WL_CONNECTED) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(40, 5);
      oled.print("WIFI");
      oled.setTextSize(1.5);
      oled.setCursor(40, 17);
      oled.print("Dang ket noi..");
      for (int i = 0; i < FRAME_COUNT_wifiOLED; i++) {
        clearRectangle(0, 0, 32, 32);
        oled.drawBitmap(0, 0, wifiOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
        oled.display();
        delay(FRAME_DELAY);
      }
      countConnect++;
      if (countConnect >= 15) {
        Serial.println("Ket noi Wifi that bai");
        Serial.println("Kiem tra SSID & PASS");
        Serial.println("Ket noi Wifi: ESP32 de cau hinh");
        Serial.println("IP: 192.168.4.1");
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setCursor(40, 5);
        oled.print("Ket noi Wifi:");
        oled.setCursor(40, 17);
        oled.print("ESP32 Tingbox");
        oled.setCursor(40, 38);
        oled.print("Dia chi IP:");
        oled.setCursor(40, 50);
        oled.print("192.168.4.1");
        clearRectangle(0, 0, 32, 64); // Xóa vùng hiển thị
        oled.drawBitmap(0, 16, settingOLED[0], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1); // Vẽ icon cài đặt
        oled.display();
        digitalWrite(BUZZER, BUZZER_ON);
        delay(2000);
        digitalWrite(BUZZER, BUZZER_OFF);
        enterConfigAPMode();
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Da ket noi Wifi: ");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println((char*)wifi_ssid.c_str());
      buzzerBeep(3);
      Serial.println("Trạng thái WiFi: " + String(WiFi.status()));
      Serial.println("Địa chỉ IP: " + WiFi.localIP().toString());
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(40, 5);
      oled.print("WIFI");
      oled.setTextSize(1.5);
      oled.setCursor(40, 17);
      oled.print("Da ket noi.");
      oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
      delay(3000);
      enableReadSepay = 1;
      screenOLED = SCREEN1;
      enableShow = ENABLE;
    }
  }
}

/**
 * 🌐 Chuyển ESP32 sang chế độ Access Point
 * Tạo điểm truy cập WiFi và khởi động máy chủ web để cấu hình.
 */
void enterConfigAPMode() {
  Serial.println("==> Vào chế độ cấu hình WiFi qua AP <==");
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("ESP AP IP: "); Serial.println(WiFi.softAPIP());
  setupWebServer();
  while (true) {
    server.handleClient();
    delay(2);
  }
}

/**
 * 📄 Tạo trang HTML cho giao diện cấu hình
 * @return Chuỗi HTML của trang web cấu hình
 */
String htmlPage() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8' />
<title>Cấu hình ESP32 Tingbox</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{background:#222;font-family:sans-serif;color:#fff;text-align:center;}
h2{color:#4CAF50;}
form{background:#333;border-radius:10px;display:inline-block;padding:30px 40px;margin-top:50px;box-shadow:0 4px 16px #0008}
input[type=text],input[type=password]{width:250px;padding:10px;margin:10px 0;border:none;border-radius:5px;}
button{
  background:linear-gradient(90deg,#00c6ff,#0072ff);
  color:#fff;padding:10px 60px;border:none;border-radius:6px;cursor:pointer;font-size:1.2em;margin-top:15px;
}
button:hover{background:#4CAF50;}
label{float:left;font-weight:bold}
.form-group{margin-bottom: 18px;text-align:left;}
.alert{background:#2ecc40;color:#fff;border-radius:5px;padding:12px 18px;display:none;margin-top:18px}
</style>
</head>
<body>
<h2>CẤU HÌNH ESP32 Tingbox</h2>
<form id='configForm' onsubmit='return submitForm();'>
  <div class='form-group'>
    <label>WiFi SSID:</label><br>
    <input name='ssid' type='text' id='ssid' required>
  </div>
  <div class='form-group'>
    <label>WiFi Password:</label><br>
    <input name='pass' type='text' id='pass' required>
  </div>
  <div class='form-group'>
    <label>Mã Script Google Sheet:</label><br>
    <input name='script' type='text' id='script' required>
  </div>
  <button type='submit'>Cài đặt</button>
  <div class='alert' id='alertMessage'></div>
</form>
<script>
window.onload = function(){
  fetch('/getcfg').then(r=>r.json()).then(cfg=>{
    document.getElementById('ssid').value=cfg.ssid;
    document.getElementById('pass').value=cfg.pass;
    document.getElementById('script').value=cfg.script;
  });
}
function submitForm() {
  let ssid = document.getElementById('ssid').value,
      pass = document.getElementById('pass').value,
      script = document.getElementById('script').value;
  fetch('/savecfg', {
    method: 'POST',
    headers: {'Content-Type':'application/x-www-form-urlencoded'},
    body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}&script=${encodeURIComponent(script)}`
  }).then(resp=>resp.text()).then(msg => {
    alert("Cài đặt thành công");
    //setTimeout(()=>{location.reload()}, 2000);
  });
  return false;
}
</script>
</body>
</html>
)rawliteral";
  return page;
}

/**
 * 🌐 Xử lý yêu cầu GET cho trang chủ
 * Gửi trang HTML cấu hình về client
 */
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

/**
 * 📡 Lấy cấu hình hiện tại
 * Gửi cấu hình WiFi và Google Script dưới dạng JSON
 */
void handleGetCfg() {
  String json = "{\"ssid\":\"" + wifi_ssid + "\",\"pass\":\"" + wifi_pass + "\",\"script\":\"" + script_code + "\"}";
  server.send(200, "application/json", json);
}

/**
 * 💾 Lưu cấu hình mới
 * Lưu SSID, mật khẩu và mã Google Script, sau đó khởi động lại ESP32
 */
void handleSaveCfg() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String script = server.arg("script");
  saveConfig(ssid, pass, script);
  wifi_ssid = ssid;
  wifi_pass = pass;
  script_code = script;
  server.send(200, "text/plain", "&#9989; Đã lưu cấu hình, ESP32 sẽ khởi động lại...");
  enableShow = DISABLE;
  screenOLED = SCREEN13;
  delay(5000);
  ESP.restart();
}

/**
 * 🌐 Thiết lập máy chủ web
 * Đăng ký các tuyến xử lý yêu cầu HTTP
 */
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/getcfg", HTTP_GET, handleGetCfg);
  server.on("/savecfg", HTTP_POST, handleSaveCfg);
  server.begin();
}

/**
 * 🖨️ In cấu hình lưu trữ
 * Hiển thị SSID, mật khẩu và mã Google Script ra Serial
 */
void printPrefs() {
  Serial.println("----- Thông tin prefs ESP32 -----");
  Serial.println("SSID    : " + wifi_ssid);
  Serial.println("PASS    : " + wifi_pass);
  Serial.println("SCRIPT  : " + script_code);
  Serial.println("---------------------------------");
}

/**
 * 💾 Tải cấu hình từ bộ nhớ
 * Đọc SSID, mật khẩu và mã Google Script từ bộ nhớ không xóa
 */
void loadConfig() {
  prefs.begin("config", true);
  wifi_ssid = prefs.getString("ssid", "wifi_config");
  wifi_pass = prefs.getString("pass", "12345678");
  script_code = prefs.getString("script", "");

  prefs.end();
}

/**
 * 💾 Lưu cấu hình vào bộ nhớ
 * @param ssid SSID WiFi
 * @param pass Mật khẩu WiFi
 * @param script Mã Google Script
 */
void saveConfig(const String& ssid, const String& pass, const String& script) {
  prefs.begin("config", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("script", script);
  prefs.end();
}

/**
 * 🖌️ Xóa vùng hình chữ nhật trên OLED
 * @param x1 Tọa độ X bắt đầu
 * @param y1 Tọa độ Y bắt đầu
 * @param x2 Tọa độ X kết thúc
 * @param y2 Tọa độ Y kết thúc
 */
void clearRectangle(int x1, int y1, int x2, int y2) {
  for (int i = y1; i < y2; i++) {
    oled.drawLine(x1, i, x2, i, 0);
  }
}

/**
 * 🖌️ Xóa toàn bộ màn hình OLED
 */
void clearOLED() {
  oled.clearDisplay();
  oled.display();
}

/**
 * 🖌️ Vẽ tam giác góc dưới trái trên OLED
 * @param color Màu tam giác (WHITE hoặc BLACK)
 */
void drawTriangleBottomLeft(uint16_t color) {
  int x0 = 0;
  int y0 = SCREEN_HEIGHT - 1;
  oled.fillTriangle(x0, y0, x0 + 5, y0, x0, y0 - 5, color);
}

/**
 * 🖌️ Vẽ hai tam giác ở góc trên và dưới bên trái
 * @param color Màu tam giác (WHITE hoặc BLACK)
 */
void drawLeftCornerTriangles(uint16_t color) {
  int x_left = 0;
  
  // 🔺 Tam giác trên trái
  oled.fillTriangle(x_left, 0, x_left + 5, 0, x_left, 5, color);

  // 🔻 Tam giác dưới trái
  int y_bottom = SCREEN_HEIGHT - 1;
  oled.fillTriangle(x_left, y_bottom, x_left + 5, y_bottom, x_left, y_bottom - 5, color);
}
/**
 * 📣 Phát tiếng còi
 * @param numberBeep Số lần còi kêu
 */
void buzzerBeep(int numberBeep) {
  for (int i = 0; i < numberBeep; ++i) {
    digitalWrite(BUZZER, BUZZER_ON);
    delay(100);
    digitalWrite(BUZZER, BUZZER_OFF);
    delay(100);
  }
}