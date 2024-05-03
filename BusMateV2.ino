#include <Arduino.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <Adafruit_PN532.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_Fingerprint.h>
#include <ArduinoJson.h>

#define SDA_PIN 21                                                                           // Chân kết nối với chân SDA của PN532 (một loại cảm biến RFID/NFC)
#define SCL_PIN 22                                                                           // Chân kết nối với chân SCL của PN532
#define FIREBASE_FUNCTION_URL "https://us-central1-attendanceschoolbus.cloudfunctions.net/"  // base URL
#define BAUDRATE 57600                                                                       // Tốc độ baud của UART
#define SER_BUF_SIZE 1024                                                                    // Kích thước buffer cho dữ liệu đọc từ UART
#define GREEN_PIN 33                                                                         // Chân kết nối với đèn LED
#define RED_PIN 32                                                                           // Chân kết nối với đèn LED
#define BLUE_PIN 25                                                                          // Chân kết nối với đèn LED
#define BUTTON 34                                                                            //chân kết nối với Button
#define COMMON_ANODE                                                                         // bỏ cmt dòng này nếu dùng anode
#define BUZZLE 26
                                    
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);                           // Sử dụng thư viện PN532 để tương tác với cảm biến NFC
Preferences preferences;                                        // Đối tượng Preferences để quản lý lưu trữ thông tin cài đặt trong bộ nhớ flash
WiFiClientSecure sslClient;                                     // Sử dụng thư viện WiFiClientSecure để thiết lập kết nối an toàn với máy chủ
// Đối tượng HTTPClient để thực hiện yêu cầu HTTP
HTTPClient http1;
HTTPClient http2;
HTTPClient http3;
HTTPClient http4;

String value_key_13MHz = "";   // Chuỗi để lưu trữ giá trị từ khóa của mô-đun 13MHz

int preStateButton = LOW;  // Lưu trạng thái trước đó của nút
bool isRegister = false;   // đăng ký/đăng nhập

String deviceId = "1";  // for test only

const int Analog_channel_pin = 35;
float voltage_value; // Chuyển sang kiểu float để lưu giá trị điện áp.
unsigned long lastMillis_battery = 0; // Biến lưu thời gian lần cuối gọi hàm


/**
2 s:C=US, O=Google Trust Services LLC, CN=GTS Root R1
   i:C=BE, O=GlobalSign nv-sa, OU=Root CA, CN=GlobalSign Root CA
   a:PKEY: rsaEncryption, 4096 (bit); sigalg: RSA-SHA256
   v:NotBefore: Jun 19 00:00:42 2020 GMT; NotAfter: Jan 28 00:00:42 2028 GMT
*/
const char *root_ca =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIFYjCCBEqgAwIBAgIQd70NbNs2+RrqIQ/E8FjTDTANBgkqhkiG9w0BAQsFADBX\n"
  "MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE\n"
  "CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIwMDYx\n"
  "OTAwMDA0MloXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT\n"
  "GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFIx\n"
  "MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAthECix7joXebO9y/lD63\n"
  "ladAPKH9gvl9MgaCcfb2jH/76Nu8ai6Xl6OMS/kr9rH5zoQdsfnFl97vufKj6bwS\n"
  "iV6nqlKr+CMny6SxnGPb15l+8Ape62im9MZaRw1NEDPjTrETo8gYbEvs/AmQ351k\n"
  "KSUjB6G00j0uYODP0gmHu81I8E3CwnqIiru6z1kZ1q+PsAewnjHxgsHA3y6mbWwZ\n"
  "DrXYfiYaRQM9sHmklCitD38m5agI/pboPGiUU+6DOogrFZYJsuB6jC511pzrp1Zk\n"
  "j5ZPaK49l8KEj8C8QMALXL32h7M1bKwYUH+E4EzNktMg6TO8UpmvMrUpsyUqtEj5\n"
  "cuHKZPfmghCN6J3Cioj6OGaK/GP5Afl4/Xtcd/p2h/rs37EOeZVXtL0m79YB0esW\n"
  "CruOC7XFxYpVq9Os6pFLKcwZpDIlTirxZUTQAs6qzkm06p98g7BAe+dDq6dso499\n"
  "iYH6TKX/1Y7DzkvgtdizjkXPdsDtQCv9Uw+wp9U7DbGKogPeMa3Md+pvez7W35Ei\n"
  "Eua++tgy/BBjFFFy3l3WFpO9KWgz7zpm7AeKJt8T11dleCfeXkkUAKIAf5qoIbap\n"
  "sZWwpbkNFhHax2xIPEDgfg1azVY80ZcFuctL7TlLnMQ/0lUTbiSw1nH69MG6zO0b\n"
  "9f6BQdgAmD06yK56mDcYBZUCAwEAAaOCATgwggE0MA4GA1UdDwEB/wQEAwIBhjAP\n"
  "BgNVHRMBAf8EBTADAQH/MB0GA1UdDgQWBBTkrysmcRorSCeFL1JmLO/wiRNxPjAf\n"
  "BgNVHSMEGDAWgBRge2YaRQ2XyolQL30EzTSo//z9SzBgBggrBgEFBQcBAQRUMFIw\n"
  "JQYIKwYBBQUHMAGGGWh0dHA6Ly9vY3NwLnBraS5nb29nL2dzcjEwKQYIKwYBBQUH\n"
  "MAKGHWh0dHA6Ly9wa2kuZ29vZy9nc3IxL2dzcjEuY3J0MDIGA1UdHwQrMCkwJ6Al\n"
  "oCOGIWh0dHA6Ly9jcmwucGtpLmdvb2cvZ3NyMS9nc3IxLmNybDA7BgNVHSAENDAy\n"
  "MAgGBmeBDAECATAIBgZngQwBAgIwDQYLKwYBBAHWeQIFAwIwDQYLKwYBBAHWeQIF\n"
  "AwMwDQYJKoZIhvcNAQELBQADggEBADSkHrEoo9C0dhemMXoh6dFSPsjbdBZBiLg9\n"
  "NR3t5P+T4Vxfq7vqfM/b5A3Ri1fyJm9bvhdGaJQ3b2t6yMAYN/olUazsaL+yyEn9\n"
  "WprKASOshIArAoyZl+tJaox118fessmXn1hIVw41oeQa1v1vg4Fv74zPl6/AhSrw\n"
  "9U5pCZEt4Wi4wStz6dTZ/CLANx8LZh1J7QJVj2fhMtfTJr9w4z30Z209fOU0iOMy\n"
  "+qduBmpvvYuR7hZL6Dupszfnw0Skfths18dG9ZKb59UhvmaSGZRVbNQpsg3BZlvi\n"
  "d0lIKO2d1xozclOzgjXPYovJJIultzkMu34qQb9Sz/yilrbCgj8=\n"
  "-----END CERTIFICATE-----\n";

void setup(void) {
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUTTON, INPUT);   //Cài đặt chân  ở trạng thái đọc dữ liệu
  pinMode(BUZZLE, OUTPUT);  //còi
  digitalWrite(BUZZLE, 1);


  Serial.begin(9600);                                  // Khởi động cổng serial cho PC
  MySerial.setRxBufferSize(SER_BUF_SIZE);              // Đặt kích thước buffer đọc cho Serial
  MySerial.begin(BAUDRATE, SERIAL_8N1, RxPin, TxPin);  // Bắt đầu Serial với tốc độ baud và cấu hình nhất định
  nfc.begin();                                         // Bắt đầu kết nối với đầu đọc RFID 13.56MHz
  sslClient.setCACert(root_ca);                        // Sử dụng chứng chỉ SSL

  initialize_RFID_13MHz(); 
  setupWifi();
 

  // Tạo các nhiệm vụ
  xTaskCreatePinnedToCore(taskRFID125kHzFunction, "RFID125kHz", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskRFID13MHzFunction, "RFID13MHz", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSendData2CloudFunction, "sendData2Cloud", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskFingerprintFunction, "Fingerprint", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskButtonFunction, "Button", 10000, NULL, 1, NULL, 1);
  Serial.println("Authentication success. System ready to operate.");
  setColor(0, 255, 0);  // green
}


void loop(void) {
  
}

// hàm đo thời lương pin còn bao nhiêu volt
// void battery() {
//   int ADC_VALUE = analogRead(Analog_channel_pin);
//   Serial.print("ADC VALUE = ");
//   Serial.println(ADC_VALUE);

//   // Tính toán điện áp đầu vào dựa trên công thức chia áp
//   float Vin = ((float)ADC_VALUE * 3.3) / 4095.0; // Tính toán điện áp đầu vào (trước khi chia áp)
//   //3v3: đây là giá trị điện áp tham chiếu. ADC sẽ chuyển đổi tín hiệu analog thành một giá trị số dựa trên độ lớn của tín hiệu analog so với điện áp tham chiếu này.
//   //4095: Đây là giá trị tối đa mà ADC có thể đo được. Trong hầu hết các vi điều khiển thông thường, ADC thường có độ phân giải cố định, tức là số bit mà nó có thể chuyển đổi. 4095 là giá trị tối đa mà một ADC 12-bit có thể đo được. 
//   //Mỗi bit bổ sung sẽ làm tăng độ chia nhỏ giữa các giá trị được đo, cung cấp độ chính xác cao hơn.

//   // Áp dụng công thức chia áp để tính toán điện áp đầu vào thực tế đến chân ADC
//   voltage_value = (Vin * 2.02) ; //+0.46; //
//   //2,01: tổng 2 điện trở
//   //0,46: sai số 

//   Serial.print("Voltage = ");
//   Serial.print(voltage_value);
//   Serial.println(" volts");
// }

//Hàm khởi tạo cho module RFID 13.56MHz
void initialize_RFID_13MHz() {
  // Lấy thông tin firmware version của module RFID
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Không tìm thấy board PN53x");
  }
  // Cấu hình chế độ hoạt động cho module RFID
  nfc.SAMConfig();
}

//Hàm xử lý thông tin từ thẻ RFID 13.56MHz có tham khảo thư viện
String process_RFID_13MHz(uint8_t uid[], uint8_t uidLength) {
  String value = "";
  // Duyệt qua các byte trong UID của thẻ RFID và thêm vào chuỗi kết quả
  for (uint8_t i = 0; i < uidLength; i++) {
    value += uid[i];
  }
  // In ra thông tin UID của thẻ RFID
  Serial.println(">>>>>>>>>>>>>>>13.56MHz");
  Serial.println(value);
  return value;
}


//hàm xử lí nút
void taskButtonFunction(void *pvParameters) {
  for (;;) {
    int buttonStatus = digitalRead(BUTTON);  // Trạng thái hiện tại của nút
    if (buttonStatus != preStateButton) {
      // Trạng thái của nút đã thay đổi
      if (buttonStatus == HIGH) {
        // Nút được nhấn
        isRegister = !isRegister;
        if (isRegister) {
          setColor(0, 0, 255);  // blue
        } else {
          setColor(0, 255, 0);  // green
        }
        delay(200);
      }
    }
    preStateButton = buttonStatus;
    battery();
  }
}

//
void Buzzle() {
  digitalWrite(BUZZLE, 0);
  delay(100);
  digitalWrite(BUZZLE, 1);
}

//upload lên server
void taskSendData2CloudFunction(void *pvParameters) {
  for (;;) {

    // Nếu giá trị value_key_13 được thay đồi thì mới update
    if (value_key_13MHz.length() > 0) {
      Serial.print("Leng 13.56MHz: ");
      Serial.println(value_key_13MHz.length());
      isRegister ? registerIdStudentFunction(value_key_13MHz, false, deviceId, http1) : updateAttendance(value_key_13MHz, 0, http1);
      // Đặt lại giá trị để chuẩn bị cho lần đọc tiếp theo
      value_key_13MHz = "";
    }

    unsigned long currentMillis = millis();  // Lấy thời gian hiện tại
    // Kiểm tra xem đã đủ 10 phút chưa
    if (currentMillis - lastMillis_battery >= 10UL * 60 * 1000) {  // Kiểm tra sau mỗi 10 phút
      lastMillis_battery = currentMillis;                          // Cập nhật thời gian lần cuối gọi hàm

      updateAttendance(String(voltage_value), 0, http4);
    }

    // Delay giữa các lần lặp của nhiệm vụ
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//in ra thẻ 13
void taskRFID13MHzFunction(void *pvParameters) {
  for (;;) {
    // Khai báo biến để lưu trữ UID của thẻ RFID 13.56MHz
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;

    // Kiểm tra xem có thẻ RFID 13.56MHz nằm trong phạm vi đọc hay không
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
      Buzzle();
      Serial.println(">>>>>>>>>>check 13.56Mhz");
      value_key_13MHz = process_RFID_13MHz(uid, uidLength);
      Serial.print("value 13.56MHz: ");
      Serial.println(value_key_13MHz);
      delay(1000);
    }

    // Delay giữa các lần lặp của nhiệm vụ
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//tạo màu led
void setColor(int red, int green, int blue) {
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif

  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

// khởi tạo Wifi
void setupWifi() {
  // Đọc thông tin SSID từ bộ nhớ flash
  preferences.begin("wifi-credentials", false);
  String savedSSID = preferences.getString("ssid", "");

  // Nếu có thông tin SSID đã lưu, thử kết nối với nó
  if (savedSSID.length() > 0) {
    Serial.println("Trying to connect using saved WiFi credentials...");

    // Đọc thông tin mật khẩu từ bộ nhớ flash
    String savedPassword = preferences.getString("password", "");

    // Nếu có thông tin mật khẩu đã lưu, thêm nó vào quy trình kết nối
    if (savedPassword.length() > 0) {
      WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    } else {
      WiFi.begin(savedSSID.c_str());
    }

    // Đợi kết nối
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    // Kiểm tra xem kết nối đã thành công hay không
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi using saved credentials");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      preferences.end();
      return;
    }
  }

  // Nếu không có hoặc kết nối thất bại, tạo AP để cấu hình
  Serial.println("Starting WiFi configuration portal.");
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("AutoConnectAP2")) {
    Serial.println("Failed to connect and hit timeout");
    // Đợi 10 giây để cho bạn nhấn reset
    delay(10000);
    // Đặt lại để thử lại, hãy chắc chắn là bạn đã xóa cài đặt cứng trước đó
    ESP.restart();
    delay(5000);
  }

  // Lưu SSID và mật khẩu vào bộ nhớ flash nếu kết nối thành công
  preferences.putString("ssid", WiFi.SSID());
  preferences.putString("password", WiFi.psk());
  preferences.end();

  // In ra thông tin kết nối WiFi
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/**
hàm gửi ID của thẻ RFID hoặc ID của vân tay tương ứng với mỗi học sinh
id: id cuar ther RFID, vân tay
key: 0: rfid, 1: vân tay
*/
void updateAttendance(String id, int key, HTTPClient &http) {
  String url = FIREBASE_FUNCTION_URL;
  url.concat("updateAttendance");

  http.begin(sslClient, url);
  http.addHeader("Content-Type", "application/json");

  // Tạo dữ liệu JSON để gửi
  String jsonData = "{ \"id\":" + id + ", \"key\":" + String(key) + "}";

  int httpResponseCode = http.POST(jsonData);

  // Xử lý kết quả
  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Server response: " + payload);
    // to do
  } else {
    Serial.println("Error on HTTP request");
  }
  http.end();
}

/**
đăng ký thẻ mới, vân tay mới cho học sinh chưa có
id: id thẻ, vân tay,
isFinger: true: đăng ký cho vân tay, false: đăng ký RFID
deviceId: mã của mỗi hộp BlackBox // to do
**/
void registerIdStudentFunction(String id, bool isFinger, String deviceId, HTTPClient &http) {
  String url = FIREBASE_FUNCTION_URL;
  url.concat("sendRegisterStudentIdRequest");

  http.begin(sslClient, url);
  http.addHeader("Content-Type", "application/json");

  // Tạo dữ liệu JSON để gửi
  String jsonData = "{ \"deviceId\":" + deviceId + ", \"id\":" + id + ", \"isFinger\":" + isFinger + "}";

  int httpResponseCode = http.POST(jsonData);

  // Xử lý kết quả
  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Server response: " + payload);
    // to do
  } else {
    Serial.println("Error on HTTP request");
  }
  http.end();
}


