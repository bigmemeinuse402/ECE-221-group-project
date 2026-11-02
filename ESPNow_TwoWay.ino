#include <WiFi.h>
#include <esp_now.h>

// Replace with the OTHER ESP32's MAC address
uint8_t peerAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

bool ready = false;

// Runs whenever a message arrives
void onReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  Serial.print("Received: ");

  for (int i = 0; i < len; i++) {
    Serial.write(data[i]);
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed!");
    return;
  }

  // Register receiving function
  esp_now_register_recv_cb(onReceive);

  // Register the other ESP32
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    return;
  }

  ready = true;
  Serial.println("Two-way ESP-NOW ready!");
}

void loop() {
  if (!ready) {
    delay(1000);
    return;
  }

  const char message[] = "Hello from ESP32!";

  esp_err_t result = esp_now_send(
    peerAddress,
    (const uint8_t *)message,
    sizeof(message)
  );

  if (result == ESP_OK) {
    Serial.println("Message queued!");
  } else {
    Serial.println("Send error!");
  }

  delay(2000);
}

