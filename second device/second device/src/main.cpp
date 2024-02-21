#include <Arduino.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <esp_camera.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long previousMillis = 0;
const long interval = 300000;  // 5 minutes in milliseconds

const int trigPin = 2; // ESP32's D2 pin
const int echoPin = 3; // ESP32's D3 pin
long duration;
int distance;
bool measurementChanged = false;

#define SERVICE_UUID        "67574d19-745c-4cbd-ab5a-5079ac6cdee3"
#define CHARACTERISTIC_UUID "60d11b39-7fea-46c8-8c04-edb67d530b1f"

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    BLEDevice::init("Sensor_Device");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setValue("Hello World");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it on your phone!");
}

void captureAndSendPicture() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    String base64Image = base64::encode(fb->buf, fb->len);
    String message = "Distance: " + String(distance) + " cm, Image: " + base64Image;

    pCharacteristic->setValue(message.c_str());
    pCharacteristic->notify();
    Serial.println("Notify value: " + message);

    esp_camera_fb_return(fb);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        int newDistance = duration * 0.034 / 2;

        if (newDistance != distance) {
            distance = newDistance;
            measurementChanged = true;
        } else {
            measurementChanged = false;
        }
    }

    if (deviceConnected && measurementChanged) {
        captureAndSendPicture();
    }

    if (deviceConnected && measurementChanged) {
        pCharacteristic->setValue("Connect Device name: Sensor_Device");
        pCharacteristic->notify();
        Serial.println("Notify value: Successfully connected Sensor_Device");
    }

    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    delay(1000);
}
