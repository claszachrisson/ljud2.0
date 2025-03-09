#include "ble.h"

#include <NimBLECharacteristic.h>
#include <NimBLEConnInfo.h>
#include <NimBLEUtils.h>
#include "../uint3.h"

#define VOLUME_BLE_UUID "1200"

class VolumeCharacteristic : public NimBLECharacteristicCallbacks {
    void (*volume_write_callback)(uint3);

public:
    explicit VolumeCharacteristic(void (*volume_write_callback)(uint3)) {
        this->volume_write_callback = volume_write_callback;
    }

    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
        printf("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
        if (pCharacteristic->getUUID().equals(NimBLEUUID(VOLUME_BLE_UUID))) {
            if (pCharacteristic->getValue().length() == 1) {
                const uint3 val = {
                    .value = pCharacteristic->getValue().data()[0],
                };
                volume_write_callback(val);
                return;
            }
        }

        ESP_LOGW("BLE", "Characteristic write to unknown UUID %s: %s",
                 pCharacteristic->getUUID().toString().c_str(),
                 pCharacteristic->getValue().c_str());
    }

    /**
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override {
        printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    /** Peer subscribed to notifications/indications */
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if (subValue == 0) {
            str += " Unsubscribed to ";
        } else if (subValue == 1) {
            str += " Subscribed to notifications for ";
        } else if (subValue == 2) {
            str += " Subscribed to indications for ";
        } else if (subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        printf("%s\n", str.c_str());
    }
};
