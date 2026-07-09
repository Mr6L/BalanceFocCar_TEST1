#include "BluetoothCarControl.h"

#include "APP.h"
#include "ButtonAndBattery.h"
#include "UserConfig.h"

#include <cstring>

#if XIAOZHI_BLE_ENABLE
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#endif

#define BLE_CAR_TAG "XiaoZhiBLE"

#if XIAOZHI_BLE_ENABLE
static BLECharacteristic *bleCommandCharacteristic = nullptr;
static volatile bool bleConnected = false;
static volatile bool powerOffRequested = false;
static QueueHandle_t bleCommandQueue = nullptr;

struct BleCommandMessage
{
    char payload[128];
};

static void notifyBle(const String &message)
{
    if (!bleConnected || bleCommandCharacteristic == nullptr)
    {
        return;
    }

    bleCommandCharacteristic->setValue((uint8_t *)message.c_str(), message.length());
    bleCommandCharacteristic->notify();
}

static String unwrapCommand(String payload)
{
    payload.trim();
    if (payload.startsWith("[") && payload.endsWith("]"))
    {
        payload = payload.substring(1, payload.length() - 1);
        payload.trim();
    }
    return payload;
}

static bool nextToken(const String &payload, int &cursor, String &token)
{
    if (cursor > payload.length())
    {
        return false;
    }

    const int next = payload.indexOf(',', cursor);
    if (next < 0)
    {
        token = payload.substring(cursor);
        cursor = payload.length() + 1;
    }
    else
    {
        token = payload.substring(cursor, next);
        cursor = next + 1;
    }
    token.trim();
    return token.length() > 0;
}

static void notifyJsonOk(const String &json)
{
    notifyBle("OK:" + json);
}

static bool switchValue(const String &value, bool defaultValue)
{
    String lower = value;
    lower.toLowerCase();

    if (lower == "1" || lower == "on" || lower == "true" || lower == "yes")
    {
        return true;
    }
    if (lower == "0" || lower == "off" || lower == "false" || lower == "no")
    {
        return false;
    }
    return defaultValue;
}

static void handleCarCommand(const String &action)
{
    String lower = action;
    lower.toLowerCase();

    if (lower == "down")
    {
        notifyBle("OK:[Car,down]");
        powerOffRequested = true;
        return;
    }

    if (lower == "status")
    {
        notifyJsonOk(BalanceCarControlStatusJson());
        return;
    }

    notifyBle("ERR:unknown car command");
}

static void handleDriveCommand(const String &body, int cursor)
{
    String action;
    if (!nextToken(body, cursor, action))
    {
        notifyBle("ERR:missing drive action");
        return;
    }

    String lower = action;
    lower.toLowerCase();

    if (lower == "stop")
    {
        notifyJsonOk(BalanceCarStopDrive());
        return;
    }

    if (lower == "distance")
    {
        String metersText;
        String speedText;
        if (!nextToken(body, cursor, metersText))
        {
            notifyBle("ERR:missing distance meters");
            return;
        }
        const float speed =
            nextToken(body, cursor, speedText) ? speedText.toFloat() : XIAOZHI_BLE_DEFAULT_MOVE_SPEED_RATIO;
        notifyJsonOk(BalanceCarMoveDistance(metersText.toFloat(), speed));
        return;
    }

    if (lower == "turn")
    {
        String degreesText;
        String speedText;
        if (!nextToken(body, cursor, degreesText))
        {
            notifyBle("ERR:missing turn degrees");
            return;
        }
        const float speed = nextToken(body, cursor, speedText) ? speedText.toFloat() : XIAOZHI_BLE_DEFAULT_TURN_RATIO;
        notifyJsonOk(BalanceCarTurnInPlace(degreesText.toFloat(), speed));
        return;
    }

    if (lower == "manual")
    {
        String throttleText;
        String turnText;
        String durationText;
        if (!nextToken(body, cursor, throttleText) ||
            !nextToken(body, cursor, turnText) ||
            !nextToken(body, cursor, durationText))
        {
            notifyBle("ERR:manual needs throttle,turn,duration_ms");
            return;
        }
        notifyJsonOk(BalanceCarDriveFor(
            throttleText.toFloat(),
            turnText.toFloat(),
            (unsigned long)durationText.toInt()));
        return;
    }

    notifyBle("ERR:unknown drive command");
}

static void handleRgbLightCommand(const String &body, int cursor, bool front)
{
    String action;
    if (!nextToken(body, cursor, action))
    {
        notifyJsonOk(front ? BalanceCarSetFrontLight(true, 255, 255, 255) : BalanceCarSetRearLight(true, 255, 0, 0));
        return;
    }

    String lower = action;
    lower.toLowerCase();

    if (lower == "off")
    {
        notifyJsonOk(front ? BalanceCarSetFrontLight(false, 0, 0, 0) : BalanceCarSetRearLight(false, 0, 0, 0));
        return;
    }

    if (lower == "set")
    {
        String rText;
        String gText;
        String bText;
        if (!nextToken(body, cursor, rText) ||
            !nextToken(body, cursor, gText) ||
            !nextToken(body, cursor, bText))
        {
            notifyBle(front ? "ERR:rgb front set needs r,g,b" : "ERR:rgb rear set needs r,g,b");
            return;
        }

        notifyJsonOk(front ? BalanceCarSetFrontLight(true, rText.toInt(), gText.toInt(), bText.toInt())
                           : BalanceCarSetRearLight(true, rText.toInt(), gText.toInt(), bText.toInt()));
        return;
    }

    const bool enable = switchValue(lower, true);
    notifyJsonOk(front ? BalanceCarSetFrontLight(enable, 255, 255, 255) : BalanceCarSetRearLight(enable, 255, 0, 0));
}

static void handleRgbCommand(const String &body, int cursor)
{
    String action;
    if (!nextToken(body, cursor, action))
    {
        notifyBle("ERR:missing rgb action");
        return;
    }

    String lower = action;
    lower.toLowerCase();

    if (lower == "off")
    {
        notifyJsonOk(BalanceCarSetRgb(false, 0, 0, 0));
        return;
    }

    if (lower == "status")
    {
        notifyJsonOk(BalanceCarControlStatusJson());
        return;
    }

    if (lower == "front")
    {
        handleRgbLightCommand(body, cursor, true);
        return;
    }

    if (lower == "rear" || lower == "back")
    {
        handleRgbLightCommand(body, cursor, false);
        return;
    }

    if (lower == "set")
    {
        String rText;
        String gText;
        String bText;
        if (!nextToken(body, cursor, rText) ||
            !nextToken(body, cursor, gText) ||
            !nextToken(body, cursor, bText))
        {
            notifyBle("ERR:rgb set needs r,g,b");
            return;
        }
        notifyJsonOk(BalanceCarSetRgb(true, rText.toInt(), gText.toInt(), bText.toInt()));
        return;
    }

    notifyBle("ERR:unknown rgb command");
}

static void handleBlePayload(String payload)
{
    payload.trim();

    if (payload == "[hello world]")
    {
        notifyBle("OK:[hello world]");
        return;
    }

    const String body = unwrapCommand(payload);
    int cursor = 0;
    String group;
    if (!nextToken(body, cursor, group))
    {
        notifyBle("ERR:empty command");
        return;
    }

    String lower = group;
    lower.toLowerCase();

    if (lower == "car")
    {
        String action;
        if (!nextToken(body, cursor, action))
        {
            notifyBle("ERR:missing car action");
            return;
        }
        handleCarCommand(action);
        return;
    }

    if (lower == "drive")
    {
        handleDriveCommand(body, cursor);
        return;
    }

    if (lower == "rgb")
    {
        handleRgbCommand(body, cursor);
        return;
    }

    notifyBle("ERR:unknown command");
}

class CarBleServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server) override
    {
        bleConnected = true;
        Serial.println("[XiaoZhiBLE] client connected");
    }

    void onDisconnect(BLEServer *server) override
    {
        bleConnected = false;
        Serial.println("[XiaoZhiBLE] client disconnected, restart advertising");
        BLEDevice::startAdvertising();
    }
};

class CarBleCommandCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override
    {
        auto value = characteristic->getValue();
        if (value.length() == 0)
        {
            return;
        }

        if (bleCommandQueue == nullptr)
        {
            Serial.println("[XiaoZhiBLE] command queue is not ready");
            return;
        }

        BleCommandMessage message = {};
        const size_t copyLength =
            value.length() < sizeof(message.payload) - 1 ? value.length() : sizeof(message.payload) - 1;
        memcpy(message.payload, value.data(), copyLength);
        message.payload[copyLength] = '\0';

        if (xQueueSend(bleCommandQueue, &message, 0) != pdTRUE)
        {
            Serial.println("[XiaoZhiBLE] command queue full, drop packet");
        }
    }
};

static void BluetoothCarControlLoop(void *pvParameters)
{
    BleCommandMessage message;
    for (;;)
    {
        while (bleCommandQueue != nullptr && xQueueReceive(bleCommandQueue, &message, 0) == pdTRUE)
        {
            String payload(message.payload);
            payload.trim();

            Serial.print("[XiaoZhiBLE] rx: ");
            Serial.println(payload);

            handleBlePayload(payload);
        }

        if (powerOffRequested)
        {
            powerOffRequested = false;
            Serial.println("[XiaoZhiBLE] power off requested");
            vTaskDelay(pdMS_TO_TICKS(200));
            balanceCarPowerOff();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void initBlePeripheral()
{
    BLEDevice::init(XIAOZHI_BLE_DEVICE_NAME);
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new CarBleServerCallbacks());

    BLEService *service = server->createService(XIAOZHI_BLE_SERVICE_UUID);
    bleCommandCharacteristic = service->createCharacteristic(
        XIAOZHI_BLE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR |
            BLECharacteristic::PROPERTY_NOTIFY);

    bleCommandCharacteristic->setCallbacks(new CarBleCommandCallbacks());
    bleCommandCharacteristic->addDescriptor(new BLE2902());
    bleCommandCharacteristic->setValue("BalanceCar ready");

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(XIAOZHI_BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.print("[XiaoZhiBLE] advertising as ");
    Serial.println(XIAOZHI_BLE_DEVICE_NAME);
}
#endif

void BluetoothCarControlTask::startTask()
{
#if XIAOZHI_BLE_ENABLE
    bleCommandQueue = xQueueCreate(8, sizeof(BleCommandMessage));
    if (bleCommandQueue == nullptr)
    {
        Serial.println("[XiaoZhiBLE] command queue create failed");
        return;
    }

    initBlePeripheral();
    xTaskCreatePinnedToCore(BluetoothCarControlLoop, "XiaoZhi BLE", 4096, NULL, 1, NULL, 0);
#else
    Serial.println("[XiaoZhiBLE] disabled");
#endif
}

BluetoothCarControlTask BluetoothCarControl;
