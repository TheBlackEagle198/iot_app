#include <MQTT.h>
#include <WebServer.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include "GlobalConfig.h"
#include "RadioType.h"
#include "Timer.h"
#include "SendStrategy.h"
#include "Preferences.h"
#include <WiFi.h>
#include <DNSServer.h>
/* NOTE: 
*    As of 28 June 2024, DNSServer library has some patches which did not 
*  increment the library version, thus those updates need to be manually added
*  in the library files.
*  Depends on: https://github.com/espressif/arduino-esp32/commit/d91271019ce0fb545ed9c2b6f51d27c86520f273
*/
#include "strings.h"
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <TFT_eSPI.h>

/// @brief MQTT setup
#define INCOMING_TOPIC "B2H"
#define OUTGOING_TOPIC "H2B"

#define COMMAND_FORCE_UPDATE "force_update"
#define COMMAND_CHANGE_STRATEGY "strategy"
#define COMMAND_CHANGE_SEND_INTERVAL "send_interval"
#define COMMAND_CHANGE_THRESHOLD "threshold"

/// @brief storage setup
#define MAX_SSID 20
#define MAX_WIFi_PASSWORD 20
#define MAX_MQTT_SERVER 20
#define MAX_MQTT_USER 20
#define MAX_MQTT_PASSWORD 20
#define PREFERENCES_NAMESPACE "hub"


/// @brief timers setup
#define CONNECT_MODE_LONG_PRESS_TIME 1000
#define CONNECT_MODE_EXPIRATION 20000
#define FACTORY_RESET_LONG_PRESS_TIME 10000
#define WIFI_CONNECTION_TIMEOUT 7500
#define MQTT_CONNECTION_TIMEOUT 300

/// @brief display setup
#define HORIZONTAL_SPACING 5
#define VERTICAL_SPACING 20
#define CIRCLE_RADIUS 5
#define TEXT_TOP_MARGIN 30

/// @brief pin setup
#define PIN_RADIO_CE             GPIO_NUM_21
#define PIN_RADIO_CNS            GPIO_NUM_22
#define PIN_CONNECT_BUTTON       GPIO_NUM_35
#define PIN_FACTORY_RESET_BUTTON GPIO_NUM_0
#define PIN_RADIO_MISO           GPIO_NUM_27
#define PIN_RADIO_MOSI           GPIO_NUM_26
#define PIN_RADIO_SCK            GPIO_NUM_25
#define PIN_RADIO_CS             GPIO_NUM_33

// global constants ************************************************************
const String hubID = "00:26:85:22";

const String apSSID = "HUB_" + hubID + "_AP";
const String apPassword = "password";

const byte MQTT_MAX_RETRIES = 5;
const byte DNS_PORT = 53;

/// @brief text to display on the tft screen
const char* display_texts[] = {
    "WiFi",
    "MQTT",
    "Radio",
    "Connect Mode"
};

enum DisplayItem {
    WIFI,
    MQTT,
    RADIO,
    CONNECT_MODE,
    DISPLAY_ITEM_COUNT
};

enum class DisplayEvent {
    WIFI_ON,
    WIFI_OFF,
    MQTT_ON,
    MQTT_OFF,
    RADIO_ON,
    CONNECT_MODE_ON,
    CONNECT_MODE_OFF,
};

enum class CommandType
{
    FORCE_UPDATE,
    CHANGE_STRATEGY,
    CHANGE_THRESHOLD,
    CHANGE_DELAY,
    LAST
};

enum class WiFiConnectionStatus
{
    NO_WIFI,
    WIFI_CONNECTING,
    NO_MQTT,
    MQTT_CONNECTING,
    CONNECTED
} volatile currentState;

template <int MAX_SIZE>
class GlobalToNodeIdsMap
{
private:
    uint16_t _nodeIDs[MAX_SIZE];
    GLOBAL_ID_T _GIDs[MAX_SIZE];

    uint32_t _lastElementIndex;

public:
    GlobalToNodeIdsMap() : _lastElementIndex(0)
    {
        memset(this->_nodeIDs, 0, sizeof(this->_nodeIDs));
        memset(this->_GIDs, 0, sizeof(this->_GIDs));
    }

    /// @brief Gets the global id associated with the given node id
    /// @param[in] nodeId - the node id to search for
    /// @param[out] GID - the global id if found
    /// @return true if the global id was found, false otherwise
    bool getByNodeID(NODE_ID_T nodeId, GLOBAL_ID_T &GID)
    {

        for (uint32_t i = 0; i <= this->_lastElementIndex; i++)
        {
            if (nodeId == this->_nodeIDs[i])
            {
                GID = this->_GIDs[i];
                return true;
            }
        }
        return false;
    }

    /// @brief Gets the node id associated with the given global id
    /// @param gid - the global id to search for
    /// @param nodeId - the node id if found
    /// @return true if the node id was found, false otherwise
    bool getByGID(GLOBAL_ID_T gid, NODE_ID_T &nodeId)
    {
        for (uint32_t i = 0; i <= this->_lastElementIndex; i++)
        {
            if (gid == this->_GIDs[i])
            {
                nodeId = this->_nodeIDs[i];
                return true;
            }
        }
        return false;
    }

    bool addSet(NODE_ID_T nodeID, GLOBAL_ID_T GID)
    {
        if (this->_lastElementIndex == MAX_SIZE - 2)
            return false;
        this->_lastElementIndex++;
        this->_nodeIDs[this->_lastElementIndex] = nodeID;
        this->_GIDs[this->_lastElementIndex] = GID;
        return true;
    }
};

// global variables ***********************************************************
// radio variables
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CNS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
uint8_t radioChannel = 0;
SPIClass hspi(HSPI); // used for spi radio communication
int writeAttempts = 0;

// internet-related variables
WiFiClient wifiNetworkClient;
MQTTClient mqttClient;
GlobalToNodeIdsMap<255> globalToNodeIdsMap;
char ssid[MAX_SSID];
char wifiPassword[MAX_WIFi_PASSWORD];

char mqttServer[MAX_MQTT_SERVER];
char mqttUser[MAX_MQTT_USER];
char mqttPassword[MAX_MQTT_PASSWORD];

IPAddress apIP(192, 168, 1, 4);
DNSServer dnsServer;
WebServer webServer(80);

byte mqttAttempts = 0;

volatile bool isHTTPServerRunning = false;
volatile bool isDNSServerRunning = false;
volatile bool isConfigPortalRunning = false;

volatile bool shouldTryWiFi = false;
volatile bool shouldTryMqtt = false;

volatile bool triedWiFiConfig = false;
volatile bool triedMqttConfig = false;

volatile bool isMQTTConnected = false;
volatile bool isWiFiConnected = false;

/// @brief Timers 
Timer wifiConnectionTimer(WIFI_CONNECTION_TIMEOUT);
Timer mqttConnectionTimer(MQTT_CONNECTION_TIMEOUT);
Timer connectModeExpirationTimer(CONNECT_MODE_EXPIRATION);
Timer connectButtonTimer(CONNECT_MODE_LONG_PRESS_TIME);
Timer factoryResetButtonTimer(FACTORY_RESET_LONG_PRESS_TIME);

/// @brief task handles
TaskHandle_t fsmTask;
TaskHandle_t radioTask;
TaskHandle_t btnTask;
TaskHandle_t connectModeExpiredTask;
TaskHandle_t displayTask;

/// @brief others
Preferences preferences;
auto tft = TFT_eSPI();

esp_event_loop_handle_t updateDisplayLoop;
ESP_EVENT_DEFINE_BASE(UPDATE_DISPLAY);
volatile bool ranOnce = false;

bool isInConnectMode = false;
uint8_t connectButtonCurrentState = LOW;
uint8_t connectButtonLastState = HIGH;
uint8_t factoryResetButtonCurrentState = LOW;
uint8_t factoryResetButtonLastState = HIGH;

// function declarations *******************************************************

void updateCircle(DisplayItem item, uint32_t color);
void updateDisplayHandler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);
void drawItem(DisplayItem item);
void displayTaskFn(void* pvParams);

/// @brief writes a message to the mesh
/// @param[in] data - the message to be sent via radio mesh
/// @param[in] msg_type - the type of message that will be used in the radio header
/// @param[in] size - the size of data (in bytes)
/// @param[in] recvNodeId - the node id of the recieving node
void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId);

/// @brief callback used when recieving an mqtt message
/// forwards the radio command
void messageReceivedCb(String &topic, String &payload);

/// @brief Reads radio data and publishes it to mqtt broker
void processRadioData();

/// @brief Sets up radio connection
void initRadio();

/// @brief Finds the next available node id
uint16_t getNewNodeID();

/// @brief Converts a global id to a string with hex values separated by colons
String globalIdToString(GLOBAL_ID_T gid);

/// @brief Converts a string with hex values separated by colons to a global id
GLOBAL_ID_T stringToGlobalId(const String &buffer);

/// @brief Initializes tasks
void initTasks();

CommandType getCommand(String &commandString);

void runRadio(void* pvParameters);
void runButton(void* pvParams);
void checkConnectModeExpired(void* pvParams);
void switchState(WiFiConnectionStatus newState);
bool connectToBroker();
void startDNS();
void stopDNS();
void startWebServer();
void stopWebServer();
void stopPortal();
void handleWiFiRoute();
void handleMqttRoute();
void handleHomeRoute();
void initHttpServer();
void startPortal(wifi_mode_t wifiMode);
void factoryReset();
void runPortal();
void storeWiFi();
void storeMQTT();
void tryStoredWiFi();
void tryStoredMQTT();
void runFSM(void* pvParameters);

// main program ***************************************************************
void setup()
{
#ifndef ENVIRONMENT
    while (true)
    {
    }
#endif
    hspi.begin(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI, PIN_RADIO_CS); // SCK, MISO, MOSI, SS
    esp_task_wdt_init(10, false); // Initialize the Task Watchdog Timer. The timeout is set to 10 seconds.

    Serial.begin(115200);

#if ENVIRONMENT == DEVELOPMENT
    while (!Serial)
    {
    }
#endif

    initTasks();
    initHttpServer();

    Serial.println("Setup complete!");
}

void loop() {}
// function definitions ********************************************************
void initTasks()
{
    xTaskCreatePinnedToCore(
        runFSM,
        "connection_fsm",
        90000,
        NULL,
        1,
        &fsmTask,
        0);

    xTaskCreatePinnedToCore(
        runRadio,
        "radio_task",
        10000,
        NULL,
        1,
        &radioTask,
        1);

    xTaskCreatePinnedToCore(
        runButton,
        "btn_task",
        3000,
        NULL,
        1,
        &btnTask,
        1);
    xTaskCreatePinnedToCore(
        checkConnectModeExpired,
        "connect_mode_expired_task",
        1000,
        NULL,
        1,
        &connectModeExpiredTask,
        1);

    xTaskCreatePinnedToCore(
        displayTaskFn,
        "display_update_task",
        10000,
        NULL,
        1,
        &displayTask,
        1);
}

void updateCircle(DisplayItem item, uint32_t color) {
    tft.fillCircle(
        CIRCLE_RADIUS + HORIZONTAL_SPACING,
        item * VERTICAL_SPACING + CIRCLE_RADIUS + TEXT_TOP_MARGIN,
        CIRCLE_RADIUS, color);
}

void updateDisplayHandler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    DisplayEvent event = (DisplayEvent)id;
    switch (event) {
        case DisplayEvent::WIFI_ON:
            updateCircle(DisplayItem::WIFI, TFT_GREEN);
            Serial.println("WiFi is connected");
            break;
        case DisplayEvent::WIFI_OFF:
            updateCircle(DisplayItem::WIFI, TFT_RED);
            Serial.println("WiFi is disconnected");
            break;
        case DisplayEvent::MQTT_ON:
            updateCircle(DisplayItem::MQTT, TFT_GREEN);
            Serial.println("MQTT is connected");
            break;
        case DisplayEvent::MQTT_OFF:
            updateCircle(DisplayItem::MQTT, TFT_RED);
            Serial.println("MQTT is disconnected");
            break;
        case DisplayEvent::RADIO_ON:
            updateCircle(DisplayItem::RADIO, TFT_GREEN);
            Serial.println("Radio is on");
            break;
        case DisplayEvent::CONNECT_MODE_ON:
            updateCircle(DisplayItem::CONNECT_MODE, TFT_GREEN);
            Serial.println("Connect mode is on");
            break;
        case DisplayEvent::CONNECT_MODE_OFF:
            updateCircle(DisplayItem::CONNECT_MODE, TFT_RED);
            Serial.println("Connect mode is off");
            break;
        default:
            break;
    }
}

void drawItem(DisplayItem item) {
    updateCircle(item, TFT_RED);
    tft.setCursor(2*CIRCLE_RADIUS + 2*HORIZONTAL_SPACING, item * VERTICAL_SPACING + CIRCLE_RADIUS + TEXT_TOP_MARGIN);
    tft.print(display_texts[item]);
}

void initDisplay() {
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    for (int i = 0; i < DISPLAY_ITEM_COUNT; i++) {
        drawItem((DisplayItem)i);
    }
}

void displayTaskFn(void* pvParams) {
    // setup the event loop
    esp_event_loop_args_t updateDisplayLoopArgs = {
        .queue_size = 10,
        .task_name = "update_display_task",
        .task_priority = 1,
        .task_stack_size = 10000,
        .task_core_id = 0
    };

    esp_event_loop_create(&updateDisplayLoopArgs, &updateDisplayLoop);

    // register the handler
    esp_event_handler_instance_register_with(
        updateDisplayLoop,
        UPDATE_DISPLAY,
        ESP_EVENT_ANY_ID,
        updateDisplayHandler,
        NULL,
        NULL
    );

    // initialize the tft display
    initDisplay();

    // allow the radio and fsm tasks to run
    xTaskNotifyGive(radioTask);
    xTaskNotifyGive(fsmTask);

    // run the loop
    while (1) {
        esp_event_loop_run(updateDisplayLoop, 100);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void runRadio(void* pvParams) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    initRadio();

    while (1) {
        mesh.update();
        mesh.DHCP();
        processRadioData();
        esp_task_wdt_reset();
        delay(5);
    }
}

void runButton(void* pvParams) {
    pinMode(PIN_CONNECT_BUTTON, INPUT_PULLUP);
    pinMode(PIN_FACTORY_RESET_BUTTON, INPUT_PULLUP);

    while(1) {
        // connect button
        connectButtonCurrentState = digitalRead(PIN_CONNECT_BUTTON);
        if (connectButtonLastState == HIGH && connectButtonCurrentState == LOW) {
            connectButtonTimer.reset();
        } else if (connectButtonCurrentState == HIGH && connectButtonLastState == LOW) {
            if (connectButtonTimer.elapsed()) {
                Serial.println("Connect mode button on");
                isInConnectMode = true;
                ESP_ERROR_CHECK(
                    esp_event_post_to(
                        updateDisplayLoop,
                        UPDATE_DISPLAY,
                        (int32_t)DisplayEvent::CONNECT_MODE_ON,
                        nullptr,
                        0,
                        portMAX_DELAY)
                );
                connectModeExpirationTimer.reset();
            }
        }
        connectButtonLastState = connectButtonCurrentState;

        // factory reset button
        factoryResetButtonCurrentState = digitalRead(PIN_FACTORY_RESET_BUTTON);
        if (factoryResetButtonLastState == HIGH && factoryResetButtonCurrentState == LOW) {
            factoryResetButtonTimer.reset();
            heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
        } else if (factoryResetButtonCurrentState == HIGH && factoryResetButtonLastState == LOW) {
            if (factoryResetButtonTimer.elapsed()) {
                Serial.println("Factory reset button on");
                factoryReset();
            }
        }
        factoryResetButtonLastState = factoryResetButtonCurrentState;
        esp_task_wdt_reset();
        delay(5);
    }
}

void checkConnectModeExpired(void* pvParams) {
    while (1) {
        if (isInConnectMode && connectModeExpirationTimer.elapsed()) {
            Serial.println("Connect mode expired!");
            isInConnectMode = false;
            ESP_ERROR_CHECK(
                esp_event_post_to(
                    updateDisplayLoop,
                    UPDATE_DISPLAY,
                    (int32_t)DisplayEvent::CONNECT_MODE_OFF,
                    nullptr,
                    0,
                    portMAX_DELAY)
            );
        } 
        esp_task_wdt_reset();
        delay(5);
    }
}

void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId)
{
    mesh.update();
    if (!mesh.write(data, msg_type, size, recvNodeId))
    {
        writeAttempts++;
        Serial.println("Send failed");
        if (writeAttempts > 5)
        {
            Serial.println("Send failed too many times, giving up");
            writeAttempts = 0;
            return;
        }
        safeWriteToMesh(data, msg_type, size, recvNodeId);
    }
    else
    {
        Serial.println("Send OK");
        writeAttempts = 0;
    }
}

void onMqttMsg(String &topic, String &payload)
{
    Serial.println("Incoming message for topic " + topic + ":\n" + payload);

    // parse the incoming topic such that we get everything after INCOMING_TOPIC
    uint8_t moduleGidStart = topic.indexOf(INCOMING_TOPIC) + strlen(INCOMING_TOPIC) + 1;
    uint8_t commandStart = topic.indexOf("/", moduleGidStart) + 1;

    String moduleGidStr = topic.substring(moduleGidStart, commandStart - 1);
    String command = topic.substring(commandStart);

    Serial.println("Module GID: " + moduleGidStr);
    GLOBAL_ID_T gid = stringToGlobalId(moduleGidStr);
    Serial.println(gid, HEX);

    NODE_ID_T nodeId;
    if (globalToNodeIdsMap.getByGID(gid, nodeId))
    {
        Serial.println("Found node id: " + String(nodeId));
        // send the command to the module

        CommandType commandType = getCommand(command);
        uint8_t empty = 0;
        SendStrategy newStrategy = SendStrategy::SEND_ON_CHANGE;
        DELAY_T newDelay = 0;
        switch (commandType)
        {
        case CommandType::FORCE_UPDATE:
            safeWriteToMesh(&empty, (uint8_t)RadioType::GET, sizeof(empty), nodeId);
            break;
        case CommandType::CHANGE_STRATEGY:
            if (payload == "on_change")
            {
                newStrategy = SendStrategy::SEND_ON_CHANGE;
                safeWriteToMesh(&newStrategy, (uint8_t)RadioType::CHANGE_STRATEGY, sizeof(newStrategy), nodeId);
            }
            else if (payload == "always")
            {
                newStrategy = SendStrategy::SEND_ALWAYS;
                safeWriteToMesh(&newStrategy, (uint8_t)RadioType::CHANGE_STRATEGY, sizeof(newStrategy), nodeId);
            }
            break;
        case CommandType::CHANGE_DELAY:
            newDelay = payload.toInt();
            Serial.println("New delay: " + String(newDelay));
            safeWriteToMesh(&newDelay, (uint8_t)RadioType::CHANGE_DELAY, sizeof(newDelay), nodeId);
            break;
        case CommandType::CHANGE_THRESHOLD:
            safeWriteToMesh(payload.c_str(), (uint8_t)RadioType::CHANGE_THRESHOLD, payload.length(), nodeId);
            break;
        default:
            break;
        }
        Serial.println("Command sent!");
    }
    else
    {
        Serial.println("Node id not found!");
    }
}

void printNetworkStatus()
{
    for (int i = 0; i < mesh.addrListTop; i++)
    {
        Serial.print("Address: ");
        Serial.println(mesh.addrList[i].address);
        Serial.print("Node id: ");
        Serial.println(mesh.addrList[i].nodeID);
    }
}

union IncomingMessage
{
    TEMPERATURE_T temperature;
    HUMIDITY_T humidity;
    BOOLEAN_T boolean;
    POTENTIOMETER_T potentiometer;
    DELAY_T delay;
    SendStrategy strategy;
    char threshold[20];
};

void processRadioData()
{
    if (network.available())
    {
        String topic(hubID + "/" + OUTGOING_TOPIC + "/");
        RF24NetworkHeader header;
        uint16_t msg_len;
        msg_len = network.peek(header);

        Serial.println("Received radio message!");
        Serial.println(header.toString());
        // printNetworkStatus();

        String subTopic;

        if (header.type == (unsigned char)RadioType::GID_NEGOTIATION && isInConnectMode){
            GLOBAL_ID_T GID;
            network.read(header, &GID, sizeof(GID));
            Serial.print("Got a node id request from a module with gid 0x");
            Serial.println(GID, HEX);

            NODE_ID_T newNodeID = mesh.getNodeID(header.from_node);
            // Serial.println(globalToNodeIdsMap.getByGID(GID, newNodeID));
            if (!globalToNodeIdsMap.getByGID(GID, newNodeID))
            {
                // the gid is not in the map; assign it an unused node id
                newNodeID = getNewNodeID();
                globalToNodeIdsMap.addSet(newNodeID, GID);
            }
            else
            {
                Serial.println("GID already in map!");
            }

            Serial.print("Sending node id ");
            Serial.println(newNodeID);
            safeWriteToMesh(&newNodeID,
                            (uint8_t)RadioType::GID_NEGOTIATION,
                            sizeof(newNodeID),
                            mesh.getNodeID(header.from_node));
            return;
        }

        GLOBAL_ID_T currentGID = 0;
        globalToNodeIdsMap.getByNodeID(
            mesh.getNodeID(header.from_node),
            currentGID);
        if (currentGID == 0) {
            uint8_t tempBuffer[msg_len];
            Serial.println("Node id not found in map; ignore the message!");
            network.read(header, tempBuffer, msg_len);
            return;
        }
        topic += globalIdToString(currentGID) + "/";
        String brokerMessage;
        IncomingMessage incomingMsg;
        switch (header.type)
        {
        // sensor data
        case (unsigned char)RadioType::TEMPERATURE:
            topic += "temperature";
            network.read(header, &incomingMsg.temperature, sizeof(incomingMsg.temperature));
            brokerMessage = String(incomingMsg.temperature, 1);
            break;
        case (unsigned char)RadioType::HUMIDITY:
            topic += "humidity";
            network.read(header, &incomingMsg.humidity, sizeof(incomingMsg.humidity));
            brokerMessage = String(incomingMsg.humidity, 1);
            break;
        case (unsigned char)RadioType::BOOLEAN:
            topic += "boolean";
            network.read(header, &incomingMsg.boolean, sizeof(incomingMsg.boolean));
            brokerMessage = String(incomingMsg.boolean);
            break;
        case (unsigned char)RadioType::POTENTIOMETER:
            topic += "potentiometer";
            network.read(header, &incomingMsg.potentiometer, sizeof(incomingMsg.potentiometer));
            brokerMessage = String(incomingMsg.potentiometer);
            break;
        // commands
        case (unsigned char)RadioType::CHANGE_STRATEGY:
            topic += COMMAND_CHANGE_STRATEGY;
            network.read(header, &incomingMsg.strategy, sizeof(incomingMsg.strategy));
            if (incomingMsg.strategy == SendStrategy::SEND_ALWAYS)
            {
                brokerMessage = "always";
            }
            else
            {
                brokerMessage = "on_change";
            }
            break;
        case (unsigned char)RadioType::CHANGE_THRESHOLD:
            topic += COMMAND_CHANGE_THRESHOLD;
            network.read(header, &incomingMsg.threshold, sizeof(incomingMsg.threshold));
            brokerMessage = String(incomingMsg.threshold);
            break;
        case (unsigned char)RadioType::CHANGE_DELAY:
            topic += COMMAND_CHANGE_SEND_INTERVAL;
            network.read(header, &incomingMsg.delay, sizeof(incomingMsg.delay));
            brokerMessage = String(incomingMsg.delay);
            break;
        // default
        default:
            topic += "error";
            Serial.print("Unknown radio message type: ");
            Serial.println(header.type);
            return;
        }
        Serial.println("Publishing to mqtt broker: ");
        Serial.print("Topic: ");
        Serial.println(topic);
        Serial.print("Message: ");
        Serial.println(brokerMessage);
        if (!mqttClient.publish(topic, brokerMessage, true, 0))
        {
            Serial.println("Failed to publish to mqtt broker!");
        }
    }
}

void initRadio()
{
    // do the lookup
    // Set the nodeID manually to a temporary value
    mesh.setNodeID(MESH_MAX_NODE_ID);

    for (uint8_t i = RADIO_MIN_CHANNEL; i < RADIO_MAX_CHANNEL; i++)
    {
        // Connect to the mesh
        Serial.print("Checking channel ");
        Serial.println(i);
        radio.begin(&hspi);

        if (!mesh.begin(i, RF24_1MBPS, 5000U))
        {
            Serial.println("Available!");
            mesh.releaseAddress();
            radioChannel = i;
            break;
        }
    }

    // setup radio communication
    Serial.println("\nInitializing radio...");
    mesh.setNodeID(0);
    Serial.print("Set radio node id to ");
    Serial.println(mesh.getNodeID());

    // Connect to the mesh
    if (!mesh.begin(radioChannel, RF24_1MBPS, 7500U))
    {
        // if mesh.begin() returns false for a master node, then radio.begin() returned false.
        Serial.println("Radio hardware not responding.");
        ESP.restart();
    }

    radio.setPALevel(RF24_PA_MIN, 0);
    ESP_ERROR_CHECK(
        esp_event_post_to(
            updateDisplayLoop,
            UPDATE_DISPLAY,
            (int32_t)DisplayEvent::RADIO_ON,
            nullptr,
            0,
            portMAX_DELAY)
    );
}

String globalIdToString(GLOBAL_ID_T gid)
{
    String gidStr = "";
    uint8_t curr_byte;

    for (int i = sizeof(GLOBAL_ID_T) - 1; i >= 0; i--)
    {
        curr_byte = (gid >> (i * 8)) & 0xFF;
        gidStr += String(curr_byte < 10 ? "0" : "") + String(curr_byte, HEX) + (i != 0 ? ":" : "");
    }
    return gidStr;
}

uint16_t getNewNodeID()
{
    // starts from 1 because 0 is always the master node
    for (uint8_t i = 1; i < 255; i++)
    {
        Serial.print("Checking node id ");
        Serial.print(i);
        Serial.print("... ");
        if (mesh.getAddress(i) < 0)
        {
            return i;
        }
        Serial.println(" taken!");
    }
    Serial.println("No more available node ids!");
    return -1;
}

GLOBAL_ID_T stringToGlobalId(const String &buffer)
{
    GLOBAL_ID_T result = 0;
    const char delim[] = ":";
    char bufferCopy[buffer.length() + 1];
    strcpy(bufferCopy, buffer.c_str());
    char *byteToken = strtok(bufferCopy, delim);
    while (byteToken != nullptr)
    {
        result = (result << 8) | strtol(byteToken, nullptr, 16);
        byteToken = strtok(NULL, delim);
    }
    return result;
}

CommandType getCommand(String &commandString)
{
    if (commandString == COMMAND_FORCE_UPDATE)
    {
        return CommandType::FORCE_UPDATE;
    }
    else if (commandString == COMMAND_CHANGE_STRATEGY)
    {
        return CommandType::CHANGE_STRATEGY;
    }
    else if (commandString == COMMAND_CHANGE_SEND_INTERVAL)
    {
        return CommandType::CHANGE_DELAY;
    }
    else if (commandString == COMMAND_CHANGE_THRESHOLD)
    {
        return CommandType::CHANGE_THRESHOLD;
    }
    return CommandType::LAST;
}

void switchState(WiFiConnectionStatus newState)
{
    currentState = newState;
    ranOnce = false;
    Serial.print("Switching to state: ");
    switch (currentState)
    {
    case WiFiConnectionStatus::NO_WIFI:
        Serial.println("NO_WIFI");
        break;
    case WiFiConnectionStatus::WIFI_CONNECTING:
        Serial.println("WIFI_CONNECTING");
        break;
    case WiFiConnectionStatus::NO_MQTT:
        Serial.println("NO_MQTT");
        break;
    case WiFiConnectionStatus::MQTT_CONNECTING:
        Serial.println("MQTT_CONNECTING");
        break;
    case WiFiConnectionStatus::CONNECTED:
        Serial.println("CONNECTED");
        break;
    }
}

bool connectToBroker()
{
    // connect to mqtt broker
    Serial.print("Connecting to MQTT broker[");
    mqttClient.setHost(mqttServer);
    Serial.print(mqttServer);
    Serial.print(", ");
    Serial.print(mqttUser);
    Serial.print(", ");
    Serial.print(mqttPassword);
    Serial.print("]...");
    if (!mqttClient.connect("arduino", mqttUser, mqttPassword))
    {
        Serial.print("Fail: ");
        Serial.println(mqttClient.lastError());
        return false;
    }
    Serial.println("Success!");
    return true;
}

void startDNS()
{
    if (!isDNSServerRunning)
    {
        isDNSServerRunning = dnsServer.start(DNS_PORT, "*", apIP);
    }
}

void stopDNS()
{
    if (isDNSServerRunning)
    {
        dnsServer.stop();
        isDNSServerRunning = false;
    }
}

void startWebServer()
{
    if (!isHTTPServerRunning)
    {
        webServer.begin();
        isHTTPServerRunning = true;
    }
}

void stopWebServer()
{
    if (isHTTPServerRunning)
    {
        webServer.stop();
        isDNSServerRunning = false;
    }
}

void stopPortal()
{
    if (!isConfigPortalRunning) return;
    WiFi.mode(WIFI_STA);
    WiFi.softAPdisconnect(true);
    stopWebServer();
    stopDNS();
    isConfigPortalRunning = false;
}

void handleWiFiRoute()
{
    if (webServer.method() == HTTP_POST)
    {
        webServer.sendHeader("Location", "/", true);
        webServer.send(302, "text/plain", "");
        if (currentState == WiFiConnectionStatus::NO_WIFI)
        {
            shouldTryWiFi = true;
            memset(ssid, 0, MAX_SSID);
            memset(wifiPassword, 0, MAX_WIFi_PASSWORD);
            strncpy(ssid, webServer.arg("ssid").c_str(), MAX_SSID);
            strncpy(wifiPassword, webServer.arg("password").c_str(), MAX_WIFi_PASSWORD);
        }
    }
    else if (webServer.method() == HTTP_GET)
    {
        webServer.send(200, "text/html", FPSTR(HTML_WIFI));
    }
}

void handleMqttRoute()
{
    if (webServer.method() == HTTP_POST)
    {
        Serial.println("req: POST");
        Serial.println((uint8_t)currentState);
        webServer.sendHeader("Location", "/", true);
        webServer.send(302, "text/plain", "");
        if (currentState == WiFiConnectionStatus::NO_MQTT)
        {
            shouldTryMqtt = true;
            Serial.println("req: Setting mqtt data");
            memset(mqttServer, 0, MAX_MQTT_SERVER);
            memset(mqttUser, 0, MAX_MQTT_USER);
            memset(mqttPassword, 0, MAX_MQTT_PASSWORD);
            strncpy(mqttServer, webServer.arg("server").c_str(), MAX_MQTT_SERVER);
            strncpy(mqttUser, webServer.arg("user").c_str(), MAX_MQTT_USER);
            strncpy(mqttPassword, webServer.arg("password").c_str(), MAX_MQTT_PASSWORD);
        }
    }
    else if (webServer.method() == HTTP_GET)
    {
        webServer.send(200, "text/html", FPSTR(HTML_MQTT));
    }
}

void handleHomeRoute()
{
    if (webServer.method() == HTTP_GET)
    {
        webServer.send(200, "text/html", FPSTR(HTML_LANDING_PAGE));
    }
}

void initHttpServer()
{
    webServer.on("/canonical.html", handleHomeRoute);
    webServer.on("/redirect", handleHomeRoute);

    webServer.on("/wifi", handleWiFiRoute);
    webServer.on("/wifiStatus", []()
                 {
    String response ="{\"status\":\"";
    response += isWiFiConnected ? "" : "not ";
    response += "connected\"}";
    webServer.send(200, "application/json", response); });
    webServer.on("/mqtt", handleMqttRoute);
    webServer.on("/mqttStatus", []()
                 {
    String response ="{\"status\":\"";
    response += isMQTTConnected ? "" : "not ";
    response += "connected\"}";
    webServer.send(200, "application/json", response); });

    webServer.on("/landingPage.css", []()
                 { webServer.send(200, "text/css", FPSTR(CSS_LANDING_PAGE)); });

    webServer.on("/formPage.css", []()
                 { webServer.send(200, "text/css", FPSTR(CSS_FORM)); });

    webServer.on("/updateWiFi.js", []()
                 { webServer.send(200, "text/javascript", FPSTR(JS_UPDATE_WIFI)); });

    webServer.on("/updateMQTT.js", []()
                 { webServer.send(200, "text/javascript", FPSTR(JS_UPDATE_MQTT)); });

    webServer.on("/", handleHomeRoute);

    // replay to all requests with same HTML
    webServer.onNotFound([]()
                         {
    String responseHTML = ""
                      "<!DOCTYPE html><html lang='en'><head>"
                      "<meta name='viewport' content='width=device-width'>"
                      "<title>CaptivePortal</title></head><body>"
                      "<h1>Hello World!</h1><p>This is a captive portal example."
                      " All requests will be redirected here.</p></body></html>";
    webServer.send(200, "text/html", responseHTML);
    Serial.println(webServer.method() == HTTP_GET ? "GET" : "POST");
    Serial.println(webServer.uri()); });
}

void factoryReset()
{
    if (preferences.begin(PREFERENCES_NAMESPACE))
    {
        preferences.clear();
    }
    else
    {
        Serial.println("Failed to open preferences!");
    }
    preferences.end();
    ESP.restart();
}

void startPortal(wifi_mode_t wifiMode)
{
    WiFi.mode(wifiMode);
    if (isConfigPortalRunning) return;
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(apSSID, apPassword);
    startWebServer();
    startDNS();
    isConfigPortalRunning = true;
}

void runPortal()
{
    if (!isConfigPortalRunning) return;
    webServer.handleClient();
}

void storeWiFi()
{
    if (preferences.begin(PREFERENCES_NAMESPACE))
    {
        preferences.putString("ssid", ssid);
        preferences.putString("wifiPassword", wifiPassword);
    }
    else
    {
        Serial.println("Failed to open preferences");
    }
    preferences.end();
}

void storeMQTT()
{
    if (preferences.begin(PREFERENCES_NAMESPACE))
    {
        preferences.putString("mqttServer", mqttServer);
        preferences.putString("mqttUser", mqttUser);
        preferences.putString("mqttPassword", mqttPassword);
    }
    else
    {
        Serial.println("Failed to open preferences");
    }
    preferences.end();
}

void tryStoredWiFi()
{
    Serial.println("Checking stored WiFi data...");

    memset(ssid, 0, MAX_SSID);
    memset(wifiPassword, 0, MAX_WIFi_PASSWORD);
    // load wifi configuration from eeprom
    if (preferences.begin(PREFERENCES_NAMESPACE, true))
    {
        Serial.println("Preferences opened");
        if (preferences.isKey("ssid") && preferences.isKey("wifiPassword")) {
            preferences.getString("ssid", ssid, MAX_SSID);
            Serial.println(ssid);
            preferences.getString("wifiPassword", wifiPassword, MAX_WIFi_PASSWORD);
            Serial.println(wifiPassword);
        } else {
            Serial.println("No WiFi credentials stored!");
        }
    }
    else
    {
        Serial.println("Failed to open preferences");
    }
    preferences.end();
    if (strlen(ssid) > 0 && strlen(wifiPassword) > 0)
    {
        shouldTryWiFi = true;
    }
    triedWiFiConfig = true;
}

void tryStoredMQTT()
{
    Serial.println("Checking stored MQTT data...");

    memset(mqttServer, 0, MAX_MQTT_SERVER);
    memset(mqttUser, 0, MAX_MQTT_USER);
    memset(mqttPassword, 0, MAX_MQTT_PASSWORD);
    // load wifi configuration from eeprom
    if (preferences.begin(PREFERENCES_NAMESPACE, true))
    {
        if (preferences.isKey("mqttServer") && preferences.isKey("mqttUser") && preferences.isKey("mqttPassword")) {
            preferences.getString("mqttServer", mqttServer, MAX_MQTT_SERVER);
            preferences.getString("mqttUser", mqttUser, MAX_MQTT_USER);
            preferences.getString("mqttPassword", mqttPassword, MAX_MQTT_PASSWORD);
        } else {
            Serial.println("No MQTT credentials stored!");
        }
    }
    else
    {
        Serial.println("Failed to open preferences");
    }
    preferences.end();

    if (strlen(mqttServer) > 0 && strlen(mqttUser) > 0 && strlen(mqttPassword))
    {
        shouldTryMqtt = true;
    }
    triedMqttConfig = true;
}

void runFSM(void* pvParameters)
{
    (void)pvParameters;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    WiFi.persistent(false);

    Serial.print("fsm task running on core ");
    Serial.println(xPortGetCoreID());
    currentState = WiFiConnectionStatus::NO_WIFI;
    while (1) {
        switch (currentState)
        {
        case WiFiConnectionStatus::NO_WIFI:
            if (!ranOnce)
            {
                isWiFiConnected = false;
                isMQTTConnected = false;
                ranOnce = true;
                shouldTryWiFi = false;
                if (!triedWiFiConfig)
                {
                    tryStoredWiFi();
                    if (!shouldTryWiFi)
                    {
                        startPortal(WIFI_AP);
                    }
                }
                else
                {
                    startPortal(WIFI_AP);
                }
            }
            if (shouldTryWiFi)
            {
                shouldTryWiFi = false;
                wifiConnectionTimer.reset();
                Serial.print("Connecting to WiFi [");
                Serial.print(ssid);
                Serial.print(", ");
                Serial.print(wifiPassword);
                Serial.println("]");
                WiFi.mode(WIFI_AP_STA);
                WiFi.begin(ssid, wifiPassword);
                switchState(WiFiConnectionStatus::WIFI_CONNECTING);
            }
            runPortal();
            break;
        case WiFiConnectionStatus::WIFI_CONNECTING:
            if (!ranOnce)
            {
                ranOnce = true;
                isWiFiConnected = false;
                wifiConnectionTimer.reset();
            }
            if (WiFi.status() == WL_CONNECTED)
            {
                storeWiFi();
                isWiFiConnected = true;
                switchState(WiFiConnectionStatus::NO_MQTT);
                break;
            }
            if (wifiConnectionTimer.elapsed())
            {
                switchState(WiFiConnectionStatus::NO_WIFI);
                break;
            }
            break;
        case WiFiConnectionStatus::NO_MQTT:
            if (!ranOnce)
            {
                ESP_ERROR_CHECK(
                    esp_event_post_to(
                        updateDisplayLoop,
                        UPDATE_DISPLAY,
                        (int32_t)DisplayEvent::WIFI_ON,
                        nullptr,
                        0,
                        portMAX_DELAY)
                );
                ranOnce = true;
                shouldTryMqtt = false;
                if (!triedMqttConfig)
                {   
                    tryStoredMQTT();
                    if (!shouldTryMqtt)
                    {
                        startPortal(WIFI_AP_STA);
                    }
                }
                else
                {
                    startPortal(WIFI_AP_STA);
                }
            }
            if (WiFi.status() != WL_CONNECTED)
            {
                switchState(WiFiConnectionStatus::NO_WIFI);
                ESP_ERROR_CHECK(
                    esp_event_post_to(
                        updateDisplayLoop,
                        UPDATE_DISPLAY,
                        (int32_t)DisplayEvent::WIFI_OFF,
                        nullptr,
                        0,
                        portMAX_DELAY)
                );
                break;
            }
            if (shouldTryMqtt)
            {
                shouldTryMqtt = false;
                switchState(WiFiConnectionStatus::MQTT_CONNECTING);
                break;
            }
            runPortal();
            break;
        case WiFiConnectionStatus::MQTT_CONNECTING:
            if (!ranOnce)
            {
                ranOnce = true;
                stopPortal();
                isMQTTConnected = false;
                mqttAttempts = 0;
                mqttClient.onMessage(onMqttMsg);
                mqttClient.begin(mqttServer, wifiNetworkClient);
                mqttClient.setWill((hubID + "/status").c_str(), "offline", true, 1);
                mqttConnectionTimer.reset();
            }
            if (mqttConnectionTimer.elapsed())
            {
                mqttConnectionTimer.reset();
                mqttAttempts++;
                if (connectToBroker())
                {
                    storeMQTT();
                    isMQTTConnected = true;
                    switchState(WiFiConnectionStatus::CONNECTED);
                    break;
                }
            }
            if (mqttAttempts > MQTT_MAX_RETRIES)
            {
                switchState(WiFiConnectionStatus::NO_MQTT);
                ESP_ERROR_CHECK(
                    esp_event_post_to(
                        updateDisplayLoop,
                        UPDATE_DISPLAY,
                        (int32_t)DisplayEvent::MQTT_OFF,
                        nullptr,
                        0,
                        portMAX_DELAY)
                );
                break;
            }
            break;
        case WiFiConnectionStatus::CONNECTED:
            if (!ranOnce)
            {
                ESP_ERROR_CHECK(
                    esp_event_post_to(
                        updateDisplayLoop,
                        UPDATE_DISPLAY,
                        (int32_t)DisplayEvent::MQTT_ON,
                        nullptr,
                        0,
                        portMAX_DELAY)
                );
                ranOnce = true;
                stopPortal();
                mqttClient.publish((hubID + "/status").c_str(), "online", true, 1);
                mqttClient.subscribe(hubID + "/" + INCOMING_TOPIC + "/#");
            }
            if (!mqttClient.loop())
            {
                Serial.println("MQTT connection lost! Attempting to reconnect...");
                // the mqtt client connection has been lost
                // check the wifi connection and reconnect if necessary
                if (WiFi.status() != WL_CONNECTED)
                {
                    Serial.print("WiFi connection lost! Current status: ");
                    Serial.println(WiFi.status());
                    switchState(WiFiConnectionStatus::NO_WIFI);
                    break;
                }
                else
                {
                    switchState(WiFiConnectionStatus::MQTT_CONNECTING);
                    break;
                }
            }
            break;
        }
        esp_task_wdt_reset();
        delay(10);
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
