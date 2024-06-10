#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <MQTT.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include "GlobalConfig.h"
#include "RadioType.h"
#include "CronJob.h"
#include "CronJobQueue.h"
#include "Timer.h"
#include "SendStrategy.h"

/// @brief MQTT setup
#define INCOMING_TOPIC "B2H"
#define OUTGOING_TOPIC "H2B"
#define COMMAND_FORCE_UPDATE "force_update"
#define COMMAND_CHANGE_STRATEGY "strategy"
#define COMMAND_CHANGE_SEND_INTERVAL "send_interval"
#define COMMAND_CHANGE_THRESHOLD "threshold"

#define LONG_PRESS_TIME 1000
#define CONNECT_MODE_EXPIRATION 20000

/// @brief pin setup
#define PIN_RADIO_CE  4
#define PIN_RADIO_CNS 2
#define PIN_CONNECT_BUTTON 5

// global constants ************************************************************
const char* mqttServer = "192.168.1.252";
const int mqttPort = 1883;
const String hubID = "00:26:85:22";

const String apSSID = "HUB" + hubID + "_AP";
const String apPassword = "password";

enum class CommandType {
    FORCE_UPDATE,
    CHANGE_STRATEGY,
    CHANGE_THRESHOLD,
    CHANGE_DELAY,
    LAST
};

template <int MAX_SIZE>
class GlobalToNodeIdsMap {
private:
    uint16_t _nodeIDs[MAX_SIZE];
    GLOBAL_ID_T _GIDs[MAX_SIZE];

    uint32_t _lastElementIndex;

public:
    GlobalToNodeIdsMap() : _lastElementIndex(0) {
        memset(this->_nodeIDs, 0, sizeof(this->_nodeIDs));
        memset(this->_GIDs, 0, sizeof(this->_GIDs));
    }

    /// @brief Gets the global id associated with the given node id
    /// @param[in] nodeId - the node id to search for
    /// @param[out] GID - the global id if found
    /// @return true if the global id was found, false otherwise
    bool getByNodeID(NODE_ID_T nodeId, GLOBAL_ID_T& GID) {
        
        for (uint32_t i = 0; i <= this->_lastElementIndex; i++) {
            if (nodeId == this->_nodeIDs[i]) {
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
    bool getByGID(GLOBAL_ID_T gid, NODE_ID_T& nodeId) {
        for (uint32_t i = 0; i <= this->_lastElementIndex; i++) {
            if (gid == this->_GIDs[i]) {
                nodeId = this->_nodeIDs[i];
                return true;
            }
        }
        return false;
    }

    bool addSet(NODE_ID_T nodeID, GLOBAL_ID_T GID) {
        if (this->_lastElementIndex == MAX_SIZE - 2) return false;
        this->_lastElementIndex++;
        this->_nodeIDs[this->_lastElementIndex] = nodeID;
        this->_GIDs[this->_lastElementIndex] = GID;
        return true;
    }
};

// global variables ***********************************************************
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CNS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
// ^^ rf mesh variables
WiFiClient wifiNetworkClient;
MQTTClient mqttClient;
// ^^ mqtt variables
GlobalToNodeIdsMap<255> globalToNodeIdsMap;
CronJobQueue<10> cronJobs;

bool isInConnectMode = false;
uint8_t connectButtonCurrentState = LOW;
uint8_t connectButtonLastState = HIGH;
Timer connectModeExpirationTimer(CONNECT_MODE_EXPIRATION);
Timer longPressTimer(LONG_PRESS_TIME);

// function declarations *******************************************************

void connectToBroker();

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

/// @brief Sets up internet connection
void initWiFi();

/// @brief Setups mqtt client and connects to mqtt broker
void initMqtt();

/// @brief Sets up radio connection
void initRadio();

/// @brief Finds the next available node id
uint16_t getNewNodeID();

/// @brief Converts a global id to a string with hex values separated by colons
String globalIdToString(GLOBAL_ID_T gid);

/// @brief Converts a string with hex values separated by colons to a global id
GLOBAL_ID_T stringToGlobalId(const String& buffer);

/// @brief Initializes the cron jobs
void initCronJobs();

CommandType getCommand(String &commandString);

// main program ***************************************************************
void setup() {
    #ifndef ENVIRONMENT
    while(true) {}
    #endif

    Serial.begin(115200);
    
    #if ENVIRONMENT == DEVELOPMENT
    while (!Serial) {}
    #endif


    initWiFi();
    initMqtt();
    connectToBroker();
    initCronJobs();
    initRadio();

    pinMode(PIN_CONNECT_BUTTON, INPUT_PULLUP);

    Serial.println("Setup complete!");
}

void loop() {
    cronJobs.runJobs();
}
int last = 0;
// function definitions ********************************************************
void initCronJobs() {
    // deals with mqtt and wifi
    cronJobs.addJob(new CronJob(10, []() {
        if (!mqttClient.loop()) {
            Serial.println("MQTT connection lost! Attempting to reconnect...");
            // the mqtt client connection has been lost
            // check the wifi connection and reconnect if necessary
            if (WiFi.status() != WL_CONNECTED) {
                Serial.print("WiFi connection lost! Current status: ");
                Serial.println(WiFi.status());
                initWiFi();
            }
            connectToBroker();
        }
    }));
    // deals with radio
    cronJobs.addJob(new CronJob(1, []() {
        processRadioData();
        mesh.update();
        mesh.DHCP();
    }));
    cronJobs.addJob(new CronJob(1, []() {
        connectButtonCurrentState = digitalRead(PIN_CONNECT_BUTTON);
        if (connectButtonLastState == HIGH && connectButtonCurrentState == LOW) {
            longPressTimer.reset();
        } else if (connectButtonCurrentState == HIGH && connectButtonLastState == LOW) {
            if (longPressTimer.elapsed()) {
                Serial.println("Long press detected");
                isInConnectMode = true;
                connectModeExpirationTimer.reset();
            }
        }
        connectButtonLastState = connectButtonCurrentState;
    }));
    cronJobs.addJob(new CronJob(1, []() {
        if (isInConnectMode && connectModeExpirationTimer.elapsed()) {
            Serial.println("Connect mode expired!");
            isInConnectMode = false;
        }
    }));
}

void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId)
{
    if (!mesh.write(data, msg_type, size, recvNodeId))
    {

        // If a write fails, check connectivity to the mesh network
        if (!mesh.checkConnection())
        {
            Serial.print("RF24 failure reason: ");
            radio.printDetails();
            // refresh the network address
            Serial.println("Renewing Address");
            if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS)
            {
                // If address renewal fails, reconfigure the radio and restart the mesh
                // This allows recovery from most if not all radio errors
                mesh.begin();
            }
        }
        else
        {
            Serial.println("Send fail, Test OK");
        }
    }
    else
    {
        Serial.println("Send OK");
    }
}

void messageReceivedCb(String &topic, String &payload) {
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
    if (globalToNodeIdsMap.getByGID(gid, nodeId)) {
        Serial.println("Found node id: " + String(nodeId));
        // send the command to the module
        
        CommandType commandType = getCommand(command);
        uint8_t empty = 0;
        SendStrategy newStrategy = SendStrategy::SEND_ON_CHANGE;
        switch (commandType) {
            case CommandType::FORCE_UPDATE:
                safeWriteToMesh(&empty, (uint8_t)RadioType::GET, sizeof(empty), nodeId);
                break;
            case CommandType::CHANGE_STRATEGY:
                if (payload == "on_change") {
                    newStrategy = SendStrategy::SEND_ON_CHANGE;
                    safeWriteToMesh(&newStrategy, (uint8_t)RadioType::CHANGE_STRATEGY, sizeof(newStrategy), nodeId);
                } else if (payload == "always") {
                    newStrategy = SendStrategy::SEND_ALWAYS;
                    safeWriteToMesh(&newStrategy, (uint8_t)RadioType::CHANGE_STRATEGY, sizeof(newStrategy), nodeId);
                }
                break;
            case CommandType::CHANGE_DELAY:
                safeWriteToMesh(payload.c_str(), (uint8_t)RadioType::CHANGE_DELAY, payload.length(), nodeId);
                break;
            case CommandType::CHANGE_THRESHOLD:
                safeWriteToMesh(payload.c_str(), (uint8_t)RadioType::CHANGE_THRESHOLD, payload.length(), nodeId);
                break;
            default:
                break;
        }
        Serial.println("Command sent!");
    } else {
        Serial.println("Node id not found!");
    }

}

void printNetworkStatus() {
    for (int i = 0; i < mesh.addrListTop; i++) {
        Serial.print("Address: ");
        Serial.println(mesh.addrList[i].address);
        Serial.print("Node id: ");
        Serial.println(mesh.addrList[i].nodeID);
    }
}

union IncomingMessage {
    TEMPERATURE_T temperature;
    HUMIDITY_T humidity;
    BOOLEAN_T boolean;
    POTENTIOMETER_T potentiometer;
};

void processRadioData() {
    if (network.available()) {
        String topic(hubID + "/" + OUTGOING_TOPIC + "/");
        RF24NetworkHeader header;
        network.peek(header);

        Serial.println("Received radio message!");
        Serial.println(header.toString());
        // printNetworkStatus();
        
        String subTopic;

        if (header.type == (unsigned char)RadioType::GID_NEGOTIATION) { //  && isInConnectMode
            GLOBAL_ID_T GID;
            network.read(header, &GID, sizeof(GID));
            Serial.print("Got a node id request from a module with gid 0x");
            Serial.println(GID, HEX);

            NODE_ID_T newNodeID = mesh.getNodeID(header.from_node);
            // Serial.println(globalToNodeIdsMap.getByGID(GID, newNodeID));
            if (!globalToNodeIdsMap.getByGID(GID, newNodeID)) {
                // the gid is not in the map; assign it an unused node id
                newNodeID = getNewNodeID();
                globalToNodeIdsMap.addSet(newNodeID, GID);
            } else {
                Serial.println("GID already in map!");
            }

            Serial.print("Sending node id ");
            Serial.println(newNodeID);
            safeWriteToMesh(&newNodeID, 
                            (unsigned char)RadioType::GID_NEGOTIATION, 
                            sizeof(newNodeID), 
                            mesh.getNodeID(header.from_node));
            return;
        }

        GLOBAL_ID_T currentGID = 0;
        globalToNodeIdsMap.getByNodeID(
            mesh.getNodeID(header.from_node),
            currentGID
        );
        topic += globalIdToString(currentGID) + "/";
        String brokerMessage;
        IncomingMessage incomingMsg;
        switch (header.type) {
            // sensor data
            case (unsigned char)RadioType::TEMPERATURE:
                topic += "temperature";
                network.read(header, &incomingMsg.temperature, sizeof(incomingMsg));
                brokerMessage = String(incomingMsg.temperature, 1);
                break;
            case (unsigned char)RadioType::HUMIDITY:
                topic += "humidity";
                network.read(header, &incomingMsg, sizeof(incomingMsg));
                brokerMessage = String(incomingMsg.humidity, 1);
                break;
            case (unsigned char)RadioType::BOOLEAN:
                topic += "boolean";
                network.read(header, &incomingMsg, sizeof(incomingMsg));
                brokerMessage = String(incomingMsg.boolean);
                break;
            case (unsigned char)RadioType::POTENTIOMETER:
                topic += "potentiometer";
                network.read(header, &incomingMsg, sizeof(incomingMsg));
                brokerMessage = String(incomingMsg.potentiometer);
                break;
            // commands
            // case (unsigned char)RadioType::CHANGE_STRATEGY:
            //     topic += "strategy";
            //     Serial.print("Strategy: ");
            //     if ((SendStrategy)incomingMsg.data == SendStrategy::SEND_ALWAYS) {
            //         brokerMessage = "always";
            //     } else {
            //         brokerMessage = "on_change";
            //     }
            //     break;
            // case (unsigned char)RadioType::CHANGE_THRESHOLD:
            //     topic += "threshold";
            //     brokerMessage = String(incomingMsg.data);
            //     break;
            // case (unsigned char)RadioType::CHANGE_DELAY:
            //     topic += "delay";
            //     brokerMessage = String(incomingMsg.data);
            //     break;
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
        if (!mqttClient.publish(topic, brokerMessage, true, 0)) {
            Serial.println("Failed to publish to mqtt broker!");
        }
    }
}

void initWiFi() {
    WiFiManager wm;

    if(!wm.autoConnect(apSSID.c_str(), apPassword.c_str())) {
        Serial.println("Failed to connect to wifi");
        ESP.restart();
    } 
    else {
        Serial.println("Wifi connection successful! Connected to " + WiFi.SSID());
    }
}

void connectToBroker() {
    // connect to mqtt broker
    Serial.print("Connecting to MQTT broker[");
    Serial.print(mqttServer);
    Serial.print("]...");
    while (!mqttClient.connect("arduino", "hub", "cqLnvEYq")) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" connected!");
    mqttClient.publish((hubID + "/status").c_str(), "online", true, 1);
    delay(100);
    mqttClient.subscribe(hubID + "/" + INCOMING_TOPIC + "/#");
}

void initMqtt() {
    // initialize mqtt client instance
    mqttClient.begin(mqttServer, mqttPort, wifiNetworkClient);
    mqttClient.setCleanSession(false);
    mqttClient.setWill((hubID + "/status").c_str(), "offline", true, 1);

    // setup mqtt client's callbacks
    mqttClient.onMessage(messageReceivedCb);
}

void initRadio() {
    // setup radio communication
    Serial.println("\nInitializing radio...");
    mesh.setNodeID(0);
    Serial.print("Set radio node id to ");
    Serial.println(mesh.getNodeID());

    // Connect to the mesh
    if (!mesh.begin()) {
        // if mesh.begin() returns false for a master node, then radio.begin() returned false.
        Serial.println("Radio hardware not responding.");
        ESP.restart();
    }

    radio.setPALevel(RF24_PA_MIN, 0);
}

String globalIdToString(GLOBAL_ID_T gid) {
    String gidStr = "";
    uint8_t curr_byte;

    for (int i = sizeof(GLOBAL_ID_T) - 1; i >= 0 ; i--) {
        curr_byte = (gid >> (i * 8)) & 0xFF;
        gidStr += String(curr_byte < 10 ? "0" : "") + String(curr_byte, HEX) + (i != 0 ? ":" : "");
    }
    return gidStr;
}

uint16_t getNewNodeID() {
    // starts from 1 because 0 is always the master node
    for (uint8_t i = 1; i < 255; i++) {
        Serial.print("Checking node id ");
        Serial.print(i);
        Serial.print("... ");
        if (mesh.getAddress(i) < 0) {
            return i;
        }
        Serial.println(" taken!");
    }
    Serial.println("No more available node ids!");
    return -1;
}

GLOBAL_ID_T stringToGlobalId(const String& buffer) {
    GLOBAL_ID_T result = 0;
    const char delim[] = ":";
    char bufferCopy[buffer.length() + 1];
    strcpy(bufferCopy, buffer.c_str());
    char *byteToken = strtok(bufferCopy, delim);
    while (byteToken != nullptr) {
        result = (result << 8) | strtol(byteToken, nullptr, 16);
        byteToken = strtok(NULL, delim);
    }
    return result;
}

CommandType getCommand(String &commandString) {
    if (commandString == COMMAND_FORCE_UPDATE) {
        return CommandType::FORCE_UPDATE;
    } else if (commandString == COMMAND_CHANGE_STRATEGY) {
        return CommandType::CHANGE_STRATEGY;
    } else if (commandString == COMMAND_CHANGE_SEND_INTERVAL) {
        return CommandType::CHANGE_DELAY;
    } else if (commandString == COMMAND_CHANGE_THRESHOLD) {
        return CommandType::CHANGE_THRESHOLD;
    }
    return CommandType::LAST;
}