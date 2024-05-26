#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <MQTT.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include "GlobalConfig.h"
#include "RadioType.h"
#include "Payload.h"
#include "CronJob.h"
#include "CronJobQueue.h"

/// @brief MQTT setup
#define INCOMING_TOPIC "B2H"
#define OUTGOING_TOPIC "H2B"
#define FORCE_UPDATE "forceUpdate"

/// @brief pin setup
#define PIN_RADIO_CE  4
#define PIN_RADIO_CNS 2

// global constants ************************************************************
const char* mqttServer = "192.168.1.252";
const int mqttPort = 1883;
const String hubID = "00:26:85:22";

const String apSSID = "HUB" + hubID + "_AP";
const String apPassword = "password";

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
    bool getByNodeID(uint16_t nodeId, GLOBAL_ID_T& GID) {
        
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
    bool getByGID(GLOBAL_ID_T gid, uint16_t& nodeId) {
        for (uint32_t i = 0; i <= this->_lastElementIndex; i++) {
            if (gid == this->_GIDs[i]) {
                nodeId = this->_nodeIDs[i];
                return true;
            }
        }
        return false;
    }

    bool addSet(uint16_t nodeID, GLOBAL_ID_T GID) {
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

/// @brief Initializes the cron jobs
void initCronJobs();

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

    Serial.println("Setup complete!");
}

void loop() {
    cronJobs.runJobs();
    delay(10);
}
int last = 0;
// function definitions ********************************************************
void initCronJobs() {
    // deals with mqtt and wifi
    cronJobs.addJob(new CronJob(1, []() {
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
}

void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId)
{
    if (!mesh.write(data, msg_type, size, recvNodeId))
    {

        // If a write fails, check connectivity to the mesh network
        if (!mesh.checkConnection())
        {
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
    
    // // parse the incoming topic such that we get everything after INCOMING_TOPIC
    // // has a +1 because of the last /
    // startIndex = topic.indexOf(INCOMING_TOPIC)+strlen(INCOMING_TOPIC) + 1;
    // char a[100] usefulTopic;
    // String usefulTopic(topic.c_str() + startIndex)
    // if (usefulTopic.indexOf(FORCE_UPDATE) > 0) {
    //     // send a radio message to the module
        
    // }
}

void printNetworkStatus() {
    for (int i = 0; i < mesh.addrListTop; i++) {
        Serial.print("Address: ");
        Serial.println(mesh.addrList[i].address);
        Serial.print("Node id: ");
        Serial.println(mesh.addrList[i].nodeID);
    }
}

void processRadioData() {
    if (network.available()) {
        String topic(hubID + "/");
        RF24NetworkHeader header;
        network.peek(header);

        Serial.println("Received radio message!");
        Serial.println(header.toString());
        // printNetworkStatus();
        
        Payload incomingMsg;
        String subTopic;

        if (header.type == (unsigned char)RadioType::GID_NEGOTIATION) {
            GLOBAL_ID_T GID;
            network.read(header, &GID, sizeof(GID));
            Serial.print("Got a node id request from a module with gid 0x");
            Serial.println(GID, HEX);

            uint16_t newNodeID = mesh.getNodeID(header.from_node);
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

        network.read(header, &incomingMsg, sizeof(incomingMsg));
        String gidBuffer = globalIdToString(incomingMsg.global_id);
        topic += gidBuffer + "/";

        String receivedData;
        switch (header.type) {
            case (unsigned char)RadioType::TEMPERATURE:
                topic += "temperature";
                Serial.print("Temperature: ");
                receivedData = String(((float)incomingMsg.data) / 10.0);
                break;
            case (unsigned char)RadioType::HUMIDITY:
                topic += "humidity";
                Serial.print("Humidity: ");
                receivedData = String(((float)incomingMsg.data) / 10.0);
                break;
            case (unsigned char)RadioType::BOOLEAN:
                topic += "boolean";
                Serial.print("Boolean: ");
                receivedData = String((bool)incomingMsg.data);
                break;
            case (unsigned char)RadioType::POTENTIOMETER:
                topic += "potentiometer";
                Serial.print("Potentiometer: ");
                receivedData = String((POTENTIOMETER_T)incomingMsg.data);
                break;
            default:
                topic += "error";
                Serial.print("Unknown radio message type: ");
                Serial.println(header.type);
                return;
        }
        Serial.println(receivedData);
        if (!mqttClient.publish(topic, receivedData)) {
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
    while (!mqttClient.connect(("arduino", "user", "password"))) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" connected!");
    delay(100);
    mqttClient.subscribe(hubID + "/" + INCOMING_TOPIC);
}

void initMqtt() {
    // initialize mqtt client instance
    mqttClient.begin(mqttServer, mqttPort, wifiNetworkClient);

    // setup mqtt client's callbacks
    mqttClient.onMessage(messageReceivedCb);
}

void initRadio() {
    // setup radio communication
    Serial.println("\nInitializing radio...");
    mesh.setNodeID(0);
    Serial.print("Set radio node id to ");
    Serial.println(mesh.getNodeID());

    // Set the PA Level to MIN and disable LNA for testing & power supply related issues
    radio.begin();
    radio.setPALevel(RF24_PA_MIN, 0);

    // Connect to the mesh
    if (!mesh.begin()) {
        // if mesh.begin() returns false for a master node, then radio.begin() returned false.
        Serial.println("Radio hardware not responding.");
        ESP.restart();
    }
    Serial.println("Mesh connection successful!\n");
}

String globalIdToString(GLOBAL_ID_T gid) {
    String gidStr = "";

    for (int i = 3; i >= 0 ; i--) {
        gidStr += String((gid >> (i * 8)) & 0xFF, HEX) + (i != 0 ? ":" : "");
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
// GLOBAL_ID_T stringToGlobalId(const String& buffer) {
//     const char delim[] = ":";
//     char *byteToken = strtok(buffer.c_str(), delim);
//     while (byteToken != nullptr) {

//         byteToken = strtok(NULL, delim);
//     }
//     return 0;
// }