#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RadioType.h"
#include "Payload.h"
#include "CronJob.h"
#include "GlobalConfig.h"


#define PIN_POT A0 ///< the pin where the potentiometer is connected

#define MESH_MASTER_NODE_ID 0 ///< The ID of the master node
#define MESH_MAX_NODE_ID 255 ///< The maximum node ID

/// related to the mesh library; added just in case
#ifdef MESH_NOMASTER
#undef MESH_NOMASTER
#endif

/// global variables ***********************************************************
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
GLOBAL_ID_T MODULE_GID = 0x00000000;

const POTENTIOMETER_T THRESHOLD = 100; ///< the threshold for sending updates to the master node

POTENTIOMETER_T readPot; ///< the last read value of the potentiometer
POTENTIOMETER_T potLastSent; ///< the last value of the potentiometer sent to the master node

CronJob sendUpdate; ///< the job that sends updates to the master node

/// function prototypes ********************************************************

/// @brief prints the identifiers of the module(GID, nodeID, mesh address)
void printIdentifiers();

/// @brief retrieves the global id from EEPROM
/// @return the global id read from EEPROM
GLOBAL_ID_T readGIDFromEEPROM();

/// @brief Blink the LED on the board three times
void blinkFail();

/// @brief Retries the mesh connection until it succeeds
void retryMeshConnection();

/// @brief Safely writes to the mesh, checking for connection and retrying if necessary
void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId = (uint8_t)0U);

/// main code ******************************************************************
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial)
    {
        // some boards need this because of native USB capability
    }

    // load the global id
    MODULE_GID = readGIDFromEEPROM();

    // Set the nodeID manually to a temporary value
    mesh.setNodeID(MESH_MAX_NODE_ID);

    // Set the PA Level to MIN and disable LNA for testing & power supply related issues
    radio.begin();
    radio.setPALevel(RF24_PA_MIN, 0);

    if (!radio.isChipConnected())
    {
        Serial.println("Radio hardware not responding.");
        while (1) {
            blinkFail();
        }
    }

    // Connect to the mesh
    Serial.println("Connecting to the mesh...");

    if (!mesh.begin()) {
        Serial.println("Mesh failed to start.");
        while (1) {
            blinkFail();
        }
    } else {
        // change the temporary node id to the first available address
        // this prevents collisions with other nodes that may be starting up
        for (uint16_t i = MESH_MAX_NODE_ID - 1; i > 0; i--) {
            if (mesh.getAddress(i) < 0) {
                // found an unassigned address
                mesh.setNodeID(i);
                retryMeshConnection();
                break;
            }
        }

        Serial.print("Temporary identifers: ");
        printIdentifiers();
    }

    // not sure if this update is needed; added it just in case
    mesh.update();

    // perform the nodeID neogitation based on the GID
    // initialize the negotiation
    safeWriteToMesh((void*)&MODULE_GID, (uint8_t)RadioType::GID_NEGOTIATION, sizeof(GLOBAL_ID_T));

    // wait for a response from the master node
    Serial.print("Waiting for master node");
    while (!network.available()) {
        Serial.print(".");
        mesh.update();
    }
    Serial.println();
    Serial.println("Done waiting for master node");

    RF24NetworkHeader header;
    network.peek(header);
    if (header.type == (uint8_t)RadioType::GID_NEGOTIATION) {
        uint16_t receivedId;
        Serial.print("Master sent back a GID_NEGOTIATION message, read ");
        Serial.print(network.read(header, &receivedId, sizeof(receivedId)));
        Serial.println(" bytes");
        // if (network.read(header, &receivedId, sizeof(receivedId)) < sizeof(receivedId)) {
        //     Serial.println("Failed to read GID from master node");
        //     while (1) {
        //         blinkFail();
        //     }
        // }
        Serial.print("Recieved NodeID: ");
        Serial.println(receivedId);

        mesh.setNodeID(receivedId);
        retryMeshConnection();
    } else {
        Serial.println("Master sent back something else");
        while (1) {
            blinkFail();
        }
    }

    printIdentifiers();

    sendUpdate.init(500, []() {
        POTENTIOMETER_T readPot = analogRead(PIN_POT);
        if (abs(readPot - potLastSent) > THRESHOLD) {
            Serial.print("Current potentiometer value: ");
            Serial.println(readPot);
            Serial.print("Last sent potentiometer value: ");
            Serial.println(potLastSent);
            Serial.print("Difference: ");
            Serial.println(abs(readPot - potLastSent));
            potLastSent = readPot;
            Payload payload;
            payload.global_id = MODULE_GID;
            payload.data = readPot;
            safeWriteToMesh(&payload, (uint8_t)RadioType::POTENTIOMETER, sizeof(payload));
        }
    });

    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    mesh.update();
    sendUpdate.run();

    if (network.available()) {
        RF24NetworkHeader header;
        network.peek(header);
        Serial.println(header.toString());
    }
}

/// function definitions *******************************************************

void printIdentifiers() {
    Serial.print("Current GID: ");
    Serial.print(MODULE_GID, HEX);
    Serial.print(" Current nodeID: ");
    Serial.print(mesh.getNodeID());
    Serial.print(" Current mesh address: ");
    Serial.println(mesh.mesh_address, BIN);
}

GLOBAL_ID_T readGIDFromEEPROM() {
    GLOBAL_ID_T gid = 0;
    EEPROM.get(0, gid);
    return gid;
}

void blinkFail() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay (1000);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(5000);
}

void retryMeshConnection() {
    do
    {
        Serial.print(F("Reconnecting to the mesh... Current identifers: "));
        printIdentifiers();
    } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
}

void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId = (uint8_t)0U)
{
    if (!mesh.write(data, msg_type, size, recvNodeId))
    {

        // If a write fails, check connectivity to the mesh network
        if (!mesh.checkConnection())
        {
            retryMeshConnection();
        }
        else
        {
            Serial.println("Send fail, Still connected to mesh");
        }
    }
    else
    {
        Serial.print("Send OK; self node id:");
        Serial.println(mesh.getNodeID());
    }
}
