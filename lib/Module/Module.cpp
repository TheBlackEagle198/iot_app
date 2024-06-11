#include "Module.h"
#include <SPI.h>
#include <EEPROM.h>
#include "RadioType.h"

Module::Module(GLOBAL_ID_T defaultGlobalId, uint8_t cePin, uint8_t csPin, uint8_t buttonPin, uint8_t statusLedPin) :
    radio(cePin, csPin), network(radio), mesh(radio, network), connectButtonPin(buttonPin), connectButtonTimer(1000), statusLedPin(statusLedPin),
    sendTimer(sendInterval), statusLedBlinkTimer(100), waitForMasterTimer(1500){
    globalId = this->readGIDFromEEPROM();
    if (globalId == 0) {
        globalId = defaultGlobalId;
    }
    Serial.println("Hello, world!");
}

GLOBAL_ID_T Module::readGIDFromEEPROM() {
    GLOBAL_ID_T gid = 0;
    EEPROM.get(0, gid);
    return gid;
}

void Module::retryMeshConnection() {
    do
    {
        Serial.print(F("Reconnecting to the mesh... Current address: "));
        Serial.println(this->mesh.mesh_address, OCT);
    } while (this->mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
}

void Module::safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId) {
    delay(10); // helps with radio stability
    if (!mesh.write(data, msg_type, size, recvNodeId)) {
        // If a write fails, check connectivity to the mesh network
        if (!mesh.checkConnection()) {
            retryMeshConnection();
        }
        else {
            Serial.println("Send fail, Still connected to mesh");
        }
    }
    else {
        Serial.print("Send OK; self node id:");
        Serial.print(mesh.getNodeID());
        Serial.print("self mesh id:");
        Serial.println(mesh.mesh_address, OCT);
    }
}

bool Module::initRadio() {
    // Set the nodeID manually to a temporary value
    mesh.setNodeID(MESH_MAX_NODE_ID);

    // Connect to the mesh
    Serial.println("Connecting to the mesh...");
    if (!mesh.begin(97, RF24_1MBPS, 2000)) {
        Serial.println("Mesh failed to start.");
        return false;
    } else {
        // change the temporary node id to the first available address
        // this prevents collisions with other nodes that may be starting up
        for (uint16_t nodeId = MESH_MAX_NODE_ID - 1; nodeId > 0; nodeId--) {
            int currentMeshAddress = mesh.getAddress(nodeId);
            if (currentMeshAddress <= 0) {
                // found an unassigned address
                mesh.setNodeID(nodeId);
                retryMeshConnection();
                break;
            }
            else {
                Serial.print("Address ");
                Serial.print(nodeId);
                Serial.print(" is assigned to node ");
                Serial.println(currentMeshAddress);
            }
        }

        Serial.print("Recieved new temporary address: ");
        Serial.println(mesh.mesh_address, OCT);
    }

    mesh.update();

    // perform the nodeID neogitation based on the GID
    // initialize the negotiation
    safeWriteToMesh((void*)&globalId, (uint8_t)RadioType::GID_NEGOTIATION, sizeof(GLOBAL_ID_T));

    // wait for a response from the master node
    Serial.print("Waiting for master node");
    waitForMasterTimer.reset();
    while (!network.available()) {
        mesh.update();
        if (waitForMasterTimer.elapsed()) {
            Serial.println("Timeout waiting for master node");
            return false;
        }
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
        Serial.print("Recieved NodeID: ");
        Serial.println(receivedId);

        mesh.releaseAddress();
        mesh.setNodeID(receivedId);
    } else {
        Serial.println("Master sent back something else");
        return false;
    }
    return true;
}

void Module::init() {
    initSubmodule();
    pinMode(statusLedPin, OUTPUT);
    pinMode(connectButtonPin, INPUT_PULLUP);
    currentState = ConnectionState::NOT_CONNECTED;
}

void Module::run() {
    switch(currentState) {
        case ConnectionState::CONNECTED:
            if (!radioModuleAttached) {
                switchState(ConnectionState::NOT_CONNECTED);
                return;
            }
            if (!ranOnce) {
                digitalWrite(statusLedPin, LOW);
                ranOnce = true;
            }
            if (statusLedBlinkTimer.elapsed()) {
                digitalWrite(statusLedPin, LOW);
            }
            mesh.update();
            if (network.available()) {
                RF24NetworkHeader header;
                uint16_t incomingBytesCount;
                incomingBytesCount = network.peek(header);
                Serial.println(header.toString());
                mesh.update();
                if (header.type == (uint8_t)RadioType::CHANGE_STRATEGY) {
                    SendStrategy newStrategy;
                    network.read(header, &newStrategy, sizeof(newStrategy));
                    changeStrategy(newStrategy);
                } else if (header.type == (uint8_t)RadioType::GET) {
                    // read that 0 byte
                    uint8_t dummy = 0;
                    network.read(header, &dummy, sizeof(dummy));
                    sendData(true);
                } else if(header.type == (uint8_t)RadioType::CHANGE_DELAY) {
                    DELAY_T newInterval = 0;
                    network.read(header, &newInterval, sizeof(newInterval));
                    sendTimer.setInterval(newInterval); // set the new interval (in ms)
                    sendTimer.reset();
                    Serial.println("Changed send interval to " + String(newInterval) + "ms");
                    safeWriteToMesh(&newInterval, (uint8_t)RadioType::CHANGE_DELAY, sizeof(newInterval));
                } else {
                    handleRadioMessage(header, incomingBytesCount);
                }
            }
            switch (sendStrategy) {
                case SendStrategy::SEND_ON_CHANGE:
                    if (shouldSend()) {
                        sendData();
                        digitalWrite(statusLedPin, HIGH);
                        statusLedBlinkTimer.reset();
                    }
                    break;
                case SendStrategy::SEND_ALWAYS:
                    if (sendTimer.elapsed()) {
                        sendData(true);
                        digitalWrite(statusLedPin, HIGH);
                        statusLedBlinkTimer.reset();
                        sendTimer.reset();
                    }
                    break;
            }
            break;
        case ConnectionState::NOT_CONNECTED:
            if (!ranOnce) {
                digitalWrite(statusLedPin, HIGH);
                mesh.begin();
                ranOnce = true;
                if (!radio.isChipConnected()) {
                    Serial.println("Radio hardware not responding.");
                } else {
                    radioModuleAttached = true;
                }
            }
            break;
        case ConnectionState::CONNECTING:
            if (!radioModuleAttached) {
                switchState(ConnectionState::NOT_CONNECTED);
                return;
            }
            if (!ranOnce) {
                ranOnce = true;
                if (initRadio()) {
                    switchState(ConnectionState::RETRY_CONNECTION);
                } else {
                    switchState(ConnectionState::NOT_CONNECTED);
                }
            }
            break;
        case ConnectionState::RETRY_CONNECTION:
            if (!radioModuleAttached) {
                switchState(ConnectionState::NOT_CONNECTED);
                return;
            }
            if (!ranOnce) {
                Serial.println("Retrying connection");
                retryMeshConnection();
                Serial.println("done");
                ranOnce = true;
                switchState(ConnectionState::CONNECTED);
            }
            break;
    }
    connectButtonState = digitalRead(connectButtonPin);
    if (lastConnectButtonState == HIGH && connectButtonState == LOW) {
        connectButtonTimer.reset();
    } else if (connectButtonState == HIGH && lastConnectButtonState == LOW) {
        if (connectButtonTimer.elapsed()) {
            Serial.println("Long press detected");
            switchState(ConnectionState::CONNECTING);
        }
    }
    lastConnectButtonState = connectButtonState;
}

void Module::switchState(ConnectionState newState) {
    Serial.print("Switching state to ");
    Serial.println(stateToStr(newState));
    ranOnce = false;
    currentState = newState;
}

inline const char* Module::stateToStr(ConnectionState state) {
    switch (state) {
    case ConnectionState::CONNECTED:
        return "CONNECTED";
    case ConnectionState::NOT_CONNECTED:
        return "NOT_CONNECTED";
    case ConnectionState::CONNECTING:
        return "CONNECTING";
    case ConnectionState::RETRY_CONNECTION:
        return "RETRY_CONNECTION";
    default:
        return "UNKNOWN";
    }
}

void Module::changeStrategy(SendStrategy strategy) {
    if (strategy == SendStrategy::SEND_ALWAYS) {
        sendTimer.reset();
    }
    sendStrategy = strategy;
    safeWriteToMesh(&sendStrategy, (uint8_t)RadioType::CHANGE_STRATEGY, sizeof(SendStrategy));
}