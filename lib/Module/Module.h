#pragma once
#include <stdint.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include "GlobalConfig.h"
#include "Timer.h"
#include "SendStrategy.h"

#define DEFAULT_INTERVAL 1000 ///< default interval for sending updates to the master node

#define MESH_MASTER_NODE_ID 0 ///< The ID of the master node
#define MAX_WRITE_ATTEMPTS 5 ///< The maximum number of write attempts before giving up

/// related to the mesh library; added just in case
#ifdef MESH_NOMASTER
#undef MESH_NOMASTER
#endif

enum ConnectionState {
    CONNECTED,
    NOT_CONNECTED,
    CONNECTING,
    RETRY_CONNECTION
};

class Module {
protected:
    RF24 radio;
    RF24Network network;
    RF24Mesh mesh;
    GLOBAL_ID_T globalId;

    uint8_t radioChannel = RADIO_MIN_CHANNEL;

    bool radioModuleAttached = false;

    // stuff for the button
    uint8_t connectButtonState;
    uint8_t lastConnectButtonState;
    uint8_t connectButtonPin;
    Timer connectButtonTimer;

    // stuff for sending data
    SendStrategy sendStrategy = SendStrategy::SEND_ON_CHANGE;
    uint32_t sendInterval = DEFAULT_INTERVAL;
    Timer sendTimer;
    ConnectionState currentState = NOT_CONNECTED;
    bool ranOnce = false;

    uint8_t statusLedPin;
    Timer statusLedBlinkTimer;

    Timer waitForMasterTimer;

    uint8_t writeAttempts = 0;
public:
    Module(GLOBAL_ID_T defaultGlobalId, uint8_t cePin, uint8_t csPin, uint8_t buttonPin, uint8_t statusledPin);

    virtual void initSubmodule() = 0;

    // initializes the module
    void init();

    // main function of the module; should be called periodically
    void run();

    /// @brief retrieves the global id from EEPROM
    /// @return the global id read from EEPROM
    GLOBAL_ID_T readGIDFromEEPROM();

    /// @brief Retries the mesh connection until it succeeds
    void retryMeshConnection();

    /// @brief Safely writes to the mesh, checking for connection and retrying if necessary
    void safeWriteToMesh(const void *data, uint8_t msg_type, size_t size, uint8_t recvNodeId = (uint8_t)0U);

    // @brief inititlizes the radio connection by performing gid negotiation
    // @return true if the radio connection was successful, false otherwise
    bool initRadio();

    // @brief converts a connection state to a string
    // @return the string representation of the connection state
    inline const char* stateToStr(ConnectionState state);

    // @brief switches the connection state of the module
    void switchState(ConnectionState newState);

    // changes the send strategy; DON'T switch to any states in this method
    void changeStrategy(SendStrategy strategy);

    // @brief checks if the module should send data i.e. if the threshodls are met
    virtual bool shouldSend() = 0;

    // @brief sends the data to the master node
    virtual void sendData(bool force = false) = 0;

    virtual void handleRadioMessage(RF24NetworkHeader header, uint16_t incomingBytesCount) = 0;

    // @brief sends the current strategy
    void radioSendStrategy();

    // @brief sends the current send interval
    void radioSendDelay();

    virtual void radioSendThreshold() = 0;
};