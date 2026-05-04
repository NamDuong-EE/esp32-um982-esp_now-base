#ifndef MOCK_LORAWAN_APP_H
#define MOCK_LORAWAN_APP_H

#include <Arduino.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DeviceClass_e {
  CLASS_A = 0,
  CLASS_B,
  CLASS_C
} DeviceClass_t;

typedef enum LoRaMacRegion_e {
  LORAMAC_REGION_AS923 = 0,
  LORAMAC_REGION_EU868,
  LORAMAC_REGION_US915
} LoRaMacRegion_t;

typedef enum DeviceState_e {
  DEVICE_STATE_INIT = 0,
  DEVICE_STATE_JOIN,
  DEVICE_STATE_CYCLE,
  DEVICE_STATE_SLEEP
} DeviceState_t;

typedef enum eMcps {
    /*!
     * Unconfirmed LoRaMAC frame
     */
    MCPS_UNCONFIRMED,
    /*!
     * Confirmed LoRaMAC frame
     */
    MCPS_CONFIRMED,
    /*!
     * Multicast LoRaMAC frame
     */
    MCPS_MULTICAST,
    /*!
     * Proprietary frame
     */
    MCPS_PROPRIETARY,
} Mcps_t;

typedef enum eLoRaMacEventInfoStatus {
    /*!
     * Service performed successfully
     */
    LORAMAC_EVENT_INFO_STATUS_OK = 0,
    /*!
     * An error occurred during the execution of the service
     */
    LORAMAC_EVENT_INFO_STATUS_ERROR,
    /*!
     * A Tx timeout occurred
     */
    LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT,
    /*!
         * An Rx timeout occurred on receive window 1
         */
    LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT,
    /*!
     * An Rx timeout occurred on receive window 2
     */
    LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT,
    /*!
     * An Rx error occurred on receive window 1
     */
    LORAMAC_EVENT_INFO_STATUS_RX1_ERROR,
    /*!
     * An Rx error occurred on receive window 2
     */
    LORAMAC_EVENT_INFO_STATUS_RX2_ERROR,
    /*!
     * An error occurred in the join procedure
     */
    LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL,
    /*!
     * A frame with an invalid downlink counter was received. The
     * downlink counter of the frame was equal to the local copy
     * of the downlink counter of the node.
     */
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED,
    /*!
     * The MAC could not retransmit a frame since the MAC decreased the datarate. The
     * payload size is not applicable for the datarate.
     */
    LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR,
    /*!
     * The node has lost MAX_FCNT_GAP or more frames.
     */
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS,
    /*!
     * An address error occurred
     */
    LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL,
    /*!
     * message integrity check failure
     */
    LORAMAC_EVENT_INFO_STATUS_MIC_FAIL,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOST,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND,
} LoRaMacEventInfoStatus_t;

typedef enum eLoRaMacRxSlot
{
    /*!
     * LoRaMAC receive window 1
     */
    RX_SLOT_WIN_1,
    /*!
     * LoRaMAC receive window 2
     */
    RX_SLOT_WIN_2,
    /*!
     * LoRaMAC receive window 2 for class c - continuous listening
     */
    RX_SLOT_WIN_CLASS_C,
    /*!
     * LoRaMAC class b ping slot window
     */
    RX_SLOT_WIN_PING_SLOT,
    /*!
     * LoRaMAC class b multicast slot window
     */
    RX_SLOT_WIN_MULTICAST_SLOT,
    
    RX_SLOT_WOTA,
}LoRaMacRxSlot_t;

typedef struct sMcpsIndication {
    /*!
     * MCPS-Indication type
     */
    Mcps_t McpsIndication;
    /*!
     * Status of the operation
     */
    LoRaMacEventInfoStatus_t Status;
    /*!
     * Multicast
     */
    uint8_t Multicast;
    /*!
     * Application port
     */
    uint8_t Port;
    /*!
     * Downlink datarate
     */
    uint8_t RxDatarate;
    /*!
     * RxDone datarate
     */
    uint8_t RxDoneDatarate;
    /*!
     * Frame pending status
     */
    uint8_t FramePending;
    /*!
     * Pointer to the received data stream
     */
    uint8_t *Buffer;
    /*!
     * Size of the received data stream
     */
    uint8_t BufferSize;
    /*!
     * Indicates, if data is available
     */
    bool RxData;
    /*!
     * Rssi of the received packet
     */
    int16_t Rssi;
    /*!
     * Snr of the received packet
     */
    int8_t Snr;
    /*!
     * Receive window
     */
    LoRaMacRxSlot_t RxSlot;
    /*!
     * Set if an acknowledgement was received
     */
    bool AckReceived;
    /*!
     * The downlink counter value for the received frame
     */
    uint32_t DownLinkCounter;
#ifdef CONFIG_LWAN
    bool DevTimeAnsReceived;
    bool LinkCheckAnsReceived;
    bool UplinkNeeded;
#endif    
} McpsIndication_t;

extern uint8_t deviceState;
extern uint32_t txDutyCycleTime;

#ifndef LORAWAN_DEVEUI_AUTO
#define LORAWAN_DEVEUI_AUTO 0
#endif

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#ifdef __cplusplus
}
#endif

class MockLoRaWAN {
 public:
  void generateDeveuiByChipID();
  void init(DeviceClass_t cls, LoRaMacRegion_t region);
  void setDefaultDR(uint8_t dr);
  void join();
  void cycle(uint32_t dutyCycle);
  void sleep(DeviceClass_t cls);

  void resetMockState();

  bool generateDeveuiByChipIDCalled = false;
  bool initCalled = false;
  bool setDefaultDRCalled = false;
  bool joinCalled = false;
  bool cycleCalled = false;
  bool sleepCalled = false;

  DeviceClass_t lastClass = CLASS_A;
  LoRaMacRegion_t lastRegion = LORAMAC_REGION_AS923;
  uint8_t lastDefaultDR = 0;
  uint32_t lastCycleDuty = 0;
};

extern MockLoRaWAN LoRaWAN;

#endif
