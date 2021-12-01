/**
 * @file core_modbus.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**
 * @file core_modbus.h
 */
#ifndef CORE_MODBUS_H
#define CORE_MODBUS_H

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C"
{
#endif
/* *INDENT-ON* */

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>

/* MODBUS_DO_NOT_USE_CUSTOM_CONFIG allows building the MODBUS library
 * without a custom config. If a custom config is provided, the
 * MODBUS_DO_NOT_USE_CUSTOM_CONFIG macro should not be defined. */
#ifndef MODBUS_DO_NOT_USE_CUSTOM_CONFIG
/* Include custom config file before other headers. */
#include "core_modbus_config.h"
#endif

/* Include config defaults header to get default values of configs not
 * defined in core_modbus_config.h file. */
#include "core_modbus_config_defaults.h"

// /* Include MODBUS serializer library. */
// #include "core_modbus_serializer.h"

/* Include transport interface. */
#include "transport_interface.h"

/**
 * @cond DOXYGEN_IGNORE
 * The current version of this library.
 */
#define MODBUS_LIBRARY_VERSION "v0.0.1"
  /** @endcond */

  /* Exported typedefs ---------------------------------------------------------*/
#define modbusHTONS(x) FreeRTOS_htons(x)
#define modbusHTONL(x) FreeRTOS_htonl(x)
#define modbusNTOHS(x) FreeRTOS_ntohs(x)
#define modbusNTOHL(x) FreeRTOS_ntohl(x)

  /* Exported typedefs ---------------------------------------------------------*/
  struct ModbusServer;
  // struct ModbusClient;
  struct ModbusServerConfig;
  // typedef struct ModbusServerConfig_s ModbusServerConfig_t;
  // typedef struct modbus_server_s ModbusServer_t;
  // typedef struct modbus_client_config_s modbus_client_config_t;
  // typedef struct modbus_client_s ModbusClient_t;

  /* Structures defined in this file. */
  struct ModbusCounter;
  struct ModbusRequest;
  struct ModbusMBAPHeader;
  struct ModbusPDU;

  /**
   * @ingroup modbus
   * Connection state codes */
  typedef enum ModbusStatus
  {
    // MODBUSSuccess = 0,      /**< Function completed successfully. */
    // MODBUSBadParameter,     /**< At least one parameter was invalid. */
    // MODBUSNoMemory,         /**< A provided buffer was too small. */
    // MODBUSSendFailed,       /**< The transport send function failed. */
    // MODBUSRecvFailed,       /**< The transport receive function failed. */
    // MODBUSBadResponse,      /**< An invalid packet was received from the server. */
    // MODBUSBadRequest,       /**< An invalid packet was received from the client. */
    // ModbusServerRefused,    /**< The server refused a CONNECT or SUBSCRIBE. */
    // MODBUSNoDataAvailable,  /**< No data available from the transport interface. */
    // MODBUSIllegalState,     /**< An illegal state in the state record. */
    // MODBUSStateCollision,   /**< A collision with an existing state record entry. */
    // MODBUSKeepAliveTimeout, /**< Timeout while waiting for PINGRESP. */
    MODBUS_STATE_SUCCESS = 0, /** Function completed successfully. */
    // MODBUS_STATE_DISCONNECTED,    /** Disconnected */
    // MODBUS_STATE_ACCEPTED,        /** Accepted */
    MODBUS_STATE_BAD_REQUEST,  /** An invalid packet was received from the client */
    MODBUS_STATE_REFUSED_MBAP, /** Refused Modbus Application Protocol header */
    // MODBUS_STATE_REFUSED_SERVER,  /** Refused server */
    // MODBUS_STATE_LISTENING,       /** Server listening on connections */
    // MODBUS_STATE_ERROR,           /** TCP Error occured */
    // MODBUS_STATE_SERVER_SHUTDOWN, /** Server shutdown */
    // MODBUS_STATE_RETRIES,         /** Retries */
    // MODBUS_STATE_TIMEOUT,         /** Timeout */
    // MODBUS_STATE_OUT_OF_MEMORY,   /** Out of memory */
  } ModbusStatus_t;

  /**
   * @ingroup modbus
   * Modbus Application Protocol Specification V1.1b3 (p. 48) */
  typedef enum ModbusException
  {
    MODBUS_EXC_NO_EXCEPTION = 0x00,
    /**
     * The function code received in the query is not an
     * allowable action for the server. This may be
     * because the function code is only applicable to
     * newer devices, and was not implemented in the
     * unit selected. It could also indicate that the server
     * is in the wrong state to process a request of this
     * type, for example because it is unconfigured and
     * is being asked to return register values.
     */
    MODBUS_EXC_ILLEGAL_FUNCTION = 0x01,
    /**
     * The data address received in the query is not an
     * allowable address for the server. More
     * specifically, the combination of reference number
     * and transfer length is invalid. For a controller with
     * 100 registers, the PDU addresses the first register
     * as 0, and the last one as 99. If a request is
     * submitted with a starting register address of 96
     * and a quantity of registers of 4, then this request
     * will successfully operate (address-wise at least)
     * on registers 96, 97, 98, 99. If a request is
     * submitted with a starting register address of 96
     * and a quantity of registers of 5, then this request
     * will fail with Exception Code 0x02 “Illegal Data
     * Address” since it attempts to operate on registers
     * 96, 97, 98, 99 and 100, and there is no register
     * with address 100.
     */
    MODBUS_EXC_ILLEGAL_DATA_ADDRESS = 0x02,
    /**
     * A value contained in the query data field is not an
     * allowable value for server. This indicates a fault in
     * the structure of the remainder of a complex
     * request, such as that the implied length is
     * incorrect. It specifically does NOT mean that a
     * data item submitted for storage in a register has a
     * value outside the expectation of the application
     * program, since the Modbus protocol is unaware
     * of the significance of any particular value of any
     * particular register.
     */
    MODBUS_EXC_ILLEGAL_DATA_VALUE = 0x03,
    /**
     * An unrecoverable error occurred while the server
     * was attempting to perform the requested action.
     */
    MODBUS_EXC_SLAVE_DEVICE_FAILURE = 0x04,
    /**
     * Specialized use in conjunction with programming
     * commands.
     * The server has accepted the request and is
     * processing it, but a long duration of time will be
     * required to do so. This response is returned to
     * prevent a timeout error from occurring in the
     * client. The client can next issue a Poll Program
     * Complete message to determine if processing is
     * completed.
     */
    MODBUS_EXC_ACKNOWLEDGE = 0x05,
    /**
     * Specialized use in conjunction with programming
     * commands.
     * The server is engaged in processing a long–
     * duration program command. The client should
     * retransmit the message later when the server is
     * free.
     */
    MODBUS_EXC_SLAVE_DEVICE_BUSY = 0x06,
    /**
     * Specialized use in conjunction with function codes
     * 20 and 21 and reference type 6, to indicate that
     * the extended file area failed to pass a consistency
     * check.
     * The server attempted to read record file, but
     * detected a parity error in the memory. The client
     * can retry the request, but service may be required
     * on the server device.
     */
    MODBUS_EXC_MEMORY_PARITY_ERROR = 0x08,
    /**
     * Specialized use in conjunction with gateways,
     * indicates that the gateway was unable to allocate
     * an internal communication path from the input port
     * to the output port for processing the request.
     * Usually means that the gateway is misconfigured
     * or overloaded.
     */
    MODBUS_EXC_GATEWAY_PATH_UNAVAILABLE = 0x0A,
    /**
     * Specialized use in conjunction with gateways,
     * indicates that no response was obtained from the
     * target device. Usually means that the device is not
     * present on the network.
     */
    MODBUS_EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B
  } ModbusException_t;

  /* Modbus Request structure ------------------------------------------------*/

  typedef struct ModbusCounter
  {
    uint16_t message;               /* Bus Message Count */
    uint16_t crc_error;             /* Bus Communication Error Count */
    uint16_t exception_error;       /* Bus Exception Error Count */
    uint16_t server_message;        /* Server Message Count */
    uint16_t server_no_response;    /* Server No Response Count */
    uint16_t server_nak;            /* Server Negative Acknowledge (NAK) Count */
    uint16_t server_busy;           /* Server Busy Count */
    uint16_t bus_character_overrun; /* Bus Character Overrun Count */
  } ModbusCounter_t;

  /** Modbus Application Protocol header (MBAP) - 7 Byte */
  typedef struct ModbusMBAPHeader
  {
    uint16_t usTID;           /* Transaction Identifier - 2 Byte */
    uint16_t usPID;           /* Protocol Identifier - 2 Byte */
    uint16_t usLength;        /* Number of bytes - 2 Byte */
    uint8_t ucUnitIdentifier; /* Unit Identifier - 1 Byte */
  } ModbusMBAPHeader_t;

  /** Modbus Protocol Data Unit (PDU) - 253 Byte */
  typedef struct ModbusPDU
  {
    uint8_t ucFunctionCode; /** Modbus Function Code - 1 Byte */
    /** Protocol Data - This field is function code dependent and usually contains
     * information such as variable references, variable counts, data offsets,
     * sub-function codes etc. */
    uint8_t ucData[MODBUS_MAX_PDU_BUFFER_LEN - 1];
  } ModbusPDU_t;

  /**
   * @ingroup mqtt_struct_types
   * @brief Modbus packet (request/response) parameters.
   */
  typedef struct ModbusPacketInfo
  {
    ModbusMBAPHeader_t xMBAP; /* Modbus Application Protocol header (MBAP) - 7 Byte */
    ModbusPDU_t xPDU;         /* Modbus Protocol Data Unit (PDU) - up to 253 Byte */
  } ModbusPacketInfo_t;

  /* Modbus state callback function typedefs ----------------------------------*/
  /**
   * @ingroup modbus
   * Function prototype for modbus server state callback. Called when server has
   * changed its state after initialisation and listening or shutdown.
   *
   * @param {server} Modbus server itself
   * @param {arg} Additional *server_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   */
  typedef void (*modbus_server_state_cb_t)(struct ModbusServer *server, void *arg);

  // /**
  //  * @ingroup modbus
  //  * Function prototype for modbus client connection state callback. Called
  //  * whenever the client state changes (connected, disconnected, timeout, ...).
  //  *
  //  * @param {client} Modbus client itself
  //  * @param {arg} Additional *client_arg to pass to the callback function
  //  *        @see ModbusServerConfig_t
  //  */
  // typedef void (*modbus_client_state_cb_t)(struct ModbusClient *client, void *arg);

  // /**
  //  * @ingroup modbus
  //  * Function prototype for modbus client heartbeat callback. Called
  //  * periodically depending on heartbeat timer (MODBUS_CYCLIC_TIMER_INTERVAL).
  //  *
  //  * @param {client} Modbus client which called the heartbeat timer
  //  * @param {arg} Additional *client_arg to pass to the callback function
  //  *        @see ModbusServerConfig_t
  //  */
  // typedef void (*modbus_client_heartbeat_cb_t)(struct ModbusClient *client, void *arg);

  // /**
  //  * @ingroup modbus
  //  * Function prototype for modbus client request callback. Called whenever the
  //  * client receives a modbus request. If response_len > 0 than the response
  //  * could not be sent yet and should be handled by callback function.
  //  *
  //  * @param {client} Modbus client which sent the request
  //  * @param {arg} Additional *client_arg to pass to the callback function
  //  *        @see ModbusServerConfig_t
  //  */
  // typedef void (*modbus_client_request_cb_t)(struct ModbusClient *client, void *arg);

  /* Modbus application callbacks function typedefs ----------------------------*/
  /**
   * @ingroup modbus
   * Function prototype for modbus read bits (coils, discret inputs) callback.
   * This function prototype is valid for requested function codes 0x01 (read
   * coils) and 0x02 (read discrete inputs).
   *
   * @param {tx_buffer} Modbus client transfer buffer where the response must be
   *        writte to (bits state)
   * @param {addr} Requested starting address (0x0000 to 0xFFFF)
   * @param {len} Requested quantity of bits (1 to 2000)
   * @param {arg} Additional *callback_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   *
   * @return {exception} Modbus exception codes @see ModbusException_t
   */
  typedef ModbusException_t (*modbus_read_bits_cb_t)(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);

  /**
   * @ingroup modbus
   * Function prototype for modbus read registers (holdregs, input register)
   * callback. This function prototype is valid for requested function codes 0x03
   * (read holding registers) and 0x04 (read input registers). To ensure correct
   * endianness (big-endian), use the function lwip_htons to convert host to
   * network.
   *
   * @param {tx_buffer} Modbus client transfer buffer where the response must be
   *        writte to (register values)
   * @param {addr} Requested starting address (0x0000 to 0xFFFF)
   * @param {len} Requested quantity of registers (1 to 125)
   * @param {arg} Additional *callback_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   *
   * @return {exception} Modbus exception codes @see ModbusException_t
   */
  typedef ModbusException_t (*modbus_read_registers_cb_t)(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);

  /**
   * @ingroup modbus
   * Function prototype for modbus write bits (coils, discret inputs) callback.
   * This function prototype is valid for requested function codes 0x05 (write
   * single coil) and 0x0F (write multiple coils). The write sequence is given as
   * bit stream for both function codes. Example of a request to write a series of
   * 10 bits starting at address 20:
   *
   * rx_buffer:  | 1  1  0  0  1  1  0  1  | X  X  X  X  X  X  0  1  |
   * bit address:| 27 26 25 24 23 22 21 20 | –  –  –  –  –  –  29 28 |
   *
   * The first byte transmitted (CD hex) addresses outputs 27-20, with the least
   * significant bit addressing the lowest output (20) in this set.
   * The next byte transmitted (01 hex) addresses outputs 29-28, with the least
   * significant bit addressing the lowest output (28) in this set.
   *
   * @param {rx_buffer} Modbus client receive buffer with requested write sequence
   * @param {addr} Requested starting address (0x0000 to 0xFFFF)
   * @param {len} Requested quantity of bits (1 to 2000)
   * @param {arg} Additional *callback_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   *
   * @return {exception} Modbus exception codes @see ModbusException_t
   */
  typedef ModbusException_t (*modbus_write_bits_cb_t)(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);

  /**
   * @ingroup modbus
   * Function prototype for modbus write registers (holdregs, input register)
   * callback. This function prototype is valid for requested function codes 0x06
   * (write single register) and 0x10 (write multiple registers). The write
   * sequence is given as uint16_t values for both function codes. To ensure
   * correct endianness (big-endian), use the function lwip_ntohs to convert
   * network to host. Example of a request to write two registers starting at 2 to
   * 00 0A and 01 02 hex:
   *
   * rx_buffer:        | 0x00 0x0A | 0x01 0x02 |
   * register address: |     2     |     3     |
   *
   * @param {rx_buffer} Modbus client receive buffer with requested write sequence
   * @param {addr} Requested starting address (0x0000 to 0xFFFF)
   * @param {len} Requested quantity of bits (1 to 2000)
   * @param {arg} Additional *callback_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   *
   * @return {exception} Modbus exception codes @see ModbusException_t
   */
  typedef ModbusException_t (*modbus_write_registers_cb_t)(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);

  /**
   * @ingroup modbus
   * Function prototype for modbus default callback. This function prototype is
   * valid for all user defined function codes (65 to 72/100 to 110). Therefore,
   * the tx and rx_buffers are given to perform read/write requests.
   *
   * @param {function_code} Requested user function code (65 to 72/100 to 110)
   * @param {rx_buffer} Modbus client receive buffer with requested write sequence
   * @param {request_len} Requested length in bytes
   * @param {tx_buffer} Modbus client transfer buffer where the response must be
   *        writte to
   * @param {request_len} Response length in bytes
   * @param {arg} Additional *callback_arg to pass to the callback function
   *        @see ModbusServerConfig_t
   *
   * @return {exception} Modbus exception codes @see ModbusException_t
   */
  typedef ModbusException_t (*modbus_default_request_cb_t)(const uint8_t function_code, const uint8_t *rx_buffer, const uint16_t request_len, uint8_t *tx_buffer, uint16_t *response_len, void *arg);

  /* Server configuration ----------------------------------------------------*/
  /**
   * @ingroup modbus
   * Server information and connection parameters, must be set before calling @modbus_server_init
   */
  typedef struct ModbusServerConfig
  {
    // /** Server address information */
    // // const ip_addr_t *ip_addr;
    // uint16_t port;

    // /** Timeout/watchdog time in seconds, 0 to disable watchdog timer for clients */
    // uint16_t timeout;

    // /** Heartbeat, seconds between each cyclic timer call */
    // uint16_t heartbeat;

    /** Server identifier */
    uint8_t server_id;
    const char *server_name;

    // /** Server state change callback */
    // void *status_arg;
    // modbus_server_state_cb_t server_state_cb;

    // /** Clients connect/heartbeat/request callback */
    // void *client_arg;
    // modbus_client_state_cb_t client_connect_cb;
    // modbus_client_heartbeat_cb_t client_heartbeat_cb;
    // modbus_client_request_cb_t client_request_cb;

    /** Application callbacks for read/write */
    void *callback_arg;
    // modbus_read_bits_cb_t read_bits_0x01_cb;
    // modbus_read_bits_cb_t read_input_bits_0x02_cb;
    // modbus_read_registers_cb_t read_registers_0x03_cb;
    // modbus_read_registers_cb_t read_input_registers_0x04_cb;
    // modbus_write_bits_cb_t write_bit_0x05_cb;
    // modbus_write_bits_cb_t write_bits_0x0F_cb;
    // modbus_write_registers_cb_t write_register_0x06_cb;
    // modbus_write_registers_cb_t write_registers_0x10_cb;
    // modbus_default_request_cb_t default_request_cb;
  } ModbusServerConfig_t;

  /** Modbus server */
  typedef struct ModbusServer
  {
    uint8_t server_id;
    const char *server_name;

    // /** Timers and timeouts */
    // uint16_t timeout;
    // uint16_t heartbeat;

    /** Connection state */
    // ModbusStatus_t state;
    // struct altcp_pcb *pcb;

    // /** Server state callback */
    // void *status_arg;
    // modbus_server_state_cb_t state_cb;

    // /** Client connection callback */
    // void *client_arg;
    // modbus_client_state_cb_t client_connect_cb;
    // modbus_client_heartbeat_cb_t client_heartbeat_cb;
    // modbus_client_request_cb_t client_request_cb;

    /** Application callbacks */
    void *callback_arg;
    // modbus_read_bits_cb_t read_bits_0x01_cb;
    // // xModbusReadCoilsHook_0x01
    // modbus_read_bits_cb_t read_input_bits_0x02_cb;
    // // xModbusReadInputBitsHook_0x02
    // modbus_read_registers_cb_t read_registers_0x03_cb;
    // // xModbusReadHoldregsHook_0x03
    // modbus_read_registers_cb_t read_input_registers_0x04_cb;
    // // xModbusReadInputRegistersHook_0x04
    // modbus_write_bits_cb_t write_bit_0x05_cb;
    // // xModbusWriteBitHook_0x05
    // modbus_write_bits_cb_t write_bits_0x0F_cb;
    // // xModbusWriteBitsHook_0x0F
    // modbus_write_registers_cb_t write_register_0x06_cb;
    // // xModbusWriteHoldregHook_0x06
    // modbus_write_registers_cb_t write_registers_0x10_cb;
    // // xModbusWriteHoldregsHook_0x10
    // modbus_default_request_cb_t default_request_cb;
    // // xModbusUserFunctionHook_0xXX

    /** Client list */
    uint16_t client_len; /* Number of known clients */
    // ModbusClient_t *client_list[MODBUS_NUM_CLIENTS]; /* Known clients list */
  } ModbusServer_t;

  /* Client configuration ----------------------------------------------------*/
  // /** Modbus client */
  // typedef struct ModbusClient
  // {
  //   ModbusServer_t *server;

  //   /** Timers and timeouts */
  //   uint16_t timeout;
  //   uint16_t watchdog;
  //   uint16_t cyclic_tick;

  //   /** Connection state */
  //   // ip_addr_t ip;
  //   uint16_t port;
  //   ModbusStatus_t state;
  //   // struct altcp_pcb *conn;

  //   /** Counters and diagnostic */
  //   uint8_t retries;
  //   uint16_t diagnostic;
  //   struct ModbusCounter counter;

  //   /** Client callbacks */
  //   void *client_arg;
  //   modbus_client_state_cb_t connect_cb;
  //   modbus_client_heartbeat_cb_t heartbeat_cb;
  //   modbus_client_request_cb_t request_cb;

  //   /* Modbus Application Protocol header */
  //   struct ModbusMBAPHeader mbap;

  //   /** Output data */
  //   struct ModbusPDU response;
  //   uint16_t response_len;

  //   /** Input data */
  //   struct ModbusPDU request;
  //   uint16_t request_len;

  // } ModbusClient_t;

  /* Exported functions ------------------------------------------------------*/

  ModbusException_t xModbusReadCoilsHook_0x01(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusReadInputBitsHook_0x02(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusReadHoldregsHook_0x03(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusReadInputRegistersHook_0x04(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusWriteBitHook_0x05(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusWriteHoldregHook_0x06(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusWriteBitsHook_0x0F(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusWriteHoldregsHook_0x10(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
  ModbusException_t xModbusUserFunctionHook_0xXX(const uint8_t ucFunctionCode, const uint8_t *rx_buffer, const uint16_t usRequestLength, uint8_t *tx_buffer, uint16_t *response_len, void *arg);

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */

#endif /* ifndef CORE_MODBUS_H */
