/**
 * @file modbus.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <string.h>
#include <math.h>

/* MODBUS library includes. */
#include "core_modbus.h"

ModbusException_t xModbusReadCoilsHook_0x01(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
ModbusException_t xModbusWriteBitHook_0x05(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
ModbusException_t xModbusReadHoldregsHook_0x03(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);
ModbusException_t xModbusWriteHoldregHook_0x06(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg);
ModbusException_t xModbusUserFunctionHook_0xXX(const uint8_t ucFunctionCode, const uint8_t *rx_buffer, const uint16_t usRequestLength, uint8_t *tx_buffer, uint16_t *response_len, void *arg);
ModbusException_t xModbusReadInputRegistersHook_0x04(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg);

/* Private user code ---------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x01 (read coils) and 0x02
 * (read discrete inputs).
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        written to (bits state)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of bits (1 to 2000)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusReadCoilsHook_0x01(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  return xModbusReadInputBitsHook_0x02(tx_buffer, addr, len, arg);
}
ModbusException_t
xModbusReadInputBitsHook_0x02(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogInfo(("addr %d (0x%04X) / len %d",
           addr, addr, len));
  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t address = addr + i;
    // switch (addr)
    // {
    tx_buffer[i] = (addr % 2);
    // }
  }

  /* All done, return without exception */
  return MODBUS_EXC_NO_EXCEPTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x03 (read holding registers).
 * To ensure correct endianness (big-endian), use the function modbusHTONS/
 * modbusHTONL to convert host to network.
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to (register values)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of registers (1 to 125)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusReadHoldregsHook_0x03(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogInfo(("addr %d (0x%04X) / len %d",
           addr, addr, len));

  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t address = addr + i;
    // switch (address)
    // {
    tx_buffer[i] = modbusHTONS(address);
    // }
  }

  /* All done, return without exception */
  return MODBUS_EXC_NO_EXCEPTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x04 (read input registers).
 * To ensure correct endianness (big-endian), use the function modbusHTONS/
 * modbusHTONL to convert host to network.
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to (register values)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of registers (1 to 125)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusReadInputRegistersHook_0x04(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogInfo(("addr %d (0x%04X) / len %d)",
           addr, addr, len));

  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t address = addr + i;
    // switch (address)
    // {
    tx_buffer[i] = modbusHTONS(address);
    // }
  }

  /* All done, return without exception */
  return MODBUS_EXC_NO_EXCEPTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x05 (write single coil) and
 * 0x0F (write multiple coils). The write sequence is given as bit stream for
 * both function codes. Example of a request to write a series of 10 bits
 * starting at address 20:
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
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusWriteBitHook_0x05(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  return xModbusWriteBitsHook_0x0F(rx_buffer, addr, len, arg);
}
ModbusException_t
xModbusWriteBitsHook_0x0F(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogInfo(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X ...",
           addr, addr, len, rx_buffer[0]));

  for (uint16_t i = 0; i < len; i++)
  {
    int value = rx_buffer[(uint16_t)floorf((float)i / 8.0F)] & (0b1 << (i % 8));
    uint16_t address = addr + i;

    // switch (address)
    // {
    return MODBUS_EXC_SLAVE_DEVICE_FAILURE;
    // }
  }

  /* All done, return without exception */
  return MODBUS_EXC_NO_EXCEPTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x06 (write single register)
 * and 0x10 (write multiple registers). The write sequence is given as uint16_t
 * values for both function codes. To ensure correct endianness (big-endian),
 * use the function lwip_ntohs to convert network to host. Example of a request
 * to write two registers starting at 2 to 00 0A and 01 02 hex:
 *
 * rx_buffer:        | 0x00 0x0A | 0x01 0x02 |
 * register address: |     2     |     3     |
 *
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of bits (1 to 2000)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusWriteHoldregHook_0x06(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  return xModbusWriteHoldregsHook_0x10(rx_buffer, addr, len, arg);
}
ModbusException_t
xModbusWriteHoldregsHook_0x10(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogInfo(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X 0x%04X ...",
           addr, addr, len, rx_buffer[0], rx_buffer[1]));

  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t value = modbusNTOHS(rx_buffer[i]);
    // uint32_t value = modbusNTOHL(rx_buffer[i]); i++;
    uint16_t address = addr + i;

    // switch (address)
    // {
    return MODBUS_EXC_SLAVE_DEVICE_FAILURE;
    // }
  }

  /* All done, return without exception */
  return MODBUS_EXC_NO_EXCEPTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for all user defined function codes (65 to 72/100 to 110).
 * Therefore, the tx and rx_buffers are given to perform read/write requests.
 *
 * @param {ucFunctionCode} Requested user function code (65 to 72/100 to 110)
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {request_len} Requested length in bytes
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to
 * @param {request_len} Response length in bytes
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
ModbusException_t
xModbusUserFunctionHook_0xXX(const uint8_t ucFunctionCode, const uint8_t *rx_buffer, const uint16_t usRequestLength, uint8_t *tx_buffer, uint16_t *response_len, void *arg)
{
  /* usRequestLength includes function code (1 byte) */
  LogInfo(("function code 0x%02X / usRequestLength %d",
           ucFunctionCode, usRequestLength));

  switch (ucFunctionCode)
  {
    // case 65:
    //   /* All done, return without exception */
    //   return MODBUS_EXC_NO_EXCEPTION;
    //   break;

  default:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
  }
}
/*-----------------------------------------------------------*/

/***************************** END OF FILE ************************************/
