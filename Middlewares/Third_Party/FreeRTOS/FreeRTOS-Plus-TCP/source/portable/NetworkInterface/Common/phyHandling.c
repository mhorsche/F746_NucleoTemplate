/*
 * FreeRTOS+TCP <DEVELOPMENT BRANCH>
 * Copyright (C) 2022 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @brief
 * Handling of Ethernet PHY's
 * PHY's communicate with an EMAC either through
 * a Media-Independent Interface (MII), or a Reduced Media-Independent Interface (RMII).
 * The EMAC can poll for PHY ports on 32 different addresses. Each of the PHY ports
 * shall be treated independently.
 *
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "phyHandling.h"

#define phyMIN_PHY_ADDRESS 0
#define phyMAX_PHY_ADDRESS 31

#if defined(PHY_LS_HIGH_CHECK_TIME_MS) || defined(PHY_LS_LOW_CHECK_TIME_MS)
#error please use the new defines with 'ipconfig' prefix
#endif

#ifndef ipconfigPHY_LS_HIGH_CHECK_TIME_MS

/* Check if the LinkStatus in the PHY is still high after 15 seconds of not
 * receiving packets. */
#define ipconfigPHY_LS_HIGH_CHECK_TIME_MS 15000U
#endif

#ifndef ipconfigPHY_LS_LOW_CHECK_TIME_MS
/* Check if the LinkStatus in the PHY is still low every second. */
#define ipconfigPHY_LS_LOW_CHECK_TIME_MS 1000U
#endif

/* As the following 3 macro's are OK in most situations, and so they're not
 * included in 'FreeRTOSIPConfigDefaults.h'.
 * Users can change their values in the project's 'FreeRTOSIPConfig.h'. */
#ifndef phyPHY_MAX_RESET_TIME_MS
#define phyPHY_MAX_RESET_TIME_MS 1000U
#endif

#ifndef phyPHY_MAX_NEGOTIATE_TIME_MS
#define phyPHY_MAX_NEGOTIATE_TIME_MS 3000U
#endif

#ifndef phySHORT_DELAY_MS
#define phySHORT_DELAY_MS 50U
#endif

/* Naming and numbering of basic PHY registers. */
#define phyREG_00_BMCR 0x00U      /* Basic Mode Control Register. */
#define phyREG_01_BMSR 0x01U      /* Basic Mode Status Register. */
#define phyREG_02_PHYSID1 0x02U   /* PHYS ID 1 */
#define phyREG_03_PHYSID2 0x03U   /* PHYS ID 2 */
#define phyREG_04_ADVERTISE 0x04U /* Advertisement control reg */

/* Naming and numbering of extended PHY registers. */
#define phyREG_05_ANLPAR 0x05U  /* Auto Negotiation Link Partner Ability Register */
#define phyREG_06_ANER 0x06U    /* Auto Negotiation Expansion Register */
#define phyREG_07_ANNPTXR 0x07U /* Auto Negotiation Next Page TX Register */
#define phyREG_08_ANNPRXR 0x08U /* Auto Negotiation Next Page RX Register */
#define phyREG_0D_MMDCTR 0x0DU  /* MMD Access Control Register */
#define phyREG_0E_MMDADR 0x0EU  /* MMD Access Address/Data Register */
#define phyREG_10_PHYSTS 0x10U  /* PHY status register Offset */
#define phyREG_11_MODER 0x11U   /* Mode Control/Status Register */
#define phyREG_12_SMODER 0x12U  /* Special Modes Register */
#define phyREG_18_TDRPAT 0x18U  /* TDR Patterns/Delay Control Register */
#define phyREG_19_PHYCR 0x19U   /* TDR Control/Status Register */
#define phyREG_1A_SYMERR 0x1AU  /* Symbol Error Counter Register */
#define phyREG_1B_SCTRL 0x1BU   /* Special Control/Status Indications Register */
#define phyREG_1C_CLENGTH 0x1CU /* Cable Length Register */
#define phyREG_1D_ISFR 0x1DU    /* Interrupt Source Flag Register */
#define phyREG_1E_IMR 0x1EU     /* Interrupt Mask Register */
#define phyREG_1F_PHYSPCS 0x1FU /* PHY Special Control/Status Register */

/* MDIO Manageable Device (MMD) Registers ------------------------------------*/
#define phyMMD_PCS ((uint16_t)3)                  /* PCS */
#define phyMMD_PCSDP1R ((uint16_t)5)              /* PCS MMD Devices Present 1 Register */
#define phyMMD_PCSDP2R ((uint16_t)6)              /* PCS MMD Devices Present 2 Register */
#define phyMMD_WUCSR ((uint16_t)32784)            /* Wakeup Control and Status Register */
#define phyMMD_WUF_CFGA ((uint16_t)32785)         /* Wakeup Filter Configuration Register A */
#define phyMMD_WUF_CFGB ((uint16_t)32786)         /* Wakeup Filter Configuration Register B */
#define phyMMD_WUF_MASK_127_112 ((uint16_t)32801) /* Wakeup Filter Byte Mask Registers [127:112] */
#define phyMMD_WUF_MASK_111_96 ((uint16_t)32802)  /* Wakeup Filter Byte Mask Registers [111:96] */
#define phyMMD_WUF_MASK_95_80 ((uint16_t)32803)   /* Wakeup Filter Byte Mask Registers [95:80] */
#define phyMMD_WUF_MASK_79_64 ((uint16_t)32804)   /* Wakeup Filter Byte Mask Registers [79:64] */
#define phyMMD_WUF_MASK_63_48 ((uint16_t)32805)   /* Wakeup Filter Byte Mask Registers [63:48] */
#define phyMMD_WUF_MASK_47_32 ((uint16_t)32806)   /* Wakeup Filter Byte Mask Registers [47:32] */
#define phyMMD_WUF_MASK_31_16 ((uint16_t)32807)   /* Wakeup Filter Byte Mask Registers [31:16] */
#define phyMMD_WUF_MASK_15_0 ((uint16_t)32808)    /* Wakeup Filter Byte Mask Registers [15:0] */
#define phyMMD_RX_ADDRA ((uint16_t)32865)         /* MAC Receive Address A Register */
#define phyMMD_RX_ADDRB ((uint16_t)32866)         /* MAC Receive Address B Register */
#define phyMMD_RX_ADDRC ((uint16_t)32867)         /* MAC Receive Address C Register */
#define phyMMD_MCFGR ((uint16_t)32868)            /* Miscellaneous Configuration Register */

/* Bit fields for 'phyREG_00_BMCR', the 'Basic Mode Control Register'. */
#define phyBMCR_FULL_DUPLEX 0x0100U /* Full duplex. */
#define phyBMCR_AN_RESTART 0x0200U  /* Auto negotiation restart. */
#define phyBMCR_ISOLATE 0x0400U     /* 1 = Isolates 0 = Normal operation. */
#define phyBMCR_POWERDOWN 0x0800U   /* Select the power down mode */
#define phyBMCR_AN_ENABLE 0x1000U   /* Enable auto negotiation. */
#define phyBMCR_SPEED_100 0x2000U   /* Select 100Mbps. */
#define phyBMCR_LOOPBACK 0x4000U    /* Select loop-back mode */
#define phyBMCR_RESET 0x8000U       /* Reset the PHY. */

/* Bit fields for 'phyREG_01_BMSR', the 'Basic Mode Status Register'. */
#define phyBMSR_EXT_CAP ((uint16_t)0x0001U)          /* Extended Capabilities */
#define phyBMSR_JABBER_DETECTION ((uint16_t)0x0002U) /* Jabber condition detected */
#define phyBMSR_LINK_STATUS ((uint16_t)0x0004U)      /* Link Status */
#define phyBMSR_AN_ABILITY ((uint16_t)0x0008U)       /* Able to perform Auto-Negotiation function */
#define phyBMSR_REMOTE_FAULT ((uint16_t)0x0010U)     /* Remote fault condition detected */
#define phyBMSR_AN_COMPLETE ((uint16_t)0x0020U)      /* Auto-Negotiation process completed */
#define phyBMSR_EXT_STATUS ((uint16_t)0x0100U)       /* Extended status information in register 15 */
#define phyBMSR_HALFDUPLEX_100M ((uint16_t)0x0200U)  /* Able to perform half-duplex mode at 100 Mb/s */
#define phyBMSR_FULLDUPLEX_100M ((uint16_t)0x0400U)  /* Able to perform full-duplex mode at 100 Mb/s */
#define phyBMSR_HALFDUPLEX_10M ((uint16_t)0x0800U)   /* Able to perform half-duplex mode at 10 Mb/s */
#define phyBMSR_FULLDUPLEX_10M ((uint16_t)0x1000U)   /* Able to perform full-duplex mode at 10 Mb/s */
#define phyBMSR_HALFDUPLEX_TX ((uint16_t)0x2000U)    /* 100BASE-TX with half-duplex */
#define phyBMSR_FULLDUPLEX_TX ((uint16_t)0x4000U)    /* 100BASE-TX with full-duplex */
#define phyBMSR_BASE_T4 ((uint16_t)0x8000U)          /* Able for 100BASE-T4 mode */

/* Bit fields for 'phyREG_19_PHYCR', the 'PHY Control Register'. */
#define phyPHYCR_MDIX_EN 0x8000U    /* Enable Auto MDIX. */
#define phyPHYCR_MDIX_FORCE 0x4000U /* Force MDIX crossed. */

#define phyPHYSTS_LINK_STATUS 0x0001U   /* PHY Link mask */
#define phyPHYSTS_SPEED_STATUS 0x0002U  /* PHY Speed mask */
#define phyPHYSTS_DUPLEX_STATUS 0x0004U /* PHY Duplex mask */

/* Bit fields for 'phyREG_1F_PHYSPCS
 *   001 = 10BASE-T half-duplex
 *   101 = 10BASE-T full-duplex
 *   010 = 100BASE-TX half-duplex
 *   110 = 100BASE-TX full-duplex
 */
#define phyPHYSPCS_SPEED_MASK 0x000CU
#define phyPHYSPCS_SPEED_10 0x0004U
#define phyPHYSPCS_FULL_DUPLEX 0x0010U

/* Description of all capabilities that can be advertised to
 * the peer (usually a switch or router).
 */
#define phyADVERTISE_CSMA 0x0001U    /* Supports IEEE 802.3u: Fast Ethernet at 100 Mbit/s */
#define phyADVERTISE_10HALF 0x0020U  /* Try for 10mbps half-duplex. */
#define phyADVERTISE_10FULL 0x0040U  /* Try for 10mbps full-duplex. */
#define phyADVERTISE_100HALF 0x0080U /* Try for 100mbps half-duplex. */
#define phyADVERTISE_100FULL 0x0100U /* Try for 100mbps full-duplex. */

#define phyADVERTISE_ALL                         \
  (phyADVERTISE_10HALF | phyADVERTISE_10FULL |   \
   phyADVERTISE_100HALF | phyADVERTISE_100FULL | \
   phyADVERTISE_CSMA)

/* Interrupt Source Flag Register 29 (ISFR) ----------------------------------*/
#define phyISFR_INT8 ((uint16_t)0x0100U)
#define phyISFR_WOL phyISFR_INT8 /* Wake on LAN (WoL) event detected */

#define phyISFR_INT7 ((uint16_t)0x0080U)
#define phyISFR_ENERGYON phyISFR_INT7 /* ENERGYON generated */

#define phyISFR_INT6 ((uint16_t)0x0040U)
#define phyISFR_ANC phyISFR_INT6 /* Auto-Negotiation complete */

#define phyISFR_INT5 ((uint16_t)0x0020U)
#define phyISFR_RFD phyISFR_INT5 /* Remote Fault Detected */

#define phyISFR_INT4 ((uint16_t)0x0010U)
#define phyISFR_LD phyISFR_INT4 /* Link Down (link status negated) */

#define phyISFR_INT3 ((uint16_t)0x0008U)
#define phyISFR_ANLPA phyISFR_INT3 /* Auto-Negotiation LP Acknowledge */

#define phyISFR_INT2 ((uint16_t)0x0004U)
#define phyISFR_PDF phyISFR_INT2 /* Parallel Detection Fault */

#define phyISFR_INT1 ((uint16_t)0x0002U)
#define phyISFR_ANPR phyISFR_INT1 /* Auto-Negotiation Page Received */

/* Wakeup control and status register 3.32784 (WUCSR) ------------------------*/
#define phyWUCSR_CLEAR_MASK ((uint16_t)0b0111101100001111) // 0x7B0F = 0b0111 1011 0000 1111
/*  BITS DESCRIPTION                                                                      TYPE   DEFAULT
 *    15 Interface Disable                                                                 R/W        0b
 *       0 = RMII interface enabled                                                       NASR
 *       1 = RMII interface disabled. Outputs driven to a low level and inputs ignored. */
#define phyWUCSR_RMII_ENL ((uint16_t)0x8000U)

/* 14:13 LED1 Function Select                                                              R/W        0b
 *       00 = LED1 functions as Link/Activity.                                            NASR
 *       01 = LED1 functions as nINT.
 *       10 = LED1 functions as nPME.
 *       11 = LED1 functions as Link Speed. */
#define phyWUCSR_LED1 ((uint16_t)0x6000U)

/* 12:11 LED2 Function Select                                                              R/W        0b
 *       00 = LED2 functions as Link Speed.                                               NASR
 *       01 = LED2 functions as nINT.
 *       10 = LED2 functions as nPME.
 *       11 = LED2 functions as Link/Activity. */
#define phyWUCSR_LED2 ((uint16_t)0x1800U)

/*    10 RESERVED                                                                           RO         - */

/*     9 nPME Self Clear                                                                   R/W        0b
 *       0 = nPME pin is not self clearing.                                               NASR
 *       1 = nPME pin is self clearing. */
#define phyWUCSR_nPME ((uint16_t)0x0200U)

/*     8 WoL Configured                                                                    R/W        0b
 *       This bit may be set by software after the WoL registers are configured. This     NASR
 *       sticky bit (and all other WoL related register bits) is reset only via a power
 *       cycle or a pin reset, allowing software to skip programming of the WoL registers
 *       in response to a WoL event. */
#define phyWUCSR_WoL ((uint16_t)0x0100U)

/*     7 Perfect DA Frame Received (PFDA_FR)                                              R/WC        0b
 *       The MAC sets this bit upon receiving a valid frame with a destination address    NASR
 *       that matches the physical address. */
#define phyWUCSR_PFDA_FR ((uint16_t)0x0080U)

/*     6 that matches the physical address.                                               R/WC        0b
 *       The MAC sets this bit upon receiving a valid remote Wakeup Frame.                NASR */
#define phyWUCSR_WUFR ((uint16_t)0x0040U)

/*     5 Magic Packet Received (MPR)                                                      R/WC        0b
 *       The MAC sets this bit upon receiving a valid Magic Packet.                       NASR */
#define phyWUCSR_MPR ((uint16_t)0x0020U)

/*     4 Broadcast Frame Received (BCAST_FR)                                              R/WC        0b
 *       The MAC Sets this bit upon receiving a valid broadcast frame.                    NASR */
#define phyWUCSR_BCAST_FR ((uint16_t)0x0010U)

/*     3 Perfect DA Wakeup Enable (PFDA_EN)                                                R/W        0b
 *       When set, remote wakeup mode is enabled and the MAC is capable of waking         NASR
 *       up on receipt of a frame with a destination address that matches the physical
 *       address of the device. The physical address is stored in the MAC Receive
 *       Address A Register (RX_ADDRA), MAC Receive Address B Register
 *       (RX_ADDRB) and MAC Receive Address C Register (RX_ADDRC). */
#define phyWUCSR_PFDA_EN ((uint16_t)0x0008U)

/*     2 Wakeup Frame Enable (WUEN)                                                        R/W        0b
 *       When set, remote wakeup mode is enabled and the MAC is capable of detecting      NASR
 *       Wakeup Frames as programmed in the Wakeup Filter. */
#define phyWUCSR_WUEN ((uint16_t)0x0004U)

/*     1 Magic Packet Enable (MPEN)                                                        R/W        0b
 *       When set, Magic Packet wakeup mode is enabled.                                   NASR */
#define phyWUCSR_MPEN ((uint16_t)0x0002U)

/*     0 Broadcast Wakeup Enable (BCST_EN)                                                 R/W        0b
 *       When set, remote wakeup mode is enabled and the MAC is capable of waking         NASR
 *       up from a broadcast frame. */
#define phyWUCSR_BCST_EN ((uint16_t)0x0001U)

/* Read/write to MDIO Manageable Device (MMD) Register. */
static BaseType_t xPhyMMDRegister(EthernetPhy_t *pxPhyObject, BaseType_t xPhyAddress,
                                  BaseType_t xRegister, uint32_t *pulValue, BaseType_t xReadWrite);
#define xPhyReadMMDRegister(pxPhyObject, xPhyAddress, xRegister, pulValue) xPhyMMDRegister(pxPhyObject, xPhyAddress, xRegister, pulValue, 0)
#define xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, xRegister, pulValue) xPhyMMDRegister(pxPhyObject, xPhyAddress, xRegister, pulValue, 1)

/* Send a reset command to a set of PHY-ports. */
static uint32_t xPhyReset(EthernetPhy_t *pxPhyObject,
                          uint32_t ulPhyMask);
/*-----------------------------------------------------------*/

static const char *pcPhyName(uint32_t ulPhyID)
{
  switch (ulPhyID)
  {
  case PHY_ID_LAN8720:
    return "LAN8720";
  case PHY_ID_LAN8742A:
    return "LAN8742A";
  // case PHY_ID_KSZ8051: // same ID as PHY_ID_KSZ8041
  // case PHY_ID_KSZ8081: // same ID as PHY_ID_KSZ8041
  case PHY_ID_KSZ8041:
    return "KSZ8041";
  case PHY_ID_KSZ8081MNXIA:
    return "KSZ8081MNXIA";
  case PHY_ID_KSZ8863:
    return "KSZ8863";
  case PHY_ID_DP83848I:
    return "DP83848I";
  default:
    return "unknown";
  }
}
/*-----------------------------------------------------------*/

static BaseType_t xHas_1F_PHYSPCS(uint32_t ulPhyID)
{
  BaseType_t xResult = pdFALSE;

  switch (ulPhyID)
  {
  case PHY_ID_LAN8720:
  case PHY_ID_LAN8742A:
  case PHY_ID_KSZ8041:

  /*
   *      case PHY_ID_KSZ8051: // same ID as 8041
   *      case PHY_ID_KSZ8081: // same ID as 8041
   */
  case PHY_ID_KSZ8081MNXIA:

  case PHY_ID_KSZ8863:
  default:
    /* Most PHY's have a 1F_PHYSPCS */
    xResult = pdTRUE;
    break;

  case PHY_ID_DP83848I:
  case PHY_ID_DP83TC811S:
  case PHY_ID_TM4C129X:
  case PHY_ID_MV88E6071:
    /* Has no 0x1F register "PHY Special Control Status". */
    break;
  }

  return xResult;
}
/*-----------------------------------------------------------*/

static BaseType_t xHas_19_PHYCR(uint32_t ulPhyID)
{
  BaseType_t xResult = pdFALSE;

  switch (ulPhyID)
  {
  case PHY_ID_LAN8742A:
  case PHY_ID_DP83848I:
  case PHY_ID_TM4C129X:
    xResult = pdTRUE;
    break;

  case PHY_ID_MV88E6071: /* Marvell 88E6071 */
  default:
    /* Most PHY's do not have a 19_PHYCR */
    break;
  }

  return xResult;
}
/*-----------------------------------------------------------*/

static BaseType_t xHas_0E_MMDADR(uint32_t ulPhyID)
{
  BaseType_t xResult;

  switch (ulPhyID)
  {

    // case PHY_ID_KSZ8041:
    // case PHY_ID_KSZ8051: // same ID as 8041
    // case PHY_ID_KSZ8081: // same ID as 8041

    // case PHY_ID_KSZ8081MNXIA:

    // case PHY_ID_KSZ8863:
    // case PHY_ID_DP83848I:

  case PHY_ID_LAN8720:
  case PHY_ID_LAN8742A:

    xResult = pdTRUE;
    break;

  default:
    /* Most PHY's  do not have a 0E_MMDADR */
    xResult = pdFALSE;
    break;
  }

  return xResult;
}
/*-----------------------------------------------------------*/

static BaseType_t xHas_1D_ISFR(uint32_t ulPhyID)
{
  BaseType_t xResult;

  switch (ulPhyID)
  {

    // case PHY_ID_KSZ8041:
    // case PHY_ID_KSZ8051: // same ID as 8041
    // case PHY_ID_KSZ8081: // same ID as 8041

    // case PHY_ID_KSZ8081MNXIA:

    // case PHY_ID_KSZ8863:
    // case PHY_ID_DP83848I:

  case PHY_ID_LAN8720:
  case PHY_ID_LAN8742A:

    xResult = pdTRUE;
    break;

  default:
    /* Most PHY's  do not have a 1D_ISFR */
    xResult = pdFALSE;
    break;
  }

  return xResult;
}
/*-----------------------------------------------------------*/

/**
 * @brief  Read/write MDIO Manageable Device (MMD) Register.
 *
 * @param  lwip: Handle to lwip_t structure that contains all configuration
 *                 settings.
 * @param  address: MMD Register address
 * @param  value: Value pointer to read/write
 * @param  read_write: 1 for writing, 0 for reading
 * @return None
 */
static BaseType_t xPhyMMDRegister(EthernetPhy_t *pxPhyObject, BaseType_t xPhyAddress, BaseType_t xRegister, uint32_t *pulValue, BaseType_t xReadWrite)
{
  BaseType_t xResult;

  /* Set MMD access control register, [15:14] read function (0b00 = Address), [4:0] MMD device address (3 = PCS) */
  if (pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_0D_MMDCTR, (0x0000 | phyMMD_PCS)) != 0)
  {
    FreeRTOS_printf(("xPhyMMDRegister: Writing MMD access control failed\n"));
    xResult = -1;
  }
  else
  {
    /* Set MMD access address, [15:0] MMD Register Address */
    if (pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_0E_MMDADR, (uint32_t)xRegister) != 0)
    {
      FreeRTOS_printf(("xPhyMMDRegister: Writing MMD access address failed\n"));
      xResult = -1;
    }
    else
    {
      /* Set MMD access control register, [15:14] read function (0b01 = Data, no post increment), [4:0] MMD device address (3 = PCS) */
      if (pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_0D_MMDCTR, (0x4000 | phyMMD_PCS)) != 0)
      {
        FreeRTOS_printf(("xPhyMMDRegister: Writing MMD access address failed\n"));
        xResult = -1;
      }
      else
      {
        if (xReadWrite == 0)
        {
          /* Read MMD access address, [15:0] MMD Register Data */
          if (pxPhyObject->fnPhyRead(xPhyAddress, phyREG_0E_MMDADR, pulValue) != 0)
          {
            FreeRTOS_printf(("xPhyReadMMDRegister 3.%d failed\n", xRegister));
            xResult = -1;
          }
          else
          {
            FreeRTOS_debug_printf(("xPhyReadMMDRegister 3.%d: 0x%04X\n", xRegister, *pulValue));
            xResult = 0;
          }
        }
        else
        {
          /* Write MMD access data, [15:0] MMD Register Data */
          if (pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_0E_MMDADR, *pulValue) != 0)
          {
            FreeRTOS_printf(("xPhyWriteMMDRegister 3.%d failed\n", xRegister));
            xResult = -1;
          }
          else
          {
            FreeRTOS_debug_printf(("xPhyWriteMMDRegister 3.%d: 0x%04X\n", xRegister, *pulValue));
            xResult = 0;
          }
        }
      }
    }
  }

  return xResult;
}
/*-----------------------------------------------------------*/

/* Initialise the struct and assign a PHY-read and -write function. */
void vPhyInitialise(EthernetPhy_t *pxPhyObject,
                    xApplicationPhyReadHook_t fnPhyRead,
                    xApplicationPhyWriteHook_t fnPhyWrite)
{
  memset((void *)pxPhyObject, 0, sizeof(*pxPhyObject));

  pxPhyObject->fnPhyRead = fnPhyRead;
  pxPhyObject->fnPhyWrite = fnPhyWrite;
}
/*-----------------------------------------------------------*/

/* Discover all PHY's connected by polling 32 indexes ( zero-based ) */
BaseType_t xPhyDiscover(EthernetPhy_t *pxPhyObject)
{
  BaseType_t xPhyAddress;

  pxPhyObject->xPortCount = 0;

  for (xPhyAddress = phyMIN_PHY_ADDRESS; xPhyAddress <= phyMAX_PHY_ADDRESS; xPhyAddress++)
  {
    uint32_t ulLowerID = 0U;

    pxPhyObject->fnPhyRead(xPhyAddress, phyREG_03_PHYSID2, &ulLowerID);

    /* A valid PHY id can not be all zeros or all ones. */
    if ((ulLowerID != (uint16_t)~0U) && (ulLowerID != (uint16_t)0U))
    {
      uint32_t ulUpperID;
      uint32_t ulPhyID;

      pxPhyObject->fnPhyRead(xPhyAddress, phyREG_02_PHYSID1, &ulUpperID);
      ulPhyID = (((uint32_t)ulUpperID) << 16) | (ulLowerID & 0xFFF0U);

      pxPhyObject->ucPhyIndexes[pxPhyObject->xPortCount] = xPhyAddress;
      pxPhyObject->ulPhyIDs[pxPhyObject->xPortCount] = ulPhyID;

      pxPhyObject->xPortCount++;
      FreeRTOS_printf(("xPhyDiscover: found new PHY address 0x%lX (ID 0x%lX / %s)\n",
                       xPhyAddress, ulPhyID, pcPhyName(pxPhyObject->ulPhyIDs[0])));

      /* See if there is more storage space. */
      if (pxPhyObject->xPortCount == ipconfigPHY_MAX_PORTS)
      {
        break;
      }
    }
  }

  return pxPhyObject->xPortCount;
}
/*-----------------------------------------------------------*/

/* Send a reset command to a set of PHY-ports. */
static uint32_t xPhyReset(EthernetPhy_t *pxPhyObject,
                          uint32_t ulPhyMask)
{
  uint32_t ulDoneMask, ulConfig;
  TickType_t xRemainingTime;
  TimeOut_t xTimer;
  BaseType_t xPhyIndex;

  /* A bit-mask of PHY ports that are ready. */
  ulDoneMask = 0U;

  /* Set the RESET bits high. */
  for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

    /* Read Control register. */
    pxPhyObject->fnPhyRead(xPhyAddress, phyREG_00_BMCR, &ulConfig);
    pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_00_BMCR, ulConfig | phyBMCR_RESET);
  }

  xRemainingTime = (TickType_t)pdMS_TO_TICKS(phyPHY_MAX_RESET_TIME_MS);
  vTaskSetTimeOutState(&xTimer);

  /* The reset should last less than a second. */
  for (;;)
  {
    for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

      pxPhyObject->fnPhyRead(xPhyAddress, phyREG_00_BMCR, &ulConfig);

      if ((ulConfig & phyBMCR_RESET) == 0)
      {
        FreeRTOS_printf(("xPhyReset: PHY address 0x%02lX ready\n", (int)xPhyAddress));
        ulDoneMask |= (1U << xPhyIndex);
      }
    }

    if (ulDoneMask == ulPhyMask)
    {
      break;
    }

    if (xTaskCheckForTimeOut(&xTimer, &xRemainingTime) != pdFALSE)
    {
      FreeRTOS_printf(("xPhyReset: phyBMCR_RESET timed out (done 0x%02lX)\n", (unsigned int)ulDoneMask));
      break;
    }

    /* Block for a while */
    vTaskDelay(pdMS_TO_TICKS(phySHORT_DELAY_MS));
  }

  /* Clear the reset bits. */
  for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    if ((ulDoneMask & (1U << xPhyIndex)) == 0U)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

      /* The reset operation timed out, clear the bit manually. */
      pxPhyObject->fnPhyRead(xPhyAddress, phyREG_00_BMCR, &ulConfig);
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_00_BMCR, ulConfig & ~phyBMCR_RESET);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(phySHORT_DELAY_MS));

  return ulDoneMask;
}
/*-----------------------------------------------------------*/

BaseType_t xPhyConfigure(EthernetPhy_t *pxPhyObject,
                         const PhyProperties_t *pxPhyProperties)
{
  uint32_t ulConfig, ulAdvertise;
  BaseType_t xPhyIndex;

  if (pxPhyObject->xPortCount < 1)
  {
    FreeRTOS_printf(("xPhyConfigure: No PHY's detected.\n"));
    return -1;
  }

  /* The expected ID for the 'LAN8742A'  is 0x0007c130. */
  /* The expected ID for the 'LAN8720'   is 0x0007c0f0. */
  /* The expected ID for the 'DP83848I'  is 0x20005C90. */

  /* Set advertise register. */
  if ((pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_AUTO) && (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_AUTO))
  {
    ulAdvertise = phyADVERTISE_ALL;
    /* Reset auto-negotiation capability. */
  }
  else
  {
    /* Always select protocol 802.3u. */
    ulAdvertise = phyADVERTISE_CSMA;

    if (pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_AUTO)
    {
      if (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_FULL)
      {
        ulAdvertise |= phyADVERTISE_10FULL | phyADVERTISE_100FULL;
      }
      else
      {
        ulAdvertise |= phyADVERTISE_10HALF | phyADVERTISE_100HALF;
      }
    }
    else if (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_AUTO)
    {
      if (pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_10)
      {
        ulAdvertise |= phyADVERTISE_10FULL | phyADVERTISE_10HALF;
      }
      else
      {
        ulAdvertise |= phyADVERTISE_100FULL | phyADVERTISE_100HALF;
      }
    }
    else if (pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_100)
    {
      if (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_FULL)
      {
        ulAdvertise |= phyADVERTISE_100FULL;
      }
      else
      {
        ulAdvertise |= phyADVERTISE_100HALF;
      }
    }
    else
    {
      if (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_FULL)
      {
        ulAdvertise |= phyADVERTISE_10FULL;
      }
      else
      {
        ulAdvertise |= phyADVERTISE_10HALF;
      }
    }
  }

  /* Send a reset command to a set of PHY-ports. */
  xPhyReset(pxPhyObject, xPhyGetMask(pxPhyObject));

  for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
    uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

    /* Write advertise register. */
    pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_04_ADVERTISE, ulAdvertise);

    /*
     *      AN_EN        AN1         AN0       Forced Mode
     *        0           0           0        10BASE-T, Half-Duplex
     *        0           0           1        10BASE-T, Full-Duplex
     *        0           1           0        100BASE-TX, Half-Duplex
     *        0           1           1        100BASE-TX, Full-Duplex
     *      AN_EN        AN1         AN0       Advertised Mode
     *        1           0           0        10BASE-T, Half/Full-Duplex
     *        1           0           1        100BASE-TX, Half/Full-Duplex
     *        1           1           0        10BASE-T Half-Duplex
     *                                         100BASE-TX, Half-Duplex
     *        1           1           1        10BASE-T, Half/Full-Duplex
     *                                         100BASE-TX, Half/Full-Duplex
     */

    /* Read Control register. */
    pxPhyObject->fnPhyRead(xPhyAddress, phyREG_00_BMCR, &ulConfig);

    ulConfig &= ~(phyBMCR_SPEED_100 | phyBMCR_FULL_DUPLEX);

    ulConfig |= phyBMCR_AN_ENABLE;

    if ((pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_100) || (pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_AUTO))
    {
      ulConfig |= phyBMCR_SPEED_100;
    }
    else if (pxPhyProperties->ucSpeed == (uint8_t)PHY_SPEED_10)
    {
      ulConfig &= ~phyBMCR_SPEED_100;
    }

    if ((pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_FULL) || (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_AUTO))
    {
      ulConfig |= phyBMCR_FULL_DUPLEX;
    }
    else if (pxPhyProperties->ucDuplex == (uint8_t)PHY_DUPLEX_HALF)
    {
      ulConfig &= ~phyBMCR_FULL_DUPLEX;
    }

    if (xHas_19_PHYCR(ulPhyID))
    {
      uint32_t ulPhyControl;
      /* Read PHY Control register. */
      pxPhyObject->fnPhyRead(xPhyAddress, phyREG_19_PHYCR, &ulPhyControl);

      /* Clear bits which might get set: */
      ulPhyControl &= ~(phyPHYCR_MDIX_EN | phyPHYCR_MDIX_FORCE);

      if (pxPhyProperties->ucMDI_X == PHY_MDIX_AUTO)
      {
        ulPhyControl |= phyPHYCR_MDIX_EN;
      }
      else if (pxPhyProperties->ucMDI_X == PHY_MDIX_CROSSED)
      {
        /* Force direct link = Use crossed RJ45 cable. */
        ulPhyControl &= ~phyPHYCR_MDIX_FORCE;
      }
      else
      {
        /* Force crossed link = Use direct RJ45 cable. */
        ulPhyControl |= phyPHYCR_MDIX_FORCE;
      }

      /* update PHY Control Register. */
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_19_PHYCR, ulPhyControl);
    }

    FreeRTOS_printf(("xPhyConfigure: Autonego advertise 0x%04lX (BMCR 0x%04lX)\n", ulAdvertise, ulConfig));
  }

  /* Keep these values for later use. */
  pxPhyObject->ulBCRValue = ulConfig & ~phyBMCR_ISOLATE;
  pxPhyObject->ulACRValue = ulAdvertise;

  return 0;
}
/*-----------------------------------------------------------*/

/* xPhyFixedValue(): this function is called in case auto-negotiation is disabled.
 * The caller has set the values in 'xPhyPreferences' (ucDuplex and ucSpeed).
 * The PHY register phyREG_00_BMCR will be set for every connected PHY that matches
 * with ulPhyMask. */
BaseType_t xPhyFixedValue(EthernetPhy_t *pxPhyObject,
                          uint32_t ulPhyMask)
{
  BaseType_t xPhyIndex;
  uint32_t ulValue, ulBitMask = (uint32_t)1U;

  ulValue = (uint32_t)0U;

  if (pxPhyObject->xPhyPreferences.ucDuplex == PHY_DUPLEX_FULL)
  {
    ulValue |= phyBMCR_FULL_DUPLEX;
  }

  if (pxPhyObject->xPhyPreferences.ucSpeed == PHY_SPEED_100)
  {
    ulValue |= phyBMCR_SPEED_100;
  }

  for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++, ulBitMask <<= 1)
  {
    if ((ulPhyMask & ulBitMask) != 0lu)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

      /* Enable Auto-Negotiation. */
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_00_BMCR, ulValue);
    }
  }

  return 0;
}
/*-----------------------------------------------------------*/

/* xPhyStartAutoNegotiation() is the alternative xPhyFixedValue():
 * It sets the BMCR_AN_RESTART bit and waits for the auto-negotiation completion
 * ( phyBMSR_AN_COMPLETE ). */
BaseType_t xPhyStartAutoNegotiation(EthernetPhy_t *pxPhyObject,
                                    uint32_t ulPhyMask)
{
  uint32_t xPhyIndex, ulDoneMask, ulBitMask;
  uint32_t ulPHYLinkStatus, ulRegValue;
  TickType_t xRemainingTime;
  TimeOut_t xTimer;

  if (ulPhyMask == (uint32_t)0U)
  {
    return 0;
  }

  for (xPhyIndex = 0; xPhyIndex < (uint32_t)pxPhyObject->xPortCount; xPhyIndex++)
  {
    if ((ulPhyMask & (1lu << xPhyIndex)) != 0lu)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

      /* Enable Auto-Negotiation. */
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_04_ADVERTISE, pxPhyObject->ulACRValue);
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_00_BMCR, pxPhyObject->ulBCRValue | phyBMCR_AN_RESTART);
    }
  }

  xRemainingTime = (TickType_t)pdMS_TO_TICKS(phyPHY_MAX_NEGOTIATE_TIME_MS);
  vTaskSetTimeOutState(&xTimer);
  ulDoneMask = 0;

  /* Wait until the auto-negotiation will be completed */
  for (;;)
  {
    ulBitMask = (uint32_t)1U;

    for (xPhyIndex = 0; xPhyIndex < (uint32_t)pxPhyObject->xPortCount; xPhyIndex++, ulBitMask <<= 1)
    {
      if ((ulPhyMask & ulBitMask) != 0lu)
      {
        if ((ulDoneMask & ulBitMask) == 0lu)
        {
          BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

          pxPhyObject->fnPhyRead(xPhyAddress, phyREG_01_BMSR, &ulRegValue);

          if ((ulRegValue & phyBMSR_AN_COMPLETE) != 0)
          {
            ulDoneMask |= ulBitMask;
          }
        }
      }
    }

    if (ulPhyMask == ulDoneMask)
    {
      break;
    }

    if (xTaskCheckForTimeOut(&xTimer, &xRemainingTime) != pdFALSE)
    {
      FreeRTOS_printf(("xPhyStartAutoNegotiation: phyBMSR_AN_COMPLETE timed out (done 0x%02lX)\n", (unsigned int)ulDoneMask));
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(phySHORT_DELAY_MS));
  }

  if (ulDoneMask != (uint32_t)0U)
  {
    ulBitMask = (uint32_t)1U;
    pxPhyObject->ulLinkStatusMask &= ~(ulDoneMask);

    for (xPhyIndex = 0; xPhyIndex < (uint32_t)pxPhyObject->xPortCount; xPhyIndex++, ulBitMask <<= 1)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
      uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

      if ((ulDoneMask & ulBitMask) == (uint32_t)0U)
      {
        continue;
      }

      /* Clear the 'phyBMCR_AN_RESTART'  bit. */
      pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_00_BMCR, pxPhyObject->ulBCRValue);

      pxPhyObject->fnPhyRead(xPhyAddress, phyREG_01_BMSR, &ulRegValue);

      if ((ulRegValue & phyBMSR_LINK_STATUS) != 0)
      {
        ulPHYLinkStatus |= phyBMSR_LINK_STATUS;
        pxPhyObject->ulLinkStatusMask |= ulBitMask;
      }
      else
      {
        ulPHYLinkStatus &= ~(phyBMSR_LINK_STATUS);
      }

      if (ulPhyID == PHY_ID_KSZ8081MNXIA)
      {
        uint32_t ulControlStatus;

        pxPhyObject->fnPhyRead(xPhyAddress, 0x1E, &ulControlStatus);

        switch (ulControlStatus & 0x07)
        {
        case 0x01:
        case 0x05:
          /*	[001] = 10BASE-T half-duplex */
          /*	[101] = 10BASE-T full-duplex */
          /* 10 Mbps. */
          ulRegValue |= phyPHYSTS_SPEED_STATUS;
          break;

        case 0x02:
        case 0x06:
          /*	[010] = 100BASE-TX half-duplex */
          /*	[110] = 100BASE-TX full-duplex */
          break;
        }

        switch (ulControlStatus & 0x07)
        {
        case 0x05:
        case 0x06:
          /*	[101] = 10BASE-T full-duplex */
          /*	[110] = 100BASE-TX full-duplex */
          /* Full duplex. */
          ulRegValue |= phyPHYSTS_DUPLEX_STATUS;
          break;

        case 0x01:
        case 0x02:
          /*	[001] = 10BASE-T half-duplex */
          /*	[010] = 100BASE-TX half-duplex */
          break;
        }
      }
      else if (ulPhyID == PHY_ID_KSZ8795)
      {
        /* KSZ8795 has a different mapping for the Port Operation Mode Indication field
         *   in the phyREG_1F_PHYSPCS than other similar PHYs:
         *     010 = 10BASE-T half-duplex
         *     101 = 10BASE-T full-duplex
         *     011 = 100BASE-TX half-duplex
         *     110 = 100BASE-TX full-duplex
         */
        uint32_t ulControlStatus = 0u;
        uint32_t ulPortOperationMode = 0u;
        pxPhyObject->fnPhyRead(xPhyAddress, phyREG_1F_PHYSPCS, &ulControlStatus);
        ulPortOperationMode = (ulControlStatus >> 8u) & 0x07u;

        ulRegValue = 0;

        /* Detect 10baseT operation */
        if ((0x02u == ulPortOperationMode) || (0x05u == ulPortOperationMode))
        {
          ulRegValue |= phyPHYSTS_SPEED_STATUS;
        }

        /* Detect full duplex operation */
        if ((0x05u == ulPortOperationMode) || (0x06u == ulPortOperationMode))
        {
          ulRegValue |= phyPHYSTS_DUPLEX_STATUS;
        }
      }
      else if (xHas_1F_PHYSPCS(ulPhyID))
      {
        /* 31 RW PHY Special Control Status */
        uint32_t ulControlStatus;

        pxPhyObject->fnPhyRead(xPhyAddress, phyREG_1F_PHYSPCS, &ulControlStatus);
        ulRegValue = 0;

        if ((ulControlStatus & phyPHYSPCS_FULL_DUPLEX) != 0)
        {
          ulRegValue |= phyPHYSTS_DUPLEX_STATUS;
        }

        if ((ulControlStatus & phyPHYSPCS_SPEED_MASK) == phyPHYSPCS_SPEED_10)
        {
          ulRegValue |= phyPHYSTS_SPEED_STATUS;
        }
      }
      else
      {
        /* Read the result of the auto-negotiation. */
        pxPhyObject->fnPhyRead(xPhyAddress, phyREG_10_PHYSTS, &ulRegValue);
      }

      FreeRTOS_printf(("xPhyStartAutoNegotiation: Autonego ready (0x%04lX / %s duplex / %u mbit / link status %s)\n",
                       ulRegValue,
                       (ulRegValue & phyPHYSTS_DUPLEX_STATUS) ? "full" : "half",
                       (ulRegValue & phyPHYSTS_SPEED_STATUS) ? 10 : 100,
                       ((ulPHYLinkStatus |= phyBMSR_LINK_STATUS) != 0) ? "high" : "low"));

      if ((ulRegValue & phyPHYSTS_DUPLEX_STATUS) != (uint32_t)0U)
      {
        pxPhyObject->xPhyProperties.ucDuplex = PHY_DUPLEX_FULL;
      }
      else
      {
        pxPhyObject->xPhyProperties.ucDuplex = PHY_DUPLEX_HALF;
      }

      if ((ulRegValue & phyPHYSTS_SPEED_STATUS) != 0)
      {
        pxPhyObject->xPhyProperties.ucSpeed = PHY_SPEED_10;
      }
      else
      {
        pxPhyObject->xPhyProperties.ucSpeed = PHY_SPEED_100;
      }
    }
  } /* if( ulDoneMask != ( uint32_t) 0U ) */

  return 0;
}
/*-----------------------------------------------------------*/

BaseType_t xPhyCheckLinkStatus(EthernetPhy_t *pxPhyObject,
                               BaseType_t xHadReception,
                               BaseType_t xForceAndSkipTimeout)
{
  uint32_t ulStatus, ulBitMask = 1U;
  BaseType_t xPhyIndex;
  BaseType_t xNeedCheck = pdFALSE;

  if (xForceAndSkipTimeout == 0 && xHadReception > 0)
  {
    /* A packet was received. No need to check for the PHY status now,
     * but set a timer to check it later on. */
    vTaskSetTimeOutState(&(pxPhyObject->xLinkStatusTimer));
    pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS(ipconfigPHY_LS_HIGH_CHECK_TIME_MS);

    for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++, ulBitMask <<= 1)
    {
      if ((pxPhyObject->ulLinkStatusMask & ulBitMask) == 0UL)
      {
        pxPhyObject->ulLinkStatusMask |= ulBitMask;
        xNeedCheck = pdTRUE;
      }
    }
  }
  else if (xForceAndSkipTimeout > 0 ||
           (xTaskCheckForTimeOut(&(pxPhyObject->xLinkStatusTimer), &(pxPhyObject->xLinkStatusRemaining)) != pdFALSE))
  {
    /* Frequent checking the PHY Link Status can affect for the performance of Ethernet controller.
     * As long as packets are received, no polling is needed.
     * Otherwise, polling will be done when the 'xLinkStatusTimer' expires. */
    for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++, ulBitMask <<= 1)
    {
      BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];

      if (pxPhyObject->fnPhyRead(xPhyAddress, phyREG_01_BMSR, &ulStatus) == 0)
      {
        if (!!(pxPhyObject->ulLinkStatusMask & ulBitMask) != !!(ulStatus & phyBMSR_LINK_STATUS))
        {
          if ((ulStatus & phyBMSR_LINK_STATUS) != 0)
          {
            pxPhyObject->ulLinkStatusMask |= ulBitMask;
          }
          else
          {
            pxPhyObject->ulLinkStatusMask &= ~(ulBitMask);
          }
          xNeedCheck = pdTRUE;
        }
      }
    }

    vTaskSetTimeOutState(&(pxPhyObject->xLinkStatusTimer));

    if ((pxPhyObject->ulLinkStatusMask & (ulBitMask >> 1)) != 0)
    {
      /* The link status is high, so don't poll the PHY too often. */
      pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS(ipconfigPHY_LS_HIGH_CHECK_TIME_MS);
    }
    else
    {
      /* The link status is low, polling may be done more frequently. */
      pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS(ipconfigPHY_LS_LOW_CHECK_TIME_MS);
    }
  }

  if (xNeedCheck == pdTRUE)
  {
    FreeRTOS_printf(("xPhyCheckLinkStatus: LinkStatus 0x%02lX (port count %d)\n",
                     pxPhyObject->ulLinkStatusMask, pxPhyObject->xPortCount));
  }

  return xNeedCheck;
}
/*-----------------------------------------------------------*/

/* Return non-zero in case the action failed. Currently only supports one port
 * xPortCount == 1. */
BaseType_t xPhyPrintRegisters(EthernetPhy_t *pxPhyObject)
{
  if (pxPhyObject->xPortCount != 1)
  {
    FreeRTOS_printf(("xPhyPrintRegisters: No or Multiple PHY's detected.\n"));
    return -1;
  }
  BaseType_t xPhyIndex = 0;

  // for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
    // uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

    uint32_t ulValue = 0;
    for (BaseType_t xRegister = 0; xRegister < 0x1F; xRegister++)
    {
      if (pxPhyObject->fnPhyRead(xPhyAddress, xRegister, &ulValue) == 0)
      {
        FreeRTOS_printf(("xPhyPrintRegisters %d (0x%02X): 0x%04X.\n", xRegister, xRegister, ulValue));
      }
    }
    for (BaseType_t xRegister = 5; xRegister <= 6; xRegister++)
    {
      xPhyReadMMDRegister(pxPhyObject, xPhyAddress, xRegister, &ulValue);
    }
    for (BaseType_t xRegister = 32784; xRegister <= 32786; xRegister++)
    {
      xPhyReadMMDRegister(pxPhyObject, xPhyAddress, xRegister, &ulValue);
    }
    for (BaseType_t xRegister = 32801; xRegister <= 32808; xRegister++)
    {
      xPhyReadMMDRegister(pxPhyObject, xPhyAddress, xRegister, &ulValue);
    }
    for (BaseType_t xRegister = 32865; xRegister <= 32868; xRegister++)
    {
      xPhyReadMMDRegister(pxPhyObject, xPhyAddress, xRegister, &ulValue);
    }
  }
  return 0;
}
/*-----------------------------------------------------------*/

/* Return non-zero in case the action failed. Currently only supports one port
 * xPortCount == 1. */
BaseType_t xPhySetInterruptMode(EthernetPhy_t *pxPhyObject, BaseType_t xInterruptMode)
{
  if (pxPhyObject->xPortCount != 1)
  {
    FreeRTOS_printf(("xPhySetInterruptMode: No or Multiple PHY's detected.\n"));
    return -1;
  }
  BaseType_t xPhyIndex = 0;

  // for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
    uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

    if (xHas_0E_MMDADR(ulPhyID))
    {
      // BaseType_t xResult;

      /* Set Wakeup Control and Status Register (WUCSR) LED2 functions [12:11]:
       *    0b00 = LED2 functions as Link Speed.
       *    0b01 = LED2 functions as nINT.
       *    0b10 = LED2 functions as nPME.
       *    0b11 = LED2 functions as Link/Activity. */
      uint32_t ulWUCSR = 0;
      /* xResult = */ xPhyReadMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, &ulWUCSR);
      if (xInterruptMode == 1)
      {
        ulWUCSR = (0b01 << 11);

        /* Enable Link Down (link status negated) in Interrupt Mask Register (ISM) */
        uint32_t ulIMR;
        /* xResult = */ pxPhyObject->fnPhyRead(xPhyAddress, phyREG_1E_IMR, &ulIMR);
        ulIMR |= phyISFR_LD;
        /* xResult = */ pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_1E_IMR, ulIMR);
      }
      else
      {
        ulWUCSR = (0b11 << 11);
      }
      if (xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, &ulWUCSR) == 0)
      {
        FreeRTOS_printf(("xPhySetInterruptMode: LED2 function set to '%s' (WUCSR 0x%04X).\n",
                         (xInterruptMode ? "nINT" : "Link/Activity"), ulWUCSR));
        return 0;
      }
    }
  }
  return -1;
}
/*-----------------------------------------------------------*/

/* Return non-zero in case the action failed. Currently only supports one port
 * xPortCount == 1. */
BaseType_t xPhySetMagicPacketDetection(EthernetPhy_t *pxPhyObject, uint8_t *pucMACAddress)
{
  if (pxPhyObject->xPortCount != 1)
  {
    FreeRTOS_printf(("xPhySetMagicPacketDetection: No or Multiple PHY's detected.\n"));
    return -1;
  }
  BaseType_t xPhyIndex = 0;

  // for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  {
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
    uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

    if (xHas_0E_MMDADR(ulPhyID))
    {
      BaseType_t xResult;

      /* 1. Set the desired MAC address to cause the wake event in the
       *    RX_ADDR registers.
       *
       * The MAC address must be loaded into the RX_ADDRA, RX_ADDRB, and
       * RX_ADDRC registers in the proper byte order. For example, a MAC
       * address of 12:34:56:78:9A:BC should be loaded into these
       * registers as follows:
       *    RX_ADDRA = BC9Ah
       *    RX_ADDRB = 7856h
       *    RX_ADDRC = 3412h */
      uint32_t ulADDRA = 0;

      /* MAC Receive Address A Register (12:34:56:78:9A:BC > 0xBC9A) */
      ulADDRA = (uint32_t)(pucMACAddress[5] << 8 | pucMACAddress[4]);
      xResult = xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_RX_ADDRA, &ulADDRA);

      /* MAC Receive Address B Register (12:34:56:78:9A:BC > 0x7856) */
      ulADDRA = (uint32_t)(pucMACAddress[3] << 8 | pucMACAddress[2]);
      xResult = xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_RX_ADDRB, &ulADDRA);

      /* MAC Receive Address C Register (12:34:56:78:9A:BC > 0x3412) */
      ulADDRA = (uint32_t)(pucMACAddress[1] << 8 | pucMACAddress[0]);
      xResult = xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_RX_ADDRC, &ulADDRA);

      /* 2. Set the Magic Packet Enable (MPEN) bit of the Wakeup Control
       *    and Status Register (WUCSR) */
      uint32_t ulWUCSR = 0;
      xResult = xPhyReadMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, &ulWUCSR);
      /* Set MPR to clear previouse interrupts */
      ulWUCSR |= (phyWUCSR_MPEN | phyWUCSR_MPR);
      ulWUCSR &= ~phyWUCSR_RMII_ENL;
      xResult = xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, &ulWUCSR);

      /* 3. Set bit 8 (WoL event indicator) in the Interrupt Mask Register
       *    to enable WoL. */
      uint32_t ulIMR = 0;
      xResult = pxPhyObject->fnPhyRead(xPhyAddress, phyREG_1E_IMR, &ulIMR);
      ulIMR |= phyISFR_WOL;
      xResult = pxPhyObject->fnPhyWrite(xPhyAddress, phyREG_1E_IMR, ulIMR);

      if (xResult == 0)
      {
        FreeRTOS_printf(("xPhySetMagicPacketDetection: Magic packet detection for %02X:%02X:%02X:%02X:%02X:%02X (IMR 0x%04X / WUCSR 0x%04X).\n",
                         pucMACAddress[0], pucMACAddress[1], pucMACAddress[2], pucMACAddress[3], pucMACAddress[4], pucMACAddress[5], ulIMR, ulWUCSR));
        return 0;
      }
    }
  }
  return -1;
}
/*-----------------------------------------------------------*/

/* Return non-zero in case the action failed. Currently only supports one port
 * xPortCount == 1. */
BaseType_t xPhyHandleInterrupt(EthernetPhy_t *pxPhyObject,
                               uint32_t *pulISFR,
                               uint32_t *pulWUCSR)
{
  if (pxPhyObject->xPortCount != 1)
  {
    FreeRTOS_printf(("xPhyHandleInterrupt: No or Multiple PHY's detected.\n"));
    return -1;
  }
  BaseType_t xPhyIndex = 0;

  // for (xPhyIndex = 0; xPhyIndex < pxPhyObject->xPortCount; xPhyIndex++)
  // {
  BaseType_t xPhyAddress = pxPhyObject->ucPhyIndexes[xPhyIndex];
  uint32_t ulPhyID = pxPhyObject->ulPhyIDs[xPhyIndex];

  if (xHas_1D_ISFR(ulPhyID))
  {
    if (pxPhyObject->fnPhyRead(xPhyAddress, phyREG_1D_ISFR, pulISFR) == 0)
    {
      if (*pulISFR > 0)
      {
        if (*pulISFR & phyISFR_WOL)
        {
          if (xHas_0E_MMDADR(ulPhyID))
          {
            /* Wake on LAN (WoL) event detected */
            xPhyReadMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, pulWUCSR);
            FreeRTOS_debug_printf(("xPhyHandleInterrupt: Wake on LAN event 0x%04X occured (ISFR 0x%04X)\n",
                                   *pulWUCSR, *pulISFR));

            /* Clear all handled WoL event flags by writing 1 to the belonging WoL event flag */
            uint32_t ulClear = *pulWUCSR;
            xPhyWriteMMDRegister(pxPhyObject, xPhyAddress, phyMMD_WUCSR, &ulClear);

            // /* Broadcast Frame Received */
            // if (*pulWUCSR & phyWUCSR_BCAST_FR)
            // {
            //     /* Clear BCAST_FR flag */
            //     ulClear = (ulClear & (phyWUCSR_CLEAR_MASK | phyWUCSR_BCAST_FR));
            // }
            // /* Magic Packet Received */
            // if (*pulWUCSR & phyWUCSR_MPR)
            // {
            //     /* Clear MPR flag */
            //     ulClear = (ulClear & (phyWUCSR_CLEAR_MASK | phyWUCSR_MPR));
            // }
            // /* Remote Wakeup Frame Received */
            // if (*pulWUCSR & phyWUCSR_WUFR)
            // {
            //     /* Clear WUFR flag */
            //     ulClear = (ulClear & (phyWUCSR_CLEAR_MASK | phyWUCSR_WUFR));
            // }
            // /* Perfect DA Frame Received */
            // if (*pulWUCSR & phyWUCSR_PFDA_FR)
            // {
            //     /* Clear PFDA_FR flag */
            //     ulClear = (ulClear & (phyWUCSR_CLEAR_MASK | phyWUCSR_PFDA_FR));
            // }
          }
        }

        // if (*pulISFR & phyISFR_ENERGYON)
        // { /* ENERGYON generated */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: ENERGYON generated (ISFR 0x%04X)\n", *pulISFR));
        // }
        // if (*pulISFR & phyISFR_ANC)
        // { /* Auto-Negotiation complete */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Auto-Negotiation complete (ISFR 0x%04X)\n", *pulISFR));
        // }
        // if (*pulISFR & phyISFR_RFD)
        // { /* Remote Fault Detected */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Remote Fault Detected (ISFR 0x%04X)\n", *pulISFR));
        // }
        // if (*pulISFR & phyISFR_LD)
        // { /* Link DOWN (link status negated) */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Link DOWN (ISFR 0x%04X)\n", *pulISFR));
        //     // ethernetif_set_link(lwip);
        // }
        // if (*pulISFR & phyISFR_ANLPA)
        // { /* Auto-Negotiation LP Acknowledge */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Auto-Negotiation LP Acknowledge (ISFR 0x%04X)\n", *pulISFR));
        // }
        // if (*pulISFR & phyISFR_PDF)
        // { /* Parallel Detection Fault */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Parallel Detection Fault (ISFR 0x%04X)\n", *pulISFR));
        // }
        // if (*pulISFR & phyISFR_ANPR)
        // { /* Auto-Negotiation Page Received */
        //     FreeRTOS_debug_printf(("xPhyHandleInterrupt: Auto-Negotiation Page Received (ISFR 0x%04X)\n", *pulISFR));
        // }
      }
      return 0;
    }
  }
  // }

  return -1;
}
/*-----------------------------------------------------------*/
