/**************************************************************************************************
  Filename:       zcl_homelink.c
  Revised:        $Date: 2014-06-03 16:29:28 -0700 (Tue, 03 Jun 2014) $
  Revision:       $Revision: 38778 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_homelink.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_key.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include "hal_flash.h"

#include <stdlib.h>
#include <math.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define NUM_BUTTONS 3 // Number of buttons on the switch

#define HOMELINK_SW_BLANKING 2000 // 2 seconds for switch blanking
#define HOMELINK_SW_RESET 1000 // time to hold all three buttons for reset

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclHomelink_TaskID;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclHomelink_BindDstAddr;

bool initState = 0;
static uint8 seqNum = 1;
uint8 savedKeys = 0;

uint8 giSwScreenMode = SW_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclHomelink_NwkState = DEV_INIT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void getTemperature(void);

static void zclHomelink_zigbeeReset(void);
static void zclHomelink_SendCmdToggle(int endpoint);
static void zclHomelink_HandleKeys( byte shift, byte keys );
static void zclHomelink_BasicResetCB( void );
static void zclHomelink_UARTInit(void);
static void zclHomelink_UARTCallback(uint8 port, uint8 event);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclHomelink_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclHomelink_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclHomelink_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclHomelink_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclHomelink_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclHomelink_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclHomelink_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclHomelink_CmdCallbacks =
{
  zclHomelink_BasicResetCB,               // Basic Cluster Reset command
  NULL,                 // Identify command
  NULL,                                   // Identify Trigger Effect command
  NULL,         // Identify Query Response command
  NULL,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};


/*********************************************************************
 * STATUS STRINGS
 */

/*********************************************************************
 * @fn          zclHomelink_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclHomelink_Init( byte task_id )
{
  zclHomelink_UARTInit();

  zclHomelink_TaskID = task_id;

  // Set destination address to indirect for bindings
  zclHomelink_BindDstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclHomelink_BindDstAddr.endPoint = 0;
  zclHomelink_BindDstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclHomelink_SimpleDesc[0] );
  zclHA_Init( &zclHomelink_SimpleDesc[1] );
  zclHA_Init( &zclHomelink_SimpleDesc[2] );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( ENDPOINT, &zclHomelink_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( ENDPOINT+1, &zclHomelink_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( ENDPOINT+2, &zclHomelink_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( ENDPOINT, SAMPLESW_MAX_ATTRIBUTES, zclHomelink_Attrs );
  zcl_registerAttrList( ENDPOINT+1, SAMPLESW_MAX_ATTRIBUTES, zclHomelink_Attrs );
  zcl_registerAttrList( ENDPOINT+2, SAMPLESW_MAX_ATTRIBUTES, zclHomelink_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclHomelink_TaskID );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclHomelink_TaskID );

  ZDO_RegisterForZDOMsg( zclHomelink_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( zclHomelink_TaskID, Match_Desc_rsp );
}


#define ADC_TEMP_25C 1480
#define ADC_TEMP_BITS_PER_C 4.5
    
void getTemperature(void)
{
  int16 value;

  HalAdcSetReference(HAL_ADC_REF_125V);
  TR0 |= 1;
  ATEST |= 1;
  value = HalAdcRead(HAL_ADC_CHANNEL_TEMP, HAL_ADC_RESOLUTION_12);
  ATEST &= ~1;
  TR0 &= ~1;
  HalAdcSetReference(HAL_ADC_REF_AVDD);
  
  zclHomelink_Temperature = 2500 + ((value - ADC_TEMP_25C) * (int16)(100 / ADC_TEMP_BITS_PER_C));
}

void zclHomelink_zigbeeReset(void)
{
  uint16 clusters[2];
  zAddrType_t tempAddr = {{0}, (afAddrMode_t)AddrGroup};
  aps_Group_t tempGroup = {0, "SwitchX"};
  int i;
  
  for (i = HAL_NV_PAGE_BEG; i <= (HAL_NV_PAGE_BEG + HAL_NV_PAGE_CNT); i++)
  {
    HalFlashErase(i);
  }
  
  clusters[0] = ZCL_CLUSTER_ID_GEN_ON_OFF;
  clusters[1] = ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL;
  
  BindSetDefaultNV();
  aps_GroupsSetDefaultNV();
  
  // Initialize default groups.  Each button has a group ID built from the MAC.
  // The top 2 bits are for the endpoints.
  for(i = 0; i < NUM_BUTTONS; i ++) {
    tempGroup.ID =  i << 14 | (aExtendedAddress[1] & 0x3F) << 8 | aExtendedAddress[0];
    tempGroup.name[6] = '1' + i;
    tempAddr.addr.shortAddr = tempGroup.ID;
    bindAddEntry(ENDPOINT+i, &tempAddr, 0, 2, clusters);
    aps_AddGroup(ENDPOINT+i, &tempGroup);
  }
  BindWriteNV();
  aps_GroupsWriteNV();

  Onboard_soft_reset();
}

/*********************************************************************
 * @fn          zclHomelink_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclHomelink_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclHomelink_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclHomelink_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclHomelink_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclHomelink_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclHomelink_NwkState == DEV_ZB_COORD) ||
               (zclHomelink_NwkState == DEV_ROUTER)   ||
               (zclHomelink_NwkState == DEV_END_DEVICE) )
          {
            if (initState == 0)
            {
              initState = 1;
            }
          }
          break;
          
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Discard unknown events
  return 0;
}

void zclHomelink_SendCmdToggle(int endpoint) {
  zclGeneral_SendOnOff_CmdToggle( endpoint, &zclHomelink_BindDstAddr, FALSE, seqNum++ );
}

/*********************************************************************
 * @fn      zclHomelink_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *
 * @return  none
 */
static void zclHomelink_HandleKeys( byte shift, byte keys )
{

  if ((keys & (HAL_KEY_SW_2 | HAL_KEY_SW_3 | HAL_KEY_SW_4)) == (HAL_KEY_SW_2 | HAL_KEY_SW_3 | HAL_KEY_SW_4)) // Press all three for one second to reset device
  {
    // Check if keys still pressed after 500 ms debounce delay (in hal_key.c)
    if ((HalKeyRead() & (HAL_KEY_SW_2 | HAL_KEY_SW_3 | HAL_KEY_SW_4)) == (HAL_KEY_SW_2 | HAL_KEY_SW_3 | HAL_KEY_SW_4))
    {
      zclHomelink_zigbeeReset();
    }
    return;
  }
  osal_stop_timerEx(zclHomelink_TaskID, HOMELINK_RESET_EVT);

  // Check if any keys still pressed after 800 ms debounce delay (in hal_key.c)
  if (HalKeyRead() & (HAL_KEY_SW_2 | HAL_KEY_SW_3 | HAL_KEY_SW_4))
  {
    // If so do nothing, probably programming mode
  }
  else if (keys & HAL_KEY_SW_2)
  {
    if (osal_get_timeoutEx(zclHomelink_TaskID, HOMELINK_SW2_EVT) == 0)
    {
      zclHomelink_SendCmdToggle(ENDPOINT);
    }
    osal_start_timerEx( zclHomelink_TaskID, HOMELINK_SW2_EVT, HOMELINK_SW_BLANKING );
  }
  else if (keys & HAL_KEY_SW_3)
  {
    if (osal_get_timeoutEx(zclHomelink_TaskID, HOMELINK_SW3_EVT) == 0)
    {
      zclHomelink_SendCmdToggle(ENDPOINT+1);
    }
    osal_start_timerEx( zclHomelink_TaskID, HOMELINK_SW3_EVT, HOMELINK_SW_BLANKING );
  }
  else if (keys & HAL_KEY_SW_4)
  {
    if (osal_get_timeoutEx(zclHomelink_TaskID, HOMELINK_SW4_EVT) == 0)
    {
      zclHomelink_SendCmdToggle(ENDPOINT+2);
    }
    osal_start_timerEx( zclHomelink_TaskID, HOMELINK_SW4_EVT, HOMELINK_SW_BLANKING );
  }
}

/*********************************************************************
 * @fn      zclHomelink_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclHomelink_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclHomelink_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclHomelink_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclHomelink_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclHomelink_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclHomelink_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclHomelink_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclHomelink_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclHomelink_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclHomelink_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclHomelink_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclHomelink_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclHomelink_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclHomelink_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclHomelink_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclHomelink_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclHomelink_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclHomelink_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclHomelink_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclHomelink_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclHomelink_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHomelink_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER

void zclHomelink_UARTInit(void)
{
  halUARTCfg_t uartConfig;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = HAL_UART_FLOW_OFF;
  uartConfig.flowControlThreshold = HAL_UART_DMA_RX_MAX >> 1;
  uartConfig.rx.maxBufSize        = HAL_UART_DMA_RX_MAX;
  uartConfig.tx.maxBufSize        = HAL_UART_DMA_RX_MAX;
  uartConfig.idleTimeout          = 50;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = zclHomelink_UARTCallback;

  /* Start UdART */
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig);
}

void zclHomelink_UARTCallback(uint8 port, uint8 event)
{
  uint8 ch;
  while (Hal_UART_RxBufLen(port))
  {
    HalUARTRead (port, &ch, 1);
  }
  HalUARTWrite(HAL_UART_PORT_0, (uint8 *)"\r", 1);
}

/****************************************************************************
****************************************************************************/


