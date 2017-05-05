/*******************************************************************************
 Cosas utiles
*******************************************************************************/

/*******************************************************************************
 Notificcaciones directas al una conexion establecida (connHandled)
*******************************************************************************/
static void gateNotify();
static void gateNotifyCB(linkDBItem_t *pLinkItem);
static uint16 o=0;

static void performPeriodicTask( void )
{
  o++;
  gateNotify(); // <- forma correcta de hacerlo
/*    
      attHandleValueNoti_t noti;          
      noti.pValue = GATT_bm_alloc( 0,
                                   ATT_HANDLE_VALUE_NOTI, 
                                   20, NULL );
      if ( noti.pValue != NULL )
      {
        noti.handle = 0x0001;
        noti.len = 20;
        for (uint8 i=0; i < noti.len; i++)
        {noti.pValue[i] = 0;}
        noti.pValue[0] = o & 0x00FF;
        noti.pValue[1] = (o >> 8) &0x00FF;

        if ( GATT_Notification( 0, &noti, FALSE ) != SUCCESS )
        {
          GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
        }
      }
*/      
}

static void gateNotify()
{
  // Execute linkDB callback to send notification
  linkDB_PerformFunc( gateNotifyCB );
}

static void gateNotifyCB( linkDBItem_t *pLinkItem )
{
  if ( pLinkItem->stateFlags & LINK_CONNECTED )
  {
    uint16 value = GATTServApp_ReadCharCfg( pLinkItem->connectionHandle,
                                            battLevelClientCharCfg );
    if ( value & GATT_CLIENT_CFG_NOTIFY )
    {
      attHandleValueNoti_t noti;
          
      noti.pValue = GATT_bm_alloc( pLinkItem->connectionHandle, /* <-ver estructura completa en linkdb.h*/
                                   ATT_HANDLE_VALUE_NOTI, 
                                   1, NULL );
      if ( noti.pValue != NULL )
      {
        noti.handle = 0x0001;
        noti.len = 20;
        for (uint8 i=0; i < noti.len; i++)
        {noti.pValue[i] = 0;}
        noti.pValue[0] = o & 0x00FF;
        noti.pValue[1] = (o >> 8) &0x00FF;

        if ( GATT_Notification( pLinkItem->connectionHandle, &noti, FALSE ) != SUCCESS )
        {
          GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
        }
      }
    }
  }
}
/*
no he cosegido hacerlo funcionar con las funcionses de L2CAP:
extern bStatus_t L2CAP_SendData( uint16 connHandle, l2capPacket_t *pPkt );
extern void *L2CAP_bm_alloc( uint16 size );

no he cosegido hacerlo funcionar con las funcionses de HCI:
extern hciStatus_t HCI_SendDataPkt( uint16 connHandle,
                                    uint8  pbFlag,
                                    uint16 pktLen,
                                    uint8  *pData );
limitaciones de GATT functions:
maximo payload=20 (23-sizeof(len)-sizeof(handle))
*/
/*******************************************************************************
*******************************************************************************/


/*******************************************************************************
 Uso de Heap
*******************************************************************************/
/*
modificar en los ficheros de configuracion: 
#define INT_HEAP_LEN              3000
para cambiar el tamanio del heap, cuidado con el Stack

modificar el tamanio de la pilas en la configuracion del proyecto

maxima memoria ram 8K pero el SATCK ocupa el 60%
*/
/*******************************************************************************
*******************************************************************************/


/*******************************************************************************
 Registro de eventos de sistema para recibir HCI mensajes
*******************************************************************************/
#include "gap.h" //oJo, puesto para probar HCI
#include "hci_event.h" //oJo, puesto para probar HCI
#include "hci_ext.h" //oJo, puesto para probar HCI
#include "linkdb.h" //oJo, puesto para probar HCI

typedef struct
{
  uint8  pktType;
  uint16 opCode;
  uint8  len;
  uint8  *pData;
} hciExtCmd_t;

static uint8 processEvents(osal_event_hdr_t *pMsg );

/*............................................................................*/
/* es necesario registrar las tareas que queremos que nos devuelvan dataos*/
void DragonFly_Gate_Init(uint8 task_id)
{
  dragonflyGate_TaskID = task_id;

  // oJo
//  HCI_ExtTaskRegister( dragonflyGate_TaskID );
  
//  GAP_RegisterForMsgs( dragonflyGate_TaskID ); <- defined in the GAPPeripheralRole_Init()
//  VOID GATT_InitClient();

//  GATTServApp_RegisterForMsg( dragonflyGate_TaskID );
//  GATT_RegisterForReq( dragonflyGate_TaskID );

  
  // Initialize GATT attributes
  Gate_AddService();  
  
  // Register callback with SimpleGATTprofile
  VOID Gate_RegisterAppCBs(&DragonFly_GateCBs);
  
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 gateData[GATE_DATA_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    Gate_SetParameter(SENSOR_DATA, GATE_DATA_LEN, gateData);
  }

  // Setup a delayed profile startup
  osal_set_event(dragonflyGate_TaskID, DRAGONFLY_GATE_STAR_EVT);
}


/*............................................................................*/
static uint8 processExtMsg( hciPacket_t *pMsg )
{
  uint8 deallocateIncoming;
  bStatus_t stat = SUCCESS;
  uint8 rspDataLen = 0;
  hciExtCmd_t msg;
  uint8 *pBuf = pMsg->pData;
  
  uint8 rspBuf[56];

  // Parse the header
  msg.pktType = *pBuf++;
  msg.opCode = BUILD_UINT16( pBuf[0], pBuf[1] );
  pBuf += 2;

  msg.len = *pBuf++;
  msg.pData = pBuf;

  switch( msg.opCode >> 7 )
  {
    case HCI_EXT_L2CAP_SUBGRP:
//      stat = processExtMsgL2CAP( (msg.opCode & 0x007F), &msg, &rspDataLen );
      break;

    case HCI_EXT_ATT_SUBGRP:
//      stat = processExtMsgATT( (msg.opCode & 0x007F), &msg );
      break;

    case HCI_EXT_GATT_SUBGRP:
//      stat = processExtMsgGATT( (msg.opCode & 0x007F), &msg, &rspDataLen );
      break;

    case HCI_EXT_GAP_SUBGRP:
//      stat = processExtMsgGAP( (msg.opCode & 0x007F), &msg, &rspDataLen );
      break;

    case HCI_EXT_UTIL_SUBGRP:
//      stat = processExtMsgUTIL( (msg.opCode & 0x007F), &msg, &rspDataLen );
      break;

    default:
      stat = FAILURE;
      break;
  }

  // Deallocate here to free up heap space for the serial message set out HCI.
  VOID osal_msg_deallocate( (uint8 *)pMsg );
  deallocateIncoming = FALSE;

  // Send back an immediate response
  rspBuf[0] = LO_UINT16( HCI_EXT_GAP_CMD_STATUS_EVENT );
  rspBuf[1] = HI_UINT16( HCI_EXT_GAP_CMD_STATUS_EVENT );
  rspBuf[2] = stat;
  rspBuf[3] = LO_UINT16( 0xFC00 | msg.opCode );
  rspBuf[4] = HI_UINT16( 0xFC00 | msg.opCode );
  rspBuf[5] = rspDataLen;

  // IMPORTANT!! Fill in Payload (if needed) in case statement

  HCI_SendControllerToHostEvent( HCI_VE_EVENT_CODE, (6 + rspDataLen), rspBuf );

  return ( deallocateIncoming );
}

/*............................................................................*/
static uint8 processEvents( osal_event_hdr_t *pMsg )
{
  uint8 msgLen = 0;
  uint8 *pBuf = NULL;
  uint8 allocated = FALSE;
  uint8 deallocateIncoming = TRUE;
  
  static uint8 out_msg[44];

  VOID osal_memset( out_msg, 0, sizeof ( out_msg ) );

  switch ( pMsg->event )
  {
    case GAP_MSG_EVENT:
//      pBuf = processEventsGAP( (gapEventHdr_t *)pMsg, out_msg, &msgLen, &allocated, &deallocateIncoming );
      msgLen = 2;
      pBuf[0] = 44; pBuf[1] = 40;   
      break;

    case L2CAP_SIGNAL_EVENT:
//      pBuf = processEventsL2CAP( (l2capSignalEvent_t *)pMsg, out_msg, &msgLen );
      msgLen = 2;
      pBuf[0] = 44; pBuf[1] = 41;   
      break;

    case L2CAP_DATA_EVENT:
//      pBuf = processDataL2CAP( (l2capDataEvent_t *)pMsg, out_msg, &msgLen, &allocated );
      msgLen = 2;
      pBuf[0] = 44; pBuf[1] = 42;   
      break;
      
    case GATT_MSG_EVENT:
//      pBuf = processEventsGATT( (gattMsgEvent_t *)pMsg, out_msg, &msgLen, &allocated );
      msgLen = 2;
      pBuf[0] = 44; pBuf[1] = 43;   
      break;
#ifndef GATT_DB_OFF_CHIP
    case GATT_SERV_MSG_EVENT:
//      pBuf = processEventsGATTServ( (gattEventHdr_t *)pMsg, out_msg, &msgLen );
      msgLen = 2;
      pBuf[0] = 44; pBuf[1] = 44;   
      break;
#endif
    default:
      break; // ignore
  }

  // Deallocate here to free up heap space for the serial message set out HCI.
  if ( deallocateIncoming )
  {
    VOID osal_msg_deallocate( (uint8 *)pMsg );
  }

  if ( msgLen )
  {
    HCI_SendControllerToHostEvent( HCI_VE_EVENT_CODE,  msgLen, pBuf );
  }

  if ( (pBuf != NULL) && (allocated == TRUE) )
  {
    osal_mem_free( pBuf );
  }

  return ( FALSE );
}

/*............................................................................*/
uint16 DragonFly_Gate_ProcessEvent(uint8 task_id, uint16 events)
{  
  if(events & SYS_EVENT_MSG)
  {
// BT Events (see in hal_tl.h)
#define HCI_COMMAND_COMPLETE_EVENT_CODE                0x0E
    
// Vendor Specific Event Code (see in hal_tl.h)
#define HCI_VE_EVENT_CODE                              0xFF

    static hciPacket_t *pMsg;

    if((pMsg = (hciPacket_t *)osal_msg_receive(dragonflyGate_TaskID)) != NULL)
    {
      uint8 dealloc = TRUE;

      // Process incoming messages
      switch(pMsg->hdr.event)
      {
        // Incoming HCI extension message
        case HCI_EXT_CMD_EVENT:
          dealloc = processExtMsg( pMsg );
          break;

        case HCI_GAP_EVENT_EVENT:
          {
            if ( pMsg->hdr.status == HCI_COMMAND_COMPLETE_EVENT_CODE )
            {
              hciEvt_CmdComplete_t *pkt = (hciEvt_CmdComplete_t *)pMsg;
              osal_msg_hdr_t *msgHdr;
              uint8 len;

              msgHdr = (osal_msg_hdr_t *)pMsg;
              msgHdr--; // Backup to the msg header

              len = (uint8)(msgHdr->len - sizeof ( hciEvt_CmdComplete_t ));
              
              // needed "hci_event.h"
              HCI_SendCommandCompleteEvent(HCI_COMMAND_COMPLETE_EVENT_CODE, 
                                           pkt->cmdOpcode, 
                                           len, 
                                           pkt->pReturnParam );
            }
            else if ( pMsg->hdr.status == HCI_VE_EVENT_CODE )
            {
              hciEvt_VSCmdComplete_t *pkt = (hciEvt_VSCmdComplete_t *)pMsg;

              HCI_SendCommandCompleteEvent(HCI_VE_EVENT_CODE, 
                                           pkt->cmdOpcode,
                                           pkt->length, 
                                           pkt->pEventParam);
            }
          }
          break;
        
        default:
          {
          dealloc = processEvents( (osal_event_hdr_t *)pMsg );
          }
          break;
      }

      // Release the OSAL message
      if ( dealloc )
      {
        VOID osal_msg_deallocate( (uint8 *)pMsg );
      }
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
////////////////////////////////////////////////////////////////////////////////  

/*******************************************************************************
 TEST para Registro de eventos de sistema para recibir HCI mensajes
*******************************************************************************/
/*
  Mirar a ver si podemos acceder directamente a los mensajes en L2CAP.
  Utilizar el ejemplo hci_ext_appGeneric.c y observa como registra la capa L2CAP

  // Register with L2CAP Generic channel
  VOID L2CAP_RegisterApp( hciExtApp_TaskID, L2CAP_CID_GENERIC );
  
  y en servicios de sistema:
          case L2CAP_DATA_EVENT:
          {
            l2capDataEvent_t *pPkt = (l2capDataEvent_t *)pMsg;

            processL2CAPDataMsg( pPkt );

            // Free the buffer - payload
            if ( pPkt->pkt.pPayload )
            {
              osal_bm_free( pPkt->pkt.pPayload );
            }
          }
          break;
  
  podemos registrar cualquire vanal (L2CAP_CID_GENERIC) pare ceciver los datas
  que nos interesan!!!
*/
  
/*******************************************************************************
*******************************************************************************/

/*******************************************************************************
 Desregistrar una caracteristica en tiempo real
*******************************************************************************/
// mirar en hci_ext_app.c
/*
      case HCI_EXT_GATT_DEL_SERVICE:
      {
        uint16 handle = BUILD_UINT16( pBuf[0], pBuf[1] );

        if ( handle == 0x0000 )
        {
          // Service is not registered with GATT yet
          freeAttrRecs( &service );

          totalAttrs = 0;
        }
        else
        {
          gattService_t serv;

          // Service is already registered with the GATT Server
          stat = GATT_DeregisterService( handle, &serv );
          if ( stat == SUCCESS )
          {
            freeAttrRecs( &serv );
          }
        }
        
        stat = SUCCESS;
      }
      break;

  */
/*******************************************************************************
*******************************************************************************/

  
/*******************************************************************************
 Mandatory configurations
*******************************************************************************/
#define FEATURE_OAD   
#define OAD_BIM
#define OAD_KEEP_NV_PAGES
#define HALNODEBUG
#define OSAL_CBTIMER_NUM_TASKS    1
/*******************************************************************************
*******************************************************************************/
