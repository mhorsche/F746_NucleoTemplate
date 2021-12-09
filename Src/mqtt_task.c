/**
 * @file mqtt_task.c
 * @author horsche (horsche@li.plus)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "mqtt_task.h"

/* Standard includes. */
#include <string.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* MQTT library includes. */
#include "core_mqtt.h"

/* Exponential backoff retry include. */
#include "backoff_algorithm.h"

/* Transport interface include. */
#include "using_plaintext.h"

/* Private define ------------------------------------------------------------*/

#ifndef ipconfigCLIENT_IDENTIFIER
/**
 * @brief The MQTT client identifier used in this example.  Each client identifier
 * must be unique so edit as required to ensure no two clients connecting to the
 * same broker use the same client identifier.
 *
 * @note Appending __TIME__ to the client id string will reduce the possibility of a
 * client id collision in the broker. Note that the appended time is the compilation
 * time. This client id can cause collision, if more than one instance of the same
 * binary is used at the same time to connect to the broker.
 */
#define ipconfigCLIENT_IDENTIFIER "STM"__TIME__
#endif

#ifndef ipconfigMQTT_STACK_SIZE_MQTT_TASK
/**
 * @brief Stack size needed for prvMQTTTask(), a bit of a guess.
 */
#define ipconfigMQTT_STACK_SIZE_MQTT_TASK (configMINIMAL_STACK_SIZE * 10)
#endif

#ifndef ipconfigMQTT_PRIORITY_MQTT_TASK
/**
 * @brief The priority of prvMQTTTask(). Should be lower than the
 * IP-task and the task running in NetworkInterface.c.
 */
#define ipconfigMQTT_PRIORITY_MQTT_TASK 3
#endif

#ifndef ipconfigMQTT_BROKER_ENDPOINT
/**
 * @brief MQTT broker end point to connect to.
 *
 * @note If you would like to setup an MQTT broker for running this demo,
 * please see `mqtt_broker_setup.txt`.
 *
 * #define ipconfigMQTT_BROKER_ENDPOINT     "insert here."
 */
#error "Define ipconfigMQTT_BROKER_ENDPOINT in FreeRTOSIPConfig.h (e.g '192.168.2.1')."
#endif

#ifndef ipconfigMQTT_BROKER_PORT
/**
 * @brief The port of the MQTT broker.
 *
 * #define ipconfigMQTT_BROKER_PORT         ( insert here. )
 */
#warning "Using default MQTT port number '1883'. Define ipconfigMQTT_BROKER_PORT in FreeRTOSIPConfig.h to avoid this warning."
#define ipconfigMQTT_BROKER_PORT (1883)
#endif

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define mqtttaskRETRY_MAX_ATTEMPTS (5U)

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed operation
 *  with server.
 */
#define mqtttaskRETRY_MAX_BACKOFF_DELAY_MS (5000U)

/**
 * @brief The base back-off delay (in milliseconds) to use for network operation retry
 * attempts.
 */
#define mqtttaskRETRY_BACKOFF_BASE_MS (500U)

/**
 * @brief Timeout for receiving CONNACK packet in milliseconds.
 */
#define mqtttaskCONNACK_RECV_TIMEOUT_MS (1000U)

/**
 * @brief Dimensions a file scope buffer currently used to send and receive MQTT data
 * from a socket.
 */
#define mqtttaskSHARED_BUFFER_SIZE (500U)

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define mqtttaskPROCESS_LOOP_TIMEOUT_MS (10U)

/**
 * @brief Timeout for MQTT_ReceiveLoop in milliseconds.
 */
#define mqtttaskRECEIVE_LOOP_TIMEOUT_MS (0U)

/**
 * @brief Keep alive time reported to the broker while establishing an MQTT connection.
 *
 * It is the responsibility of the Client to ensure that the interval between
 * Control Packets being sent does not exceed the this Keep Alive value. In the
 * absence of sending any other Control Packets, the Client MUST send a
 * PINGREQ Packet.
 */
#define mqtttaskKEEP_ALIVE_TIMEOUT_SECONDS (60U)

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define mqtttaskTRANSPORT_SEND_RECV_TIMEOUT_MS (200U)

#define MILLISECONDS_PER_SECOND (1000U)                                      /**< @brief Milliseconds per second. */
#define MILLISECONDS_PER_TICK (MILLISECONDS_PER_SECOND / configTICK_RATE_HZ) /**< Milliseconds per FreeRTOS tick. */

/**
 * @brief Publish and un-/subscibe queue element sizes.
 */
#define REQUEST_ITEM_SIZE sizeof(MQTTRequest_t)

/* Private macros ------------------------------------------------------------*/

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Each compilation unit that consumes the NetworkContext must define it.
 * It should contain a single pointer to the type of your desired transport.
 * When using multiple transports in the same compilation unit, define this pointer as void *.
 *
 * @note Transport stacks are defined in FreeRTOS-Plus/Source/Application-Protocols/network_transport.
 */
struct NetworkContext
{
  PlaintextTransportParams_t *pParams;
};

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief MQTT task handle to avoid task getting started multiple times. Notification
 * is given in vApplicationIPNetworkEventHook as soon the network is up.
 */
TaskHandle_t xMQTTTaskHandle;

/**
 * @brief The task used to demonstrate the MQTT API.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvMQTTTask(void *pvParameters);

/**
 * @brief Check un-/subscribe queue and handle queued request.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 */
static void prvMQTTHandleQueue(MQTTContext_t *pxMQTTContext);

/**
 * @brief Connect to MQTT broker with reconnection retries.
 *
 * If connection fails, retry is attempted after a timeout.
 * Timeout value will exponentially increase until maximum
 * timeout value is reached or the number of attempts are exhausted.
 *
 * @param[out] pxNetworkContext The parameter to return the created network context.
 *
 * @return The status of the final connection attempt.
 */
static PlaintextTransportStatus_t prvConnectToServerWithBackoffRetries(NetworkContext_t *pxNetworkContext);

/**
 * @brief Sends an MQTT Connect packet over the already connected TCP socket.
 *
 * @param[in, out] pxMQTTContext MQTT context pointer.
 * @param[in] pxNetworkContext Network context.
 *
 */
static void prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext, NetworkContext_t *pxNetworkContext);

/**
 * @brief Un-/subscribes to the given topic. In the case of a Subscribe ACK failure,
 * then subscription is retried using an exponential backoff strategy with jitter.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] pxMessageItem List element containing topic information.
 */
static void prvMQTTMessageWithBackoffRetries(MQTTContext_t *pxMQTTContext, MQTTMessageItem_t *pxMessageItem);

/**
 * @brief The application callback function for getting the incoming publish
 * and incoming acks reported from the MQTT library.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] pxDeserializedInfo Deserialized information from the incoming packet.
 */
static void prvMQTTEventCallback(MQTTContext_t *pxMQTTContext, MQTTPacketInfo_t *pxPacketInfo, MQTTDeserializedInfo_t *pxDeserializedInfo);

/**
 * @brief Process incoming Publish message.
 *
 * @param[in] pxPublishInfo is a pointer to structure containing deserialized
 * Publish message.
 */
static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo);

/**
 * @brief Function to update variable #pxMessageItem->xAckStatus for published
 * messages with QoS1. Called by the event callback after processing an incoming
 * PUBACK packet.
 *
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] usPacketId Packet ID generated by #MQTT_GetPacketId.
 */
static void prvMQTTUpdatePubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId);

/**
 * @brief Function to update variable #pxMessageItem->xAckStatus for published
 * messages with QoS2. Called by the event callback after processing an incoming
 * PUBREC packet.
 *
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] usPacketId Packet ID generated by #MQTT_GetPacketId.
 */
static void prvMQTTUpdatePubRecStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId);

/**
 * @brief Function to update variable #pxMessageItem->xAckStatus for published
 * messages with QoS2. Called by the event callback after processing an incoming
 * PUBCOMP packet.
 *
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] usPacketId Packet ID generated by #MQTT_GetPacketId.
 */
static void prvMQTTUpdatePubCompStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId);

/**
 * @brief Function to update variable #pxMessageItem->xAckStatus with status
 * information from Subscribe ACK. Called by the event callback after processing
 * an incoming SUBACK packet.
 *
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] usPacketId Packet ID generated by #MQTT_GetPacketId.
 */
static void prvMQTTUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId);

/**
 * @brief Function to delete subscription element from list and free allocated 
 * memory. Called by the event callback after processing an incoming UNSUBACK packet.
 *
 * @param[in] Server response to the subscription request.
 * @param[in] usPacketId Packet ID generated by #MQTT_GetPacketId.
 */
static void prvMQTTUpdateUnsubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId);

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t prvGetTimeMs(void);

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucSharedBuffer[mqtttaskSHARED_BUFFER_SIZE];

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static MQTTFixedBuffer_t xBuffer = {
    .pBuffer = ucSharedBuffer,
    .size = mqtttaskSHARED_BUFFER_SIZE};

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

/** 
 * @brief The list of all subscribed topics. The SUBACK status
 * is updated when the event callback processes a SUBACK or UNSUBACK.
 */
static List_t xMQTTMessageList;

/**
 * @brief The variable used to hold the queue's data structure.
 */
static StaticQueue_t xMQTTStaticQueue;

/**
 * @brief The array to use as the queue's storage area.  This must be at least
 * uxQueueLength * uxItemSize bytes.
 */
uint8_t ucMQTTQueueStorageArea[MQTT_STATE_ARRAY_MAX_COUNT * REQUEST_ITEM_SIZE];

/**
 * @brief MQTT request (publish, un-/subscribe) queue.
 */
QueueHandle_t xMQTTQueue;

/* Private user code ---------------------------------------------------------*/

void vMQTTInstall(void)
{
  if (xMQTTTaskHandle == NULL)
  {
    /* Initialise list holding all subscribed topics. */
    vListInitialise(&xMQTTMessageList);

    /* Create queue which handle publish and un-/subscribe. */
    xMQTTQueue = xQueueCreateStatic(MQTT_STATE_ARRAY_MAX_COUNT, REQUEST_ITEM_SIZE, ucMQTTQueueStorageArea, &xMQTTStaticQueue);

    /* ucMQTTQueueStorageArea was not NULL so xMQTTQueue should not be NULL. */
    configASSERT(xMQTTQueue);

    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the MQTT
     * broker.
     *
     * Also see https://www.freertos.org/mqtt/mqtt-agent-demo.html? for an
     * alternative run time model whereby coreMQTT runs in an autonomous
     * background agent task.  Executing the MQTT protocol in an agent task
     * removes the need for the application writer to explicitly manage any MQTT
     * state or call the MQTT_ProcessLoop() API function. Using an agent task
     * also enables multiple application tasks to more easily share a single
     * MQTT connection. */
    xTaskCreate(prvMQTTTask, "MQTT", ipconfigMQTT_STACK_SIZE_MQTT_TASK, NULL, ipconfigMQTT_PRIORITY_MQTT_TASK, &xMQTTTaskHandle);
  }
}
/*-----------------------------------------------------------*/

bool vMQTTPublish(const char *pTopicName, uint16_t topicNameLength, const void *pPayload, size_t xPayloadLength, MQTTQoS_t qos, bool retain, MQTTPublishCallback_t vPublishCallback)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Initialised to pdFALSE. */

  if (xMQTTQueue == NULL)
  {
    LogDebug(("Publish queue is not yet initialised, could not append topic '%.*s'.",
              topicNameLength, pTopicName));
    return false;
  }

  LogDebug(("Publish '%.*s' (%d bytes).",
            topicNameLength, pTopicName, xPayloadLength));

  /* New publish request. */
  MQTTRequest_t xPublishRequest;
  (void)memset((void *)&xPublishRequest, 0x00, sizeof(xPublishRequest));

  /* Subscription type is PUBLISH */
  xPublishRequest.ucPacketType = MQTT_PACKET_TYPE_PUBLISH;

  /* Duplicate message is not implemented yet */
  xPublishRequest.xDup = 0;

  /* Subscribe to the topic filter. */
  xPublishRequest.xQoS = qos;
  strncpy(xPublishRequest.ucTopic, pTopicName, MIN(topicNameLength, sizeof(xPublishRequest.ucTopic)));
  xPublishRequest.usTopicLength = strlen(xPublishRequest.ucTopic);

  xPublishRequest.pPayload = pPayload;
  xPublishRequest.xPayloadLength = xPayloadLength;
  xPublishRequest.xRetain = retain;

  /* Set user event callback function.  This can be used to free payload resources. */
  xPublishRequest.vPublishCallback = vPublishCallback;

  /* Send MQTTPublishInfo_t object.  Don't block if the queue is already full. */
  if (xQueueSendFromISR(xMQTTQueue, (void *)&xPublishRequest, &xHigherPriorityTaskWoken) != pdPASS)
  {
    /* Failed to post the message. */
    LogWarn(("Failed to append message '%s', queue is full.",
             pTopicName));
  }

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
   * xQueueSendFromISR() then a task that has a priority above the
   * priority of the currently executing task was unblocked and a context
   * switch should be performed to ensure the ISR returns to the unblocked
   * task.  In most FreeRTOS ports this is done by simply passing
   * xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
   * variables value, and perform the context switch if necessary.  Check the
   * documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return true;
}
/*-----------------------------------------------------------*/

bool vMQTTSubscribe(const char *pTopicName, uint16_t topicNameLength, MQTTQoS_t qos, MQTTPublishCallback_t vPublishCallback)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Initialised to pdFALSE. */

  if (xMQTTQueue == NULL)
  {
    LogDebug(("Subscribe queue is not yet initialised, could not append topic '%.*s'.",
              topicNameLength, pTopicName));
    return false;
  }

  LogDebug(("Subscribe '%.*s'.",
            topicNameLength, pTopicName));

  /* New subscription request. */
  MQTTRequest_t xSubscriptionRequest;
  (void)memset((void *)&xSubscriptionRequest, 0x00, sizeof(xSubscriptionRequest));

  /* Subscription type is SUBSCRIBE */
  xSubscriptionRequest.ucPacketType = MQTT_PACKET_TYPE_SUBSCRIBE;

  /* Subscribe to the topic filter. */
  xSubscriptionRequest.xQoS = qos;
  strncpy(xSubscriptionRequest.ucTopic, pTopicName, MIN(topicNameLength, sizeof(xSubscriptionRequest.ucTopic)));
  xSubscriptionRequest.usTopicLength = strlen(xSubscriptionRequest.ucTopic);

  /* Set user event callback function */
  xSubscriptionRequest.vPublishCallback = vPublishCallback;

  /* Send MQTTPublishInfo_t object.  Don't block if the queue is already full. */
  if (xQueueSendFromISR(xMQTTQueue, (void *)&xSubscriptionRequest, &xHigherPriorityTaskWoken) != pdPASS)
  {
    /* Failed to post the subscription. */
    LogWarn(("Failed to append subscription '%.*s', queue is full.",
             topicNameLength, pTopicName));
  }

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
   * xQueueSendFromISR() then a task that has a priority above the
   * priority of the currently executing task was unblocked and a context
   * switch should be performed to ensure the ISR returns to the unblocked
   * task.  In most FreeRTOS ports this is done by simply passing
   * xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
   * variables value, and perform the context switch if necessary.  Check the
   * documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return true;
}
/*-----------------------------------------------------------*/

bool vMQTTUnsubscribe(const char *pTopicName, uint16_t topicNameLength)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Initialised to pdFALSE. */

  if (xMQTTQueue == NULL)
  {
    LogDebug(("Unsubscribe queue is not yet initialised, could not append topic '%.*s'.",
              topicNameLength, pTopicName));
    return false;
  }

  LogInfo(("Unsubscribe '%.*s'.",
           topicNameLength, pTopicName));

  /* New unsubscription request. */
  MQTTRequest_t xUnsubscriptionRequest;
  (void)memset((void *)&xUnsubscriptionRequest, 0x00, sizeof(xUnsubscriptionRequest));

  /* Subscription type is UNSUBSCRIBE */
  xUnsubscriptionRequest.ucPacketType = MQTT_PACKET_TYPE_UNSUBSCRIBE;

  /* Subscribe to the topic filter. */
  strncpy(xUnsubscriptionRequest.ucTopic, pTopicName, MIN(topicNameLength, sizeof(xUnsubscriptionRequest.ucTopic)));
  xUnsubscriptionRequest.usTopicLength = strlen(xUnsubscriptionRequest.ucTopic);

  /* Send MQTTPublishInfo_t object.  Don't block if the queue is already full. */
  if (xQueueSendFromISR(xMQTTQueue, (void *)&xUnsubscriptionRequest, &xHigherPriorityTaskWoken) != pdPASS)
  {
    /* Failed to post the subscription. */
    LogWarn(("Failed to append unsubscription '%.*s', queue is full.",
             topicNameLength, pTopicName));
  }

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
   * xQueueSendFromISR() then a task that has a priority above the
   * priority of the currently executing task was unblocked and a context
   * switch should be performed to ensure the ISR returns to the unblocked
   * task.  In most FreeRTOS ports this is done by simply passing
   * xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
   * variables value, and perform the context switch if necessary.  Check the
   * documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return true;
}
/*-----------------------------------------------------------*/

static void prvMQTTTask(void *pvParameters)
{
  NetworkContext_t xNetworkContext = {0};
  PlaintextTransportParams_t xPlaintextTransportParams = {0};
  MQTTContext_t xMQTTContext;
  MQTTStatus_t xMQTTStatus;
  PlaintextTransportStatus_t xNetworkStatus;

  /* Remove compiler warnings about unused parameters. */
  (void)pvParameters;

  /* Set the pParams member of the network context with desired transport. */
  xNetworkContext.pParams = &xPlaintextTransportParams;

  /* Wait until the network is up before creating the servers.  The notification
   * is given from the network event hook 'vApplicationIPNetworkEventHook'.
   *    xTaskNotifyGive(xMQTTTaskHandle);
   */
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  ulGlobalEntryTimeMs = prvGetTimeMs();

  /* Attempt to connect to the MQTT broker. If connection fails, retry after
   * a timeout. The timeout value will exponentially increase until the
   * maximum number of attempts are reached or the maximum timeout value is
   * reached. The function below returns a failure status if the TCP connection
   * cannot be established to the broker after the configured number of attempts. */
  xNetworkStatus = prvConnectToServerWithBackoffRetries(&xNetworkContext);
  configASSERT(xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS);

  if (xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS)
  {
    /* Sends an MQTT Connect packet over the already connected TCP socket,
       * and waits for a connection acknowledgment (CONNACK) packet. */
    LogInfo(("Creating an MQTT connection to '%s:%d'.",
             ipconfigMQTT_BROKER_ENDPOINT,
             ipconfigMQTT_BROKER_PORT));
    prvCreateMQTTConnectionWithBroker(&xMQTTContext, &xNetworkContext);
    BSP_LED_On(LED_BLUE);
  }

  for (;;)
  {
    /* Check for any requests. */
    prvMQTTHandleQueue(&xMQTTContext);

    /* Process the incoming publish. */
    // xMQTTStatus = MQTT_ProcessLoop(&xMQTTContext, mqtttaskPROCESS_LOOP_TIMEOUT_MS);
    xMQTTStatus = MQTT_ReceiveLoop(&xMQTTContext, mqtttaskRECEIVE_LOOP_TIMEOUT_MS);
    configASSERT(xMQTTStatus == MQTTSuccess);
  }

  /**************************** Disconnect. *****************************/
  {
    /* Send an MQTT Disconnect packet over the connected TCP socket.
     * There is no corresponding response for a disconnect packet. After
     * sending the disconnect, the client must close the network connection. */
    LogInfo(("Disconnecting the MQTT connection with '%s:%d'.",
             ipconfigMQTT_BROKER_ENDPOINT,
             ipconfigMQTT_BROKER_PORT));
    xMQTTStatus = MQTT_Disconnect(&xMQTTContext);
    configASSERT(xMQTTStatus == MQTTSuccess);

    /* Close the network connection. */
    xNetworkStatus = Plaintext_FreeRTOS_Disconnect(&xNetworkContext);
    configASSERT(xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS);
    BSP_LED_Off(LED_BLUE);
  }
}
/*-----------------------------------------------------------*/

static void prvMQTTHandleQueue(MQTTContext_t *pxMQTTContext)
{
  /* Check for pending subscriptions in queue. */
  MQTTRequest_t xRequest;
  UBaseType_t xWaiting = uxQueueMessagesWaiting(xMQTTQueue);

  while (xQueueReceive(xMQTTQueue, &(xRequest), (TickType_t)10) == pdPASS)
  {
    UBaseType_t xPending = xWaiting - uxQueueMessagesWaiting(xMQTTQueue);
    /* xPending is not used when LIBRARY_LOG_LEVEL is below LOG_INFO. */
    (void)xPending;

    switch (xRequest.ucPacketType)
    {
    case MQTT_PACKET_TYPE_PUBLISH:
      /* Publish to given topic. */
      LogInfo(("Request (%ld/%ld): Publish to topic '%.*s'",
               xPending, xWaiting,
               xRequest.usTopicLength,
               xRequest.ucTopic));
      {
        /* Malloc new topic filter structure. */
        MQTTMessageItem_t *pxPublishItem;
        pxPublishItem = (MQTTMessageItem_t *)pvPortMalloc(sizeof(*pxPublishItem));
        (void)memset((void *)pxPublishItem, 0x00, sizeof(*pxPublishItem));

        pxPublishItem->xRequest = xRequest;
        pxPublishItem->xAckStatus = MQTTSubAckFailure;

        /* Set pointer value and append list item. */
        listSET_LIST_ITEM_OWNER(&(pxPublishItem->xListItem), (void *)pxPublishItem);
        listINSERT_END(&xMQTTMessageList, &(pxPublishItem->xListItem));

        prvMQTTMessageWithBackoffRetries(pxMQTTContext, pxPublishItem);
      }
      break;

    case MQTT_PACKET_TYPE_SUBSCRIBE:
      /* Subscribe to given topic. */
      LogInfo(("Request (%ld/%ld): Subscribe to topic '%.*s'",
               xPending, xWaiting,
               xRequest.usTopicLength,
               xRequest.ucTopic));
      {
        /* Iterate through list and check for any topic that already subscribed to. */
        const MiniListItem_t *pxEnd;
        const ListItem_t *pxIterator;
        pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
        for (pxIterator = (const ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (const ListItem_t *)pxEnd; pxIterator = (const ListItem_t *)listGET_NEXT(pxIterator))
        {
          MQTTMessageItem_t *pxSubscriptionItem;
          pxSubscriptionItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

          /* Check for topic length before strncmp for performance reasons. */
          if ((xRequest.usTopicLength == pxSubscriptionItem->xRequest.usTopicLength) &&
              (0 == strncmp(pxSubscriptionItem->xRequest.ucTopic, xRequest.ucTopic, xRequest.usTopicLength)))
          {
            LogWarn(("Already subscribed to topic '%.*s' with maximum QoS %u.",
                     xRequest.usTopicLength,
                     xRequest.ucTopic,
                     xRequest.xQoS));
            return;
          }
        }

        /* Malloc new topic filter structure. */
        MQTTMessageItem_t *pxSubscriptionItem;
        pxSubscriptionItem = (MQTTMessageItem_t *)pvPortMalloc(sizeof(*pxSubscriptionItem));
        (void)memset((void *)pxSubscriptionItem, 0x00, sizeof(*pxSubscriptionItem));

        pxSubscriptionItem->xRequest = xRequest;
        pxSubscriptionItem->xAckStatus = MQTTSubAckFailure;

        /* Set pointer value and append list item. */
        listSET_LIST_ITEM_OWNER(&(pxSubscriptionItem->xListItem), (void *)pxSubscriptionItem);
        listINSERT_END(&xMQTTMessageList, &(pxSubscriptionItem->xListItem));

        prvMQTTMessageWithBackoffRetries(pxMQTTContext, pxSubscriptionItem);
      }
      break;

    case MQTT_PACKET_TYPE_UNSUBSCRIBE:
      /* Unsubscribe from given topic. */
      LogInfo(("Request (%ld/%ld): Unsubscribe from topic '%.*s'",
               xPending, xWaiting,
               xRequest.usTopicLength,
               xRequest.ucTopic));
      {
        /* Iterate through list and check for any unsubscribed topics. */
        const MiniListItem_t *pxEnd;
        const ListItem_t *pxIterator;
        pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
        for (pxIterator = (const ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (const ListItem_t *)pxEnd; pxIterator = (const ListItem_t *)listGET_NEXT(pxIterator))
        {
          MQTTMessageItem_t *pxUnsubscriptionItem;
          pxUnsubscriptionItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

          /* Check for topic length before strncmp for performance reasons. */
          if ((xRequest.usTopicLength == pxUnsubscriptionItem->xRequest.usTopicLength) &&
              (0 == strncmp(pxUnsubscriptionItem->xRequest.ucTopic, xRequest.ucTopic, xRequest.usTopicLength)))
          {
            pxUnsubscriptionItem->xRequest.ucPacketType = MQTT_PACKET_TYPE_UNSUBSCRIBE;
            pxUnsubscriptionItem->xAckStatus = MQTTSubAckFailure;
            prvMQTTMessageWithBackoffRetries(pxMQTTContext, pxUnsubscriptionItem);
            return;
          }
        }

        LogWarn(("No subscription with topic '%.*s' found.",
                 xRequest.usTopicLength,
                 xRequest.ucTopic));
      }
      break;

    default:
      LogWarn(("Unknown subscription type for topic '%.*s'.",
               xRequest.usTopicLength,
               xRequest.ucTopic));
      break;
    }
  }
}
/*-----------------------------------------------------------*/

static PlaintextTransportStatus_t prvConnectToServerWithBackoffRetries(NetworkContext_t *pxNetworkContext)
{
  PlaintextTransportStatus_t xNetworkStatus;
  BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
  BackoffAlgorithmContext_t xReconnectParams;
  uint16_t usNextRetryBackOff = 0U;

  /* Initialize reconnect attempts and interval.*/
  BackoffAlgorithm_InitializeParams(&xReconnectParams,
                                    mqtttaskRETRY_BACKOFF_BASE_MS,
                                    mqtttaskRETRY_MAX_BACKOFF_DELAY_MS,
                                    mqtttaskRETRY_MAX_ATTEMPTS);

  /* Attempt to connect to MQTT broker. If connection fails, retry after
   * a timeout. Timeout value will exponentially increase till maximum
   * attempts are reached. */
  do
  {
    /* Establish a TCP connection with the MQTT broker. This example connects to
     * the MQTT broker as specified in ipconfigMQTT_BROKER_ENDPOINT and
     * ipconfigMQTT_BROKER_PORT at the top of this file. */
    LogInfo(("Create a TCP connection to %s:%d.",
             ipconfigMQTT_BROKER_ENDPOINT,
             ipconfigMQTT_BROKER_PORT));
    xNetworkStatus = Plaintext_FreeRTOS_Connect(pxNetworkContext,
                                                ipconfigMQTT_BROKER_ENDPOINT,
                                                ipconfigMQTT_BROKER_PORT,
                                                mqtttaskTRANSPORT_SEND_RECV_TIMEOUT_MS,
                                                mqtttaskTRANSPORT_SEND_RECV_TIMEOUT_MS);

    if (xNetworkStatus != PLAINTEXT_TRANSPORT_SUCCESS)
    {
      /* Generate a random number and calculate backoff value (in milliseconds) for
       * the next connection retry.
       * Note: It is recommended to seed the random number generator with a device-specific
       * entropy source so that possibility of multiple devices retrying failed network operations
       * at similar intervals can be avoided. */
      xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff(&xReconnectParams, uxRand(), &usNextRetryBackOff);

      if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted)
      {
        LogError(("Connection to the broker failed, all attempts exhausted."));
      }
      else if (xBackoffAlgStatus == BackoffAlgorithmSuccess)
      {
        LogWarn(("Connection to the broker failed. "
                 "Retrying connection with backoff and jitter."));
        vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));
      }
    }
  } while ((xNetworkStatus != PLAINTEXT_TRANSPORT_SUCCESS) && (xBackoffAlgStatus == BackoffAlgorithmSuccess));

  return xNetworkStatus;
}
/*-----------------------------------------------------------*/

static void prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext, NetworkContext_t *pxNetworkContext)
{
  MQTTStatus_t xResult;
  bool xSessionPresent;
  TransportInterface_t xTransport;

  /* For readability, error handling in this function is restricted to the use of
   * asserts(). */

  /* Fill in Transport Interface send and receive function pointers. */
  xTransport.pNetworkContext = pxNetworkContext;
  xTransport.send = Plaintext_FreeRTOS_send;
  xTransport.recv = Plaintext_FreeRTOS_recv;

  /* Initialize MQTT library. */
  xResult = MQTT_Init(pxMQTTContext, &xTransport, prvGetTimeMs, prvMQTTEventCallback, &xBuffer);
  configASSERT(xResult == MQTTSuccess);

  /* Many fields not used in this demo so start with everything at 0. */
  MQTTConnectInfo_t xConnectInfo;
  (void)memset((void *)&xConnectInfo, 0x00, sizeof(xConnectInfo));

  /* Start with a clean session i.e. direct the MQTT broker to discard any
   * previous session data. Also, establishing a connection with clean session
   * will ensure that the broker does not store any data when this client
   * gets disconnected. */
  xConnectInfo.cleanSession = true;

  /* The client identifier is used to uniquely identify this MQTT client to
   * the MQTT broker. In a production device the identifier can be something
   * unique, such as a device serial number. */
  xConnectInfo.pClientIdentifier = ipconfigCLIENT_IDENTIFIER;
  xConnectInfo.clientIdentifierLength = (uint16_t)strlen(ipconfigCLIENT_IDENTIFIER);

  /* Set MQTT keep-alive period. It is the responsibility of the application to ensure
   * that the interval between Control Packets being sent does not exceed the Keep Alive value.
   * In the absence of sending any other Control Packets, the Client MUST send a PINGREQ Packet. */
  xConnectInfo.keepAliveSeconds = mqtttaskKEEP_ALIVE_TIMEOUT_SECONDS;

  /* Send MQTT CONNECT packet to broker. LWT is not used in this demo, so it
   * is passed as NULL. */
  xResult = MQTT_Connect(pxMQTTContext, &xConnectInfo, NULL, mqtttaskCONNACK_RECV_TIMEOUT_MS, &xSessionPresent);
  // configASSERT(xResult == MQTTSuccess);
}
/*-----------------------------------------------------------*/

static void prvMQTTMessageWithBackoffRetries(MQTTContext_t *pxMQTTContext, MQTTMessageItem_t *pxMessageItem)
{
  MQTTStatus_t xResult = MQTTSuccess;
  BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
  BackoffAlgorithmContext_t xRetryParams;
  uint16_t usNextRetryBackOff = 0U;

  bool xRetry = false;

  // listCURRENT_LIST_LENGTH(&xMQTTMessageList)

  /* Get a unique packet id. */
  if (pxMessageItem->xRequest.ucPacketType != MQTT_PACKET_TYPE_PUBLISH || pxMessageItem->xRequest.xQoS != 0)
  {
    pxMessageItem->usPacketIdentifier = MQTT_GetPacketId(pxMQTTContext);
  }
  else
  {
    pxMessageItem->usPacketIdentifier = 0;
  }

  /* Initialize context for backoff retry attempts if SUBSCRIBE request fails. */
  BackoffAlgorithm_InitializeParams(&xRetryParams, mqtttaskRETRY_BACKOFF_BASE_MS, mqtttaskRETRY_MAX_BACKOFF_DELAY_MS, mqtttaskRETRY_MAX_ATTEMPTS);

  do
  {
    if (pxMessageItem->xAckStatus == MQTTSubAckFailure)
    {
      switch (pxMessageItem->xRequest.ucPacketType)
      {
      case MQTT_PACKET_TYPE_PUBLISH:
      {
        /* Copy list element to publish info structure. */
        MQTTPublishInfo_t xMQTTPublishInfo;

        xMQTTPublishInfo.retain = pxMessageItem->xRequest.xRetain;
        xMQTTPublishInfo.qos = pxMessageItem->xRequest.xQoS;
        xMQTTPublishInfo.dup = pxMessageItem->xRequest.xDup;

        xMQTTPublishInfo.pTopicName = pxMessageItem->xRequest.ucTopic;
        xMQTTPublishInfo.topicNameLength = pxMessageItem->xRequest.usTopicLength;

        xMQTTPublishInfo.pPayload = pxMessageItem->xRequest.pPayload;
        xMQTTPublishInfo.payloadLength = pxMessageItem->xRequest.xPayloadLength;

        /* Publishes a message to the given topic name. If QoS > 0, we will need 
         * to call MQTT_ReceiveLoop() or MQTT_ProcessLoop() to process the publish
         * acknowledgments (PUBACK). Packet Id is 0 for QoS0 */
        LogDebug(("Attempt to publish to topic '%.*s'.",
                  xMQTTPublishInfo.topicNameLength,
                  xMQTTPublishInfo.pTopicName));

        xResult = MQTT_Publish(pxMQTTContext, &xMQTTPublishInfo, pxMessageItem->usPacketIdentifier);
        configASSERT(xResult == MQTTSuccess);

        LogDebug(("PUBLISH sent for topic '%.*s' to broker.",
                  xMQTTPublishInfo.topicNameLength,
                  xMQTTPublishInfo.pTopicName));

        if (xMQTTPublishInfo.qos == 0)
        {
          /* QoS0 (packet id is zero) does not get any PUBACK (fire and forget). 
           * Request is already complete. */
          pxMessageItem->xAckStatus = MQTTSubAckComplete;
        }
      }
      break;

      case MQTT_PACKET_TYPE_SUBSCRIBE:
      {
        /* Copy list element to subscribe info structure. */
        MQTTSubscribeInfo_t xMQTTSubscribeInfo;

        xMQTTSubscribeInfo.qos = pxMessageItem->xRequest.xQoS;
        xMQTTSubscribeInfo.pTopicFilter = pxMessageItem->xRequest.ucTopic;
        xMQTTSubscribeInfo.topicFilterLength = pxMessageItem->xRequest.usTopicLength;

        /* Subscribe to the topic by sending a subscribe packet then waiting for a 
         * subscribe acknowledgment (SUBACK). */
        LogDebug(("Attempt to subscribe to topic '%.*s'.",
                  xMQTTSubscribeInfo.topicFilterLength,
                  xMQTTSubscribeInfo.pTopicFilter));

        xResult = MQTT_Subscribe(pxMQTTContext, &(xMQTTSubscribeInfo), 1, pxMessageItem->usPacketIdentifier);
        configASSERT(xResult == MQTTSuccess);

        LogDebug(("SUBSCRIBE sent for topic '%.*s' to broker.",
                  xMQTTSubscribeInfo.topicFilterLength,
                  xMQTTSubscribeInfo.pTopicFilter));
      }
      break;

      case MQTT_PACKET_TYPE_UNSUBSCRIBE:
      {
        /* Copy list element to unsubscribe info structure. */
        MQTTSubscribeInfo_t xMQTTUnsubscribeInfo;

        xMQTTUnsubscribeInfo.qos = pxMessageItem->xRequest.xQoS;
        xMQTTUnsubscribeInfo.pTopicFilter = pxMessageItem->xRequest.ucTopic;
        xMQTTUnsubscribeInfo.topicFilterLength = pxMessageItem->xRequest.usTopicLength;

        /* Unsubscribe from the topic by sending an unsubscribe packet then waiting
         * for a unsubscribe acknowledgment (UNSUBACK). */
        LogDebug(("Attempt to unsubscribe from topic '%.*s'.",
                  xMQTTUnsubscribeInfo.topicFilterLength,
                  xMQTTUnsubscribeInfo.pTopicFilter));

        xResult = MQTT_Unsubscribe(pxMQTTContext, &(xMQTTUnsubscribeInfo), 1, pxMessageItem->usPacketIdentifier);
        configASSERT(xResult == MQTTSuccess);

        LogDebug(("UNSUBSCRIBE sent for topic '%.*s' to broker.",
                  xMQTTUnsubscribeInfo.topicFilterLength,
                  xMQTTUnsubscribeInfo.pTopicFilter));
      }
      break;
      }
    }

    /* Reset flag before checking any ACK responses. */
    xRetry = false;

    if (pxMessageItem->xAckStatus == MQTTSubAckFailure || pxMessageItem->xAckStatus == MQTTSubAckProceed)
    {
      /* Process incoming packet from the broker. After sending the subscribe, the
       * client may receive a publish before it receives a subscribe ACK. Therefore,
       * call generic incoming packet processing function. */
      xResult = MQTT_ProcessLoop(pxMQTTContext, mqtttaskPROCESS_LOOP_TIMEOUT_MS);
      configASSERT(xResult == MQTTSuccess);
    }

    /* Check if recent subscription request has been rejected. #pxMessageItem->xAckStatus is updated
     * in the event callback to reflect the status of the SUBACK sent by the broker. It represents
     * either the QoS level granted by the server upon subscription, or acknowledgement of
     * server rejection of the subscription request. */
    if (pxMessageItem->xAckStatus == MQTTSubAckFailure)
    {
      xRetry = true;

      /* Generate a random number and calculate backoff value (in milliseconds) for
       * the next connection retry.
       * Note: It is recommended to seed the random number generator with a device-specific
       * entropy source so that possibility of multiple devices retrying failed network operations
       * at similar intervals can be avoided. */
      xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff(&xRetryParams, uxRand(), &usNextRetryBackOff);

      if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted)
      {
        LogError(("Server rejected request to topic '%s'. All retry attempts have exhausted and request is dropped.",
                  pxMessageItem->xRequest.ucTopic));

        /* Set acknowledge status to complete so the request gets freed. */
        pxMessageItem->xAckStatus = MQTTSubAckComplete;
      }
      else if (xBackoffAlgStatus == BackoffAlgorithmSuccess)
      {
        LogWarn(("Attempting to re-request to topic '%s'.",
                 pxMessageItem->xRequest.ucTopic));

        /* Backoff before the next re-subscribe attempt. */
        vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));
      }
    }

    /* Check if recent request is still proceeding. This is necessary for QoS 2 to receive PUBCOMP 
     * packet. */
    if (pxMessageItem->xAckStatus == MQTTSubAckProceed)
    {
      xRetry = true;
    }

    /* Check if recent request is complete and ready to get freed from memory and list.
     * Note: This is non-standard MQTT acknowledge status and used to avoid memory leakage. */
    if (pxMessageItem->xAckStatus == MQTTSubAckComplete)
    {
      /* Check for callback function. */
      if (pxMessageItem->xRequest.vPublishCallback != NULL)
      {
        pxMessageItem->xRequest.vPublishCallback(pxMessageItem, NULL);
      }
      /* Remove request and free message memory. */
      listREMOVE_ITEM(&(pxMessageItem->xListItem));
      vPortFree(pxMessageItem);
    }

  } while ((xRetry == true) && (xBackoffAlgStatus == BackoffAlgorithmSuccess));
}
/*-----------------------------------------------------------*/

static void prvMQTTEventCallback(MQTTContext_t *pxMQTTContext, MQTTPacketInfo_t *pxPacketInfo, MQTTDeserializedInfo_t *pxDeserializedInfo)
{
  /* MQTT context is not used here. */
  (void)pxMQTTContext;

  switch (pxPacketInfo->type)
  {
  case MQTT_PACKET_TYPE_PUBLISH:
    /* A PUBACK from the broker, containing the published payload. */
    {
      /* (pxPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH */
      prvMQTTProcessIncomingPublish(pxDeserializedInfo->pPublishInfo);
    }
    break;

  case MQTT_PACKET_TYPE_PUBACK:
    /* A PUBACK from the broker, is the response to a PUBLISH Packet with QoS level 1. */
    {
      prvMQTTUpdatePubAckStatus(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
    }
    break;

  case MQTT_PACKET_TYPE_PUBREC:
    /* A PUBREC from the broker, is the response to a PUBLISH Packet with QoS 2.
     * It is the second packet of the QoS 2 protocol exchange. */
    {
      prvMQTTUpdatePubRecStatus(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
    }
    break;

  case MQTT_PACKET_TYPE_PUBCOMP:
    /* A PUBCOMP from the broker, is the response to a PUBREL Packet. It is
     * the fourth and final packet of the QoS 2 protocol exchange. */
    {
      prvMQTTUpdatePubCompStatus(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
    }
    break;

  case MQTT_PACKET_TYPE_SUBACK:
    /* A SUBACK from the broker, containing the server response to our subscription request, has been received.
     * It contains the status code indicating server approval/rejection for the subscription to the single topic
     * requested. The SUBACK will be parsed to obtain the status code, and this status code will be stored in global
     * variable #xTopicFilterContext. */
    {
      prvMQTTUpdateSubAckStatus(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
    }
    break;

  case MQTT_PACKET_TYPE_UNSUBACK:
    /* A UNSUBACK from the broker, containing the server response to our subscription request, has been received.
     * It contains the status code indicating server approval/rejection for the subscription to the single topic
     * requested. The UNSUBACK will be parsed to obtain the status code, and this status code will be stored in global
     * variable #xTopicFilterContext. */
    {
      prvMQTTUpdateUnsubAckStatus(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
    }
    break;

  case MQTT_PACKET_TYPE_PINGRESP:
    /* Nothing to be done from application as library handles
     * PINGRESP with the use of MQTT_ProcessLoop API function. */
    {
      LogWarn(("PINGRESP should not be handled by the application callback when using MQTT_ProcessLoop."));
    }
    break;

  /* Any other packet type is invalid. */
  default:
    LogWarn(("prvMQTTProcessResponse() called with unknown packet type:(%02X).",
             pxPacketInfo->type));
  }
}
/*-----------------------------------------------------------*/

static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo)
{
  configASSERT(pxPublishInfo != NULL);

  /* Process incoming Publish. */
  LogInfo(("Incoming QoS: %d", pxPublishInfo->qos));

  /* Iterate through subscription list to check received topic. */
  const MiniListItem_t *pxEnd;
  const ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (const ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (const ListItem_t *)pxEnd; pxIterator = (const ListItem_t *)listGET_NEXT(pxIterator))
  {
    /* Verify the received publish is for the we have subscribed to. */
    MQTTMessageItem_t *pxPublishItem;
    pxPublishItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Check for topic length before strncmp for performance reasons. */
    if ((pxPublishInfo->topicNameLength == pxPublishItem->xRequest.usTopicLength) &&
        (0 == strncmp(pxPublishItem->xRequest.ucTopic, pxPublishInfo->pTopicName, pxPublishInfo->topicNameLength)))
    {
      /* pxPublishInfo holds subscribe structure. */
      LogInfo(("Incoming publish topic: '%.*s'",
               pxPublishInfo->topicNameLength,
               pxPublishInfo->pTopicName));
      LogInfo(("Incoming publish message: %.*s%s(%ld bytes)",
               MIN(pxPublishInfo->payloadLength, 20),
               pxPublishInfo->pPayload,
               ((pxPublishInfo->payloadLength > 20) ? "... " : " "),
               pxPublishInfo->payloadLength));

      /* Check for callback function */
      if (pxPublishItem->xRequest.vPublishCallback != NULL)
      {
        pxPublishItem->xRequest.vPublishCallback(pxPublishItem, pxPublishInfo);
      }

      /* All done, return after first topic that matches. */
      return;
    }
  }

  LogInfo(("Incoming Publish Topic Name: %.*s does not match subscribed topic.",
           pxPublishInfo->topicNameLength,
           pxPublishInfo->pTopicName));
}
/*-----------------------------------------------------------*/

static void prvMQTTUpdatePubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId)
{
  /* Iterate through list and check for missing PUBACK. */
  const MiniListItem_t *pxEnd;
  ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (ListItem_t *)pxEnd; pxIterator = (ListItem_t *)listGET_NEXT(pxIterator))
  {
    MQTTMessageItem_t *pxPublishItem;
    pxPublishItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Get element where packet identifier matches received packet id. */
    if (pxPublishItem->usPacketIdentifier == usPacketId)
    {
      LogDebug(("PUBACK for topic '%.*s'.",
                pxPublishItem->xRequest.usTopicLength,
                pxPublishItem->xRequest.ucTopic));

      /* Publish request is complete. Set acknowledge status so that
       * its memory gets freed and request gets removed from list. */
      pxPublishItem->xAckStatus = MQTTSubAckComplete;
      return;
    }
  }

  LogError(("Packet identifier %d does not match any published message.",
            usPacketId));
}
/*-----------------------------------------------------------*/

static void prvMQTTUpdatePubRecStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId)
{
  /* Iterate through list and check for missing PUBREC. */
  const MiniListItem_t *pxEnd;
  ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (ListItem_t *)pxEnd; pxIterator = (ListItem_t *)listGET_NEXT(pxIterator))
  {
    MQTTMessageItem_t *pxPublishItem;
    pxPublishItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Get element where packet identifier matches received packet id. */
    if (pxPublishItem->usPacketIdentifier == usPacketId)
    {
      LogDebug(("PUBREC for topic '%.*s'.",
                pxPublishItem->xRequest.usTopicLength,
                pxPublishItem->xRequest.ucTopic));

      /* Run process loop again to receive PUBCOMP. */
      pxPublishItem->xAckStatus = MQTTSubAckProceed;
      return;
    }
  }

  LogError(("Packet identifier %d does not match any published message.",
            usPacketId));
}
/*-----------------------------------------------------------*/

static void prvMQTTUpdatePubCompStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId)
{
  /* Iterate through list and check for missing PUBCOMP. */
  const MiniListItem_t *pxEnd;
  ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (ListItem_t *)pxEnd; pxIterator = (ListItem_t *)listGET_NEXT(pxIterator))
  {
    MQTTMessageItem_t *pxPublishItem;
    pxPublishItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Get element where packet identifier matches received packet id. */
    if (pxPublishItem->usPacketIdentifier == usPacketId)
    {
      LogDebug(("PUBCOMP for topic '%.*s'.",
                pxPublishItem->xRequest.usTopicLength,
                pxPublishItem->xRequest.ucTopic));

      /* Publish request is complete. Set acknowledge status so that
       * its memory gets freed and request gets removed from list. */
      pxPublishItem->xAckStatus = MQTTSubAckComplete;
      return;
    }
  }

  LogError(("Packet identifier %d does not match any published message.",
            usPacketId));
}
/*-----------------------------------------------------------*/

static void prvMQTTUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId)
{
  MQTTStatus_t xResult = MQTTSuccess;
  uint8_t *pucPayload = NULL;
  size_t ulSize = 0;
  uint32_t ulTopicCount = 0U;

  xResult = MQTT_GetSubAckStatusCodes(pxPacketInfo, &pucPayload, &ulSize);

  /* MQTT_GetSubAckStatusCodes always returns success if called with packet info
   * from the event callback and non-NULL parameters. */
  configASSERT(xResult == MQTTSuccess);

  /* Iterate through list and check for missing SUBACK. */
  const MiniListItem_t *pxEnd;
  const ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (const ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (const ListItem_t *)pxEnd; pxIterator = (const ListItem_t *)listGET_NEXT(pxIterator))
  {
    MQTTMessageItem_t *pxSubscriptionItem;
    pxSubscriptionItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Get element where packet identifier matches received packet id. */
    if (pxSubscriptionItem->usPacketIdentifier == usPacketId)
    {
      /* Get QoS as new subscription status. */
      pxSubscriptionItem->xAckStatus = pucPayload[ulTopicCount++];

      LogDebug(("SUBACK for topic '%.*s' with maximum QoS %u.",
                pxSubscriptionItem->xRequest.usTopicLength,
                pxSubscriptionItem->xRequest.ucTopic,
                pxSubscriptionItem->xAckStatus));
      return;
    }
  }

  LogError(("Packet identifier %d does not match any subscription.",
            usPacketId));
}
/*-----------------------------------------------------------*/

static void prvMQTTUpdateUnsubAckStatus(MQTTPacketInfo_t *pxPacketInfo, uint16_t usPacketId)
{
  /* Iterate through list and check for missing UNSUBACK. */
  const MiniListItem_t *pxEnd;
  ListItem_t *pxIterator;
  pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xMQTTMessageList);
  for (pxIterator = (ListItem_t *)listGET_NEXT(pxEnd); pxIterator != (ListItem_t *)pxEnd; pxIterator = (ListItem_t *)listGET_NEXT(pxIterator))
  {
    MQTTMessageItem_t *pxUnsubscriptionItem;
    pxUnsubscriptionItem = (MQTTMessageItem_t *)listGET_LIST_ITEM_OWNER(pxIterator);

    /* Get element where packet identifier matches received packet id. */
    if (pxUnsubscriptionItem->usPacketIdentifier == usPacketId)
    {
      LogDebug(("UNSUBACK for topic '%.*s'.",
                pxUnsubscriptionItem->xRequest.usTopicLength,
                pxUnsubscriptionItem->xRequest.ucTopic));

      /* Unsubscription request is complete. Set acknowledge status so that
       * its memory gets freed and request gets removed from list */
      pxUnsubscriptionItem->xAckStatus = MQTTSubAckComplete;
      return;
    }
  }

  LogError(("Packet identifier %d does not match any subscription.",
            usPacketId));
}
/*-----------------------------------------------------------*/

static uint32_t prvGetTimeMs(void)
{
  TickType_t xTickCount = 0;
  uint32_t ulTimeMs = 0UL;

  /* Get the current tick count. */
  xTickCount = xTaskGetTickCount();

  /* Convert the ticks to milliseconds. */
  ulTimeMs = (uint32_t)xTickCount * MILLISECONDS_PER_TICK;

  /* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
   * elapsed time in the application. */
  ulTimeMs = (uint32_t)(ulTimeMs - ulGlobalEntryTimeMs);

  return ulTimeMs;
}
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
