/**
 * @file mqtt_task.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-25
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
#define ipconfigCLIENT_IDENTIFIER "testClient"__TIME__
#endif

#ifndef ipconfigMQTT_STACK_SIZE_MQTT_TASK
/**
 * @brief Stack size needed for prvMQTTDemoTask(), a bit of a guess.
 */
#define ipconfigMQTT_STACK_SIZE_MQTT_TASK (configMINIMAL_STACK_SIZE * 10)
#endif

#ifndef ipconfigMQTT_PRIORITY_MQTT_TASK
/**
 * @brief The priority of prvMQTTDemoTask(). Should be lower than the 
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
 * @brief The port to use for the demo.
 *
 * #define ipconfigMQTT_BROKER_PORT         ( insert here. )
 */
#error "Using default MQTT port number '1883'. Define ipconfigMQTT_BROKER_PORT in FreeRTOSIPConfig.h to avoid this warning."
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
 * @brief The topic to subscribe and publish to in the example.
 *
 * The topic name starts with the client identifier to ensure that each demo
 * interacts with a unique topic name.
 */
#define mqtttaskTOPIC ipconfigCLIENT_IDENTIFIER "status"

/**
 * @brief The number of topic filters to subscribe.
 */
#define mqtttaskTOPIC_COUNT (1)

/**
 * @brief The MQTT message published in this example.
 */
#define mqtttaskMESSAGE "Hello World!"

/**
 * @brief Dimensions a file scope buffer currently used to send and receive MQTT data
 * from a socket.
 */
#define mqtttaskSHARED_BUFFER_SIZE (500U)

/**
 * @brief Time to wait between each cycle of the demo implemented by prvMQTTDemoTask().
 */
#define mqtttaskDELAY_BETWEEN_DEMO_ITERATIONS (pdMS_TO_TICKS(5000U))

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define mqtttaskPROCESS_LOOP_TIMEOUT_MS (500U)

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
 * @brief Delay between MQTT publishes. Note that the process loop also has a
 * timeout, so the total time between publishes is the sum of the two delays.
 */
#define mqtttaskDELAY_BETWEEN_PUBLISHES (pdMS_TO_TICKS(500U))

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define mqtttaskTRANSPORT_SEND_RECV_TIMEOUT_MS (200U)

#define MILLISECONDS_PER_SECOND (1000U)                                      /**< @brief Milliseconds per second. */
#define MILLISECONDS_PER_TICK (MILLISECONDS_PER_SECOND / configTICK_RATE_HZ) /**< Milliseconds per FreeRTOS tick. */

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
 * @brief The task used to demonstrate the MQTT API.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvMQTTDemoTask(void *pvParameters);

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
static void prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext,
                                              NetworkContext_t *pxNetworkContext);

/**
 * @brief Function to update variable #xTopicFilterContext with status
 * information from Subscribe ACK. Called by the event callback after processing
 * an incoming SUBACK packet.
 *
 * @param[in] Server response to the subscription request.
 */
static void prvUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo);

/**
 * @brief Subscribes to the topic as specified in mqtttaskTOPIC at the top of
 * this file. In the case of a Subscribe ACK failure, then subscription is
 * retried using an exponential backoff strategy with jitter.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 */
static void prvMQTTSubscribeWithBackoffRetries(MQTTContext_t *pxMQTTContext);

/**
 * @brief Publishes a message mqtttaskMESSAGE on mqtttaskTOPIC topic.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 */
static void prvMQTTPublishToTopic(MQTTContext_t *pxMQTTContext);

/**
 * @brief Unsubscribes from the previously subscribed topic as specified
 * in mqtttaskTOPIC.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 */
static void prvMQTTUnsubscribeFromTopic(MQTTContext_t *pxMQTTContext);

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t prvGetTimeMs(void);

/**
 * @brief Process a response or ack to an MQTT request (PING, SUBSCRIBE
 * or UNSUBSCRIBE). This function processes PINGRESP, SUBACK, and UNSUBACK.
 *
 * @param[in] pxIncomingPacket is a pointer to structure containing deserialized
 * MQTT response.
 * @param[in] usPacketId is the packet identifier from the ack received.
 */
static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket,
                                   uint16_t usPacketId);

/**
 * @brief Process incoming Publish message.
 *
 * @param[in] pxPublishInfo is a pointer to structure containing deserialized
 * Publish message.
 */
static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo);

/**
 * @brief The application callback function for getting the incoming publish
 * and incoming acks reported from the MQTT library.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] pxDeserializedInfo Deserialized information from the incoming packet.
 */
static void prvEventCallback(MQTTContext_t *pxMQTTContext,
                             MQTTPacketInfo_t *pxPacketInfo,
                             MQTTDeserializedInfo_t *pxDeserializedInfo);

/* Private variables ---------------------------------------------------------*/

/**
 * @brief MQTT Demo task handle to avoid task getting started multiple times.
 */
static TaskHandle_t pxMQTTDemoTask;

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucSharedBuffer[mqtttaskSHARED_BUFFER_SIZE];

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

/**
 * @brief Packet Identifier generated when Subscribe request was sent to the broker;
 * it is used to match received Subscribe ACK to the transmitted ACK.
 */
static uint16_t usSubscribePacketIdentifier;

/**
 * @brief Packet Identifier generated when Unsubscribe request was sent to the broker;
 * it is used to match received Unsubscribe response to the transmitted unsubscribe
 * request.
 */
static uint16_t usUnsubscribePacketIdentifier;

/**
 * @brief A pair containing a topic filter and its SUBACK status.
 */
typedef struct topicFilterContext
{
  const char *pcTopicFilter;
  MQTTSubAckStatus_t xSubAckStatus;
} topicFilterContext_t;

/**
 * @brief An array containing the context of a SUBACK; the SUBACK status
 * of a filter is updated when the event callback processes a SUBACK.
 */
static topicFilterContext_t xTopicFilterContext[mqtttaskTOPIC_COUNT] = {
    {mqtttaskTOPIC, MQTTSubAckFailure}};

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static MQTTFixedBuffer_t xBuffer = {
    .pBuffer = ucSharedBuffer,
    .size = mqtttaskSHARED_BUFFER_SIZE};

/* Private user code ---------------------------------------------------------*/

void vMQTTInstall(void)
{
  if (pxMQTTDemoTask == NULL)
  {
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
    xTaskCreate(prvMQTTDemoTask, "MQTT", ipconfigMQTT_STACK_SIZE_MQTT_TASK, NULL, ipconfigMQTT_PRIORITY_MQTT_TASK, &pxMQTTDemoTask);
  }
}
/*-----------------------------------------------------------*/

static void prvMQTTDemoTask(void *pvParameters)
{
  uint32_t ulPublishCount = 0U, ulTopicCount = 0U;
  const uint32_t ulMaxPublishCount = 5UL;
  NetworkContext_t xNetworkContext = {0};
  PlaintextTransportParams_t xPlaintextTransportParams = {0};
  MQTTContext_t xMQTTContext;
  MQTTStatus_t xMQTTStatus;
  PlaintextTransportStatus_t xNetworkStatus;

  /* Remove compiler warnings about unused parameters. */
  (void)pvParameters;

  /* Set the pParams member of the network context with desired transport. */
  xNetworkContext.pParams = &xPlaintextTransportParams;

  ulGlobalEntryTimeMs = prvGetTimeMs();

  for (;;)
  {
    /****************************** Connect. ******************************/

    /* Attempt to connect to the MQTT broker. If connection fails, retry after
     * a timeout. The timeout value will exponentially increase until the
     * maximum number of attempts are reached or the maximum timeout value is
     * reached. The function below returns a failure status if the TCP connection
     * cannot be established to the broker after the configured number of attempts. */
    xNetworkStatus = prvConnectToServerWithBackoffRetries(&xNetworkContext);
    // configASSERT(xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS);
    if (xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS)
    {
      BSP_LED_On(LED_BLUE);

      vTaskDelay(mqtttaskDELAY_BETWEEN_DEMO_ITERATIONS);

      /* Sends an MQTT Connect packet over the already connected TCP socket,
     * and waits for a connection acknowledgment (CONNACK) packet. */
      LogInfo(("Creating an MQTT connection to %s.", ipconfigMQTT_BROKER_ENDPOINT));
      prvCreateMQTTConnectionWithBroker(&xMQTTContext, &xNetworkContext);

      /**************************** Subscribe. ******************************/

      // /* If server rejected the subscription request, attempt to resubscribe to
      //  * the topic. Attempts are made according to the exponential backoff retry
      //  * strategy declared in backoff_algorithm.h. */
      // prvMQTTSubscribeWithBackoffRetries(&xMQTTContext);

      /******************* Publish and Keep Alive Loop. *********************/
      /* Publish messages with QoS0, then send and process Keep Alive messages. */
      for (ulPublishCount = 0; ulPublishCount < ulMaxPublishCount; ulPublishCount++)
      {
        LogInfo(("Publish to the MQTT topic %s.", mqtttaskTOPIC));
        prvMQTTPublishToTopic(&xMQTTContext);

        // /* Process the incoming publish echo. Since the application subscribed
        //  * to the same topic, the broker will send the same publish message
        //  * back to the application. */
        // LogInfo(("Attempt to receive publish message from broker."));
        // xMQTTStatus = MQTT_ProcessLoop(&xMQTTContext,
        //                                mqtttaskPROCESS_LOOP_TIMEOUT_MS);
        // configASSERT(xMQTTStatus == MQTTSuccess);

        /* Leave the connection idle for some time. */
        LogInfo(("Keeping Connection Idle..."));
        vTaskDelay(mqtttaskDELAY_BETWEEN_PUBLISHES);
      }

      /******************** Unsubscribe from the topic. *********************/
      // LogInfo(("Unsubscribe from the MQTT topic %s.", mqtttaskTOPIC));
      // prvMQTTUnsubscribeFromTopic(&xMQTTContext);

      // /* Process the incoming packet from the broker. */
      // xMQTTStatus = MQTT_ProcessLoop(&xMQTTContext,
      //                                mqtttaskPROCESS_LOOP_TIMEOUT_MS);
      // configASSERT(xMQTTStatus == MQTTSuccess);

      /**************************** Disconnect. *****************************/

      /* Send an MQTT Disconnect packet over the connected TCP socket.
     * There is no corresponding response for a disconnect packet. After
     * sending the disconnect, the client must close the network connection. */
      LogInfo(("Disconnecting the MQTT connection with %s.",
               ipconfigMQTT_BROKER_ENDPOINT));
      xMQTTStatus = MQTT_Disconnect(&xMQTTContext);
      configASSERT(xMQTTStatus == MQTTSuccess);

      /* Close the network connection. */
      xNetworkStatus = Plaintext_FreeRTOS_Disconnect(&xNetworkContext);
      configASSERT(xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS);
      BSP_LED_Off(LED_BLUE);
    }

    /* Reset SUBACK status for each topic filter after completion of
     * subscription request cycle. */
    for (ulTopicCount = 0; ulTopicCount < mqtttaskTOPIC_COUNT; ulTopicCount++)
    {
      xTopicFilterContext[ulTopicCount].xSubAckStatus = MQTTSubAckFailure;
    }

    /* Wait for some time between two iterations to ensure that we do not
     * bombard the MQTT broker. */
    LogInfo(("prvMQTTDemoTask() completed an iteration successfully. "
             "Total free heap is %u.",
             xPortGetFreeHeapSize()));
    vTaskDelay(mqtttaskDELAY_BETWEEN_DEMO_ITERATIONS);
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

static void prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext,
                                              NetworkContext_t *pxNetworkContext)
{
  MQTTStatus_t xResult;
  MQTTConnectInfo_t xConnectInfo;
  bool xSessionPresent;
  TransportInterface_t xTransport;

  /* For readability, error handling in this function is restricted to the use of
   * asserts(). */

  /* Fill in Transport Interface send and receive function pointers. */
  xTransport.pNetworkContext = pxNetworkContext;
  xTransport.send = Plaintext_FreeRTOS_send;
  xTransport.recv = Plaintext_FreeRTOS_recv;

  /* Initialize MQTT library. */
  xResult = MQTT_Init(pxMQTTContext, &xTransport, prvGetTimeMs, prvEventCallback, &xBuffer);
  configASSERT(xResult == MQTTSuccess);

  /* Many fields not used in this demo so start with everything at 0. */
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
  xResult = MQTT_Connect(pxMQTTContext,
                         &xConnectInfo,
                         NULL,
                         mqtttaskCONNACK_RECV_TIMEOUT_MS,
                         &xSessionPresent);
  // configASSERT(xResult == MQTTSuccess);
}
/*-----------------------------------------------------------*/

static void prvUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo)
{
  MQTTStatus_t xResult = MQTTSuccess;
  uint8_t *pucPayload = NULL;
  size_t ulSize = 0;
  uint32_t ulTopicCount = 0U;

  xResult = MQTT_GetSubAckStatusCodes(pxPacketInfo, &pucPayload, &ulSize);

  /* MQTT_GetSubAckStatusCodes always returns success if called with packet info
   * from the event callback and non-NULL parameters. */
  configASSERT(xResult == MQTTSuccess);

  for (ulTopicCount = 0; ulTopicCount < ulSize; ulTopicCount++)
  {
    xTopicFilterContext[ulTopicCount].xSubAckStatus = pucPayload[ulTopicCount];
  }
}
/*-----------------------------------------------------------*/

static void prvMQTTSubscribeWithBackoffRetries(MQTTContext_t *pxMQTTContext)
{
  MQTTStatus_t xResult = MQTTSuccess;
  BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
  BackoffAlgorithmContext_t xRetryParams;
  uint16_t usNextRetryBackOff = 0U;
  MQTTSubscribeInfo_t xMQTTSubscription[mqtttaskTOPIC_COUNT];
  bool xFailedSubscribeToTopic = false;
  uint32_t ulTopicCount = 0U;

  /* Some fields not used by this demo so start with everything at 0. */
  (void)memset((void *)&xMQTTSubscription, 0x00, sizeof(xMQTTSubscription));

  /* Get a unique packet id. */
  usSubscribePacketIdentifier = MQTT_GetPacketId(pxMQTTContext);

  /* Subscribe to the mqtttaskTOPIC topic filter. This example subscribes to
   * only one topic and uses QoS0. */
  xMQTTSubscription[0].qos = MQTTQoS0;
  xMQTTSubscription[0].pTopicFilter = mqtttaskTOPIC;
  xMQTTSubscription[0].topicFilterLength = (uint16_t)strlen(mqtttaskTOPIC);

  /* Initialize context for backoff retry attempts if SUBSCRIBE request fails. */
  BackoffAlgorithm_InitializeParams(&xRetryParams,
                                    mqtttaskRETRY_BACKOFF_BASE_MS,
                                    mqtttaskRETRY_MAX_BACKOFF_DELAY_MS,
                                    mqtttaskRETRY_MAX_ATTEMPTS);

  do
  {
    /* The client is now connected to the broker. Subscribe to the topic
     * as specified in mqtttaskTOPIC at the top of this file by sending a
     * subscribe packet then waiting for a subscribe acknowledgment (SUBACK).
     * This client will then publish to the same topic it subscribed to, so it
     * will expect all the messages it sends to the broker to be sent back to it
     * from the broker. This demo uses QOS0 in Subscribe, therefore, the Publish
     * messages received from the broker will have QOS0. */
    LogInfo(("Attempt to subscribe to the MQTT topic %s.", mqtttaskTOPIC));
    xResult = MQTT_Subscribe(pxMQTTContext,
                             xMQTTSubscription,
                             sizeof(xMQTTSubscription) / sizeof(MQTTSubscribeInfo_t),
                             usSubscribePacketIdentifier);
    configASSERT(xResult == MQTTSuccess);

    LogInfo(("SUBSCRIBE sent for topic %s to broker.", mqtttaskTOPIC));

    /* Process incoming packet from the broker. After sending the subscribe, the
     * client may receive a publish before it receives a subscribe ack. Therefore,
     * call generic incoming packet processing function. Since this demo is
     * subscribing to the topic to which no one is publishing, probability of
     * receiving Publish message before subscribe ack is zero; but application
     * must be ready to receive any packet.  This demo uses the generic packet
     * processing function everywhere to highlight this fact. */
    xResult = MQTT_ProcessLoop(pxMQTTContext, mqtttaskPROCESS_LOOP_TIMEOUT_MS);
    configASSERT(xResult == MQTTSuccess);

    /* Reset flag before checking suback responses. */
    xFailedSubscribeToTopic = false;

    /* Check if recent subscription request has been rejected. #xTopicFilterContext is updated
     * in the event callback to reflect the status of the SUBACK sent by the broker. It represents
     * either the QoS level granted by the server upon subscription, or acknowledgement of
     * server rejection of the subscription request. */
    for (ulTopicCount = 0; ulTopicCount < mqtttaskTOPIC_COUNT; ulTopicCount++)
    {
      if (xTopicFilterContext[ulTopicCount].xSubAckStatus == MQTTSubAckFailure)
      {
        xFailedSubscribeToTopic = true;

        /* Generate a random number and calculate backoff value (in milliseconds) for
         * the next connection retry.
         * Note: It is recommended to seed the random number generator with a device-specific
         * entropy source so that possibility of multiple devices retrying failed network operations
         * at similar intervals can be avoided. */
        xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff(&xRetryParams, uxRand(), &usNextRetryBackOff);

        if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted)
        {
          LogError(("Server rejected subscription request. All retry attempts have exhausted. Topic=%s",
                    xTopicFilterContext[ulTopicCount].pcTopicFilter));
        }
        else if (xBackoffAlgStatus == BackoffAlgorithmSuccess)
        {
          LogWarn(("Server rejected subscription request. Attempting to re-subscribe to topic %s.",
                   xTopicFilterContext[ulTopicCount].pcTopicFilter));
          /* Backoff before the next re-subscribe attempt. */
          vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));
        }

        break;
      }
    }

    configASSERT(xBackoffAlgStatus != BackoffAlgorithmRetriesExhausted);
  } while ((xFailedSubscribeToTopic == true) && (xBackoffAlgStatus == BackoffAlgorithmSuccess));
}
/*-----------------------------------------------------------*/

static uint64_t runtime;
static void prvMQTTPublishToTopic(MQTTContext_t *pxMQTTContext)
{
  MQTTStatus_t xResult;
  MQTTPublishInfo_t xMQTTPublishInfo;

  /* For readability, error handling in this function is restricted to the use of
   * asserts(). */

  /* Some fields are not used by this demo so start with everything at 0. */
  (void)memset((void *)&xMQTTPublishInfo, 0x00, sizeof(xMQTTPublishInfo));

  /* This demo uses QoS0. */
  xMQTTPublishInfo.qos = MQTTQoS0;
  xMQTTPublishInfo.retain = false;
  xMQTTPublishInfo.pTopicName = mqtttaskTOPIC;
  xMQTTPublishInfo.topicNameLength = (uint16_t)strlen(mqtttaskTOPIC);

  runtime = ullGetMicrosecondTime();
  xMQTTPublishInfo.pPayload = &runtime;
  xMQTTPublishInfo.payloadLength = sizeof(runtime);
  // xMQTTPublishInfo.pPayload = mqtttaskMESSAGE;
  // xMQTTPublishInfo.payloadLength = strlen(mqtttaskMESSAGE);

  /* Send PUBLISH packet. Packet ID is not used for a QoS0 publish. */
  xResult = MQTT_Publish(pxMQTTContext, &xMQTTPublishInfo, 0U);

  configASSERT(xResult == MQTTSuccess);
}
/*-----------------------------------------------------------*/

static void prvMQTTUnsubscribeFromTopic(MQTTContext_t *pxMQTTContext)
{
  MQTTStatus_t xResult;
  MQTTSubscribeInfo_t xMQTTSubscription[mqtttaskTOPIC_COUNT];

  /* Some fields not used by this demo so start with everything at 0. */
  (void)memset((void *)&xMQTTSubscription, 0x00, sizeof(xMQTTSubscription));

  /* Get a unique packet id. */
  usSubscribePacketIdentifier = MQTT_GetPacketId(pxMQTTContext);

  /* Subscribe to the mqtttaskTOPIC topic filter. This example subscribes to
   * only one topic and uses QoS0. */
  xMQTTSubscription[0].qos = MQTTQoS0;
  xMQTTSubscription[0].pTopicFilter = mqtttaskTOPIC;
  xMQTTSubscription[0].topicFilterLength = (uint16_t)strlen(mqtttaskTOPIC);

  /* Get next unique packet identifier. */
  usUnsubscribePacketIdentifier = MQTT_GetPacketId(pxMQTTContext);

  /* Send UNSUBSCRIBE packet. */
  xResult = MQTT_Unsubscribe(pxMQTTContext,
                             xMQTTSubscription,
                             sizeof(xMQTTSubscription) / sizeof(MQTTSubscribeInfo_t),
                             usUnsubscribePacketIdentifier);

  configASSERT(xResult == MQTTSuccess);
}
/*-----------------------------------------------------------*/

static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket,
                                   uint16_t usPacketId)
{
  uint32_t ulTopicCount = 0U;

  switch (pxIncomingPacket->type)
  {
  case MQTT_PACKET_TYPE_SUBACK:

    /* A SUBACK from the broker, containing the server response to our subscription request, has been received.
     * It contains the status code indicating server approval/rejection for the subscription to the single topic
     * requested. The SUBACK will be parsed to obtain the status code, and this status code will be stored in global
     * variable #xTopicFilterContext. */
    prvUpdateSubAckStatus(pxIncomingPacket);

    for (ulTopicCount = 0; ulTopicCount < mqtttaskTOPIC_COUNT; ulTopicCount++)
    {
      if (xTopicFilterContext[ulTopicCount].xSubAckStatus != MQTTSubAckFailure)
      {
        LogInfo(("Subscribed to the topic %s with maximum QoS %u.",
                 xTopicFilterContext[ulTopicCount].pcTopicFilter,
                 xTopicFilterContext[ulTopicCount].xSubAckStatus));
      }
    }

    /* Make sure ACK packet identifier matches with Request packet identifier. */
    configASSERT(usSubscribePacketIdentifier == usPacketId);
    break;

  case MQTT_PACKET_TYPE_UNSUBACK:
    LogInfo(("Unsubscribed from the topic %s.", mqtttaskTOPIC));
    /* Make sure ACK packet identifier matches with Request packet identifier. */
    configASSERT(usUnsubscribePacketIdentifier == usPacketId);
    break;

  case MQTT_PACKET_TYPE_PINGRESP:

    /* Nothing to be done from application as library handles
     * PINGRESP with the use of MQTT_ProcessLoop API function. */
    LogWarn(("PINGRESP should not be handled by the application "
             "callback when using MQTT_ProcessLoop."));
    break;

  /* Any other packet type is invalid. */
  default:
    LogWarn(("prvMQTTProcessResponse() called with unknown packet type:(%02X).",
             pxIncomingPacket->type));
  }
}

/*-----------------------------------------------------------*/

static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo)
{
  configASSERT(pxPublishInfo != NULL);

  /* Process incoming Publish. */
  LogInfo(("Incoming QoS : %d", pxPublishInfo->qos));

  /* Verify the received publish is for the we have subscribed to. */
  if ((pxPublishInfo->topicNameLength == strlen(mqtttaskTOPIC)) &&
      (0 == strncmp(mqtttaskTOPIC, pxPublishInfo->pTopicName, pxPublishInfo->topicNameLength)))
  {
    LogInfo(("Incoming Publish Topic Name: %.*s matches subscribed topic."
             "Incoming Publish Message : %.*s",
             pxPublishInfo->topicNameLength,
             pxPublishInfo->pTopicName,
             pxPublishInfo->payloadLength,
             pxPublishInfo->pPayload));
  }
  else
  {
    LogInfo(("Incoming Publish Topic Name: %.*s does not match subscribed topic.",
             pxPublishInfo->topicNameLength,
             pxPublishInfo->pTopicName));
  }
}

/*-----------------------------------------------------------*/

static void prvEventCallback(MQTTContext_t *pxMQTTContext,
                             MQTTPacketInfo_t *pxPacketInfo,
                             MQTTDeserializedInfo_t *pxDeserializedInfo)
{
  /* The MQTT context is not used for this demo. */
  (void)pxMQTTContext;

  if ((pxPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
  {
    prvMQTTProcessIncomingPublish(pxDeserializedInfo->pPublishInfo);
  }
  else
  {
    prvMQTTProcessResponse(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
  }
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
