/**
 * @file mqtt_task.h
 * @author horsche (horsche@li.plus)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTT_TASK_H
#define __MQTT_TASK_H

/* Includes ------------------------------------------------------------------*/
/* MQTT library includes. */
#include "core_mqtt.h"

/* Public define -------------------------------------------------------------*/
/**
 * @brief Maximum size of subscribed topic.  This defines the topic char buffer.
 */
#define mqtttaskMAX_TOPIC_LENGTH (16)

/* Public typedef ------------------------------------------------------------*/
struct MQTTSubscription;
struct MQTTMessageItem;

/**
 * @brief Callback function which is fired on publishing completed or on received
 * payload for subscribed topic.
 * 
 * @param[in] pxMessageItem List element containing topic information.
 * @param[in] pxPublishInfo is a pointer to structure containing deserialized
 * Publish message. This pointer is NULL for publish/unsubsribe complete callbacks.
 */
typedef void (*MQTTPublishCallback_t)(struct MQTTMessageItem *pxMessageItem, struct MQTTPublishInfo *pxPublishInfo);

/**
 * @brief Subsciption structure holding information for subscribed topics.
 */
typedef struct MQTTSubscription
{
  char ucTopic[mqtttaskMAX_TOPIC_LENGTH]; /**< @brief Buffer holding subscribed topic. */
  uint16_t usTopicLength;                 /**< @brief Length of subscription topic filter. */
  MQTTQoS_t xQoS;                         /**< @brief Quality of Service for subscription. */

  const void *pPayload;  /**< @brief Message payload. */
  size_t xPayloadLength; /**< @brief Message payload length. */
  bool xRetain;          /**< @brief Whether this is a retained message. */
  bool xDup;             /**< @brief Whether this is a duplicate publish message. */

  MQTTPublishCallback_t vPublishCallback; /**< @brief Callback function on incoming data. */
  uint8_t ucPacketType;                   /**< @brief MQTT_PACKET_TYPE_PUBLISH, MQTT_PACKET_TYPE_UNSUBSCRIBE or MQTT_PACKET_TYPE_SUBSCRIBE */

} MQTTRequest_t;

/**
 * @brief Single list element containing current subscription status and packet 
 * identifier.
 */
typedef struct MQTTMessageItem
{
  uint16_t usPacketIdentifier;   /**< @brief Packet identifier to match SUBACK and UNSUBACK messages. */
  MQTTSubAckStatus_t xAckStatus; /**< @brief Acknowledge status, MQTTSubAckFailure for ongoing subscription or QoS on success. */

  MQTTRequest_t xRequest; /**< @brief Actual subscription structure holding topic, QoS and callback function. */

  struct xLIST_ITEM xListItem; /* With this item the client will be bound to a List_t. */
} MQTTMessageItem_t;

/* Exported variables --------------------------------------------------------*/
/**
 * @brief MQTT task handle to avoid task getting started multiple times. Notification
 * is given in vApplicationIPNetworkEventHook as soon the network is up.
 */
extern TaskHandle_t xMQTTTaskHandle;

/* Exported functions --------------------------------------------------------*/

void vMQTTInstall(void);

bool vMQTTPublish(const char *pTopicName, uint16_t topicNameLength, const void *pPayload, size_t xPayloadLength, MQTTQoS_t qos, bool retain, MQTTPublishCallback_t vPublishCallback);
bool vMQTTSubscribe(const char *pTopicName, uint16_t topicNameLength, MQTTQoS_t qos, MQTTPublishCallback_t vPublishCallback);
bool vMQTTUnsubscribe(const char *pTopicName, uint16_t topicNameLength);

#endif /* __MQTT_TASK_H */

/***************************** END OF FILE ************************************/
