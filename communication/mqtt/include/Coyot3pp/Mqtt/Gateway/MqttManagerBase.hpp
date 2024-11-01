/**
* 
* 
* - CYT-TOOLS -
*   File: MqttManagerBase.h
*   2021 
*   Project :
*
* 
* 
*/

#pragma once

#include <string>

#include "MqttGateway.hpp"

#include <functional>
#include <thread>
#include <algorithm>
#include <map>
#include <jsoncpp/json/json.h>
#include <ostream>

#include <QObject>

#include <QTimer>


#include <Coyot3pp/Cor3/JsonSerializablePacketBase.hpp>
#include "MqttGatewayAcrp.hpp"
#include "AcrpDataPacket.hpp"
#include "AcrpTopicManagementInfoStack.hpp"

namespace coyot3{
namespace communication{
namespace mqtt{


/**
 * @brief : base class for the managers using an mqtt connection to wrap a
 *            communication API. Uses a common MqttGateway instance.
 * */
class MqttManagerBase : public QObject{
  
  Q_OBJECT
    
    friend class MqttGateway;

    public:




      struct AcrpConfig{
        bool        acrp_is_active;
        int64_t     acrp_timeout_packet_resend;
        int64_t     acrp_timeout_packet_send_failed;
        int64_t     acrp_automatic_resend_time;
        int64_t     acrp_timeout_receiver_alive_checkpacket;
        int64_t     acrp_noprot_msg_noactivity_timeout;
        int64_t     acrp_publish_dynop_interval;
        std::string acrp_default_posftix_request;
        std::string acrp_default_postfix_response;


        struct JsField{
          static const char* acrp_is_active;
          static const char* acrp_timeout_packet_resend;
          static const char* acrp_timeout_packet_send_failed;
          static const char* acrp_automatic_resend_time;
          static const char* acrp_timeout_receiver_alive_checkpacket;
          static const char* acrp_noprot_msg_noactivity_timeout; // dynop no activity timeout
          static const char* acrp_publish_dynop_interval;
          static const char* acrp_default_posftix_request;
          static const char* acrp_default_postfix_response;
        };

        struct Defaults{
        static const bool    acrp_is_active;
        static const char*   acrp_default_posftix_request;
        static const char*   acrp_default_postfix_response;
        static const int64_t acrp_timeout_packet_resend;
        static const int64_t acrp_timeout_packet_send_failed;
        static const int64_t acrp_automatic_resend_time;
        static const int64_t acrp_timeout_receiver_alive_checkpacket;
        static const int64_t acrp_noprot_msg_noactivity_timeout;
        static const int64_t acrp_publish_dynop_interval;
      };

        AcrpConfig();
        AcrpConfig(const AcrpConfig& o);
        virtual ~AcrpConfig();

        AcrpConfig& operator=(const AcrpConfig& o);
        bool    operator==(const AcrpConfig& o);

        bool    fromJson(const Json::Value& o);
        Json::Value toJson() const;

        bool    setAtmiConfig(AcrpTopicManagementInfo::Config& cfg);

      };
      
      enum class State{
         CREATED = 0
        ,INITIATED = 1
        ,STARTED = 2
        ,ERROR = 3
      };
      static const char* StateToString(State s);



      
    protected:
      MqttManagerBase(const std::string& name,QObject* parent = nullptr);
    
    public:

      
      virtual ~MqttManagerBase();
      std::string& Name();

      virtual bool Init()  = 0;
      virtual bool Start() = 0;
      virtual bool Stop()  = 0;



      bool         create_own_gateway(const Json::Value& config);
      virtual bool set_gateway(MqttGateway* gw);
      bool         unset_gateway();

      
      bool acrpIsActive();
      bool acrpIsActive(bool state);
      bool setAcrpConfig(const Json::Value& config);
    

      void debug_level_set(int level);
      void mqttGwDebugLevelSet(int debugLevel);

    signals:

      void managerError();

    
    protected:
      MqttGateway*      gateway;
      bool              gateway_owner;

      std::string                  manager_name;
      State                        state;

      int64_t           birth_ts;

      AcrpTopicManagementInfoStack tmiStack;
      /**
       * @brief : meant to be used by the user class, register a publisher
       * */
      bool acrp_register_publisher(const std::string& topic_base
                                   ,bool is_active
                                   ,const std::string& topic_postfix_req = std::string()
                                   ,const std::string& topic_postfix_res = std::string());
      
      bool acrp_register_publisher(const std::string& topic_base
                                  ,bool is_active
                                  ,AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType onPublishResultCallback
                                  ,const std::string& topic_postfix_req = std::string()
                                  ,const std::string& topic_postfix_res = std::string());

      /**
       * @brief : acrp publisher. 
       *          1) searches the topic
       *          2) if the topic is acrp-inactive, just publish the message at the base topic and invokes the on-published ok callback
       *          3) if the topic is acrp-active, 
       *          3.1) create acrp packet
       *          3.2) publish at the topicReq
       *          3.3) save the message at the stack of the ATMI (acrp topic management information, management interface)
       * */
      bool acrp_publish(const std::string& topic_base, 
                        const Json::Value& packet, 
                        MqttGateway::Priority p = MqttGateway::Priority::DEFAULT);
      
      /**
       * @brief : for an active publisher, there's one subscription done that
       *          will point to this method. Here, we will see witch was the 
       *          acrp-publisher, to stop repeating the message send, and to
       *          invoke the final publish result callback.
       * */
      bool acrp_publisher_ack_common_callback(const std::string& topic_response
                        ,const uint8_t* data
                        ,size_t data_length);
      


      /**
       * @brief base callback for the publisher giving a result for an operation.
       *        It is meant to be overloaded by the user class.
       * */
      virtual bool on_publisher_operation_result_callback(const std::string& topic
                                    , bool result
                                    , const Json::Value& sourceData);


      /**
       * @brief registration of a subscriber
       * @param topic_base : string with the base topic
       * @param callback : base callback for raw data, invoked when acrp is off
       * @param parsedDataCallback : base callback for parsed json object data when acrp is on
       * @param is_active : boolean
       * @param topic_postfix_req : postfix for acrp-request : if empty, default postfix is set
       * @param topic_postfix_res : postfix for acrp-response: if empty, default postfix is set
       * */
      bool acrp_register_subscriber(const std::string& topic_base
                                    ,AcrpTopicManagementInfo::OnSubscriberDataCallbackType callback
                                    ,AcrpTopicManagementInfo::OnSubscriberParsedDataCallbackType parsedDataCallback
                                    ,bool is_active
                                    ,const std::string& topic_postfix_req = std::string()
                                    ,const std::string& topic_postfix_res = std::string());

      /**
       * @brief common subscriber data callback. From here, the topic will select
       *        the correct final destination callback. 
       *        0) only called if 
       *        1) construct the ACRP packet
       *        2) inspect if it is already been treated.
       *        2.1) if not treated, push a copy to the atmi messages stack
       *              and invoke the final user callback
       *        2.2) if already treated, do nothing with it
       *        3) in any case, publish an acknowledge to the topic response.
       *        
       * */
      bool acrp_subscription_common_callback(const std::string& topic
                                    ,const uint8_t* data
                                    ,size_t dataSize);

      /**
       * @brief : TO BE USED AFTER REGISTERING THE ACRP TOPICS, BEFORE 
       *          INITIALIZING THE MQTT GATEWAY (the current version 1.2 does
       *          not make live subscriptions yet).
       * */
      bool acrpInitialize();


      bool mqtt_gateway_init_acrp();
      bool mqtt_gateway_init_acrp_publisher(AcrpTopicManagementInfo* tmi);
      bool mqtt_gateway_init_acrp_subscriber(AcrpTopicManagementInfo* tmi);



      typedef AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType OnPublisherOperationResultCallbackType;


      //Asynchronous communications renforce protocol
      AcrpConfig acrpConfig;


      QTimer* timerManagerChecker;


      bool acrp_start_check();
      bool acrp_stop_check();
      /**
       * @brief : targt of the timer callback
       * */
      void acrp_management_pulse();

      /**
       * @brief : analyzes a message contained by a publisher, and republish if needed.
       * @return : true if the message is to be kept at the stack
       * */
      bool acrp_management_pulse_check_msg_publisher(AcrpTopicManagementInfo* tmi,AcrpPacket* acrpPtr);
      /**
       * @brief : analyzes a message contained by a subscriber
       * @return : true if the message is to be keeped at the stack
       * */
      bool acrp_management_pulse_check_msg_subscriber(AcrpTopicManagementInfo* tmi,AcrpPacket* acrpPtr);
      

      int64_t acrp_dynopub_last_operation;
      int     acrp_dynopub_make_operations();

    private:


};

}//end namespace
}//end namespace
}//end namespace

std::ostream& operator<<(std::ostream& o,coyot3::communication::mqtt::MqttManagerBase::State s);

