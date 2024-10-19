#pragma once

#include "MqttGateway.hpp"
#include "AcrpTopicManagementInfoStack.hpp"


namespace coyot3{
namespace communication{
namespace mqtt{

CYT3MACRO_model_class_declarations(
  AcrpConfigObject
  , 
    , ( )
    , ( )
      , is_active                           , bool              , 
      , use_defaults                        , bool              , 
      , timeout_packet_resend               , int64_t           , 
      , timeout_packet_send_failed          , int64_t           , 
      , automatic_resend_time               , int64_t           , 
      , timeout_receiver_alive_checkpacket  , int64_t           , 
      , noprot_msg_noactivity_timeout       , int64_t           , 
      , publish_dynop_interval              , int64_t           , 
      , default_posftix_request             , std::string       , 
      , default_postfix_response            , std::string       , 
)



CYT3MACRO_model_class_serializable_json_declarations(
  AcrpConfigObject
  , 
  , 
    , ( ) 
    , ( )
      , is_active                           , "is_active"                           , 
      , use_defaults                        , "use_defaults"                        , 
      , timeout_packet_resend               , "timeout_packet_resend"               , 
      , timeout_packet_send_failed          , "timeout_packet_send_failed"          , 
      , automatic_resend_time               , "automatic_resend_time"               , 
      , timeout_receiver_alive_checkpacket  , "timeout_receiver_alive_checkpacket"  , 
      , noprot_msg_noactivity_timeout       , "noprot_msg_noactivity_timeout"       , 
      , publish_dynop_interval              , "publish_dynop_interval"              , 
      , default_posftix_request             , "default_posftix_request"             , 
      , default_postfix_response            , "default_postfix_response"            , 
)




CYT3MACRO_model_class_declarations(
  MqttGatewayAcrpConfig
    , MqttGatewayConfigObject
    , ( )
    , ( )
    , acrp   , AcrpConfigObject , 
)

CYT3MACRO_model_class_serializable_json_declarations(
  MqttGatewayAcrpConfig
  , MqttGatewayConfigObject
  , 
  , ( 
      acrp , "acrp", AcrpConfigObject
  )
  , ( )
)


class MqttGatewayAcrp : public MqttGateway {
  public:
      struct AcrpConfig{
        bool        acrp_is_active;
        bool        acrp_use_defaults;
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
          static const char* acrp_use_defaults;
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
          static const bool    acrp_use_defaults;
          static const char*   acrp_default_posftix_request;
          static const char*   acrp_default_postfix_response;
          static const int64_t acrp_timeout_packet_resend;
          static const int64_t acrp_timeout_packet_send_failed;
          static const int64_t acrp_automatic_resend_time;
          static const int64_t acrp_timeout_receiver_alive_checkpacket;
          static const int64_t acrp_noprot_msg_noactivity_timeout;
          static const int64_t acrp_publish_dynop_interval;
        };
        
        static const char* get_help();

        AcrpConfig();
        AcrpConfig(const AcrpConfig& o);
        virtual ~AcrpConfig();

        AcrpConfig& operator=(const AcrpConfig& o);
        bool    operator==(const AcrpConfig& o);

        bool    fromJson(const Json::Value& js);
        Json::Value toJson() const;
       

        bool    setAtmiConfig(AcrpTopicManagementInfo::Config& cfg);
        void    print_config_state();
      };

      MqttGatewayAcrp();
      explicit MqttGatewayAcrp(const Json::Value& config_source);
      virtual ~MqttGatewayAcrp();

      virtual bool set_configuration(const MqttGatewayAcrpConfig& conf);

      virtual bool Init() override;
      virtual bool Start() override;
      virtual bool Stop() override;

      /**
       * @brief : meant to be used by the user class, register a publisher
       * */
      bool acrp_register_publisher(const std::string& topic_base
                                  ,bool is_active
                                  ,int qos = 0
                                  ,const std::string& topic_postfix_req = std::string()
                                  ,const std::string& topic_postfix_res = std::string());

      bool acrp_register_publisher(const std::string& topic_base
                                  ,bool is_active
                                  ,AcrpTopicManagementInfo::OnPublisherOperationResultCallbackType onPublishResultCallback
                                  ,int qos = 0
                                  ,const std::string& topic_postfix_req = std::string()
                                  ,const std::string& topic_postfix_res = std::string());


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
                                    ,int qos = 0
                                    ,const std::string& topic_postfix_req = std::string()
                                    ,const std::string& topic_postfix_res = std::string());

      /**
       * @brief : acrp publish. 
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
      bool register_subscriber(const std::string& topic_base
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
      
  protected:
      bool mqtt_gateway_init_acrp();
      bool mqtt_gateway_init_acrp_publisher(AcrpTopicManagementInfo* tmi);
      bool mqtt_gateway_init_acrp_subscriber(AcrpTopicManagementInfo* tmi);
      bool load_acrp_configuration();
      bool import_acrp_from_config_struct();
      
      AcrpConfig acrpConfig;
      
      AcrpTopicManagementInfoStack tmiStack;
      
      //QTimer*    timerManagerChecker;
      coyot3::tools::ControlThread* manager_checker_th_;

        bool        acrp_start_check();
        bool        acrp_stop_check();

        void        acrp_management_pulse();

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


      MqttGatewayAcrpConfig config_acrp;

  private:




};


}
}
}