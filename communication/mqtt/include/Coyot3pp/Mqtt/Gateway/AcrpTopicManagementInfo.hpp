#pragma once


#include <Coyot3pp/Cor3/Coyot3.hpp>

#include "AcrpDataPacket.hpp"

#include "MqttGateway.hpp"

#define CYTOOLS_MQTTACRP_DYNOP_NOCOMMFACYTOR 10


namespace coyot3{
namespace communication{
namespace mqtt{


  CYT3MACRO_enum_class_declarations(
    AcrpTopicRole
    , 
    , SUBSCRIBER = 1
    , EMITTER    = 2
  )


  CYT3MACRO_model_class_declarations(
    AcrpTopicInformation
    ,
    , ( 
      typedef MqttGateway::OnMessageCallback  OnMessageCallback
    )
    , ( 
      role                              , ec::AcrpTopicRole , ec::AcrpTopicRole::UNKNOWN_OR_UNSET
    )
    , timeout_packet_resend               , int               , 5000
    , timeout_packet_send_failed          , int               , 3000
    , automatic_resend_time               , int               , 1000
    , timeout_receiver_alive_checkpacket  , int               , 10000
    , noprot_msg_noactivity_timeout       , int               , 3000
    , publish_dynop_interval              , int               , 5000
    , postfix_req                         , std::string       , "crp_req"
    , postfix_res                         , std::string       , "crp_res"

  )











  class AcrpTopicManagementInfo{
    public:
    struct Defaults{
      static const char*                   postfix_req;
      static const char*                   postfix_res;
      static const bool                    active;
      static const MqttGateway::Priority   qos;
    };
    struct Config{
      bool acrp_is_active;
      int64_t  acrp_timeout_packet_resend;
      int64_t  acrp_timeout_packet_send_failed;
      int64_t  acrp_automatic_resend_time;
      int64_t  acrp_timeout_receiver_alive_checkpacket;
      int64_t  acrp_noprot_msg_noactivity_timeout;
      int64_t  acrp_publish_dynop_interval;
      std::string postfix_req;
      std::string postfix_res;



      struct JsField{
        static const char* acrp_is_active;
        static const char* acrp_timeout_packet_resend;
        static const char* acrp_timeout_packet_send_failed;
        static const char* acrp_automatic_resend_time;
        static const char* acrp_timeout_receiver_alive_checkpacket;
        static const char* acrp_noprot_msg_noactivity_timeout;
        static const char* acrp_publish_dynop_interval;

      };

      Config();
      Config(const Config& o);
      virtual ~Config();

      Config& operator=(const Config& o);
      bool    operator==(const Config& o);

      bool    fromJson(const Json::Value& o);
      Json::Value toJson() const;

    };

    enum class Role{
      SUBSCRIBER = 0,
      EMITTER    = 1
    };
    static const char* RoleToString(Role r);
    friend std::ostream& operator << (std::ostream& o,Role p);

    /**
     * @brief : default callback function, in case that the user class does not
     *          define it.
     * */
    static void on_published_placebo_callback(const std::string& topic,bool result, const Json::Value& source);
    static bool on_message_placebo_callback(const std::string& topic, const uint8_t* d,size_t s);

    //after a publish operation, a callback is to be set to inform of the 
    //operation result
    // callback type void function(result, token, token_sec)
    typedef std::function<void(const std::string&, bool, const Json::Value&)> OnPublisherOperationResultCallbackType;
    //
    typedef MqttGateway::OnMessageCallback             OnSubscriberDataCallbackType;
    // 
    typedef std::function<bool(const std::string&,const Json::Value& )>
                                                       OnSubscriberParsedDataCallbackType;

    const std::string&  getTopicBase() const ;
    const std::string&  getTopicCrpReq() const ;
    const std::string&  getTopicCrpRes() const ;
    bool                isActive();
    bool                isActive(bool state);

    int64_t   getTimeoutPacketResend();
    int64_t   getTimeoutPacketSendFailed();
    int64_t   getAutomaticResendTime(); 
    int64_t   getPacketStoreExpirationTimeout();

    bool  setTimeoutPacketResendReducedTime(); ///< if topic seems disconnected, then
    bool  setTimeoutPacketResendConfigTime();

    bool setTopicBase(const std::string& t);
    bool setTopicCrpReq(const std::string& t);
    bool setTopicCrpRes(const std::string& t);


    bool        setRole(Role r);
    Role        getRole();
    
    MqttGateway::Priority getPriority();
    bool     setPriority(MqttGateway::Priority p);



    /**
     * @brief : serves a pointer to an Acrp Packet, or null if it is not managed
     * */
    AcrpPacket* getPacket(int64_t token,int64_t sec = 0);
    AcrpPacket* getPacket(const AcrpPacket& source);

    /**
     * @brief : registers a new packet and includes it at the managed stack.
     * @return : not null if operation-success.
     * 
     * */
    AcrpPacket* registerPacket(const AcrpPacket& source);

    /**
     * @brief : deletes a registered ACRP packet from the stack
     * @return : true if operation-success.
     * */
    bool unregisterPacket(int64_t token,int64_t sec=0);
    bool unregisterPacket(const AcrpPacket& acrp);

    /**
      * on data reception callback. For Role::EMITTER, we will find the 
      *  callback for the user-class when the message arrives or on error.
      * For Role::SUBSCRIBER, we will find the callback of the user-class
      * */
    OnSubscriberDataCallbackType          on_data_callback;

    OnSubscriberParsedDataCallbackType    on_parsed_data_callback;

    void updateOnSubscriberCallback(OnSubscriberDataCallbackType c);

    //link between a message token and a packet
    typedef std::map<int64_t,AcrpPacket*> TransmissionsMapType;
    TransmissionsMapType                  transmissionsMap;
    

    OnPublisherOperationResultCallbackType publish_op_result_callback;
    /**
     * @brief
     * */
    void updateOnPublishedCallback(OnPublisherOperationResultCallbackType callback);
    
    
    size_t forEachMessage(std::function<bool(AcrpPacket*)> func);



    /**
      * @brief constructor for a publisher
      * */
    AcrpTopicManagementInfo(const std::string& base
                          ,const std::string& preq = std::string()
                          ,const std::string& pres = std::string());
    AcrpTopicManagementInfo(const std::string& base
                          ,OnPublisherOperationResultCallbackType callbackOnPublishResult
                          ,const std::string& preq = std::string()
                          ,const std::string& pres = std::string());
    /**
      * @brief constructor for a role subscriber 
      */
    AcrpTopicManagementInfo(const std::string& base
                          ,OnSubscriberDataCallbackType         callback
                          ,OnSubscriberParsedDataCallbackType   callbackParsedData
                          ,const std::string& preq = std::string()
                          ,const std::string& pres = std::string());
    ~AcrpTopicManagementInfo();


      bool setConfig(const Json::Value& cfg);
      Config& getConfig();

      int64_t getDynopPublishInterval();
      int64_t getLastIncomingActivityTs();
      void    markIncomingActivity();
      int64_t getLastPacketSend();
      void    markLastPacketSend();
  protected:
    AcrpPacketManagementStack message_stack;

    std::string           topic_base;     //base topic
    int                   default_qos;    //default qos (when acrp is off)
    std::string           topic_crp_req;  //final topic with request postfix
    std::string           topic_crp_res;  //final topic with response topic.

    int64_t                   acrp_timeout_packet_send_failed;
    int64_t                   acrp_automatic_resend_time;
    int64_t                   acrp_last_incoming_activity_ts;
    int64_t                   acrp_last_packet_send;
    

    Role                      role;
    MqttGateway::Priority     qos_priority;

    Config                    acrpConfig;

  };



}//eons mqtt
}//eons wrappers
}//eons coyot3

std::ostream& operator<<(std::ostream& o, coyot3::communication::mqtt::AcrpTopicManagementInfo::Role s);
