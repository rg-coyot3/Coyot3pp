#pragma once


// ROS includes.

#include <unordered_map>
#include <stdlib.h>
#include <string>
#include <vector>
#include <list>
#include <mosquitto.h>
#include <functional>
#include <map>
#include <algorithm>
#include <thread>
#include <deque>
#include <mutex>
#include <jsoncpp/json/json.h>
#include <cstring>
#include <sstream>  
#include <iterator> 
#include <ctime>
#include <chrono>
#include <iostream> 
#include <unistd.h>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <Coyot3pp/Cor3/Coyot3.hpp>


#define MQTT_GATEWAY_CONFIG_MQTT_DEBUG          "mqtt_broker_mode_debug"
#define MQTT_GATEWAY_CONFIG_ADDRESS             "mqtt_broker_address"
#define MQTT_GATEWAY_CONFIG_PORT                "mqtt_broker_port"
#define MQTT_GATEWAY_CONFIG_QOS                 "mqtt_client_qos"
#define MQTT_GATEWAY_CONFIG_QOS_MAX             "mqtt_client_qos_max"
#define MQTT_GATEWAY_CONFIG_TIMEOUT             "mqtt_client_timeout"
#define MQTT_GATEWAY_CONFIG_MOSQUITTO_DEBUG     "mqtt_mosquitto_debug_msgs"
#define MQTT_GATEWAY_CONFIG_CLIENT_IDENTIFIER   "mqtt_mosquitto_client_identifier"


#define MQTT_GATEWAY_CONFIG_CLEAN_SESSION       "mqtt_client_clean_session"
#define MQTT_GATEWAY_CONFIG_USER                "mqtt_client_user"
#define MQTT_GATEWAY_CONFIG_PASSWORD            "mqtt_client_password"
#define MQTT_GATEWAY_CONFIG_CA_FILE             "mqtt_client_file_ca"
#define MQTT_GATEWAY_CONFIG_CERT_FILE           "mqtt_client_file_cert"
#define MQTT_GATEWAY_CONFIG_KEY_FILE            "mqtt_client_file_key"
#define MQTT_GATEWAY_CONFIG_PASSPHRASE          "mqtt_client_keyfile_passphrase"
#define MQTT_GATEWAY_CONFIG_TLS_VERSION         "mqtt_broker_tls_version"
#define MQTT_GATEWAY_CONFIG_CIPHERS             "mqtt_broker_ciphers"
#define MQTT_GATEWAY_CONFIG_CONTROLLER_REPUBLISH_MSGTIMEOUT "mqtt_controller_priority_messages_timeout"
#define MQTT_GATEWAY_CONFIG_CONTROLLER_REPUBLISH_MAXRETRIES "mqtt_controller_priority_max_retries"

#define MQTT_GATEWAY_CONFIG_DEFAULT_CIPHERS                 "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384:DHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:DHE-RSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:DHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA384:DHE-RSA-AES256-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES256-SHA:ECDHE-RSA-AES256-SHA:DHE-RSA-AES256-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES128-SHA:DHE-RSA-AES128-SHA:RSA-PSK-AES256-GCM-SHA384:DHE-PSK-AES256-GCM-SHA384:RSA-PSK-CHACHA20-POLY1305:DHE-PSK-CHACHA20-POLY1305:ECDHE-PSK-CHACHA20-POLY1305:AES256-GCM-SHA384:PSK-AES256-GCM-SHA384:PSK-CHACHA20-POLY1305:RSA-PSK-AES128-GCM-SHA256:DHE-PSK-AES128-GCM-SHA256:AES128-GCM-SHA256:PSK-AES128-GCM-SHA256:AES256-SHA256:AES128-SHA256:ECDHE-PSK-AES256-CBC-SHA384:ECDHE-PSK-AES256-CBC-SHA:SRP-RSA-AES-256-CBC-SHA:SRP-AES-256-CBC-SHA:RSA-PSK-AES256-CBC-SHA384:DHE-PSK-AES256-CBC-SHA384:RSA-PSK-AES256-CBC-SHA:DHE-PSK-AES256-CBC-SHA:AES256-SHA:PSK-AES256-CBC-SHA384:PSK-AES256-CBC-SHA:ECDHE-PSK-AES128-CBC-SHA256:ECDHE-PSK-AES128-CBC-SHA:SRP-RSA-AES-128-CBC-SHA:SRP-AES-128-CBC-SHA:RSA-PSK-AES128-CBC-SHA256:DHE-PSK-AES128-CBC-SHA256:RSA-PSK-AES128-CBC-SHA:DHE-PSK-AES128-CBC-SHA:AES128-SHA:PSK-AES128-CBC-SHA256:PSK-AES128-CBC-SHA"
#define MQTT_GATEWAY_CONFIG_DEFAULT_DEBUG_MODE              false
#define MQTT_GATEWAY_CONFIG_PACKAGE_LOCATION        "package_location"
#define MQTT_GATEWAY_CONFIG_ROSTOPIC_CONNECTIONSTATUS "control_rostopic_connection_status_state"



#define MQTT_GATEWAY_INTERNAL_QUEUE_CONTROLLER_PULSE_SECONDS    4
#define MQTT_GATEWAY_REPUBLISH_RETRIES_MAX                      5

namespace coyot3{
namespace communication{
namespace mqtt{


CYT3MACRO_model_class_declarations(
  MqttGatewayConfigObject
  , 
    , ( virtual void print_current_config_state())
    , ( )
    
      , debug_mode                          , bool        , false 
      , host_address                        , std::string , ""
      , host_port                           , int32_t     , 0
      , qos                                 , int         , -1
      , qos_max                             , int         , 2
      , timeout                             , int         , 30
      , show_debug_msgs                     , bool        , false
      , important_messages_timeout_ms       , int         , 5
      , important_messages_max_retries      , int         , 5
      , clean_session                       , bool        , true
      , client_id                           , std::string , ""
      , client_id_postfix_rand_str_length   , int         , 10
      , user                                , std::string , ""
      , password                            , std::string , ""
      , base_path                           , std::string , ""
      , certificates_relative_path          , bool        , true
      , certificates_ca                     , std::string , ""
      , certificates_cert                   , std::string , ""
      , certificates_key                    , std::string , ""
      , certificates_passphrase             , std::string , ""
      , tls_version                         , std::string , ""
      , ciphers                             , std::string , "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384:DHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:DHE-RSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:DHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA384:DHE-RSA-AES256-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES256-SHA:ECDHE-RSA-AES256-SHA:DHE-RSA-AES256-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES128-SHA:DHE-RSA-AES128-SHA:RSA-PSK-AES256-GCM-SHA384:DHE-PSK-AES256-GCM-SHA384:RSA-PSK-CHACHA20-POLY1305:DHE-PSK-CHACHA20-POLY1305:ECDHE-PSK-CHACHA20-POLY1305:AES256-GCM-SHA384:PSK-AES256-GCM-SHA384:PSK-CHACHA20-POLY1305:RSA-PSK-AES128-GCM-SHA256:DHE-PSK-AES128-GCM-SHA256:AES128-GCM-SHA256:PSK-AES128-GCM-SHA256:AES256-SHA256:AES128-SHA256:ECDHE-PSK-AES256-CBC-SHA384:ECDHE-PSK-AES256-CBC-SHA:SRP-RSA-AES-256-CBC-SHA:SRP-AES-256-CBC-SHA:RSA-PSK-AES256-CBC-SHA384:DHE-PSK-AES256-CBC-SHA384:RSA-PSK-AES256-CBC-SHA:DHE-PSK-AES256-CBC-SHA:AES256-SHA:PSK-AES256-CBC-SHA384:PSK-AES256-CBC-SHA:ECDHE-PSK-AES128-CBC-SHA256:ECDHE-PSK-AES128-CBC-SHA:SRP-RSA-AES-128-CBC-SHA:SRP-AES-128-CBC-SHA:RSA-PSK-AES128-CBC-SHA256:DHE-PSK-AES128-CBC-SHA256:RSA-PSK-AES128-CBC-SHA:DHE-PSK-AES128-CBC-SHA:AES128-SHA:PSK-AES128-CBC-SHA256:PSK-AES128-CBC-SHA"

)

CYT3MACRO_model_class_serializable_json_declarations(
  MqttGatewayConfigObject
  ,
  , 
  , ( )
  , ( )
    , debug_mode                        , "debug_mode"                        , 
    , host_address                      , "host_address"                      , 
    , host_port                         , "host_port"                         , 
    , qos                               , "qos"                               , 
    , qos_max                           , "qos_max"                           , 
    , timeout                           , "timeout"                           , 
    , show_debug_msgs                   , "show_debug_msgs"                   , 
    , important_messages_timeout_ms     , "important_messages_timeout_ms"     , 
    , important_messages_max_retries    , "important_messages_max_retries"    , 
    , clean_session                     , "clean_session"                     , 
    , client_id                         , "client_id"                         , 
    , client_id_postfix_rand_str_length , "client_id_postfix_rand_str_length" , 
    , user                              , "user"                              , 
    , password                          , "password"                          , 
    , base_path                         , "base_path"                         , 
    , certificates_relative_path        , "certificates_relative_path"        , 
    , certificates_ca                   , "certificates_ca"                   , 
    , certificates_cert                 , "certificates_cert"                 , 
    , certificates_key                  , "certificates_key"                  , 
    , certificates_passphrase           , "certificates_passphrase"           , 
    , tls_version                       , "tls_version"                       , 
    , ciphers                           , "ciphers"                           , 
)


CYT3MACRO_enum_class_declarations(
  GatewayState
  ,
    ,NOT_CONFIGURED  = 1
    ,CONFIGURED      = 2
    ,CONNECTING      = 3
    ,CONNECTED       = 4
    ,DISCONNECTED    = 5
    ,CLOSED          = 6
    ,END_OF_LIFE     = 7
    ,ERROR           = 8
)


CYT3MACRO_enum_class_declarations(
  MessagePriority
  ,
    ,DEFAULT =10
    ,LOW     = 0
    ,MEDIUM  = 1
    ,HIGH    = 2
)


/**
 * @brief : The MqttGateway class creates a wrapper with the mosquitto client
 *  instance, using as a base the libmosquitto.
 *  
 *  It is configured as an independent instance.
 * 
 *  The client instances are those who publish streams and subscribes to topics
 *  The MqttGateway instance manages the exchanges with the client intances
 *      using lambda functions.
 *  
 *  The list of event it manages are :
 *      
 *      - on_connection()
 *      - on_disconnection()
 *      - on_message()
 *      - on_published()
 *      - on_subscribed()
 *      - on_unsubscribed()
 * 
 * */
class MqttGateway
{
    friend int mosq_on_tls_certs_password_callback(char* buff,
                                    int size,
                                    int rwflag,
                                    void* userdata);

    friend void mosq_on_subscribe(struct mosquitto* client, 
                                    void* userdata,
                                    int mid,
                                    int qos_count,
                                    const int* granted_qos);
    friend void mosq_on_client_connect(struct mosquitto *mosq, void *userdata, int result);
    friend void mosq_on_client_disconnect(struct mosquitto *mosq, void *obj, int rc);
    friend void mosq_on_subscribe(struct mosquitto* client, 
                                    void* userdata,
                                    int mid,
                                    int qos_count,
                                    const int* granted_qos);
    friend void mosq_on_unsubscribe(struct mosquitto* client, void* userdata, int mid);
    friend void mosq_on_message_received(struct mosquitto* mosq,
                                    void* obj,
                                    const struct mosquitto_message* message);
    friend void mosq_on_log(struct mosquitto* client,void* userdata,int level,const char* msg);
    friend void mosq_on_publish(struct mosquitto* client,void* userdata,int mid);



  public:
    /**
     * Enumerates the state of the gateway
     * 
     * */
    enum class GatewayState{
        NOT_CONFIGURED,
        CONFIGURED,
        CONNECTING,
        CONNECTED,
        DISCONNECTED,
        CLOSED,
        END_OF_LIFE,
        ERROR
    };
    static const char* GatewayStateToString(GatewayState s);

    //corresponds to Mqtt Qos
    enum class Priority{
        DEFAULT=-1,
        LOW = 0,
        MEDIUM = 1,
        HIGH = 2
    };
    static const char* PriorityToString(Priority p);

    //wraps messages callback lambda type for client instances
    typedef std::basic_string<uint8_t>                  ByteStream;

    typedef std::function<bool(const std::string&,const uint8_t*,size_t)>  OnMessageCallback;
    typedef std::function<void(bool)>                   OnConnectionCallback;
    typedef std::function<void(bool)>                   OnDisconnectionCallback;
    
    typedef std::list<OnConnectionCallback>         OnConnectionCallbackStack;
    typedef std::list<OnDisconnectionCallback>      OnDisconnectionCallbackStack;
    
    typedef std::list<OnMessageCallback>            OnMessageCallbackCollection;
    
    typedef std::map<std::string,OnMessageCallbackCollection> TopicSubscriptionsStack;
    typedef std::map<std::string, std::string>                TopicSubscriptionsRelations;  


    typedef std::map<int,std::string>                         topic_ids_collection;

    typedef std::map<std::string,int>                         TopicQosRelations;
    /**
     * @brief : Each 'client' instance will subscribe to an mqtt topic.
     *  when the message is received, a lamba function given by the client
     *  will be invoked sending the stream as a parameter, and its size.
     * */
    struct TopicSubscription{
        std::string topic;          /**< topic to subscribe to*/
        OnMessageCallback callback; /**< callback lambda function to be invoked*/     
    };

    struct MqttMessage
    {   
        int         messageID;
        int64_t     creation_time;
        std::string topic;
        uint8_t*    payload;
        uint        size;
        Priority    prio;
        int         retries;

        MqttMessage();
        MqttMessage(const MqttMessage& o);
        MqttMessage(MqttMessage& o);
        MqttMessage(int pmsgid
                    ,const std::string& ptopic
                    ,const uint8_t* pptr
                    ,size_t psize
                    ,Priority pprio = Priority::DEFAULT);
        virtual ~MqttMessage();

        MqttMessage& operator=(const MqttMessage& o);
        MqttMessage& operator=(MqttMessage& o);
        bool         setPayload(const uint8_t* pptr,size_t psize);
        void         setCreationTimestampNow();
        void         incrementRetries();
    };

    
    struct MqttMessagesStack{
      typedef std::map<int,MqttMessage>                 MqttMessagesMap;
      typedef std::map<int,MqttMessage>::iterator       MqttMessagesMapIterator;
      typedef std::map<int,MqttMessage>::const_iterator MqttMessagesMapConstIterator;
      typedef std::pair<int,MqttMessage>                MessageStackPair;


      MqttMessagesStack();
      MqttMessagesStack(const MqttMessagesStack& o);
      virtual ~MqttMessagesStack();

      MqttMessagesStack& operator=(const MqttMessagesStack& o);
      MqttMessagesStack& operator+=(const MqttMessagesStack& o);
      MqttMessagesStack& operator+=(const MqttMessage& o);

      size_t  size() const;
      size_t  size();

      bool    empty() const;

      void clear();
      

      bool push(const MqttMessage& m);
      bool push(const MqttMessagesStack& o);
      bool exists(const MqttMessage& m);
      bool exists(int messageId);
      bool remove(const MqttMessage& m);
      bool remove(int messageId);

      size_t forEach(std::function<bool(const MqttMessage& m)> func) const;
      size_t forEach(std::function<bool(MqttMessage& m)> func);


      MqttMessagesMap   msg_map;

      std::mutex  stack_mtx_;

    };

    struct MqttGatewayConfig{
      std::string       certificates_base_path;
      bool              debug_mode;

      std::string       broker_host;
      int32_t           broker_port;
      bool              clean_session;
      
      std::string       client_id;
      bool              client_id_add_random_string;
      std::string       user;
      std::string       password;

      int               qos;
      int               qos_max; //some brokers at the cloud does not have qos2
      int64_t           connection_timeout_msecs;
      bool              mosquitto_client_debug;

      int64_t           priority_msgs_republish_interval;
      int64_t           priority_msgs_republish_max_retries;

      std::string       file_ca;
      std::string       file_cert;
      std::string       file_key;
      std::string       key_passphrase;
      std::string       tls_version;
      std::string       cyphers;
      
      struct JsFields{
        static const char* debug_mode;
        static const char* broker_host;
        static const char* broker_port;
        static const char* clean_session;
        static const char* client_id;
        static const char* client_id_add_random_string;
        static const char* user;
        static const char* password;
        static const char* qos;
        static const char* qos_max;
        static const char* connection_timeout_msecs;
        static const char* mosquitto_client_debug;
        static const char* priority_msgs_republish_interval;
        static const char* priority_msgs_republish_max_retries;
        static const char* file_ca;
        static const char* file_cert;
        static const char* file_key;
        static const char* key_passphrase;
        static const char* tls_version;
        static const char* cyphers;
        
      };
      struct Defaults{
        static const bool debug_mode;
        static const std::string broker_host;
        static const uint32_t broker_port;
        static const bool clean_session;
        static const std::string client_id;
        static const bool client_id_add_random_string;
        static const std::string user;
        static const std::string password;
        static const int qos;
        static const int qos_max;
        static const int64_t connection_timeout_msecs;
        static const bool mosquitto_client_debug;
        static const int64_t priority_msgs_republish_interval;
        static const int64_t priority_msgs_republish_max_retries;
        static const std::string file_ca;
        static const std::string file_cert;
        static const std::string file_key;
        static const std::string key_passphrase;
        static const std::string tls_version;
        static const std::string cyphers;

      };

      
      MqttGatewayConfig();
      MqttGatewayConfig(const MqttGatewayConfig& o);
      virtual ~MqttGatewayConfig();

      MqttGatewayConfig& operator=(const MqttGatewayConfig& o);
      bool          operator==(const MqttGatewayConfig& o);

      void load_defaults();

      Json::Value toJson() const;
      bool        fromJson(const Json::Value& js);

      const char* get_help() const;

    };
    typedef std::vector<TopicSubscription*> TopicSubscriptionsCollection;

    /**
     * @brief collection of disconnect functions callbacks, of a function
     *      to a void that receives a bool param.
     *      if the parameter is true  : the disconnection was expected
     *      if the parameter is false : the disconnection was unexpected 
     * */
    typedef std::vector<std::function<void(bool)>> OnDisconnectFuncCollection;
    
    /**
     * @brief collection of connection functions callbacks, of a function
     *  to a void that receives a bool param.
     *      if the parameter is true  : the connection is success
     *      if the parameter is false : the connection was refused or unavailable 
     * */
    typedef std::vector<std::function<void(bool)>> OnConnectFuncCollection;

    //singleton of callbacks for multiple instances

    /**
     * @brief : constructor of the class. will also load the configuration from
     *  the ros param system  
     * */
    MqttGateway();
    explicit MqttGateway(const Json::Value& config_source);

    /**
     * @brief will close and disconnect the client
     * */
    virtual ~MqttGateway();


    /** 
     * @brief loads the configuration and prepares the client to be connected
     * @return true if configuration was correctly loaded
     * */
    virtual bool Init();

    /**
     * @brief launches the mqtt client and connects it to the broker
     * @return true if it was correctly launched
     * */
    virtual bool Start();

    /**
     * @brief disconnects the client and closes the instance
     * @return true if the client is correctly stopped
     * */
    virtual bool Stop();


    /**
     * @brief sets the current configuration manager.
     * @return true if the configuration object is valid
     * */
    virtual bool set_configuration(const Json::Value& config);

    virtual bool set_configuration(const MqttGatewayConfigObject& conf);

    /**
     * @brief main publisher function
     * @param t : topic
     * @param payload : pointer to the stream
     * @param payload_size : length of the stream
     * @param q : QoS
     * @return boolean : true if ok - false otherwise
     * */
    virtual bool publish(const std::string& t, 
        const uint8_t* payload,
        int payload_size,
        Priority p = Priority::DEFAULT);
    /**
     * @brief : overload to permit use of strings
     * @param t : topic
     * @param m : message
     * @param q : qos
     * @return true if ok, false if ko
     * */
    bool publish(const std::string& t,
                    const std::string& m,
                    Priority p = Priority::DEFAULT);
    
    bool publish(const std::string& t,
                    const ByteStream& s,
                    Priority p = Priority::DEFAULT);

    bool publish(const std::string& t,
                    const Json::Value& s,
                    Priority p = Priority::DEFAULT);
    /** 
     * 
     * */
    void set_certificate_location_path(const std::string& l);

    /**
     * @brief : subscribe to a topic.
     * @param t : topic to subscribe to
     * @param cb : callback function 
     * @param q : qos set by default to -1
     * @return true if subscription was OK. false if KO
     * */
    bool subscribe_to_topic(const std::string& t,
        OnMessageCallback cb,Priority p = Priority::DEFAULT);
    


 
    /**
     * @brief : debuggin purposes.
     * @param option : if true, it will not try to connect to the mqtt broker.
     * */
    bool set_debug_mode(bool option);



    /**
     * @brief : to be invoked by the mosquitto wrapper
     * @param topic :
     * @param stream :
     * @param stream_size : 
     * */
    void on_mosq_message(const char* t,const void* payload,const size_t payload_size);
    
    // -------------------------------------------------------------------------
    // internal callbacks
    void on_mosq_client_connects(int result);
    void on_mosq_client_disconnects(int reason); //to-do
    void on_mosq_client_published(int message_id);
    void on_mosq_client_unsubscribe(int message_id);
    void on_mosq_client_subscribe(int message_id,int qos_count,const int* granted_qos);
    void on_mosq_client_logs(int level,const char* msg);
    int  on_mosq_key_file_passphrase_request(char* buff, int size,int rwflag);
    
    
    
    void priority_messages_controller_pulse();

    // 
    // -------------------------------------------------------------------------


    /**
     * @brief simple getter
     * */
    GatewayState getState();

    void set_debug_level(int debugLevel);

    //getters-setters : 

    bool set_max_qos_level(int l);
    bool lock_interface_io() const;
    bool lock_interface_io(bool l);

  protected:
    TopicSubscriptionsStack         topic_subs_collection;
    TopicSubscriptionsRelations     topic_subs_relations;

    TopicSubscriptionsCollection    topic_subscriptions;
    OnDisconnectFuncCollection      on_disconnect_callbacks;
    OnConnectFuncCollection         on_connect_callbacks;

    TopicQosRelations               topic_qos_relations;
    TopicQosRelations               topic_qos_rel_todo;

    
    GatewayState                    gateway_state;
    int64_t                         gwstate_last_change_ts;

    bool republished = false;

    
    bool on_mosq_message_search_relation(const std::string& in, std::string& out);

    MqttGatewayConfigObject         config;
    bool set_configuration_(const Json::Value& confStruct);
    bool update_config_from_struct_();



    /**
     * @brief : internal, subscription to a topic
     * @param t : topic
     * @param q : qos
     * @return : true on success, false otherwise
     * */
    bool pulse_mosq_subscribe();

    bool make_mosq_subscriptions();

    /**
     * @brief  unsubscribes from one topic
     * @param  t : topic 
     *  mosquitto client instance.
     * */
    bool mosq_unsubscribe(const std::string& t);

    /**
     * @brief : logging purposes
     * */
    void print_config();
    /**
     * @brief : loads the configuration loaded at the params system of ros 
     *  (from the launch file)
     * @return : true if the configuration was loaded correctly. false if the 
     *  configuration contains errors
     * */
    bool load_configuration();

    bool _load_configuration_from_jsoncpp_struct();
    Json::Value _config_source;
    MqttGatewayConfig configuration;

    /**
     * @brief : launches the communication with the broker
     * @return : true if the mqtt client was correctly launched. false on 
     *  errors.
     * */
    bool connect_to_broker();


    
    /**
     * @brief : disconnects the broker and closes the instance.
     * 
     * */
    bool disconnect_from_broker();



    std::vector<std::string>        topics_to_subscribe_to_mosq;

    


    std::thread* gateway_controller_thread = nullptr;
    bool         gateway_controller_keepalive_flag = false;

    std::thread* queue_controller_thread = nullptr;
    bool         queue_controller_keepalive_flag = false;


    bool         gateway_controller();
    bool         queue_controller();

    void         controller_pulse_make_mosq_subscription();
    bool         full_reset_connection();
    bool         copy_priority_messages_to_secondary_queue();
    bool         republish_secondary_queue();

    private:

    
    bool has_user;
    bool has_password;
    bool has_ca_file;
    bool has_cert_file;
    bool has_key_file;
    bool has_key_passphrase;
    bool has_tls_version;
    bool has_ciphers;
    

    //! MQTT client 
    struct mosquitto *client;
    bool   lock_interface_io_;
    int64_t last_confirmed_activity_ts_;



    MqttMessagesStack       stack_priority_messages_primary;
    MqttMessagesStack       stack_priority_messages_secondary;

    //int priority_republish_timeout    = 30;
    //int priority_republish_maxretries = 4;

    //std::mutex mtx_republish_queue;
    std::mutex mtx_mqtt_publish;
    //std::mutex mtx_secondary_queue;


    int     mosqclient_disconnected_timeout_secs = 10;
    int64_t mosqclient_ts_last_connection = 0;
    int64_t mosqclient_ts_last_disconnection = 0;
    bool    mosqclient_full_reset_flag = false;

    int number_of_auto_messages = 0;

    bool republishing = false;

    std::time_t time_thread;

   

    int number_of_messages_published_success = 0;
#ifdef MQTTGW_CONFIG_LOAD_METHOD_ROS_XMLRPCPP
    ros::NodeHandle*    node_handler;
    ros::Publisher      publisher_mqtt_status;
    std::string         rostopic_mqttgwstatus;

#endif
    

   
    std::function<void(const std::string&,const uint8_t* pptr,size_t psize)> user_callback_on_priority_message_send_error;
    std::function<void(const std::string&)> user_callback_on_fatal_error;

    void _debug_level_set_mod_msg(int debugLevel);
    void _debug_level_set_mod_callbacks(int debugLevel);
    void _debug_level_set_mod_connects(int debugLevel);
    void _debug_level_set_mod_configurations(int debugLevel);
    void _debug_level_set_mod_controllers(int debugLevel);
    
    
};


} //oens mqtt
} //eons communication
} //eons coyot3



    //!
    // TO-DO
    // static void on_client_has_connected(struct mosquitto *mosq,void *userdata,int result);
    // static void on_client_has_disconnected(struct mosquitto* mosq,void *obj,int rc);
    // static void on_message_has_been_received(struct mosquitto* mosq,void* obj, const struct mosquitto_message* message);
