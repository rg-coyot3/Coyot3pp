/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * 
 * @author : Ricardo GONZALEZ 
 * @email : ricardogonalm@ltlab.net 
 * */


#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>
namespace coyot3{
namespace communication{
namespace mqtt{

//! 
void MqttGateway::_debug_level_set_mod_connects(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
}
/**
 * @brief : static function that will be invoked by the mosquitto client in case
 *  the key file is ciphered. 
 * */
int mosq_on_tls_certs_password_callback(char* buff,
                int size,
                int rwflag,
                void* userdata)
{
  CLOG_INFO("mqtt-gw : STATIC : tls certs demands password");
  //search of the mosquitto instance invoker - 
  return static_cast<MqttGateway*>(userdata)->on_mosq_key_file_passphrase_request(buff,size,rwflag);
}


/**
 * @brief : static callback method when the mosquitto instance is connected.
 *  
 * */
void mosq_on_client_connect(struct mosquitto *mosq, void *userdata, int result)
{


  if(!result)
  {
    CLOG_INFO("mqtt-gw : STATIC : mosquitto client has connected.");
  }
  else
  {
    CLOG_ERROR("mqtt-gw : STATIC : ERROR : while trying to connect with"
        " mosquitto broker! stderr");
  }
    // - calls the gateway
    static_cast<MqttGateway*>(userdata)->on_mosq_client_connects(result);
    return;
}
/**
 * @brief : static function to 
 * 
 * */
void mosq_on_client_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
  if(rc == 0)
  {
    CLOG_INFO("MQTT GW : STATIC : The mosquitto client instance "
        "DISCONNECTED from the broker");
  }
  else
  {
    CLOG_WARN("MQTT GW : STATIC : !!! THE MQTT BROKER INSTANCE HAS "
        "UNEXPECTEDLY DISCONNECTED FROM THE BROKER");
  }
  
  static_cast<MqttGateway*>(obj)->on_mosq_client_disconnects(rc);
}


/**
 * @brief : static callback function for the mosquitto client when a message
 *   has been published
 * */


void mosq_on_subscribe(struct mosquitto* client, 
    void* userdata,
    int mid,
    int qos_count,
    const int* granted_qos)
{
  static_cast<MqttGateway*>(userdata)->on_mosq_client_subscribe(mid,qos_count,granted_qos);
}

/**
 * @brief : static callback function
 * */
void mosq_on_unsubscribe(struct mosquitto* client, void* userdata, int mid)
{
  static_cast<MqttGateway*>(userdata)->on_mosq_client_unsubscribe(mid);
}

/**

 * @brief : message received
 * */
void mosq_on_message_received(struct mosquitto* mosq,
                                void* obj,
                                const struct mosquitto_message* message)
{

  static_cast<MqttGateway*>(obj)->on_mosq_message(message->topic,
      static_cast<uint8_t*>(message->payload),
      message->payloadlen);
}
// /


void mosq_on_publish(struct mosquitto* client,void* userdata,int mid)
{
  static_cast<MqttGateway*>(userdata)->on_mosq_client_published(mid);
}




void mosq_on_log(struct mosquitto* client,void* userdata,int level,const char* msg)
{
  static_cast<MqttGateway*>(userdata)->on_mosq_client_logs(level,msg);
}


void errno_system_description(int err)
{
    switch(err)
    {
      case 110:
        CLOG_ERROR("ERRNO 110 = ETIMEDOUT 110 Connection timed out");        
        return;
      break;
      case 111:
        CLOG_ERROR("ERRNO 111 = CONNECTION REFUSED");
    }

}

bool MqttGateway::connect_to_broker()
{  
    last_confirmed_activity_ts_ = coyot3::tools::get_current_timestamp();
    if(config.debug_mode() == true)
    {
      CLOG_WARN("mqtt-gw : DEBUG MODE ACTIVE : THIS "
        "GATEWAY WILL NOT BE CONNECTED TO THE BROKER BUT WILL REPORT "
        "ACTIVITY");
      return true;
    }
    std::string mosqId;
    // Initializing mosquitto library.
    static int NUM_INSTANCES = 0;
    if(NUM_INSTANCES == 0)
    {
      CLOG_INFO("mqtt-gw : connect : mosquitto lib init (ONCE)");
      mosquitto_lib_init();
      NUM_INSTANCES ++;
    }
    else
    {
      CLOG_INFO("mqtt-gw : connect : mosquitto lib already initiated with"
          " a previous instance");
    }
    CLOG_INFO("mqtt-gw : connect : creating connection with config:")
    print_config();
    if(config.client_id().size() == 0)
    {
      CLOG_INFO("mqtt-gw : connect : creating mosquitto instance with "
          "no identifier");
      mosqId = coyot3::tools::generate_string_aphanumeric(10);
      client = mosquitto_new(mosqId.c_str(),true,static_cast<void*>(this));

      //client = mosquitto_new("ID - Client1",false,static_cast<void*>(this));
    }
    else
    {
      mosqId = config.client_id();
      CLOG_INFO("mqtt-gw : connect : creating mosquitto instance with "
          "identifier [" << config.client_id() << "], clean sesion[" <<  
          (config.clean_session() == true?"true":"false") << "]");
      if(config.clean_session() == true)
      {
        mosqId+="_";
        mosqId+=coyot3::tools::generate_string_aphanumeric(5);
      }
      client = mosquitto_new(mosqId.c_str(),
                              config.clean_session(), 
                              static_cast<void*>(this));    
    }

    if(client == nullptr)
    {
      std::stringstream ss;
      CLOG_ERROR("mqtt-gw : connect : FATAL ERROR! while creating "
          "mosquitto client instance. ERRORNO = ["<< errno << "]");
      ss << "error invoking mosquitto_new to create an instance. Parameters ("
        "clientId=" << mosqId << ";clean_session="
        << (config.clean_session()==true?"true":"false") << ";usrPtr="
        << (this!=nullptr?"NOT_NULL":"NULL!?") << "). ErrNo(" << errno << "):";
      switch(errno)
      {
        case ENOMEM : 
          CLOG_ERROR("mqtt-gw : connect : FATAL ERROR! NO MEMORY");
          ss << "NO MEMORY!";
          break;
        case EINVAL :
          CLOG_ERROR("mqtt-gw : connect : ");
          ss << "EINVAL";
          break;
        default:
          ss << "not-managed";
      }
      CLOG_ERROR("mqtt-gw : connect : exiting");
      if(user_callback_on_fatal_error)
      {
        CLOG_WARN("mqtt-gw : connect : invoking user callback as last-"
          "ressource");
        user_callback_on_fatal_error(ss.str());
        }
    }

    int ret_pw;
    //user and password set
    //
    if(has_user && has_password)
    {
      CLOG_INFO("mqtt-gw : connect conf : setting user and "
          "password");
      ret_pw = mosquitto_username_pw_set(client,
                                          config.user().c_str(), 
                                          config.password().c_str());

      CLOG_INFO("Username and password [" << config.user() << ":" 
        << config.password() << "]");
      
      switch(ret_pw)
      {
        case MOSQ_ERR_SUCCESS:
          CLOG_INFO("mqtt-gw : connecting to broker - user name "
              "and password has been set ok");
          break;
        case MOSQ_ERR_INVAL:
          CLOG_ERROR("mqtt-gw : connecting to server - ERROR - "
              "user name and/or password are invalid : [u:" 
              << config.user() << ",p:" << config.password() << "]");
          return false;
          break;
        case MOSQ_ERR_ERRNO:
          CLOG_ERROR("mqtt-gw : connecting to server - ERROR - "
              "Internal error . REPORTED ERROR = ["<< errno << "]");
          return false;
          break;
        default:
          CLOG_ERROR("mqtt-gw : connecting to server - ERROR - "
              "Internal error . MOSQUITTO ERROR CODE ["<< ret_pw << "]");
          return false;

      }
    }
    else
    {
      if(has_user || has_password)
      {
        CLOG_ERROR("mqtt-gw : connect conf : MUST have user and pass "
            "or none");
      }
      else
      {
        CLOG_INFO("mqtt-gw : connect conf : no user/pass config "
            "for this broker");
      }        
    }

    //tls configuration
    if(has_ca_file || has_cert_file || has_key_file)
    {

      const char* caFileLocation = nullptr;
      const char* certFileLocation = nullptr;
      const char* keyFileLocation = nullptr;
      if(has_ca_file)
      {
        caFileLocation = config.certificates_ca().c_str();
      }
      if(has_cert_file && has_key_file){
        certFileLocation = config.certificates_cert().c_str();
        keyFileLocation = config.certificates_key().c_str();
      }else if(has_cert_file != has_key_file){
        CLOG_ERROR("mqtt-gw : connecting : certificate and key paths"
          "MUST BE BOTH set, or NOT set");
      }

      CLOG_INFO("mqtt-gw : connect conf : SETTING TLS CONFIG : ca   : " << caFileLocation)
      CLOG_INFO("mqtt-gw : connect conf : SETTING TLS CONFIG : cert : " << certFileLocation)
      CLOG_INFO("mqtt-gw : connect conf : SETTING TLS CONFIG : key  : " << keyFileLocation)
      int ret_tls = mosquitto_tls_set(client, 
                                  caFileLocation,
                                  nullptr, 
                                  certFileLocation, 
                                  keyFileLocation, 
                                  mosq_on_tls_certs_password_callback);
                    
      if(mosquitto_tls_insecure_set(client,true) != MOSQ_ERR_SUCCESS){
        CLOG_WARN("mqtt-gw : connect conf : tls-insecure-set : error setting insecure")
      }

      switch(ret_tls) 
      {
        case MOSQ_ERR_SUCCESS	: 
          CLOG_INFO("mqtt-gw : connecting : tls certs set OK");
          break;  
        case MOSQ_ERR_INVAL: 
          CLOG_ERROR("mqtt-gw : connect conf : FATAL! :"\
          " Certificates are INVALID /!!!/");
          CLOG_ERROR("mqtt-gw : CHECK CERTIFICATES AT CA   FILE : ["
              << config.certificates_ca() 
              << "]\n CERT FILE : [" << config.certificates_cert() 
              << "]\nKEY  FILE[" << config.certificates_key() << "]");
          CLOG_ERROR("mqtt-gw : connection : STOPPING PROCESS");
          exit(1);
          break;
        case MOSQ_ERR_NOMEM: 
          CLOG_ERROR("mqtt-gw : connect conf : FATAL! : Out of "
              "memory");
          CLOG_ERROR("mqtt-gw : connection : STOPPING PROCESS");
          exit(1);
          break;
        default:
          CLOG_ERROR("mqtt-gw : connect conf : ERROR SETTING"\
          " CERTIFICATE, KEY AND CRT FILES");
          CLOG_ERROR("mqtt-gw : connection : STOPPING PROCESS");
          exit(1);
      }

      int ret_version = 
        mosquitto_tls_opts_set(client,
                                SSL_VERIFY_PEER,
                                config.tls_version().c_str(),
                                config.ciphers().c_str());
      std::string bufferMsg;
      switch(ret_version){
        case MOSQ_ERR_INVAL: bufferMsg = "MOSQ_ERR_INVAL";break;
        case MOSQ_ERR_NOMEM: bufferMsg = "MOSQ_ERR_NOMEM";break;
        case MOSQ_ERR_SUCCESS: bufferMsg = "MOSQ_ERR_SUCCESS";break;
        default:
          bufferMsg = "UNKNOWN-UNHANDLED-MOSQ-ERROR-VALUE!";

      }

      if(ret_version != MOSQ_ERR_SUCCESS)
      {
        CLOG_ERROR("mqtt-gw : connect conf : TLS advanced "
          "options are NOT set. : err = " << ret_version << "(" 
          << bufferMsg << ")");
        return false;
      }else{
        CLOG_INFO("mqtt-gw : connect conf : TLS options are set OK");
      }
    }

    CLOG_INFO("mqtt-gw : Setting on_connect callback");
    mosquitto_connect_callback_set(client,mosq_on_client_connect);
    CLOG_INFO("mqtt-gw : Setting on_disconnect callback function");
    mosquitto_disconnect_callback_set(client, mosq_on_client_disconnect);
    CLOG_INFO("mqtt-gw : Setting on_message callback function");
    mosquitto_message_callback_set(client,mosq_on_message_received);
    CLOG_INFO("mqtt-gw : Setting on_log callback function");
    mosquitto_log_callback_set(client,mosq_on_log);
    CLOG_INFO("mqtt-gw : setting callback on_subscription");
    mosquitto_subscribe_callback_set(client,mosq_on_subscribe);
    CLOG_INFO("mqtt-gw : setting on_unsubscribe callback");
    mosquitto_unsubscribe_callback_set(client,mosq_on_unsubscribe);

    mosquitto_publish_callback_set(client,mosq_on_publish);

    CLOG_INFO("mqtt-gw : launching controller thread");
    CLOG_INFO("now connecting to broker - locking mutex /!\\ timeout " << config.timeout());

    int res_loop_start = mosquitto_loop_start(client);
    
    switch(res_loop_start)
    {
        case MOSQ_ERR_SUCCESS:
            CLOG_INFO("mqtt-gw : connect : started mosquitto loop OK"); //
            break;
        case MOSQ_ERR_INVAL:
            CLOG_ERROR("mqtt-gw : connect : ERROR! INPUT CLIENT PARAM IS" 
                "INVALID");
            CLOG_ERROR("mqtt-gw : connect : FATAL ERROR");
            exit(1);
            break;
        case MOSQ_ERR_NOT_SUPPORTED:
            CLOG_ERROR("mqtt-gw : connect : ERROR! THREAD SUPPORT IS NOT "
                "AVAILABLE!");
            CLOG_ERROR("mqtt-gw : connect : FATAL ERROR");
            exit(1);
            break;
    }

    CLOG_INFO("Mosquitto connection flux succeded"); //
    // Create ROS subscribers

  //mosquitto_reconnect_delay_set(client,5,5,true);
    gateway_state = GatewayState::CONNECTING;

    bool retry = false;

    do
    {
      int ret_connect;
      if(retry == true){
        CLOG_WARN("mqtt-gw : connect : retrying async-connect : forcing "
          "thread lock for 1/4 of a second");
        usleep(250000);
      }
      CLOG_INFO("mqtt-gw : connect : async-connect to [" 
        << config.host_address() << ":" << config.host_port() 
        << "], with a timeout = " << config.timeout());
      int keepaliveparam = (config.timeout() < 20 ? config.timeout() : 20);
      ret_connect = mosquitto_connect_async(
        client, 
        config.host_address().c_str(), 
        config.host_port(), 
        keepaliveparam);

      switch(ret_connect)
      {
        case MOSQ_ERR_SUCCESS:
          CLOG_INFO("mqtt-gw : connect : connection success");  //
          retry = false;
          break;
        case MOSQ_ERR_INVAL:
          CLOG_ERROR("mqtt-gw : connect : ERROR : input parameters are "
              "invalid");
          CLOG_ERROR("mqtt-gw : connect : FATAL ERROR");
          retry = true;
          exit(1);
          break;
        case MOSQ_ERR_ERRNO:
            
          CLOG_ERROR("mqtt-gw : connect : ERROR : system message [" 
              << (ret_connect = errno) << "]");
          errno_system_description(ret_connect);
          // ROS_ERROR("mqtt-gw : connect : FATAL ERROR");
          // exit(1);
          break;
        default:
          CLOG_ERROR("mqtt-gw : connect : ERROR : unhandled mosquitto "
            "connection error code.");
          retry = true;
      }
    }while(retry);

  return true;
}

}
}//end of namespace
}//end of namespace wrappers