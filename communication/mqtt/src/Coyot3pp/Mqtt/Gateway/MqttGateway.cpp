/**
 * Creates a wrapper with the mosquitto mqtt client to abstract all
 *  operations to client instances.
 * 
 * @author : Ricardo GONZALEZ - Phong PHAM
 * @email : ricardogonalm@ltlab.net 
 *  
 * GPLv2+
 * */


#include <Coyot3pp/Mqtt/Gateway/MqttGateway.hpp>



namespace coyot3{
namespace communication{
namespace mqtt{


//ec::GatewayState

CYT3MACRO_enum_class_definitions(
  GatewayState
  ,
    ,NOT_CONFIGURED
    ,CONFIGURED
    ,CONNECTING
    ,CONNECTED
    ,DISCONNECTED
    ,CLOSED
    ,END_OF_LIFE
    ,ERROR
);


//ec::MessagePriority


CYT3MACRO_enum_class_definitions(
  MessagePriority
  ,
    ,DEFAULT
    ,LOW
    ,MEDIUM
    ,HIGH
)





const char* MqttGateway::MqttGatewayConfig::JsFields::debug_mode ="mqtt_broker_mode_debug";
const char* MqttGateway::MqttGatewayConfig::JsFields::broker_host ="mqtt_broker_address";
const char* MqttGateway::MqttGatewayConfig::JsFields::broker_port ="mqtt_broker_port";
const char* MqttGateway::MqttGatewayConfig::JsFields::clean_session ="mqtt_client_clean_session";
const char* MqttGateway::MqttGatewayConfig::JsFields::client_id ="mqtt_mosquitto_client_identifier";
const char* MqttGateway::MqttGatewayConfig::JsFields::client_id_add_random_string ="mqtt_mosquitto_client_identifier_add_random_string";
const char* MqttGateway::MqttGatewayConfig::JsFields::user = "mqtt_client_user";
const char* MqttGateway::MqttGatewayConfig::JsFields::password ="mqtt_client_password";
const char* MqttGateway::MqttGatewayConfig::JsFields::qos ="mqtt_client_qos";
const char* MqttGateway::MqttGatewayConfig::JsFields::qos_max ="mqtt_client_qos_max";
const char* MqttGateway::MqttGatewayConfig::JsFields::connection_timeout_msecs ="mqtt_client_timeout";
const char* MqttGateway::MqttGatewayConfig::JsFields::mosquitto_client_debug ="mqtt_mosquitto_debug_msgs";
const char* MqttGateway::MqttGatewayConfig::JsFields::priority_msgs_republish_interval ="priority_msgs_republish_interval";
const char* MqttGateway::MqttGatewayConfig::JsFields::priority_msgs_republish_max_retries ="priority_msgs_republish_max_retries";
const char* MqttGateway::MqttGatewayConfig::JsFields::file_ca = "mqtt_client_file_ca";
const char* MqttGateway::MqttGatewayConfig::JsFields::file_cert = "mqtt_client_file_cert";
const char* MqttGateway::MqttGatewayConfig::JsFields::file_key = "mqtt_client_file_key";
const char* MqttGateway::MqttGatewayConfig::JsFields::key_passphrase = "mqtt_client_keyfile_passphrase";
const char* MqttGateway::MqttGatewayConfig::JsFields::tls_version = "mqtt_broker_tls_version";
const char* MqttGateway::MqttGatewayConfig::JsFields::cyphers = "mqtt_broker_ciphers";

const bool MqttGateway::MqttGatewayConfig::Defaults::debug_mode = false;
const std::string MqttGateway::MqttGatewayConfig::Defaults::broker_host = "";
const uint32_t MqttGateway::MqttGatewayConfig::Defaults::broker_port = 0;
const bool MqttGateway::MqttGatewayConfig::Defaults::clean_session = true;
const std::string MqttGateway::MqttGatewayConfig::Defaults::client_id = "";
const bool MqttGateway::MqttGatewayConfig::Defaults::client_id_add_random_string = true;
const std::string MqttGateway::MqttGatewayConfig::Defaults::user = "";
const std::string MqttGateway::MqttGatewayConfig::Defaults::password = "";
const int MqttGateway::MqttGatewayConfig::Defaults::qos = 0;
const int MqttGateway::MqttGatewayConfig::Defaults::qos_max = 2;
const int64_t MqttGateway::MqttGatewayConfig::Defaults::connection_timeout_msecs = 20;
const bool MqttGateway::MqttGatewayConfig::Defaults::mosquitto_client_debug = false;
const int64_t MqttGateway::MqttGatewayConfig::Defaults::priority_msgs_republish_interval = 5;
const int64_t MqttGateway::MqttGatewayConfig::Defaults::priority_msgs_republish_max_retries = 5;
const std::string MqttGateway::MqttGatewayConfig::Defaults::file_ca = "";
const std::string MqttGateway::MqttGatewayConfig::Defaults::file_cert = "";
const std::string MqttGateway::MqttGatewayConfig::Defaults::file_key = "";
const std::string MqttGateway::MqttGatewayConfig::Defaults::key_passphrase = "";
const std::string MqttGateway::MqttGatewayConfig::Defaults::tls_version = "tlsv1.2";
const std::string MqttGateway::MqttGatewayConfig::Defaults::cyphers = "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384:DHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:DHE-RSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:DHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA384:DHE-RSA-AES256-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES256-SHA:ECDHE-RSA-AES256-SHA:DHE-RSA-AES256-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES128-SHA:DHE-RSA-AES128-SHA:RSA-PSK-AES256-GCM-SHA384:DHE-PSK-AES256-GCM-SHA384:RSA-PSK-CHACHA20-POLY1305:DHE-PSK-CHACHA20-POLY1305:ECDHE-PSK-CHACHA20-POLY1305:AES256-GCM-SHA384:PSK-AES256-GCM-SHA384:PSK-CHACHA20-POLY1305:RSA-PSK-AES128-GCM-SHA256:DHE-PSK-AES128-GCM-SHA256:AES128-GCM-SHA256:PSK-AES128-GCM-SHA256:AES256-SHA256:AES128-SHA256:ECDHE-PSK-AES256-CBC-SHA384:ECDHE-PSK-AES256-CBC-SHA:SRP-RSA-AES-256-CBC-SHA:SRP-AES-256-CBC-SHA:RSA-PSK-AES256-CBC-SHA384:DHE-PSK-AES256-CBC-SHA384:RSA-PSK-AES256-CBC-SHA:DHE-PSK-AES256-CBC-SHA:AES256-SHA:PSK-AES256-CBC-SHA384:PSK-AES256-CBC-SHA:ECDHE-PSK-AES128-CBC-SHA256:ECDHE-PSK-AES128-CBC-SHA:SRP-RSA-AES-128-CBC-SHA:SRP-AES-128-CBC-SHA:RSA-PSK-AES128-CBC-SHA256:DHE-PSK-AES128-CBC-SHA256:RSA-PSK-AES128-CBC-SHA:DHE-PSK-AES128-CBC-SHA:AES128-SHA:PSK-AES128-CBC-SHA256:PSK-AES128-CBC-SHA";


MqttGateway::MqttGatewayConfig::MqttGatewayConfig(){
  load_defaults();
}
MqttGateway::MqttGatewayConfig::MqttGatewayConfig(const MqttGateway::MqttGatewayConfig& o){
  *this = o;
}
MqttGateway::MqttGatewayConfig::~MqttGatewayConfig(){
  
}

MqttGateway::MqttGatewayConfig& 
MqttGateway::MqttGatewayConfig::operator=(const MqttGateway::MqttGatewayConfig& o)
{
  debug_mode = o.debug_mode;
  broker_host = o.broker_host;
  broker_port = o.broker_port;
  clean_session = o.clean_session;
  client_id = o.client_id;
  client_id_add_random_string = o.client_id_add_random_string;
  user = o.user;
  password = o.password;
  qos = o.qos;
  qos_max = o.qos_max;
  connection_timeout_msecs = o.connection_timeout_msecs;
  mosquitto_client_debug = o.mosquitto_client_debug;
  priority_msgs_republish_interval = o.priority_msgs_republish_interval;
  priority_msgs_republish_max_retries = o.priority_msgs_republish_max_retries;
  file_ca = o.file_ca;
  file_cert = o.file_cert;
  file_key = o.file_key;
  key_passphrase = o.key_passphrase;
  tls_version = o.tls_version;
  cyphers = o.cyphers;
  return *this;
}

bool 
MqttGateway::MqttGatewayConfig::operator==(const MqttGateway::MqttGatewayConfig& o){
  return (
       (debug_mode == o.debug_mode)
    && (broker_host == o.broker_host)
    && (broker_port == o.broker_port)
    && (clean_session == o.clean_session)
    && (client_id == o.client_id)
    && (client_id_add_random_string == o.client_id_add_random_string)
    && (user == o.user)
    && (password == o.password)
    && (qos == o.qos)
    && (qos_max == o.qos_max)
    && (connection_timeout_msecs == o.connection_timeout_msecs)
    && (mosquitto_client_debug == o.mosquitto_client_debug)
    && (priority_msgs_republish_interval == o.priority_msgs_republish_interval)
    && (priority_msgs_republish_max_retries == o.priority_msgs_republish_max_retries)
    && (file_ca == o.file_ca)
    && (file_cert == o.file_cert)
    && (file_key == o.file_key)
    && (key_passphrase == o.key_passphrase)
    && (tls_version == o.tls_version)
    && (cyphers == o.cyphers)
  );
}
void 
MqttGateway::MqttGatewayConfig::load_defaults(){
  debug_mode = Defaults::debug_mode;
  broker_host = Defaults::broker_host;
  broker_port = Defaults::broker_port;
  clean_session = Defaults::clean_session;
  client_id = Defaults::client_id;
  client_id_add_random_string = Defaults::client_id_add_random_string;
  user = Defaults::user;
  password = Defaults::password;
  qos = Defaults::qos;
  qos_max = Defaults::qos_max;
  connection_timeout_msecs = Defaults::connection_timeout_msecs;
  mosquitto_client_debug = Defaults::mosquitto_client_debug;
  priority_msgs_republish_interval = Defaults::priority_msgs_republish_interval;
  priority_msgs_republish_max_retries = Defaults::priority_msgs_republish_max_retries;
  file_ca = Defaults::file_ca;
  file_cert = Defaults::file_cert;
  file_key = Defaults::file_key;
  key_passphrase = Defaults::key_passphrase;
  tls_version = Defaults::tls_version;
  cyphers = Defaults::cyphers;
}

Json::Value 
MqttGateway::MqttGatewayConfig::toJson() const {
  Json::Value js;
  js[JsFields::debug_mode] = debug_mode;
  js[JsFields::broker_host] = broker_host;
  js[JsFields::broker_port] = broker_port;
  js[JsFields::clean_session] = clean_session;
  js[JsFields::client_id] = client_id;
  js[JsFields::client_id_add_random_string] = client_id_add_random_string;
  js[JsFields::user] = user;
  js[JsFields::password] = password;
  js[JsFields::qos] = qos;
  js[JsFields::qos_max] = qos_max;
  js[JsFields::connection_timeout_msecs] = connection_timeout_msecs;
  js[JsFields::mosquitto_client_debug] = mosquitto_client_debug;
  js[JsFields::priority_msgs_republish_interval] = priority_msgs_republish_interval;
  js[JsFields::priority_msgs_republish_max_retries] = priority_msgs_republish_max_retries;
  js[JsFields::file_ca] = file_ca;
  js[JsFields::file_cert] = file_cert;
  js[JsFields::file_key] = file_key;
  js[JsFields::key_passphrase] = key_passphrase;
  js[JsFields::tls_version] = tls_version;
  js[JsFields::cyphers] = cyphers;
  return js;
}

const char* 
MqttGateway::MqttGatewayConfig::get_help() const{
  std::stringstream s;
  s << " - mqtt-gateway configuration required fields: " << std::endl;
  s << " field: " << JsFields::debug_mode << " :() " << std::endl;
  s << " field: " << JsFields::broker_host << " :()" << std::endl;
  s << " field: " << JsFields::broker_port << " :()" << std::endl;
  s << " field: " << JsFields::clean_session << " :()" << std::endl;
  s << " field: " << JsFields::client_id << " :()" << std::endl;
  s << " field: " << JsFields::client_id_add_random_string << " :()" << std::endl;
  s << " field: " << JsFields::user << " :()" << std::endl;
  s << " field: " << JsFields::password << " :()" << std::endl;
  s << " field: " << JsFields::qos << " :()" << std::endl;
  s << " field: " << JsFields::qos_max << " :()" << std::endl;
  s << " field: " << JsFields::connection_timeout_msecs << " :()" << std::endl;
  s << " field: " << JsFields::mosquitto_client_debug << " :()" << std::endl;
  s << " field: " << JsFields::priority_msgs_republish_interval << " :()" << std::endl;
  s << " field: " << JsFields::priority_msgs_republish_max_retries << " :()" << std::endl;
  s << " field: " << JsFields::file_ca << " :()" << std::endl;
  s << " field: " << JsFields::file_cert << " :()" << std::endl;
  s << " field: " << JsFields::file_key << " :()" << std::endl;
  s << " field: " << JsFields::key_passphrase << " :()" << std::endl;
  s << " field: " << JsFields::tls_version << " :()" << std::endl;
  s << " field: " << JsFields::cyphers << " :()" << std::endl;
  s << std::endl;
  return s.str().c_str();
}

bool 
MqttGateway::MqttGatewayConfig::fromJson(const Json::Value& js){
  bool res = true;

  res &= coyot3::tools::json_import_value(js,JsFields::debug_mode,debug_mode);
  res &= coyot3::tools::json_import_value(js,JsFields::broker_host,broker_host);
  res &= coyot3::tools::json_import_value(js,JsFields::broker_port,broker_port);
  res &= coyot3::tools::json_import_value(js,JsFields::clean_session,clean_session);
  res &= coyot3::tools::json_import_value(js,JsFields::client_id,client_id);
  res &= coyot3::tools::json_import_value(js,JsFields::client_id_add_random_string,client_id_add_random_string);
  res &= coyot3::tools::json_import_value(js,JsFields::user,user);
  res &= coyot3::tools::json_import_value(js,JsFields::password,password);
  res &= coyot3::tools::json_import_value(js,JsFields::qos,qos);
  res &= coyot3::tools::json_import_value(js,JsFields::qos_max,qos_max);
  res &= coyot3::tools::json_import_value(js,JsFields::connection_timeout_msecs,connection_timeout_msecs);
  res &= coyot3::tools::json_import_value(js,JsFields::mosquitto_client_debug,mosquitto_client_debug);
  res &= coyot3::tools::json_import_value(js,JsFields::priority_msgs_republish_interval,priority_msgs_republish_interval);
  res &= coyot3::tools::json_import_value(js,JsFields::priority_msgs_republish_max_retries,priority_msgs_republish_max_retries);
  res &= coyot3::tools::json_import_value(js,JsFields::file_ca,file_ca);
  res &= coyot3::tools::json_import_value(js,JsFields::file_cert,file_cert);
  res &= coyot3::tools::json_import_value(js,JsFields::file_key,file_key);
  res &= coyot3::tools::json_import_value(js,JsFields::key_passphrase,key_passphrase);
  res &= coyot3::tools::json_import_value(js,JsFields::tls_version,tls_version);
  res &= coyot3::tools::json_import_value(js,JsFields::cyphers,cyphers);

  return res;
}


const char* 
MqttGateway::GatewayStateToString(GatewayState s)
{
  switch(s)
  {
    case GatewayState::NOT_CONFIGURED:return "NOT_CONFIGURED";break;
    case GatewayState::CONFIGURED:return "CONFIGURED";break;
    case GatewayState::CONNECTING:return "CONNECTING";break;
    case GatewayState::CONNECTED:return "CONNECTED";break;
    case GatewayState::DISCONNECTED:return "DISCONNECTED";break;
    case GatewayState::CLOSED:return "CLOSED";break;
    case GatewayState::END_OF_LIFE:return "END_OF_LIFE";break;
    case GatewayState::ERROR:return "ERROR";break;
    default:
        return "err_gateway_string_state_err";
  }
}

const char* 
MqttGateway::PriorityToString(Priority p)
{
  switch(p)
  {
    case Priority::DEFAULT:return "DEFAULT";break;
    case Priority::LOW:return "LOW";break;
    case Priority::MEDIUM:return "MEDIUM";break;
    case Priority::HIGH:return "HIGH";break;
    default:
        return "priority_unexpected_state";
  }
}




MqttGateway::MqttGateway()
: client(nullptr)
, gateway_controller_thread(nullptr)
, gateway_controller_keepalive_flag(false)
, queue_controller_thread(nullptr)
, queue_controller_keepalive_flag(false)
, _config_source()

, lock_interface_io_(false)
, stack_priority_messages_primary()
, stack_priority_messages_secondary()


{
    CLOG_INFO("MQTT GATEWAY INSTANCE CREATED");
}


MqttGateway::MqttGateway(const Json::Value& source)
: client(nullptr)
, gateway_controller_thread(nullptr)
, gateway_controller_keepalive_flag(false)
, queue_controller_thread(nullptr)
, queue_controller_keepalive_flag(false)
, lock_interface_io_(false)
,_config_source(source)
{
    CLOG_INFO("MQTT GATEWAY INSTANCE CREATED");
}

// destructor of the instance
MqttGateway::~MqttGateway()
{
    CLOG_INFO("mqtt-gw : destroying instance");
    if(gateway_state != GatewayState::DISCONNECTED)
    {
        //cleans the instance
        disconnect_from_broker();
    }
}

bool MqttGateway::lock_interface_io() const{
  return lock_interface_io_;
}
bool MqttGateway::lock_interface_io(bool l){
  return lock_interface_io_ = l;
}

void MqttGateway::set_debug_level(int debugLevel)
{
  CLOG_DEBUG_LEVEL_SET(debugLevel);
  _debug_level_set_mod_msg(debugLevel);
  _debug_level_set_mod_callbacks(debugLevel);
  _debug_level_set_mod_connects(debugLevel);
  _debug_level_set_mod_configurations(debugLevel);
  _debug_level_set_mod_controllers(debugLevel);
}




bool 
MqttGateway::disconnect_from_broker()
{
  std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
  if(config.debug_mode() == true){
    CLOG_INFO("mqtt-gw : disconnect : DEBUG MODE ACTIVATED - no client"
        "has been launched")
    return true;
  }
  if(client == nullptr)
  {
    CLOG_ERROR("mqtt-gw : disconnect : ERROR, NO CLIENT SEEMS TO BE "
        "CONFIGURED")
    return false;
  }
  int res_loop_stop = mosquitto_loop_stop(client,true);
  switch(res_loop_stop)
  {
    case MOSQ_ERR_SUCCESS:
        CLOG_INFO("mqtt-gw : disconnect from broker : loop stop OK")
    default:
        CLOG_WARN("mqtt-gw : disconnect from broker : loop stop error")
  }


  int res_discon = mosquitto_disconnect(client);


  switch(res_discon)
  {
    case MOSQ_ERR_SUCCESS : 
        CLOG_INFO("mqtt-gw : disconnect : successfully disconnected.")
        break;
    case MOSQ_ERR_INVAL:
        CLOG_WARN("mqtt-gw : disconnect : WARNING : invalid input")
        break;
    case MOSQ_ERR_NO_CONN:
        CLOG_WARN("mqtt-gw : disconnect : WARNING : the client was not "
            "connected")
        break;
    default:
        CLOG_WARN("mqtt-gw : disconnect : WARNING : unhandled error "
            "while disconnecting from the broker : mosq_err [%d]"
            << res_discon)
  }
  this->gateway_state = GatewayState::CLOSED;

  CLOG_INFO("mqtt-gw : disconnect : destroying mosquitto instance")

  
  mosquitto_destroy(client);


  client = nullptr;
  return (res_discon == MOSQ_ERR_SUCCESS);
}

bool 
MqttGateway::subscribe_to_topic(const std::string& t,
                    MqttGateway::OnMessageCallback cb,
                    MqttGateway::Priority p)
{
  TopicSubscriptionsStack::iterator it = topic_subs_collection.find(t.c_str());
  if(t.size() == 0)
  {
    CLOG_WARN("mqtt-gw : add-subscription : ERROR!!! : trying to make a "
      "subscription to an empty topic!!! this subscription is being ignored")
    return false;
  }
  if(p == Priority::DEFAULT){
    p = static_cast<Priority>(config.qos());
  }
  int q = static_cast<int>(p);
  if(it == topic_subs_collection.end()){
      CLOG_INFO("mqtt-gw : subscription : creating subscription to topic :"
          << t)
      OnMessageCallbackCollection newCallbackCollection;
      newCallbackCollection.push_back(cb);
      topic_subs_collection.insert(std::make_pair(t,newCallbackCollection));
      topic_qos_relations.insert(std::make_pair(t,q));
      topic_qos_rel_todo.insert(std::make_pair(t,q));
  }else{
      CLOG_INFO("mqtt-gw : subscription : updating callbacks list for "
          "topic " << t)
      topic_subs_collection[t.c_str()].push_back(cb);
      mosq_unsubscribe(t);
      if(topic_qos_relations[t] < q){
          topic_qos_relations[t] = q;

          if(topic_qos_rel_todo.find(t) != topic_qos_rel_todo.end()){
              topic_qos_rel_todo[t] = q;
          }else{
              mosq_unsubscribe(t);
              topic_qos_rel_todo.insert(std::make_pair(t,q));
          }
      }
  }   
  return true;
}

bool 
MqttGateway::set_max_qos_level(int l)
{
    if(l < 0 || l> 2){
        CLOG_ERROR("mqtt-gw : set-max-qos-level : trying to set max-qos with "
        "an invalid specification! [" << l << "]!!!")
        return false;
    }
    CLOG_INFO("mqtt-gw : set-max-qos-level : setting max QoS to : " << l)
    config.qos_max(l);
    return true;
}


bool 
MqttGateway::make_mosq_subscriptions()
{
  
  if(config.debug_mode() == true)
  {
      return true;
  }
  if(client == nullptr)
  {
      CLOG_ERROR("mqtt-gw : make subscriptions : the client is not connected "
        "/ created")
      return false;
  }

  if(topic_qos_rel_todo.size() == 0)
  {
      CLOG_WARN("mqtt-gw : make subscriptions : no subscriptions for this "
        "instance")
      return true;
  }
  
  CLOG_INFO("mqtt-gw : make subscriptions : going to subscribe to [" 
    << topic_qos_rel_todo.size() << "] topics.")
  uint  doneOk = 0;
  uint queueOrigSize = topic_qos_rel_todo.size();
  for(TopicQosRelations::iterator it = topic_qos_rel_todo.begin();
      it!=topic_qos_rel_todo.end();)
  {
    usleep(200);
    std::string t = it->first;
    if(t.size() == 0){
      CLOG_WARN("MQTT-GW : MAKE-MOSQ-SUBSCRIPTIONS : ERROR! : CURRENT TOPIC "
        "SEEMS EMPTY! MUST DEBUG! NOW IGNORING")
      TopicQosRelations::iterator toDel = it;
      it++;
      topic_qos_rel_todo.erase(toDel);
      continue;
    }
    int         q = it->second;
    int subscription_id;
    bool subscription_ok = false;
    int sub_ret;
    CLOG_INFO("mqtt-gw : make mosq subscriptions : subscribing with QoS [" 
      << q << "] to topic [" << t << "] - mtx")
    {
      std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
      if(q < 0 || q > 2)
      {
        CLOG_DEBUG(1,"mqtt-gw : pulse-mosq-subscribe : WARNING! for topic [" 
          << t << "] there is a wronq QoS specification! setting to default [" 
          << config.qos() << "]")
        q = config.qos();
      }
      if(q > config.qos_max())
      {
        CLOG_DEBUG(2,"mqtt-gw : pulse-mosq-subscribe : WARNING! for topic [" 
          << t << "] the QoS target level is superior than the maximum "
          "permitted for this client-instance. Adjusting (" << q << ">" << 
          config.qos_max() << ")")

        q = config.qos_max();
      }

      {
        //std::lock_guard<std::mutex> guard(mtx_mqtt_publish); //bug
        sub_ret = mosquitto_subscribe(client,&subscription_id,t.c_str(),q);
      }
    
    }
    
    switch(sub_ret)
    {
      case MOSQ_ERR_SUCCESS:
        subscription_ok = true;
        CLOG_INFO("mqtt-gw : mosq sub : success qos [" << q << 
            "] at topic [" << t << "] with subscription ID [" << 
            subscription_id << "]")
        break;
      case MOSQ_ERR_INVAL:
        CLOG_ERROR("mqtt-gw : mosq sub : ERROR . invalid params to "
            "subscribe to [" << t << "] with QoS [" << q << "]")
        break;
      case MOSQ_ERR_NOMEM:
        CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! NO MEMORY WHEN "
            "SUBSCRIBING TO [" << t << "]")
        break;
      case MOSQ_ERR_NO_CONN:
        CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! no connection when "
            "subscribing to topic [" << t << "]")
        break;
      case MOSQ_ERR_MALFORMED_UTF8:
        CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! malformed UTF-8 "
            "chain when refering to topic ["<< t << "]")
        break;
      default:
        CLOG_WARN("mqtt-gw : mosq sub : ERROR !!! unexpected error "
          "while  subscribing to [" << t << "]")
    }
    if(subscription_ok)
    {
      ++doneOk;
      //remove topic from queue to do.
      TopicQosRelations::iterator toDel = it;
      it++;
      topic_qos_rel_todo.erase(toDel);
    }else{
      it++;
    }

    if(it == topic_qos_rel_todo.end())
    {
      break;
    }
  }
  if(doneOk == queueOrigSize)
  {
      CLOG_INFO("mqtt-gw : topic subscriptions : correctly subscribed " 
          "to [" << doneOk << "] topics")
  }
  else
  {
      CLOG_WARN("mqtt-gw : topic subscriptions : original queue size "
          "[" << queueOrigSize << "] but only [" << doneOk << "] were " 
          "done")
  }
  return true;
}
bool 
MqttGateway::pulse_mosq_subscribe()
{
  std::string t;
  int q;
  if(config.debug_mode() == true){
    CLOG_INFO("mqtt-gw : pulse-mosq-subscribe : DEBUG-MODE-ON : no "
      "subscriptions to be done.")
    return true;
  }

  if(client == nullptr){
      CLOG_ERROR("mqtt-gw : pulse-mosq-subscribe : the client is not "
      "connected.")
      return false;
  }
  TopicQosRelations::iterator it = topic_qos_rel_todo.begin();
  if(it == topic_qos_rel_todo.end()){
      //all subscribed;
      return false;
  }
  CLOG_INFO("mqtt-gw : pulse-mosq-subscribe : subscribing with QoS [" << q 
    << "] to topic [" << t << "]")
  t = it->first;
  q = it->second;
  if(q < 0 || q > 2){
    CLOG_WARN("mqtt-gw : pulse-mosq-subscribe : WARNING! for topic [" << t 
    << "] there is a wronq QoS specification! (" << q << ") setting to default "
    "[" << config.qos() << "]")
    q = config.qos();
  }

  if(q > config.qos_max()){
    CLOG_WARN("mqtt-gw : pulse-mosq-subscribe : WARNING! for topic [" << t
      << "] the QoS target level is superior than the maximum "
      "permitted for this client-instance. Adjusting (" << q << ">" << 
      config.qos_max() << ")")
    q = config.qos_max();
  }

  int subscription_id; //for future version, to ensure the subscription.
  int sub_ret;
  
  {
    CLOG_INFO("mqtt-gw : pulse-mosq-subscribe : making subscription "
      "(lock-guard-in)")
    std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
    sub_ret = mosquitto_subscribe(client,&subscription_id,t.c_str(),q);
    CLOG_INFO("mqtt-gw : pulse-mosq-subscribe : making subscription "
      "(lock-guard-out)")
  }
  
  bool toRet = false;
  switch(sub_ret)
  {
    case MOSQ_ERR_SUCCESS:
      toRet = true;
      CLOG_INFO("mqtt-gw : mosq sub : success qos [" << q << "," << t << "]")
      break;
    case MOSQ_ERR_INVAL:
      CLOG_ERROR("mqtt-gw : mosq sub : ERROR . invalid params to "
          "subscribe to [" << t << "] with QoS [" << q << "]")
      break;
    case MOSQ_ERR_NOMEM:
      CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! NO MEMORY WHEN "
          "SUBSCRIBING TO [" << t << "]")
      break;
    case MOSQ_ERR_NO_CONN:
      CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! no connection when "
          "subscribing to topic [" << t << "]")
      break;
    case MOSQ_ERR_MALFORMED_UTF8:
      CLOG_ERROR("mqtt-gw : mosq sub : ERROR !!! malformed UTF-8 "
          "chain when refering to topic [" << t << "]")
      break;
  }
  if(!toRet){
    //TO-DO : ? repeat subscription call in some seconds.
    topic_qos_rel_todo.erase(topic_qos_rel_todo.find(t));
  }
  return toRet;
}

/**
 * @brief: once reconnected after a full reset of the mosquitto client, 
 *         it will republish all the messages of the queue. These messages 
 *         will be managed at the primary queue.
 * */
bool 
MqttGateway::republish_secondary_queue()
{
    int pos= 0;
    //std::lock_guard<std::mutex> guard(mtx_secondary_queue);
    size_t done = 0,size = stack_priority_messages_secondary.size();
    done = stack_priority_messages_secondary.forEach([&](MqttMessage& msg){
      usleep(50);
      return publish(msg.topic,msg.payload,msg.size,msg.prio);
            
    });
    stack_priority_messages_secondary.clear();
    return (done == size);
}

//
bool 
MqttGateway::mosq_unsubscribe(const std::string& t)
{
    if(config.debug_mode())
    {
        return true;
    }
    bool ret = false;
    int unsub_id;
    int unsub_ret = mosquitto_unsubscribe(client,&unsub_id,t.c_str());
    switch(unsub_ret)
    {
        case MOSQ_ERR_SUCCESS :
            ret = true; 
            CLOG_INFO("mqtt-gw : unsubscribe : successfully "
                "unsubscribed from topic [" << t << "]")
            break;
        case MOSQ_ERR_INVAL : 
            CLOG_WARN("mqtt-gw : unsubscribe : topic [" << t << "]"
                "input data is corrupt!")
            break;
        case MOSQ_ERR_NOMEM:
            CLOG_ERROR("mqtt-gw : unsubscribe : topic [" << t << "] ERROR!!! "
                "NOT ENAUGH MEMORY!!!")
            break;
        case MOSQ_ERR_NO_CONN:
            CLOG_ERROR("mqtt-gw : unsubscribe : topic [" << t << "] WARN! no "
                "connection to the broker")
            break;
        case MOSQ_ERR_MALFORMED_UTF8:
            CLOG_ERROR("mqtt-gw : unsubscribe : topic [" << t << "] ERROR! "
                "input string is not a valid UTF-8 string")
            break;
        default:
            CLOG_ERROR("mqtt-gw : unsubscribe : topic [" << t << "] unexpected "
                "behave while unsubscribing mosquitto client.")
    }
    if(!ret){
        //TO-DO : repeat unsubscription call in some seconds.
    }
    return ret;
}

bool 
MqttGateway::publish(const std::string& t,const std::string& m,Priority p)
{
  return publish(t,reinterpret_cast<const uint8_t*>(m.c_str()),static_cast<int>(m.size()),p);
}

bool 
MqttGateway::publish(const std::string& t,const ByteStream& p,Priority P)
{
  return publish(t,p.c_str(),p.size(),P);
}
bool 
MqttGateway::publish(const std::string& t,const Json::Value& s,Priority p)
{
  std::stringstream sstr;
  sstr << s;
  return publish(t,sstr.str(),p);
}

int __cyt_count_maxqos_warn = 1000;
bool 
MqttGateway::publish(const std::string& t,
                            const uint8_t* payload,
                            int payload_size,
                            Priority p)
{
    static int debug_num_sents = 0;
    CLOG_INFO("lock interface [" << lock_interface_io() << "]")
    if(lock_interface_io() == true){
      CLOG_DEBUG(5,"mqtt-gw : publish : activity is locked for this "
      "connector. Ignoring publication of msg at topic [" << t << "]")
      return true;
    }
    ++debug_num_sents;
    if(debug_num_sents > 50){
        CLOG_DEBUG(2,"mqtt-gw : publish : a group of 50 messages has "
            "been sent. current is publishing at topic [" << t << "]")
            debug_num_sents = 0;
    }
    if(config.debug_mode() == true){
        return true;
    }

        
    int q = static_cast<int>(p); //just in case
    if((q < 0) || (q > 2))
    {
        if(q != -1){
            CLOG_WARN("mqtt-gw : publish : WARN : trying to publish with an"
                " invalid QoS")
        }
        q = config.qos();
    }
    if(q > config.qos_max()){
      CLOG_WARN_THEN_SILENCE(__cyt_count_maxqos_warn,"mqtt-gw : "
        "pulse-mosq-subscribe : WARNING! for topic [" << t
        << "] the QoS target level is superior than the maximum "
        "permitted for this client-instance. Adjusting (" << q << ">" << 
        config.qos_max() << ")")
      q = config.qos_max();
    }

    int message_id;
    int pub_result;
        
    if ((q != 0))   // && ( (reset == false))) // || ((reset == true) && ((gateway_state == GatewayState::DISCONNECTED) || (gateway_state == GatewayState::CONNECTING)) )))
    {   
         
        //MqttMessage new_message;
        // new_message.size = static_cast<int>(payload_size);
        // new_message.topic = t;
        // new_message.payload = new uint8_t[payload_size];
        message_id = -1;
        //memcpy(new_message.payload,payload,payload_size);   

        //new_message.prio = MqttGateway::Priority::HIGH; // just for test

        {
          std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
          pub_result = mosquitto_publish(
            client,
            &message_id,
            t.c_str(),
            static_cast<int>(payload_size),
            static_cast<const void*>(payload),
            q,false);
        }
        CLOG_DEBUG(3,"mqtt-gw : publish : storing priority message mqttid: "
          << message_id);
        MqttMessage msg(message_id,t,payload,payload_size,p);
        stack_priority_messages_primary.push(msg);
        
        // new_message.messageID = message_id;
        // {
        //   std::lock_guard<std::mutex> guard(mtx_republish_queue);
        //   CLOG_INFO("mqtt-gw : publish : priority message pushed to queue[" << 
        //         queue_priority_messages_primary.size() << "] ID[" << message_id << 
        //         "] with Priority[" << (q==1?"MEDIUM":"HIGH") << "]");
        //     queue_priority_messages_primary.push_back(new_message);
        // }
        
    }else{
        std::lock_guard<std::mutex> guard(mtx_mqtt_publish);
        
        pub_result = mosquitto_publish(
            client,
            &message_id,
            t.c_str(),
            static_cast<int>(payload_size),
            static_cast<const void*>(payload),
            q,false);
    }
    int ret = false;

    switch(pub_result)
    {
        case MOSQ_ERR_SUCCESS:
            ret = true;
            break;
        case MOSQ_ERR_INVAL:
            CLOG_WARN("mqtt-gw : publish : WARN - invalid input params to "
                "send a stream (size=" << payload_size << ") to topic [" <<
                t << "]")
            break;
        case MOSQ_ERR_NO_CONN:
            CLOG_WARN("mqtt-gw : no connection to mqtt broker to send "
                "stream (size = " << payload_size << ") to topic [" << 
                t << "]")
            break;
        case MOSQ_ERR_PROTOCOL:
            CLOG_WARN("mqtt-gw : publish : PROTOCOL ERROR COMMUNICATING"
                " with broker while sending paload (size = " << payload_size << 
                ") to topic[" << t << "]")
            break;
        case MOSQ_ERR_PAYLOAD_SIZE:
            CLOG_WARN("mqtt-gw : publish : error! message to big (size="
                << payload_size << ") while publishing to topic " << t)
            break;
        case MOSQ_ERR_MALFORMED_UTF8:
            CLOG_WARN("mqtt-gw : publish! ERROR! topic is an invalid "
                "UTF-8 descriptor [" << t << "]")
            break;
        default:
            CLOG_WARN("mqtt-gw : publish : unhandled error while publishing"
                " a payload (size=" << payload_size <<") to topic " << t)
    }

    return ret;
}





bool 
MqttGateway::Init()
{
  CLOG_INFO("mqtt-gw : init : begin");
  return load_configuration();
}



bool 
MqttGateway::Start()
{
    CLOG_INFO("mqtt-gw : starting / launching instance")

    CLOG_INFO("mqtt-gw : launching gateway controller")
    gateway_controller();

    CLOG_INFO("mqtt-gw : launching queue controller")
    queue_controller();


    return connect_to_broker();
}



bool 
MqttGateway::Stop()
{
    CLOG_INFO("mqtt-gw : stopping instance");
    gateway_controller_keepalive_flag = false;
    queue_controller_keepalive_flag = false;
    if(this->gateway_controller_thread!=nullptr)
    {
        CLOG_INFO("mqtt-gw : Stop : waiting gateway controller thread to end")
        gateway_controller_thread->join();
        delete gateway_controller_thread;
        gateway_controller_thread = nullptr;

    }
    if(this->queue_controller_thread!=nullptr)
    {
        CLOG_INFO("mqtt-gw : Stop : waiting priority messages queue controller "
          "thread to end")
        queue_controller_thread->join();
        delete gateway_controller_thread;
        gateway_controller_thread = nullptr;
    }
    return disconnect_from_broker();
}

bool 
MqttGateway::set_debug_mode(bool option)
{
    config.debug_mode(option);
    CLOG_INFO("mqtt-gw : setting debug mode option to [" 
      << (config.debug_mode() == true?"true":"false"))
    return config.debug_mode();
}

bool 
MqttGateway::full_reset_connection()
{       
     
    CLOG_INFO("mqtt-gw : controller : closing GW")
    mosqclient_full_reset_flag = true;

    disconnect_from_broker();
    copy_priority_messages_to_secondary_queue();
    CLOG_INFO("mqtt-gw : controller : CONNECTING")
    
    topic_qos_rel_todo.clear();
    TopicQosRelations::iterator it;
    for(it = topic_qos_relations.begin();it != topic_qos_relations.end(); ++it)
    {
        topic_qos_rel_todo.insert(std::make_pair(it->first,it->second));
    }

    connect_to_broker();
    // reset = false;
    CLOG_INFO("mqtt-gw : controller : DONE")
    return true;
}


bool MqttGateway::copy_priority_messages_to_secondary_queue()
{
  stack_priority_messages_secondary = stack_priority_messages_primary;
  stack_priority_messages_primary.clear();
  return true;
}

}
}//end of namespace wrappers
}//end of namespace coyot3;


