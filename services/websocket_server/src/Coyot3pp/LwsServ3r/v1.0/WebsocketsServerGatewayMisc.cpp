#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>



namespace coyot3{
namespace services{
namespace websocket{

  const char* WebsocketsServerGateway::CallbackReasonToString(int reason)
  {
    switch(reason)
    {  
      case LWS_CALLBACK_PROTOCOL_INIT : 
        return "LWS_CALLBACK_PROTOCOL_INIT";break;
      case LWS_CALLBACK_PROTOCOL_DESTROY : 
        return "LWS_CALLBACK_PROTOCOL_DESTROY";break;
      case LWS_CALLBACK_WSI_CREATE : 
        return "LWS_CALLBACK_WSI_CREATE";break;
      case LWS_CALLBACK_WSI_DESTROY : 
        return "LWS_CALLBACK_WSI_DESTROY";break;
      case LWS_CALLBACK_WSI_TX_CREDIT_GET : 
        return "LWS_CALLBACK_WSI_TX_CREDIT_GET";break;
      case LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS : 
        return "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS";break;
      case LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS : 
        return "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS";break;
      case LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION : 
        return "LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION";break;
      /*case LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY : 
        return "LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY";break;*/
      case LWS_CALLBACK_SSL_INFO : 
        return "LWS_CALLBACK_SSL_INFO";break;
      case LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION : 
        return "LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION";break;
      case LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED : 
        return "LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED";break;
      case LWS_CALLBACK_HTTP : 
        return "LWS_CALLBACK_HTTP";break;
      case LWS_CALLBACK_HTTP_BODY : 
        return "LWS_CALLBACK_HTTP_BODY";break;
      case LWS_CALLBACK_HTTP_BODY_COMPLETION : 
        return "LWS_CALLBACK_HTTP_BODY_COMPLETION";break;
      case LWS_CALLBACK_HTTP_FILE_COMPLETION : 
        return "LWS_CALLBACK_HTTP_FILE_COMPLETION";break;
      case LWS_CALLBACK_HTTP_WRITEABLE : 
        return "LWS_CALLBACK_HTTP_WRITEABLE";break;
      case LWS_CALLBACK_CLOSED_HTTP : 
        return "LWS_CALLBACK_CLOSED_HTTP";break;
      case LWS_CALLBACK_FILTER_HTTP_CONNECTION : 
        return "LWS_CALLBACK_FILTER_HTTP_CONNECTION";break;
      case LWS_CALLBACK_ADD_HEADERS : 
        return "LWS_CALLBACK_ADD_HEADERS";break;
      case LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION : 
        return "LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION";break;
      case LWS_CALLBACK_CHECK_ACCESS_RIGHTS : 
        return "LWS_CALLBACK_CHECK_ACCESS_RIGHTS";break;
      case LWS_CALLBACK_PROCESS_HTML : 
        return "LWS_CALLBACK_PROCESS_HTML";break;
      case LWS_CALLBACK_HTTP_BIND_PROTOCOL : 
        return "LWS_CALLBACK_HTTP_BIND_PROTOCOL";break;
      case LWS_CALLBACK_HTTP_DROP_PROTOCOL : 
        return "LWS_CALLBACK_HTTP_DROP_PROTOCOL";break;
      case LWS_CALLBACK_HTTP_CONFIRM_UPGRADE : 
        return "LWS_CALLBACK_HTTP_CONFIRM_UPGRADE";break;
      case LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP : 
        return "LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP";break;
      case LWS_CALLBACK_CLOSED_CLIENT_HTTP : 
        return "LWS_CALLBACK_CLOSED_CLIENT_HTTP";break;
      case LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ : 
        return "LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ";break;
      case LWS_CALLBACK_RECEIVE_CLIENT_HTTP : 
        return "LWS_CALLBACK_RECEIVE_CLIENT_HTTP";break;
      case LWS_CALLBACK_COMPLETED_CLIENT_HTTP : 
        return "LWS_CALLBACK_COMPLETED_CLIENT_HTTP";break;
      case LWS_CALLBACK_CLIENT_HTTP_WRITEABLE : 
        return "LWS_CALLBACK_CLIENT_HTTP_WRITEABLE";break;
      case LWS_CALLBACK_CLIENT_HTTP_REDIRECT : 
        return "LWS_CALLBACK_CLIENT_HTTP_REDIRECT";break;
      case LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL : 
        return "LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL";break;
      case LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL : 
        return "LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL";break;
      case LWS_CALLBACK_ESTABLISHED : 
        return "LWS_CALLBACK_ESTABLISHED";break;
      case LWS_CALLBACK_CLOSED : 
        return "LWS_CALLBACK_CLOSED";break;
      case LWS_CALLBACK_SERVER_WRITEABLE : 
        return "LWS_CALLBACK_SERVER_WRITEABLE";break;
      case LWS_CALLBACK_RECEIVE : 
        return "LWS_CALLBACK_RECEIVE";break;
      case LWS_CALLBACK_RECEIVE_PONG : 
        return "LWS_CALLBACK_RECEIVE_PONG";break;
      case LWS_CALLBACK_WS_PEER_INITIATED_CLOSE : 
        return "LWS_CALLBACK_WS_PEER_INITIATED_CLOSE";break;
      case LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION : 
        return "LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION";break;
      case LWS_CALLBACK_CONFIRM_EXTENSION_OKAY : 
        return "LWS_CALLBACK_CONFIRM_EXTENSION_OKAY";break;
      case LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL : 
        return "LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL";break;
      case LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL : 
        return "LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL";break;
      case LWS_CALLBACK_CLIENT_CONNECTION_ERROR : 
        return "LWS_CALLBACK_CLIENT_CONNECTION_ERROR";break;
      case LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH : 
        return "LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH";break;
      case LWS_CALLBACK_CLIENT_ESTABLISHED : 
        return "LWS_CALLBACK_CLIENT_ESTABLISHED";break;
      case LWS_CALLBACK_CLIENT_CLOSED : 
        return "LWS_CALLBACK_CLIENT_CLOSED";break;
      case LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER : 
        return "LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER";break;
      case LWS_CALLBACK_CLIENT_RECEIVE : 
        return "LWS_CALLBACK_CLIENT_RECEIVE";break;
      case LWS_CALLBACK_CLIENT_RECEIVE_PONG : 
        return "LWS_CALLBACK_CLIENT_RECEIVE_PONG";break;
      case LWS_CALLBACK_CLIENT_WRITEABLE : 
        return "LWS_CALLBACK_CLIENT_WRITEABLE";break;
      case LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED : 
        return "LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED";break;
      case LWS_CALLBACK_WS_EXT_DEFAULTS : 
        return "LWS_CALLBACK_WS_EXT_DEFAULTS";break;
      case LWS_CALLBACK_FILTER_NETWORK_CONNECTION : 
        return "LWS_CALLBACK_FILTER_NETWORK_CONNECTION";break;
      case LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL : 
        return "LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL";break;
      case LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL : 
        return "LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL";break;
      case LWS_CALLBACK_GET_THREAD_ID : 
        return "LWS_CALLBACK_GET_THREAD_ID";break;
      case LWS_CALLBACK_ADD_POLL_FD : 
        return "LWS_CALLBACK_ADD_POLL_FD";break;
      case LWS_CALLBACK_DEL_POLL_FD : 
        return "LWS_CALLBACK_DEL_POLL_FD";break;
      case LWS_CALLBACK_CHANGE_MODE_POLL_FD : 
        return "LWS_CALLBACK_CHANGE_MODE_POLL_FD";break;
      case LWS_CALLBACK_LOCK_POLL : 
        return "LWS_CALLBACK_LOCK_POLL";break;
      case LWS_CALLBACK_UNLOCK_POLL : 
        return "LWS_CALLBACK_UNLOCK_POLL";break;
      case LWS_CALLBACK_CGI : 
        return "LWS_CALLBACK_CGI";break;
      case LWS_CALLBACK_CGI_TERMINATED : 
        return "LWS_CALLBACK_CGI_TERMINATED";break;
      case LWS_CALLBACK_CGI_STDIN_DATA : 
        return "LWS_CALLBACK_CGI_STDIN_DATA";break;
      case LWS_CALLBACK_CGI_STDIN_COMPLETED : 
        return "LWS_CALLBACK_CGI_STDIN_COMPLETED";break;
      case LWS_CALLBACK_CGI_PROCESS_ATTACH : 
        return "LWS_CALLBACK_CGI_PROCESS_ATTACH";break;
      case LWS_CALLBACK_SESSION_INFO : 
        return "LWS_CALLBACK_SESSION_INFO";break;
      case LWS_CALLBACK_GS_EVENT : 
        return "LWS_CALLBACK_GS_EVENT";break;
      case LWS_CALLBACK_HTTP_PMO : 
        return "LWS_CALLBACK_HTTP_PMO";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_RX : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_RX";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_RX : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_RX";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_CLOSE : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_CLOSE";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_CLOSE : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_CLOSE";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_ADOPT : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_ADOPT";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_ADOPT : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_ADOPT";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL";break;
      case LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL : 
        return "LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL";break;
      case LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL : 
        return "LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL";break;
      case LWS_CALLBACK_RAW_RX : 
        return "LWS_CALLBACK_RAW_RX";break;
      case LWS_CALLBACK_RAW_CLOSE : 
        return "LWS_CALLBACK_RAW_CLOSE";break;
      case LWS_CALLBACK_RAW_WRITEABLE : 
        return "LWS_CALLBACK_RAW_WRITEABLE";break;
      case LWS_CALLBACK_RAW_ADOPT : 
        return "LWS_CALLBACK_RAW_ADOPT";break;
      case LWS_CALLBACK_RAW_CONNECTED : 
        return "LWS_CALLBACK_RAW_CONNECTED";break;
      case LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL : 
        return "LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL";break;
      case LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL : 
        return "LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL";break;
      case LWS_CALLBACK_RAW_ADOPT_FILE : 
        return "LWS_CALLBACK_RAW_ADOPT_FILE";break;
      case LWS_CALLBACK_RAW_RX_FILE : 
        return "LWS_CALLBACK_RAW_RX_FILE";break;
      case LWS_CALLBACK_RAW_WRITEABLE_FILE : 
        return "LWS_CALLBACK_RAW_WRITEABLE_FILE";break;
      case LWS_CALLBACK_RAW_CLOSE_FILE : 
        return "LWS_CALLBACK_RAW_CLOSE_FILE";break;
      case LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL : 
        return "LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL";break;
      case LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL : 
        return "LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL";break;
      case LWS_CALLBACK_TIMER : 
        return "LWS_CALLBACK_TIMER";break;
      case LWS_CALLBACK_EVENT_WAIT_CANCELLED : 
        return "LWS_CALLBACK_EVENT_WAIT_CANCELLED";break;
      case LWS_CALLBACK_CHILD_CLOSING : 
        return "LWS_CALLBACK_CHILD_CLOSING";break;
      case LWS_CALLBACK_VHOST_CERT_AGING : 
        return "LWS_CALLBACK_VHOST_CERT_AGING";break;
      case LWS_CALLBACK_VHOST_CERT_UPDATE : 
        return "LWS_CALLBACK_VHOST_CERT_UPDATE";break;
      case LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED : 
        return "LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED";break;
      case LWS_CALLBACK_MQTT_IDLE : 
        return "LWS_CALLBACK_MQTT_IDLE";break;
      case LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED : 
        return "LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED";break;
      case LWS_CALLBACK_MQTT_SUBSCRIBED : 
        return "LWS_CALLBACK_MQTT_SUBSCRIBED";break;
      case LWS_CALLBACK_MQTT_CLIENT_WRITEABLE : 
        return "LWS_CALLBACK_MQTT_CLIENT_WRITEABLE";break;
      case LWS_CALLBACK_MQTT_CLIENT_RX : 
        return "LWS_CALLBACK_MQTT_CLIENT_RX";break;
      case LWS_CALLBACK_MQTT_UNSUBSCRIBED : 
        return "LWS_CALLBACK_MQTT_UNSUBSCRIBED";break;
      case LWS_CALLBACK_MQTT_DROP_PROTOCOL : 
        return "LWS_CALLBACK_MQTT_DROP_PROTOCOL";break;
      case LWS_CALLBACK_MQTT_CLIENT_CLOSED : 
        return "LWS_CALLBACK_MQTT_CLIENT_CLOSED";break;
      case LWS_CALLBACK_MQTT_ACK : 
        return "LWS_CALLBACK_MQTT_ACK";break;
      case LWS_CALLBACK_MQTT_RESEND : 
        return "LWS_CALLBACK_MQTT_RESEND";break;
      case LWS_CALLBACK_USER : 
        return "LWS_CALLBACK_USER";break;
      default:
        return "--UNKNOWN-LWS-CALLBACK-REASON--";
    }

  }




  void WebsocketsServerGateway::set_debug_level_mmisc_(int level)
  {
    CLOG_DEBUG_LEVEL_SET(level);
  }
  std::string WebsocketsServerGateway::name()
  {
    return std::string(_name);
  }
  bool WebsocketsServerGateway::setName(const std::string& n)
  {
    if(_name){delete _name;}
    _name = new(std::nothrow) char[n.size()];
    if(!_name)return false;
    strcpy(_name,n.c_str());
    return true;
  }


  bool 
  WebsocketsServerGateway::setServerPort(int p)
  {
    _server_port = p;
    return true;
  }

  void 
  WebsocketsServerGateway::_clean_mounts()
  {
    CLOG_DEBUG(5,"CYTWSG : " << _name << " : clean mounts : init");
    for(std::vector<struct lws_http_mount*>::iterator 
          it=_mounts_collection.begin();
          it != _mounts_collection.end();
          ++it)
    {
      if( (*it)->mountpoint )delete (*it)->mountpoint;
      if( (*it)->origin )delete (*it)->origin;
      if( (*it)->def )delete (*it)->def;
      delete *it;
    }
  }

  bool 
  WebsocketsServerGateway::_push_mount(struct lws_http_mount* m)
  {
    if(_mounts_collection.size() && m)
    {
      _mounts_collection[_mounts_collection.size() - 1]->mount_next = m;
    }
    
    _mounts_collection.push_back(m);
    if(!m)return false;
    return true;
  } 

  bool 
  WebsocketsServerGateway::setDefaultDoc404(const std::string& p)
  {
    _404_path = p;
    return true;
  }

  

  void 
  WebsocketsServerGateway::_clean_prots()
  {
    for(std::vector<struct lws_protocols*>::iterator 
          it = _protocols_collection.begin();
          it != _protocols_collection.end();
          ++it)
    {
      if( (*it)->name)delete (*it)->name;
      if( (*it)->user)delete static_cast<WsgConnectionStructure*>((*it)->user);
      delete (*it);
      
    }
    _protocols_collection.clear();
    if(_pprotocols_ptrs)delete _pprotocols_ptrs;
  }

  bool 
  WebsocketsServerGateway::setSSLCertKeyPath(
    const std::string& certPath,
    const std::string& keyPath)
  {
    _ssl_cert_path = certPath;
    _ssl_key_path  = keyPath;
    return ((_ssl_cert_path.size()) >0 && (_ssl_key_path.size()>0));
  }

  void 
  WebsocketsServerGateway::_async_comms_control_pulse()
  {
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    TCClMap::iterator i;
    for(i = _connected_clients.begin();i != _connected_clients.end();++i)
    {
      if(!i->second->standByTimeout())
      {
        CLOG_DEBUG(6,"CYTWSG : async-comms-control-pulse : for client [" 
          << i->second->getConnectionID() << "]")
        continue;
      }
      std::stringstream ss;
      ss << "CYTWSG : async-comms-control-pulse : for client [" 
        << i->second->getConnectionID() << "] : standby operation exceeded. "
        "Timeout for socket set to [" << i->second->_standby_time << "] but [" 
        << coyot3::tools::getCurrentTimestamp() << " - " 
        << i->second->_standby_start_ts 
        << " = " << (coyot3::tools::getCurrentTimestamp() 
                      - i->second->_standby_start_ts) 
        << "] milliseconds have passed.";
      CLOG_WARN(ss.str());
      i->second->appendToResponseContent(ss.str());
      i->second->sendPreparedData();
    }
  }
  WsgClient* 
  WebsocketsServerGateway::add_client(
    WsgConnectionStructure* ref,
    struct ::lws* wsi)
  {
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    bool create = false;
    TCClMap::iterator i;
    WsgClient* c= nullptr;
    if(ref->ref_id == 0)
    {
      ref->ref_id = (++_connection_struct_id_index);
      CLOG_DEBUG(5,"WSG : add client : " << _name << " : created reference id ["
      " [" << ref->ref_id << "] for incoming connection");
    }
    i = _connected_clients.find(ref->ref_id);
    if(i == _connected_clients.end()){
      create = true;
    }else{
      c = i->second;
    }
  
    if(create){
      ref->wsi = wsi;
      c = new(std::nothrow) WsgClient(ref);
      if(!c)
      {
        CLOG_ERROR("WSG : " << _name << " : add client : error adding client!");
        return nullptr;
      }
      _connected_clients.insert(TCClPair(ref->ref_id,c));
      
    }
    return c;
  }
  WsgClient* 
  WebsocketsServerGateway::get_client(WsgClient* client)
  {
    WsgClient* c = nullptr;
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    for(TCClMap::iterator 
            it= _connected_clients.begin();
            it!= _connected_clients.end();
            ++it)
    {
      if(client == it->second)
      {
        c = it->second;
      }
    }
    return c;
  }
  std::string 
  WebsocketsServerGateway::getConnectedClientsPeerInfo()
  {
    std::stringstream ss;
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    bool first = true;
    for(TCClMap::iterator 
          it = _connected_clients.begin();
          it!= _connected_clients.end();
          ++it)
    {
      if(!first) ss << ";";
      ss << it->second->getPeerInfo();
      first = false;
    }
    return ss.str();
  }


  WsgClient* 
  WebsocketsServerGateway::get_client(uint64_t id)
  {
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    TCClMap::iterator i;
    if(!id)return nullptr;
    i = _connected_clients.find(id);
    if(i == _connected_clients.end())
    {
      return nullptr;
    }
    return i->second;
  }

  bool WebsocketsServerGateway::del_client(uint64_t id)
  {
    std::lock_guard<std::mutex> guard(_connected_clients_mtx);
    TCClMap::iterator i;
    WsgClient* c;
    bool res = false;
    i = _connected_clients.find(id);
    if(i == _connected_clients.end())
    {
      return false;
    }
    c = i->second;
    res = (_connected_clients.erase(id) != 0);
    if(!c)
    {
      CLOG_ERROR("WSG : " << _name << " : del client : client for id [" 
      << id << "] pointer is null");
    }
    else{
      delete c;
    }
    
    return res;
  }
  bool WebsocketsServerGateway::del_client(WsgClient* client)
  {
    TCClMap::iterator i;
    for(i = _connected_clients.begin(); i != _connected_clients.end();++i)
    {
      if(i->second == client)
      {
        break;
      }
    }
    if(i == _connected_clients.end())
    {
      CLOG_WARN("WSG : " << _name << " : del client c* : client not found!");
      delete client;
      return false;
    }  
    _connected_clients.erase(i);
    delete client;
    return true;
  }

  bool 
  WebsocketsServerGateway::setOnWSClientConnect(
    std::function<bool(coyot3::services::websocket::WsgClient*)> callback)
  {
    callback_onwsconnects = callback;
    return true;
  }
  bool 
  WebsocketsServerGateway::setOnWSClientDisconnects(
    std::function<bool(coyot3::services::websocket::WsgClient*)> callback)
  {
    callback_onwsdisconnects = callback;
    return true;
  }
  bool 
  WebsocketsServerGateway::setOnWSClientMessage(
    std::function<bool(coyot3::services::websocket::WsgClient*,
    const uint8_t*,size_t)> callback)
  {
    callback_onwsmessage = callback;
    return true;
  }

  bool 
  WebsocketsServerGateway::setOnDynHttpCallback(
    std::function<bool(WsgClient*,
    const std::string&,
    const UriArguments&,std::string&)> callback)
  {
      callback_ondynhttpreq = callback;return true;
  };
  bool 
  WebsocketsServerGateway::setConfigMaxChunkSize(size_t s)
  {
    ((s < _WSG_CHUNK_MAX_LENGTH)
          ?_config_chunk_max_size = _WSG_CHUNK_MAX_LENGTH
          :_config_chunk_max_size = s);
    return true;
  }

  int64_t 
  WebsocketsServerGateway::asyncCommsTimeout()
  {
    return _async_comms_control_timeout;
  }

  int64_t 
  WebsocketsServerGateway::asyncCommsTimeout(int64_t timeoutMsecs)
  {
    _async_comms_control_timeout = timeoutMsecs;
    return _async_comms_control_timeout;
  }



////////////////////////////////////////////
////////////////////////////////////////////
///// HEADERS
////////////////////////////////////////////
////////////////////////////////////////////
  bool 
  WebsocketsServerGateway::serverSetHeader(
    const std::string& header,
    const std::string& value)
  {
    return headersManager.setHeader(header,value);
  }

  bool 
  WebsocketsServerGateway::serverDefaultSecurePolicyHeaders()
  {
    return headersDefaultSecurePolicy;
  }

  bool 
  WebsocketsServerGateway::serverDefaultSecurePolicyHeaders(bool activate)
  {
    return (headersDefaultSecurePolicy = activate);
  }


  bool 
  WebsocketsServerGateway::serverPrepareDefaultHeaders()
  {
    return headersManager.prepareDefaultHeaders();
  }

  bool 
  WebsocketsServerGateway::serverSetCspRule(
    const std::string& policy,
    const std::string& rule)
  {
    return headersManager.setCsp(policy,rule);
  }



  //server headers
  WebsocketsServerGateway::ServerHeaders::ServerHeaders()
  :headersMap()
  ,cspMap()
  ,headers(nullptr)
  {

  }

  WebsocketsServerGateway::ServerHeaders::ServerHeaders(const ServerHeaders& o)
  {
    *this = o;
  }
  WebsocketsServerGateway::ServerHeaders::~ServerHeaders()
  {

  }

  WebsocketsServerGateway::ServerHeaders& 
  WebsocketsServerGateway::ServerHeaders::operator=(
    const WebsocketsServerGateway::ServerHeaders& o)
  {
    headersMap = o.headersMap;
    cspMap = o.cspMap;
    headers = o.headers;
    return *this;
  }

bool          
WebsocketsServerGateway::ServerHeaders::setHeader(
  const std::string& header,const std::string& content)
{
  bool res = (headersMap.find(header) == headersMap.end());
  headersMap[header] = content;
  return res;
}
std::string   
WebsocketsServerGateway::ServerHeaders::getHeader(
  const std::string& header) const
{
  StrstrMap::const_iterator it = headersMap.find(header);
  if(it == headersMap.end())
  {
    return std::string();
  }
  return it->second;
}
std::string   WebsocketsServerGateway::ServerHeaders::stringify() const
{
  std::stringstream headers;
  
  for(const std::pair<std::string,std::string>& p: headersMap)
  {
    headers << p.first << ": " << p.second << std::endl;
  }

  return headers.str();
}
bool WebsocketsServerGateway::ServerHeaders::prepareDefaultCsp()
{
  cspMap.clear();
  setCsp("default-src"       ,"'none'");
  setCsp("img-src"           ,"'self' data: blob:");
  setCsp("script-src"        ,"'self'");
  setCsp("font-src"          ,"'self'");
  setCsp("style-src"         ,"'self'");
  setCsp("connect-src"       ,"'self' ws: wss:");
  setCsp("frame-ancestors"   ,"'none'");
  setCsp("base-uri"          ,"'none'");
  setCsp("form-action"       ,"'self'");
  return true;
}
bool  
WebsocketsServerGateway::ServerHeaders::setCsp(
  const std::string& policy,
  const std::string& rule)
{
  bool res = (cspMap.find(policy) == cspMap.end());
  cspMap[policy] = rule;
  
  std::stringstream sscsprule;    
  for(const std::pair<std::string,std::string>& p : cspMap)
  {
  
    sscsprule << p.first << " " << p.second << " ; ";
  }

  setHeader("content-security-policy",sscsprule.str());

  return res;
}
std::string   
WebsocketsServerGateway::ServerHeaders::getCsp(const std::string& policy)
{
  StrstrMap::const_iterator it = cspMap.find(policy);
  if(it == cspMap.end())
  {
    return std::string();
  }
  return it->second;
}
size_t        WebsocketsServerGateway::ServerHeaders::size()
{
  return headersMap.size();
}

bool WebsocketsServerGateway::ServerHeaders::prepareDefaultHeaders(){
  headersMap.clear();

  setHeader("x-content-type-options","nosniff");
  setHeader("x-xss-protection","1; mode=block");
  setHeader("referrer-policy","no-referrer");
  setHeader("x-frame-options","deny");
  setHeader("extra-info-custom-header","dev by the coyot3");
  prepareDefaultCsp();
  return true;
}

// HTTP/1.0 403 Forbidden
// content-security-policy: default-src 'none'; img-src 'self' data: ; script-src 'self'; font-src 'self'; style-src 'self'; connect-src 'self' ws: wss:; frame-ancestors 'none'; base-uri 'none';form-action 'self';
// x-content-type-options: nosniff
// x-xss-protection: 1; mode=block
// x-frame-options: deny
// referrer-policy: no-referrer
// content-type: text/html
// content-length: 173


// GET / HTTP/1.1
// Host: localhost:6005
// User-Agent: curl/7.58.0
// Accept: */*

//--------------------------------
// HTTP/1.1 200 OK
// content-security-policy: default-src 'none'; img-src 'self' data: ; script-src 'self'; font-src 'self'; style-src 'self'; connect-src 'self' ws: wss:; frame-ancestors 'none'; base-uri 'none';form-action 'self';
// x-content-type-options: nosniff
// x-xss-protection: 1; mode=block
// x-frame-options: deny
// referrer-policy: no-referrer
// content-type: text/html
// connection: close
struct lws_protocol_vhost_options pvo_hsbph[] = {{
	NULL, NULL, "referrer-policy:", "no-referrer"
}, {
	&pvo_hsbph[0], NULL, "x-frame-options:", "deny"
}, {
	&pvo_hsbph[1], NULL, "x-xss-protection:", "1; mode=block"
}, {
	&pvo_hsbph[2], NULL, "x-content-type-options:", "nosniff"
}, {
	&pvo_hsbph[3], NULL, "content-security-policy:",
	"default-src 'none'; img-src 'self' data: ; "
		"script-src 'self'; font-src 'self'; "
		"style-src 'self'; connect-src 'self' ws: wss:; "
		"frame-ancestors 'none'; base-uri 'none';"
		"form-action 'self';"
}, {
	&pvo_hsbph[4], NULL, "una-que-me-invento:",
	"en un lugar de la mancha "
  "de cuyo nombre no quiero acordarme "
  "looooorem ipsum in dolor sit amet!"
}};

  size_t WebsocketsServerGateway::ServerHeaders::generateLwsServerHeaders()
  {
    struct lws_protocol_vhost_options* newHeader,*previousHeader = nullptr;
    bool error = false;
    char* buffer;
    size_t gen = 0;
    for(const std::pair<std::string,std::string>& p : headersMap)
    {
      CLOG_DEBUG(3,"CYTWSG : server-headers : generate-lws-server-headers : "
        "preparing header [" << p.first << ": " << p.second << "]");
      newHeader = new(std::nothrow) struct lws_protocol_vhost_options;
      if(!newHeader){
        CLOG_ERROR("CYTWSG : server-headers : generate-lws-server-headers : "
          "FATAL ERROR allocating header!");
        exit(1);
      }
      std::string buffstr = p.first + ":\0";
      buffer = new char[buffstr.size()+1];
      memcpy(buffer,buffstr.c_str(),buffstr.size()+1);
      newHeader->name = buffer;

      buffstr = p.second + "\0";
      buffer = new char[buffstr.size()+1];
      memcpy(buffer,buffstr.c_str(),buffstr.size()+1);
      newHeader->value = buffer;
      newHeader->options = NULL;
      newHeader->next = previousHeader;

      headers = newHeader;
      previousHeader = newHeader;
      gen++;
    }
    return gen;

  }
  size_t WebsocketsServerGateway::ServerHeaders::cleanHeaders()
  {
    size_t c = 0;
    const struct lws_protocol_vhost_options* ptr;
    while(headers != nullptr)
    {
      ptr = headers->next;
      delete headers;
      headers = (struct lws_protocol_vhost_options*)ptr;
      c++;
    }
    return c;
  }

  struct lws_protocol_vhost_options* 
  WebsocketsServerGateway::ServerHeaders::getHeadersPtr()
  {
    return headers;
  }
  bool WebsocketsServerGateway::ServerHeaders::areGenerated()
  {
    return (headers != nullptr);
  }


}
} //eons wrappers
} //eons coyot3