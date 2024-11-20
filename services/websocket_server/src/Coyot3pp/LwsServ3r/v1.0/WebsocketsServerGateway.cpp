#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>


namespace coyot3{
namespace services{
namespace websocket{


WebsocketsServerGateway::WebsocketsServerGateway()
: _name(nullptr)
, _initialized(false)
, _server_thread(nullptr)
, _server_active(false)
, _async_comms_control(nullptr)
, _async_comms_control_timeout(CYTWSG_CONFIG_DEFAULT_COMMSCONTROL_TIMEOUT)
, _async_comms_control_interval(CYTWSG_CONFIG_DEFAULT_COMMSCONTROL_INTERVAL)
, _info()
, _context(nullptr)
, _mounts_collection()
, _protocols_collection()
, _pprotocols_ptrs(nullptr)
, headersManager()
,headersDefaultSecurePolicy(true)
, _server_port(8081)
, _404_path()
, _ssl_cert_path()
, _ssl_key_path()
, _connected_clients()
, _connection_struct_id_index(0)
, _config_chunk_max_size(_WSG_CHUNK_MAX_LENGTH)
{
  CLOG_INFO("WEBSOCKETS GATEWAY : constructor");
  setName("no-name");
  _server_thread = new(std::nothrow) coyot3::tools::ControlThread(std::bind(&coyot3::services::websocket::WebsocketsServerGateway::_server_loop,this));
  _async_comms_control = new(std::nothrow) coyot3::tools::ControlThread(std::bind(&coyot3::services::websocket::WebsocketsServerGateway::_async_comms_control_pulse,this));
  if(!(_server_thread) || !(_async_comms_control))
  {
    CLOG_ERROR("CYTWSG : constructor : ")
  }
  callback_ondynhttpreq     = std::bind(
    &coyot3::services::websocket::WebsocketsServerGateway::dummy_callback_on_dyn_http
    ,this
    ,std::placeholders::_1
    ,std::placeholders::_2
    ,std::placeholders::_3
    ,std::placeholders::_4);
  callback_onwsmessage      = std::bind(
    &coyot3::services::websocket::WebsocketsServerGateway::dummy_callback_on_ws_message
    ,this
    ,std::placeholders::_1
    ,std::placeholders::_2
    ,std::placeholders::_3);
  callback_onwsconnects     = std::bind(
    &coyot3::services::websocket::WebsocketsServerGateway::dummy_callback_on_ws_connects
    ,this
    ,std::placeholders::_1);
  callback_onwsdisconnects  = std::bind(
    &coyot3::services::websocket::WebsocketsServerGateway::dummy_callback_on_ws_disconnects
    ,this
    ,std::placeholders::_1);

  CLOG_INFO("WEBSOCKETS GATEWAY : constructor : OK");
}

WebsocketsServerGateway::~WebsocketsServerGateway()
{
  CLOG_WARN("WEBSOCKETS GATEWAY : " << _name << " : destructor");
  if(_server_thread && _server_active)
  {
    CLOG_DEBUG(3,"CYTWSG : " << _name << " : destructor : stopping");
    Stop();
  }
  
  if(_initialized)
  {
    CLOG_DEBUG(3,"CYTWSG : " << _name << " : destructor : freeing memory");
    if(_context)
      lws_context_destroy(_context);
    _clean_mounts();
    _clean_prots();
  }
  if(_name)delete _name;
  if(_info.error_document_404)
  {
    CLOG_DEBUG(3,"CYTWSG : " << _name << " : destructor : freeyingh (404)");
  }
  
  CLOG_DEBUG(3,"CYTWSG : " << _name << " : destructor : OK");
}

void WebsocketsServerGateway::set_debug_level(int level)
{
  CLOG_DEBUG_LEVEL_SET(level);
  set_debug_level_mcallbacks_(level);
  set_debug_level_mmisc_(level);
  set_debug_level_mstatics_(level);
}

bool WebsocketsServerGateway::Init()
{
  bool res = true;
  CLOG_DEBUG(4,"CYTWSG : " << _name << " : init : allocating mount structure");
  
  if(!_initialize_info())
  {
    CLOG_WARN("CYTWSG : " << _name << " : init : warning, no static mount point has been configured");
    return false;
  }
  res &= _initialize_lws_context();

  if(!res)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : init : error initializing");
    return false;
  }
  _initialized = true;
  return true;
}

bool WebsocketsServerGateway::Start()
{
  bool res = true;
  
  _server_thread->setInterval(1);
  _server_active = true;
  res &= _server_thread->start();

  _async_comms_control->setInterval(_async_comms_control_interval);
  res &= _async_comms_control->start();
  CLOG_INFO("CYTWSG : start : " << _name << " : [" << ((res==true)?"ok.":"!ERROR") 
    << "]");

  return res;
}


bool WebsocketsServerGateway::Stop()
{
  bool res = true;
  CLOG_INFO("CYTWSG :: " << _name << " : stop ");
  res &= _server_thread->stop();

  res &= _async_comms_control->stop();
  return res;
}


void libwebsocket_log(int level, const char* line)
{
  CLOG_WARN("TO DELETE : " << level << " : " << line);
}

// struct lws_protocol_vhost_options pvo_hsbph[] = {{
// 	NULL, NULL, "referrer-policy:", "no-referrer"
// }, {
// 	&pvo_hsbph[0], NULL, "x-frame-options:", "deny"
// }, {
// 	&pvo_hsbph[1], NULL, "x-xss-protection:", "1; mode=block"
// }, {
// 	&pvo_hsbph[2], NULL, "x-content-type-options:", "nosniff"
// }, {
// 	&pvo_hsbph[3], NULL, "content-security-policy:",
// 	"default-src 'none'; img-src 'self' data: ; "
// 		"script-src 'self'; font-src 'self'; "
// 		"style-src 'self'; connect-src 'self' ws: wss:; "
// 		"frame-ancestors 'none'; base-uri 'none';"
// 		"form-action 'self';"
// }, {
// 	&pvo_hsbph[4], NULL, "una-que-me-invento:",
// 	"en un lugar de la mancha "
//   "de cuyo nombre no quiero acordarme "
//   "looooorem ipsum in dolor sit amet!"
// }};

bool WebsocketsServerGateway::_initialize_info()
{
  CLOG_DEBUG(5,"CYTWSG : " << _name << " : initialize lws context info");
  if(_mounts_collection.size() == 0)
  {
    CLOG_WARN("CYTWSG : " << _name << " : no mount configuration has been defined");
    return false;
  }
  memset(&_info,0,sizeof(_info));
  _info.port      = _server_port;
  _info.mounts    = _mounts_collection[0];
  _info.protocols = nullptr;
  _info.gid       = static_cast<gid_t>(-1);
  _info.uid       = static_cast<uid_t>(-1);
  // _info.username  = nullptr;
  // _info.groupname = nullptr;
  // _info.unix_socket_perms = nullptr;
  _info.user = static_cast<void*>(this);
  _info.keepalive_timeout = -1;
  if((_ssl_cert_path.size() != 0) && (_ssl_key_path.size() != 0))
  {
    CLOG_INFO("CYTWWSS : " << _name << " : initializing ssl : cert[" 
      << _ssl_cert_path << "], key [" << _ssl_key_path << "]");
    _info.ssl_cert_filepath = _ssl_cert_path.c_str();
    _info.ssl_private_key_filepath = _ssl_key_path.c_str();
    _info.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT | LWS_SERVER_OPTION_REDIRECT_HTTP_TO_HTTPS;
  }


  if(headersDefaultSecurePolicy == false)
  {
    if(!headersManager.areGenerated()){headersManager.generateLwsServerHeaders();}
    _info.headers = headersManager.getHeadersPtr();
  }else{
    _info.headers = nullptr;
  }
  
  /* include protocols collection */
  //struct lws_protocols* lastprot = new struct lws_protocols();
  //memset(lastprot,0,sizeof(struct lws_protocols));
  //_protocols_collection.push_back(lastprot);
  _protocols_collection.push_back(nullptr);
  CLOG_DEBUG(5,"CYTWSG : " << _name << " : initialize info : setting "
    "protocols. Allocating [" << _protocols_collection.size() << "] positions");
  _pprotocols_ptrs = new(std::nothrow) const struct lws_protocols*[_protocols_collection.size()];
  if(!_pprotocols_ptrs)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : intiialize info : ERROR "
      "ALLOCATING PROTOCOLS. no mem??");
    return false;
  }
  int p=0;
  for(std::vector<struct lws_protocols*>::iterator it=_protocols_collection.begin();
    it != _protocols_collection.end();++it)
  {
    _pprotocols_ptrs[p] = *it;
    p++;
    CLOG_DEBUG(5," at position " << p << " pointer " << _pprotocols_ptrs[p-1]);
  }
  CLOG_DEBUG(3,"CYTWSG : " << _name << " : initialize info : protocols set");
  _info.pprotocols = _pprotocols_ptrs;
  

  

  char* path404Docc = new(std::nothrow) char[_404_path.size()+1];
  if(!path404Docc){
    CLOG_ERROR("CYTWSG : " << _name << " : initialize info : error allocating"
      " 404 doc path");
    return false;
  }
  strcpy(path404Docc,(_404_path + std::string("\0")).c_str());
  _info.error_document_404 = path404Docc; //libwebsockets version 4.1
  if(headersDefaultSecurePolicy == false)
  {
    _info.options = 
      LWS_SERVER_OPTION_VALIDATE_UTF8;
    
  }else{
    _info.options = 
      LWS_SERVER_OPTION_VALIDATE_UTF8
    | LWS_SERVER_OPTION_HTTP_HEADERS_SECURITY_BEST_PRACTICES_ENFORCE;
  }

  CLOG_INFO("CYTWSG : " << _name << " : initialize lws context info : OK : port " << _server_port);
  //lws_set_log_level(10,libwebsocket_log);
  return true;
}

bool WebsocketsServerGateway::_initialize_lws_context()
{
  CLOG_DEBUG(5,"CYTWSG : " << _name << " : initialize lws context : creating");

  _context = lws_create_context(&_info);
  
  

  if(!_context)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : initialize lws context : error initializing context");
    return false;
  }
  
  return true;
}




void WebsocketsServerGateway::_server_loop()
{
  CLOG_INFO("CYTWSG : " << _name << " : server loop : init");
  int n;
  while(_server_active)
  {
    n = lws_service(_context,1000);
    //CLOG_DEBUG(5, "CYTWSG : " << _name << " : server loop : lws_service result = " 
    //<< n << " : current clients size [" <<_connected_clients.size() << "]");
  } 
  CLOG_INFO("CYTWSG : " << _name << " : server loop : bye");
}



bool WebsocketsServerGateway::addStaticPath(
   const std::string& baseUrl
  ,const std::string& basePath
  ,const std::string& defaultFile)
{
  CLOG_INFO("CYTWSG : " << _name << " : add static path : url[" << baseUrl
  << "] : path[" << basePath << "], default(" << defaultFile << ")");
  struct lws_http_mount* newMountPoint;

  newMountPoint = new(std::nothrow) struct lws_http_mount;
  if(!newMountPoint)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : add static path (" << baseUrl 
    << "," << basePath << ") : ALLOCATION ERROR. no mem?");
    return false;
  }
  memset(newMountPoint,0,sizeof(*newMountPoint));
  char* baseUrlc     = new(std::nothrow) char[baseUrl.size()+1];
  char* basePathc    = new(std::nothrow) char[basePath.size()+1];
  char* defaultFilec = new(std::nothrow) char[defaultFile.size()+1];
  if(!baseUrlc || !basePathc || !defaultFilec)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : add static path (" << baseUrl
    << "," << basePath << ") unable to allocate memory");
  }
  memcpy(baseUrlc,(baseUrl+std::string("\0")).c_str(),baseUrl.size()+1);
  memcpy(basePathc,(basePath+std::string("\0")).c_str(),basePath.size()+1);
  memcpy(defaultFilec,(defaultFile+std::string("\0")).c_str(),defaultFile.size()+1);
  
  newMountPoint->mount_next      = nullptr;
  newMountPoint->mountpoint      = baseUrlc;
  newMountPoint->origin          = basePathc;
  //newMountPoint->origin = "/home/ricard0/DEV/COYOT3/WebSocketsServer/www/html";
  newMountPoint->def             = defaultFilec;
  newMountPoint->protocol        = nullptr; // libwebsockets version 4.1
  newMountPoint->cgienv          = nullptr;
  newMountPoint->extra_mimetypes = nullptr;
  newMountPoint->interpret       = nullptr; // libwebsockets version 4.1
  newMountPoint->cgi_timeout     = 0;
  newMountPoint->cache_max_age   = 0;
  newMountPoint->cache_revalidate= 0;
  newMountPoint->cache_intermediaries = 0;
  newMountPoint->origin_protocol = LWSMPRO_FILE;
  newMountPoint->mountpoint_len  = 1;

  _push_mount(newMountPoint);

  //protocols
  CLOG_DEBUG(5,"CYTWSG : " << _name << " : add static path : allocating protocol");
  struct lws_protocols* defprot = new(std::nothrow) struct lws_protocols();
  if(!defprot){
    CLOG_ERROR("CYTWSG : " << _name << " : add static path : ERROR ALLOCATING "
    "PROTOCOL CONFIG. no mem??");
    return false;
  }

  defprot->name                  = "defprot";
  defprot->callback              = lws_callback_http_dummy;
  defprot->per_session_data_size = 0;
  defprot->rx_buffer_size        = 0;
  defprot->id                    = 0;
  defprot->user                  = nullptr;

  _protocols_collection.push_back(defprot);
  
  CLOG_INFO("CYTWSG : " << _name << " : add static path : configured static "
    "path [" << baseUrl << " <> " << basePath << ";default=" << defaultFile 
    << "]");
  return true;
}

bool WebsocketsServerGateway::addDynamicPath(const std::string& baseUrl)
{
  struct lws_http_mount* newMountPoint;
  newMountPoint = new(std::nothrow) struct lws_http_mount;
  if(!newMountPoint)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : add dynamic path (" << baseUrl 
    <<" : ALLOCATION ERROR. no mem?");
    return false;
  }
  memset(newMountPoint,0,sizeof(*newMountPoint));
  char* baseUrlc     = new(std::nothrow) char[baseUrl.size()+1];
  if(!baseUrlc )
  {
    CLOG_ERROR("CYTWSG : " << _name << " : add dynamic path (" << baseUrl
    << ") unable to allocate memory");
    return false;
  }
  memcpy(baseUrlc,baseUrl.c_str(),baseUrl.size());
  baseUrlc[baseUrl.size()] = '\0';
  newMountPoint->mountpoint      = baseUrlc;
  newMountPoint->mountpoint_len  = baseUrl.size();
  newMountPoint->protocol        = "http";
  newMountPoint->origin_protocol = LWSMPRO_CALLBACK;
  _push_mount(newMountPoint);
  struct lws_protocols* prot = new(std::nothrow) struct lws_protocols();
  if(!prot)
  {
    CLOG_ERROR("CYTWSG : " << _name << " : add dynamic path : (" << baseUrl
    << ") impossible to allocate lws_protocols structure");
    return false;
  }
  memset(prot,0,sizeof(*prot));
  prot->name                  = "http";
  prot->callback              = callback_dynamic_http_route;
  prot->per_session_data_size = sizeof(struct WsgConnectionStructure);
  prot->rx_buffer_size        = 0;
  prot->tx_packet_size        = 0;
  prot->id                    = 0;
  //prot->user                  = static_cast<void*>(this);


    WsgConnectionStructure* ref = new(std::nothrow) WsgConnectionStructure();
    memset(ref,0,sizeof(*ref));
    ref->path             = baseUrl;
    ref->gateway_instance = this;
  prot->user                  = static_cast<void*>(ref);


  _protocols_collection.push_back(prot);
  
  CLOG_INFO("CYTWSG : " << _name << " : add dynamic path : added (" 
  << baseUrlc << ")");
  return true;


}

int WebsocketsServerGateway::broadcastMessage(const std::string& message)
{
  int r = 0;
  TCClMap::iterator i;
  std::lock_guard<std::mutex> guard(_connected_clients_mtx);
  for(i = _connected_clients.begin(); i!= _connected_clients.end();++i)
  {
    
    if((i->second)->send(message))
    {
      r++;
    }
    else{
      CLOG_WARN("WSG : " << _name << " : broadcast message : error sending "
      "message to connection id [" << (i->second)->getConnectionID() << "]");
    }
  }
  return r;
}
int WebsocketsServerGateway::broadcastMessage(const Json::Value& message)
{
  std::stringstream ss;
  ss << message;
  return broadcastMessage(ss.str());
}

bool WebsocketsServerGateway::add_minimal_ws_callback()
{
    CLOG_DEBUG(5,"CYTWSG : " << _name << " : add static path : allocating protocol");
  struct lws_protocols* defprot = new(std::nothrow) struct lws_protocols();
  if(!defprot){
    CLOG_ERROR("CYTWSG : " << _name << " : add static path : ERROR ALLOCATING "
    "PROTOCOL CONFIG. no mem??");
    return false;
  }
  defprot->name                  = "defprot";
  defprot->callback              = callback_minimal_ws;
  defprot->per_session_data_size = sizeof(struct WsgConnectionStructure);
  defprot->rx_buffer_size        = 0;
  defprot->id                    = 0;
    WsgConnectionStructure* ref = new(std::nothrow) WsgConnectionStructure();
    memset(ref,0,sizeof(*ref));
    ref->path             = "minimal ws callback";
    ref->gateway_instance = this;
  defprot->user                  = ref;

  _protocols_collection.push_back(defprot);
  return true;
}

bool 
WebsocketsServerGateway::activateSendForHoldingConnection(WsgClient* client)
{
  if(get_client(client) == nullptr){
    CLOG_WARN("CYTWSG : activate-send-for-holding-connection : "
      "client not found");
    return false;
  }
  CLOG_DEBUG(5,"CYTSWG : activate-send-for-holding-connection : "
    "sending prepared data.");
  return client->sendPreparedData();
}
size_t 
WebsocketsServerGateway::appendToResponseContent(
  WsgClient* client,
  const std::string& data)
{
  if(get_client(client) == nullptr)
  {
    CLOG_WARN("CYTWSG : appendToResponseContent : string : client not found");
  }
  return _appendToResponseContent(client,data);
}

size_t 
WebsocketsServerGateway::appendToResponseContent(
  WsgClient* client,
  const Json::Value& data)
{
  if(get_client(client) == nullptr)
  {
    CLOG_WARN("CYTWSG : appendToResponseContent : json : client not found");
  }
  std::stringstream ss; ss << data; 
  return _appendToResponseContent(client,ss.str());
}

std::string 
WebsocketsServerGateway::getClientPeerInfo(WsgClient* client)
{
  if(get_client(client) == nullptr)
  {
    return "no-client-information";
  }
  return client->getPeerInfo();
}

size_t 
WebsocketsServerGateway::_appendToResponseContent(
  WsgClient* client,
  const std::string& data)
{
  return client->appendToResponseContent(data);
}



}
}//namespace wrappers
}//namespace coyot3