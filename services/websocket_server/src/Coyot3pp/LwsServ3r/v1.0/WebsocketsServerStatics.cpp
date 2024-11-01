#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>



namespace coyot3{
namespace services{
namespace websocket{

static int callback_http(
    struct lws *wsi,
    enum lws_callback_reasons reason,
    void *user,
    void *in, 
    size_t len
){
  char* requested_uri_;
  std::string requested_uri;

  CLOG_DEBUG(6,"CYTWSG : static callback http : new request done");
  switch(reason)
  {
    case LWS_CALLBACK_CLIENT_WRITEABLE:

    case LWS_CALLBACK_HTTP:
      requested_uri_ = (char*)in;
      requested_uri  = requested_uri_;

      CLOG_DEBUG(4,"WSS STATIC : callback http : reason lws_callback_http : "
        "client requests uri = [" << requested_uri << "]");
    case LWS_CALLBACK_ESTABLISHED:
    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
    case LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH:
    case LWS_CALLBACK_CLIENT_ESTABLISHED:
    case LWS_CALLBACK_CLOSED:
    case LWS_CALLBACK_CLOSED_HTTP:
    case LWS_CALLBACK_RECEIVE:
    case LWS_CALLBACK_RECEIVE_PONG:
    case LWS_CALLBACK_CLIENT_RECEIVE:
    case LWS_CALLBACK_CLIENT_RECEIVE_PONG:
    //case LWS_CALLBACK_CLIENT_WRITEABLE: //handled
    case LWS_CALLBACK_SERVER_WRITEABLE:
    //case LWS_CALLBACK_HTTP: //handled
    case LWS_CALLBACK_HTTP_BODY:
    case LWS_CALLBACK_HTTP_BODY_COMPLETION:
    case LWS_CALLBACK_HTTP_FILE_COMPLETION:
    case LWS_CALLBACK_HTTP_WRITEABLE:
    case LWS_CALLBACK_FILTER_NETWORK_CONNECTION:
    case LWS_CALLBACK_FILTER_HTTP_CONNECTION:
    case LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED:
    case LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION:
    case LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS:
    case LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS:
    case LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION:
    case LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER:
    case LWS_CALLBACK_CONFIRM_EXTENSION_OKAY:
    case LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED:
    case LWS_CALLBACK_PROTOCOL_INIT:
    case LWS_CALLBACK_PROTOCOL_DESTROY:
    case LWS_CALLBACK_WSI_CREATE:
    case LWS_CALLBACK_WSI_DESTROY:
    case LWS_CALLBACK_GET_THREAD_ID:

	/* external poll() management support */
    case LWS_CALLBACK_ADD_POLL_FD:
    case LWS_CALLBACK_DEL_POLL_FD:
    case LWS_CALLBACK_CHANGE_MODE_POLL_FD:
    case LWS_CALLBACK_LOCK_POLL:
    case LWS_CALLBACK_UNLOCK_POLL:

    //case LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY:
    case LWS_CALLBACK_WS_PEER_INITIATED_CLOSE:


    case LWS_CALLBACK_WS_EXT_DEFAULTS:


    case LWS_CALLBACK_CGI:
    case LWS_CALLBACK_CGI_TERMINATED:
    case LWS_CALLBACK_CGI_STDIN_DATA:
    case LWS_CALLBACK_CGI_STDIN_COMPLETED:
    case LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP:
    case LWS_CALLBACK_CLOSED_CLIENT_HTTP:
    case LWS_CALLBACK_RECEIVE_CLIENT_HTTP:
    case LWS_CALLBACK_COMPLETED_CLIENT_HTTP:
    case LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ:

    /****** add new things just above ---^ ******/
    case LWS_CALLBACK_USER:

    default:
      CLOG_DEBUG(3,"nothing to do");
  }
  return 1;
}


int callback_dynamic_http_route(
     lws *wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
)
{
  CLOG_DEBUG(7,"CYTWSG : static-method dyn-http-route callback : user ptr = " << user);
  

  const lws_protocols* p = lws_get_protocol(wsi);
  
  if(!p)
  {
    CLOG_WARN("user is null!");
    return lws_callback_http_dummy(wsi, reason, user,in,len);
  }
  WsgConnectionStructure* dprorig = (WsgConnectionStructure*)(p->user);
  coyot3::services::websocket::WebsocketsServerGateway* ptr = 
    dprorig->gateway_instance;

  WsgConnectionStructure* dpr = nullptr;
  if(user)
  {
    
    dpr = (WsgConnectionStructure*)user;
    CLOG_DEBUG(7,"CYTWSG : static-method dyn-http-route callback : "
    "dpr set not null");
  }  
  CLOG_DEBUG(7,"CYTWSG : static-method dyn-http-route callback : "
  "protocol name = " << p->name);
  if(!user)
  {
    CLOG_DEBUG(5,"CYTWSG : static-method dyn-http-route callback : "
    "no user data found, serving dummy response for reason [" 
    << WebsocketsServerGateway::CallbackReasonToString(reason) << "]");
    return lws_callback_http_dummy(wsi, reason, user,in,len);
  }
  CLOG_DEBUG(6,"CYTWSG : static-method dyn-http-route callback : "
  "invoking callback for peer [" << dpr->source_peer << "]");
  return ptr->callback_dyn_http(wsi,reason,in,len,dpr);

}

  int callback_minimal_ws(
    struct lws *wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
  )
  {
    CLOG_INFO(__PRETTY_FUNCTION__ << " reached!");
    return lws_callback_http_dummy(wsi,reason,user,in,len);

  }

  void WebsocketsServerGateway::set_debug_level_mstatics_(int level)
  {
    CLOG_DEBUG_LEVEL_SET(level);
  }

}
}
}//eons coyot3