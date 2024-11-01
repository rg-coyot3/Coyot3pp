#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerCallbackReasonsHelper.hpp>


namespace coyot3{
namespace services{
namespace websocket{



const char* LwsCallbackReasonToString(int r)
{
  return LwsCallbackReasonToString(static_cast<LwsCallbackReason>(r));
}

const char* LwsCallbackReasonToString(LwsCallbackReason r)
{
  switch(r)
  {

    case LwsCallbackReason::LWS_CALLBACK_PROTOCOL_INIT : return "LWS_CALLBACK_PROTOCOL_INIT";break;
    case LwsCallbackReason::LWS_CALLBACK_PROTOCOL_DESTROY : return "LWS_CALLBACK_PROTOCOL_DESTROY";break;
    case LwsCallbackReason::LWS_CALLBACK_WSI_CREATE : return "LWS_CALLBACK_WSI_CREATE";break;
    case LwsCallbackReason::LWS_CALLBACK_WSI_DESTROY : return "LWS_CALLBACK_WSI_DESTROY";break;
    case LwsCallbackReason::LWS_CALLBACK_WSI_TX_CREDIT_GET : return "LWS_CALLBACK_WSI_TX_CREDIT_GET";break;
    case LwsCallbackReason::LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS : return "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS";break;
    case LwsCallbackReason::LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS : return "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS";break;
    case LwsCallbackReason::LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION : return "LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION";break;
    case LwsCallbackReason::LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY : return "LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY";break;
    case LwsCallbackReason::LWS_CALLBACK_SSL_INFO : return "LWS_CALLBACK_SSL_INFO";break;
    case LwsCallbackReason::LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION : return "LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION";break;
    case LwsCallbackReason::LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED : return "LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP : return "LWS_CALLBACK_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_BODY : return "LWS_CALLBACK_HTTP_BODY";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_BODY_COMPLETION : return "LWS_CALLBACK_HTTP_BODY_COMPLETION";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_FILE_COMPLETION : return "LWS_CALLBACK_HTTP_FILE_COMPLETION";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_WRITEABLE : return "LWS_CALLBACK_HTTP_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_CLOSED_HTTP : return "LWS_CALLBACK_CLOSED_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_FILTER_HTTP_CONNECTION : return "LWS_CALLBACK_FILTER_HTTP_CONNECTION";break;
    case LwsCallbackReason::LWS_CALLBACK_ADD_HEADERS : return "LWS_CALLBACK_ADD_HEADERS";break;
    case LwsCallbackReason::LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION : return "LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION";break;
    case LwsCallbackReason::LWS_CALLBACK_CHECK_ACCESS_RIGHTS : return "LWS_CALLBACK_CHECK_ACCESS_RIGHTS";break;
    case LwsCallbackReason::LWS_CALLBACK_PROCESS_HTML : return "LWS_CALLBACK_PROCESS_HTML";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_BIND_PROTOCOL : return "LWS_CALLBACK_HTTP_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_DROP_PROTOCOL : return "LWS_CALLBACK_HTTP_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_CONFIRM_UPGRADE : return "LWS_CALLBACK_HTTP_CONFIRM_UPGRADE";break;
    case LwsCallbackReason::LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP : return "LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_CLOSED_CLIENT_HTTP : return "LWS_CALLBACK_CLOSED_CLIENT_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ : return "LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ";break;
    case LwsCallbackReason::LWS_CALLBACK_RECEIVE_CLIENT_HTTP : return "LWS_CALLBACK_RECEIVE_CLIENT_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_COMPLETED_CLIENT_HTTP : return "LWS_CALLBACK_COMPLETED_CLIENT_HTTP";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_HTTP_WRITEABLE : return "LWS_CALLBACK_CLIENT_HTTP_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_HTTP_REDIRECT : return "LWS_CALLBACK_CLIENT_HTTP_REDIRECT";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL : return "LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL : return "LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_ESTABLISHED : return "LWS_CALLBACK_ESTABLISHED";break;
    case LwsCallbackReason::LWS_CALLBACK_CLOSED : return "LWS_CALLBACK_CLOSED";break;
    case LwsCallbackReason::LWS_CALLBACK_SERVER_WRITEABLE : return "LWS_CALLBACK_SERVER_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_RECEIVE : return "LWS_CALLBACK_RECEIVE";break;
    case LwsCallbackReason::LWS_CALLBACK_RECEIVE_PONG : return "LWS_CALLBACK_RECEIVE_PONG";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_PEER_INITIATED_CLOSE : return "LWS_CALLBACK_WS_PEER_INITIATED_CLOSE";break;
    case LwsCallbackReason::LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION : return "LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION";break;
    case LwsCallbackReason::LWS_CALLBACK_CONFIRM_EXTENSION_OKAY : return "LWS_CALLBACK_CONFIRM_EXTENSION_OKAY";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL : return "LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL : return "LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_CONNECTION_ERROR : return "LWS_CALLBACK_CLIENT_CONNECTION_ERROR";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH : return "LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_ESTABLISHED : return "LWS_CALLBACK_CLIENT_ESTABLISHED";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_CLOSED : return "LWS_CALLBACK_CLIENT_CLOSED";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER : return "LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_RECEIVE : return "LWS_CALLBACK_CLIENT_RECEIVE";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_RECEIVE_PONG : return "LWS_CALLBACK_CLIENT_RECEIVE_PONG";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_WRITEABLE : return "LWS_CALLBACK_CLIENT_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED : return "LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_EXT_DEFAULTS : return "LWS_CALLBACK_WS_EXT_DEFAULTS";break;
    case LwsCallbackReason::LWS_CALLBACK_FILTER_NETWORK_CONNECTION : return "LWS_CALLBACK_FILTER_NETWORK_CONNECTION";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL : return "LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL : return "LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_GET_THREAD_ID : return "LWS_CALLBACK_GET_THREAD_ID";break;
    case LwsCallbackReason::LWS_CALLBACK_ADD_POLL_FD : return "LWS_CALLBACK_ADD_POLL_FD";break;
    case LwsCallbackReason::LWS_CALLBACK_DEL_POLL_FD : return "LWS_CALLBACK_DEL_POLL_FD";break;
    case LwsCallbackReason::LWS_CALLBACK_CHANGE_MODE_POLL_FD : return "LWS_CALLBACK_CHANGE_MODE_POLL_FD";break;
    case LwsCallbackReason::LWS_CALLBACK_LOCK_POLL : return "LWS_CALLBACK_LOCK_POLL";break;
    case LwsCallbackReason::LWS_CALLBACK_UNLOCK_POLL : return "LWS_CALLBACK_UNLOCK_POLL";break;
    case LwsCallbackReason::LWS_CALLBACK_CGI : return "LWS_CALLBACK_CGI";break;
    case LwsCallbackReason::LWS_CALLBACK_CGI_TERMINATED : return "LWS_CALLBACK_CGI_TERMINATED";break;
    case LwsCallbackReason::LWS_CALLBACK_CGI_STDIN_DATA : return "LWS_CALLBACK_CGI_STDIN_DATA";break;
    case LwsCallbackReason::LWS_CALLBACK_CGI_STDIN_COMPLETED : return "LWS_CALLBACK_CGI_STDIN_COMPLETED";break;
    case LwsCallbackReason::LWS_CALLBACK_CGI_PROCESS_ATTACH : return "LWS_CALLBACK_CGI_PROCESS_ATTACH";break;
    case LwsCallbackReason::LWS_CALLBACK_SESSION_INFO : return "LWS_CALLBACK_SESSION_INFO";break;
    case LwsCallbackReason::LWS_CALLBACK_GS_EVENT : return "LWS_CALLBACK_GS_EVENT";break;
    case LwsCallbackReason::LWS_CALLBACK_HTTP_PMO : return "LWS_CALLBACK_HTTP_PMO";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_RX : return "LWS_CALLBACK_RAW_PROXY_CLI_RX";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_RX : return "LWS_CALLBACK_RAW_PROXY_SRV_RX";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_CLOSE : return "LWS_CALLBACK_RAW_PROXY_CLI_CLOSE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_CLOSE : return "LWS_CALLBACK_RAW_PROXY_SRV_CLOSE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE : return "LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE : return "LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_ADOPT : return "LWS_CALLBACK_RAW_PROXY_CLI_ADOPT";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_ADOPT : return "LWS_CALLBACK_RAW_PROXY_SRV_ADOPT";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL : return "LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL : return "LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL : return "LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL : return "LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_RX : return "LWS_CALLBACK_RAW_RX";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_CLOSE : return "LWS_CALLBACK_RAW_CLOSE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_WRITEABLE : return "LWS_CALLBACK_RAW_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_ADOPT : return "LWS_CALLBACK_RAW_ADOPT";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_CONNECTED : return "LWS_CALLBACK_RAW_CONNECTED";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL : return "LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL : return "LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_ADOPT_FILE : return "LWS_CALLBACK_RAW_ADOPT_FILE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_RX_FILE : return "LWS_CALLBACK_RAW_RX_FILE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_WRITEABLE_FILE : return "LWS_CALLBACK_RAW_WRITEABLE_FILE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_CLOSE_FILE : return "LWS_CALLBACK_RAW_CLOSE_FILE";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL : return "LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL : return "LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_TIMER : return "LWS_CALLBACK_TIMER";break;
    case LwsCallbackReason::LWS_CALLBACK_EVENT_WAIT_CANCELLED : return "LWS_CALLBACK_EVENT_WAIT_CANCELLED";break;
    case LwsCallbackReason::LWS_CALLBACK_CHILD_CLOSING : return "LWS_CALLBACK_CHILD_CLOSING";break;
    case LwsCallbackReason::LWS_CALLBACK_VHOST_CERT_AGING : return "LWS_CALLBACK_VHOST_CERT_AGING";break;
    case LwsCallbackReason::LWS_CALLBACK_VHOST_CERT_UPDATE : return "LWS_CALLBACK_VHOST_CERT_UPDATE";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED : return "LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_IDLE : return "LWS_CALLBACK_MQTT_IDLE";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED : return "LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_SUBSCRIBED : return "LWS_CALLBACK_MQTT_SUBSCRIBED";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_CLIENT_WRITEABLE : return "LWS_CALLBACK_MQTT_CLIENT_WRITEABLE";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_CLIENT_RX : return "LWS_CALLBACK_MQTT_CLIENT_RX";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_UNSUBSCRIBED : return "LWS_CALLBACK_MQTT_UNSUBSCRIBED";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_DROP_PROTOCOL : return "LWS_CALLBACK_MQTT_DROP_PROTOCOL";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_CLIENT_CLOSED : return "LWS_CALLBACK_MQTT_CLIENT_CLOSED";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_ACK : return "LWS_CALLBACK_MQTT_ACK";break;
    case LwsCallbackReason::LWS_CALLBACK_MQTT_RESEND : return "LWS_CALLBACK_MQTT_RESEND";break;
    case LwsCallbackReason::LWS_CALLBACK_USER : return "LWS_CALLBACK_USER";break;
  }
  return "unknown-lws-callback-type!";
}

}
}
}


std::ostream& operator<<(std::ostream& o,coyot3::services::websocket::LwsCallbackReason s)
{
  return (o << coyot3::services::websocket::LwsCallbackReasonToString(s));
}