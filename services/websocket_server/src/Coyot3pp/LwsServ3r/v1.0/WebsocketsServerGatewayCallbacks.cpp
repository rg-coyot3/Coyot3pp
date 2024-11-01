#include <Coyot3pp/LwsServ3r/v1.0/WebsocketsServerGateway.hpp>


//#define _WSG_CHUNK_MAX_LENGTH 65536
//#define _WSG_CHUNK_MAX_LENGTH 1024


namespace coyot3{
namespace services{
namespace websocket{


void 
WebsocketsServerGateway::set_debug_level_mcallbacks_(int level)
{
  CLOG_DEBUG_LEVEL_SET(level);
}

bool 
WebsocketsServerGateway::HasUriArguments(struct ::lws* wsi)
{
  return (lws_hdr_total_length(wsi,WSI_TOKEN_HTTP_URI_ARGS) != 0);
}
size_t 
WebsocketsServerGateway::ExtraitUriArguments(struct ::lws* wsi,UriArguments& uriargs)
{
  char buffer[1024];
  UriArguments res;
  int idx = 0;
  int cl;
  while(lws_hdr_copy_fragment(
          wsi,(char*)&buffer,sizeof(buffer),WSI_TOKEN_HTTP_URI_ARGS,idx++) != -1
  )
  {
    
    CLOG_INFO("CYTWSG : helper-obtain-uri-arguments : current index [" << idx 
      << "], obtained [" << buffer << "]");
    std::string buf(buffer);
    std::string f,s;
    f = buf.substr(0,buf.find("="));
    s = buf.substr(buf.find("=")+1);
    uriargs.insert(std::make_pair(f,s));
  }
  return uriargs.size();
}

size_t end_string_pos(const char* s,size_t l)
{
  
  for(size_t i = 0; i<l ;++i)
  {
    if(s[i] == '\r')
    {
      if((++i) < l)
      {
        if(s[i] == '\n')
        {
          return (--i);
        }
      }
    }
  }
  return -1;
}

std::string obtain_lws_header_uri(struct ::lws* wsi)
{
  std::string resp;
  uint8_t buffer[LWS_PRE + _WSG_CHUNK_MAX_LENGTH + 1024];
  int n;
  n = lws_hdr_copy(wsi,(char*)buffer,sizeof(buffer),WSI_TOKEN_GET_URI);
  if(n == -1)
  {
    return "";
  }
  CLOG_DEBUG(5,"CYTWSG : callbacks : helper-obtain-lws_header-uri : buffer size = " << n);
  resp = (char*)buffer;
  return resp;
}
std::string obtain_lws_peer(struct ::lws* wsi)
{
  std::string res;
  char buffer[LWS_PRE + 4000];
  CLOG_DEBUG(8,"CYTWSG : callbacks : helper-obtain-hws-peer : size of buff " << sizeof(buffer));
  if(
    lws_get_peer_simple(wsi,(char*)buffer,sizeof(buffer)) == nullptr
  ){
    return "";
  }
  res = buffer;
  CLOG_DEBUG(5,"CYTWSG : callbacks : helper-obtain-hws-peer : serving :" << res);
  return res = buffer;
}

int WebsocketsServerGateway::callback_dyn_http(
        struct ::lws* wsi,
        enum ::lws_callback_reasons reason,
        void* in,
        size_t len,
        WsgConnectionStructure* dpr   
){
  CLOG_DEBUG(5,"CYTWSG : callback-dyn-http :" << _name << " : on dyn http request ");
  
  uint8_t buffer[LWS_PRE + 4000]
          ,*start = &buffer[LWS_PRE]
          ,*p = start
          ,*end=&buffer[sizeof(buffer) - LWS_PRE - 1];
  char*   b;
  int     n;
  size_t  l;
  size_t  pes,copylen;
  int     bytesSent;
  WsgClient* currentClient = nullptr;
  UriArguments uriargs;

  uint8_t* incoming_msg;


  std::string basePath;
  std::string sourcePeer;
  

  currentClient = get_client(dpr->ref_id);
  if(currentClient) //client is already registered
  {
    CLOG_DEBUG(5,"CYTWSG : callback dyn http : " << _name << " : [" << CallbackReasonToString(reason) << "] flux for client [" << currentClient->getConnectionID() << "] from peer [" << currentClient->getPeerInfo() << "]");
  }
  else // in case
  {
    dpr->source_peer = obtain_lws_peer(wsi).c_str();
    CLOG_DEBUG(5,"CYTWSG : callback dyn http : " << _name << " : [" << CallbackReasonToString(reason) << "] no registered client from [" << dpr->source_peer << "]");
  }

  
  switch(reason)
  {
    //new client connects. "(VH) after the server completes a handshake with an incoming client."
    case LWS_CALLBACK_ESTABLISHED:
      CLOG_DEBUG(2,"CYTWSG : on-dyn-http callback : LWS_CALLBACK_ESTABLISHED");
      if(!currentClient)
      {
        dpr->wsi = wsi;
        currentClient = add_client(dpr,wsi);
      }

      callback_onwsconnects(currentClient);
      break;


    case LWS_CALLBACK_HTTP:
    /**
     * an http request is done.
     * 
     * 
     * 
     */


      CLOG_DEBUG(3,"CYTWSG : on-dyn-http callback : LWS_CALLBACK_ESTABLISHED");
        // n = lws_hdr_copy(wsi,(char*)buffer,sizeof(buffer),WSI_TOKEN_GET_URI);// b = new char[n - strlen((const char*)in)];// memcpy(b,buffer,n - strlen((const char*)in));
      basePath = obtain_lws_header_uri(wsi);
        // delete b;
      CLOG_DEBUG(3,"CYTWSG : " << _name << " : callback dyn http : basepath [" << basePath << "]");
      dpr->source_peer = obtain_lws_peer(wsi);

      if(lws_add_http_common_headers(wsi
                                    ,HTTP_STATUS_OK
                                    ,"text/html"
                                    ,LWS_ILLEGAL_HTTP_CONTENT_LEN
                                    ,&p
                                    ,end))
      {
        CLOG_WARN("CYTWSG : callback dyn http : " << _name << " : no content len for connection from [" << sourcePeer << "]");
        return 1;//open connection with no input data?
      }
      if(lws_finalize_write_http_header(wsi,start,&p,end)) //error writing headers for the response.
      {
        CLOG_WARN("CYTWSG : callback dyn http : " << _name << " : error : finalize write http header");
      }
      basePath = obtain_lws_header_uri(wsi);
      CLOG_DEBUG(5,"CYTWSG : callback-dyn-http : base-path [" << basePath << "]");
      if(HasUriArguments(wsi)){ ExtraitUriArguments(wsi,uriargs);}          //prepara uri arguments if available
      currentClient = get_client(dpr->ref_id);
      if(!currentClient)
      {
        CLOG_WARN("CYTWSG : callback-dyn-http : base-path [" << basePath << "] no registered found for this connection");
        currentClient = add_client(dpr,wsi);
      }


      {
        std::string response;
        bool respond_now;
        respond_now = callback_ondynhttpreq(currentClient,basePath,uriargs,response);
        currentClient->appendToResponseContent(response);
        currentClient->resetOutputBufferIndex();
        
        if(respond_now)
        {
          lws_callback_on_writable(wsi);
        }else{
          //guard this client so a response is sent after a timeout
          
          currentClient->standBy(_async_comms_control_timeout);
          //-------------------------
          //lws_set_timeout(wsi,PENDING_TIMEOUT_CLOSE_ACK,CYTWSG_CONFIG_DEFAULT_LWS_HTTPTIMEOUT_SECS);
          lws_set_timeout(wsi,PENDING_TIMEOUT_CLOSE_ACK,(_async_comms_control_timeout / 1000) + 2);
          CLOG_WARN("CYTWSG : on-http-dyn-callback : [LWS_CALLBACK_HTTP] "
            "standing by for client : [" << currentClient->getConnectionID() 
            << "] a maximum of [" << _async_comms_control_timeout 
            << " msecs], lws set to [" << ((_async_comms_control_timeout / 1000) + 2) << " seconds]");
          //-------------------------
        }
      }
      return 0;
      break;

    
    case LWS_CALLBACK_HTTP_WRITEABLE:
      basePath = obtain_lws_header_uri(wsi);
      //CLOG_INFO(" *** lws callback http writeable << base path << " << basePath);
      if(HasUriArguments(wsi))
      {
        ExtraitUriArguments(wsi,uriargs);
      } 

      n = LWS_WRITE_HTTP;
      currentClient = get_client(dpr->ref_id);
      
      if(dpr->response.size() == 0)
      {
        CLOG_WARN("CYTWSG : callback-dyn-http : LWS_CALLBACK_HTTP_WRITEABLE : response is not prepared");
        dpr->response = "CYTWSG : internal engine : no complete response formed";
        dpr->response_total_length = dpr->response.size();
        dpr->response_current_index = 0;
      }
      else
      {
        CLOG_DEBUG(5,"CYTWSG : callback dyn http : " << _name << " : response already created : size [" << dpr->response.size() << "]");
      }

      
      if(//if current response size is inferior to maximal chunk size, prepare for final transmission
          (dpr->response_current_index+_WSG_CHUNK_MAX_LENGTH) 
          > dpr->response_total_length)                              
      {
        n = LWS_WRITE_HTTP_FINAL;
        l = (dpr->response_total_length - dpr->response_current_index);
      }else{ //else, prepare a transmission of a portion of the string.
        l = _WSG_CHUNK_MAX_LENGTH;
      }
      CLOG_DEBUG(5,"CYTWSG : callback dyn http : " << _name << " : chunk size " << l << "; chunk content [" << dpr->response.substr(dpr->response_current_index,l)<< "]");
    
      memcpy(&buffer[LWS_PRE]
            ,(const void*)dpr->response.substr(dpr->response_current_index,l).c_str()
            ,l);
      dpr->response_current_index+=l;


      if(//transmit the chunk, and if there is a problem, close connection (return 1)
          lws_write(wsi
                  ,&buffer[LWS_PRE]
                  //,l+LWS_PRE
                  ,l
                  ,static_cast<enum lws_write_protocol>(n)) != (l))
      {
        CLOG_ERROR("CYTWSG CYTWSG : callback dyn http : " << _name << " : error sending chunk (size = " << (l + LWS_PRE) << ") to peer ["<< dpr->source_peer << "]");
        return 0; //if returns 1, it will kill the server
      } 
      else{//if the transmission was ok
        CLOG_DEBUG(5,"CYTWSG : callback dyn http : " << _name << " : written a total of [" << (l+LWS_PRE) << "] bytes");
      }


      if(n == LWS_WRITE_HTTP_FINAL) // if the transaction is completed
      {
        if(lws_http_transaction_completed(wsi)) //and the connection must close
        {
          return -1; //close the connection
        }
      }else{ //else, invoke this callback when this websocket insterface is ready for writing
          lws_callback_on_writable(wsi); 
      }
      return 0;
      break;

    //initialization of the communication protocol
    case LWS_CALLBACK_PROTOCOL_INIT:
      CLOG_DEBUG(3,"CYTWSG : on-dyn-http callback : LWS_CALLBACK_PROTOCOL_INIT");
      break;






    // clean client information.
    case LWS_CALLBACK_HTTP_DROP_PROTOCOL:
    case LWS_CALLBACK_WSI_DESTROY:
    case LWS_CALLBACK_CLOSED:      
      CLOG_DEBUG(2,"CYTWSG : on-dyn-http callback : LWS_CALLBACK_CLOSED");
      callback_onwsdisconnects(currentClient);
      del_client(dpr->ref_id);

      break;



    // incoming data from one websocket connection.
    case LWS_CALLBACK_RECEIVE:
      CLOG_DEBUG(3,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_RECEIVE");
      pes = end_string_pos((const char*)in,len);
      // incoming_msg = new uint8_t[len];
      incoming_msg = (uint8_t*)in;
      // memcpy(incoming_msg,in,len);
      if(pes == size_t(-1))
      {
        CLOG_DEBUG(5,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_RECEIVE : no end string : l[" << len << "]");
        copylen = len;
      }
      else
      {
        CLOG_DEBUG(3,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_RECEIVE : detected end string : l[" << len << "] at [" << pes << "]");
        copylen = pes;
      }
      //incoming_msg[len] = '\0';
      for(size_t i=0;i<copylen;++i)
      {
        dpr->ws_input_stream.push_back(incoming_msg[i]);
      }
      
      if(copylen<=(len-2)) // all the input string is formed, all input string contains a null-terminated character
      {
        callback_onwsmessage(currentClient,dpr->ws_input_stream.c_str(),dpr->ws_input_stream.size());
        dpr->ws_input_stream.clear();
      }
      if(copylen < (len-2)) //preparing chunk 
      {
        for(size_t i=(copylen+2);i<len;++i)
        {
          dpr->ws_input_stream.push_back(incoming_msg[i]);
          dpr->ws_message_last_rx_ts = coyot3::tools::getCurrentTimestamp();
        }
      }
      break;

    // websocket connection is ready to send data.
    case LWS_CALLBACK_SERVER_WRITEABLE:
      if(!currentClient)
      {
        CLOG_DEBUG(4,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_SERVER_WRITEABLE : " << _name << " : no associated client?!");
        break;
      }else if(!currentClient->getOutputBufferSize())
      {
        CLOG_DEBUG(5,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_SERVER_WRITEABLE : " << _name << " : for client"
        << currentClient->getConnectionID() << " : no output buffer to send");
        break;
      }
      CLOG_DEBUG(4,"CYTWSG : on-dyn-http-callback : LWS_CALLBACK_SERVER_WRITEABLE : " 
        << _name << " : on writeable for : " << currentClient->getConnectionID() << "]");

      bytesSent = lws_write(
                        wsi
                        ,&currentClient->getOutputBuffer()[LWS_PRE + currentClient->getOutputBufferIndex()]
                        ,currentClient->getOutputBufferSize() - currentClient->getOutputBufferIndex()
                        ,LWS_WRITE_TEXT);

      if(bytesSent == -1)                                                       //error sending data
      {
        CLOG_WARN("CYTWSG : on-dyn-http-callback : LWS_CALLBACK_SERVER_WRITEABLE : " << _name << " : " 
        << currentClient->getConnectionID() << "] : fatal error sending data");
        return -1;
      }                                                                         //sent a portion of the data
      else if(bytesSent < currentClient->getOutputBufferSize()){ 
        CLOG_DEBUG(4,"WSG : " << _name << " : callback : on writeable for :"
        << currentClient->getConnectionID() << "] : sent [" << bytesSent << "/"
        << currentClient->getOutputBufferSize() << "]");
        currentClient->outputBufferIndexInc(bytesSent);
        lws_callback_on_writable(wsi);
      }else{                                                                    //all data has been sent
        CLOG_DEBUG(4,"WSG : " << _name << " : callback : on writeable for :" 
        << currentClient->getConnectionID() << "] : sent all buffer : cleaning");
        currentClient->outputBufferClean();
      }
      break;
#if CYTWSG_DEVELOPPEMENT == true
    default:
      CLOG_WARN("CYTWSG : " << _name << " : callback dyn http : (" << basePath
      << " :) callback reason not implemented [" << CallbackReasonToString(reason) << "]");
#endif
  }
  return lws_callback_http_dummy(wsi, reason, (void*)dpr,in,len);
}






bool 
WebsocketsServerGateway::dummy_callback_on_dyn_http(
                    coyot3::services::websocket::WsgClient* client
                    ,const std::string& originPath
                    ,const UriArguments& uriargs
                    ,std::string& response)
{
  CLOG_WARN("WSG :: dummy_callback_on_dyn_http : no callback set");
  return true;
}
bool 
WebsocketsServerGateway::dummy_callback_on_ws_message(
  coyot3::services::websocket::WsgClient* client,
  const uint8_t* buffer,
  size_t length)
{
  CLOG_WARN("WSG :: dummy_callback_on_ws_message : no callback set for "
  "buffer size [" << length << "]");
  return true;
}

bool 
WebsocketsServerGateway::dummy_callback_on_ws_connects(
  coyot3::services::websocket::WsgClient* client)
{
  CLOG_WARN("WSG :: dummy_callback_on_ws_connects : no callback set");
  return true;
}

bool 
WebsocketsServerGateway::dummy_callback_on_ws_disconnects(
  coyot3::services::websocket::WsgClient* client)
{
  CLOG_WARN("WSG :: dummy_callback_on_ws_disconnects : no callback set");
  return true;
}
bool 
WebsocketsServerGateway::dummy_callback_on_error(
  coyot3::services::websocket::WsgClient* client,const std::string& what){
  CLOG_WARN("WSG :: dummy_callback_on_error : no callback set");
  return true;
}

}
}// eons wrappers
}// eons coyot3