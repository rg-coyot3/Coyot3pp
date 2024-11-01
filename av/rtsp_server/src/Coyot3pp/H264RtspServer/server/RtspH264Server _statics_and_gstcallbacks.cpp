#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>


namespace ct = coyot3::tools;

namespace coyot3{
namespace av{
namespace rtsp{

  

  namespace ct = coyot3::tools;

  bool extract_key_value_from_query(
    const std::string& query,
    const std::string& key,
    std::string& value){
    CLOG_DEBUG(4,"vsm-publisher : extract key ["<<key << "] value");
    std::size_t kp = query.find(key);
    if(kp == std::string::npos)
    {
      CLOG_WARN("vsm-publisher : extract key [" << key << "] key not found");
      return false;
    }
    kp+=key.size();
    if(kp>=query.size())
    {
      CLOG_WARN("vsm-publisher : extract key [" << key << "] query too short");
      return false;
    }
    if(query[kp++]!='=')
    {
      CLOG_WARN("vsm-publisher : extract key [" << key << "] query not valid : "
        "[" << query[kp] << "]");
      return false;
    }
    value = query.substr(kp,query.substr(kp).find_first_of(';'));
    CLOG_INFO(" value = " << value);
    return true;
  }

  //statics
  void 
  RtspH264Server::gstcallback_on_client_connect(
    GstRTSPServer* server,
    GstRTSPClient* client,
    RtspH264Server* owner){
    
    CLOG_WARN("rtsp-h264-server : static : on-client-connect : static cb");
    owner->on_client_connects(server,client);
  }





  //callbacks

  void 
  RtspH264Server::on_client_connects(
    GstRTSPServer* server,
    GstRTSPClient* client){
    
    log_info(o() << "on-client-connect : " << config_.server_name() 
      << " : configuring incoming connection");
    g_signal_connect(client,
              "options-request", 
              G_CALLBACK(RtspH264Server::gstcallback_client_requests_options),
              this);

    g_signal_connect(client,
              "teardown-request",
              G_CALLBACK(RtspH264Server::gstcallback_client_requests_teardown),
              this);

    log_info(o() << "on-onclient-connect : " << config_.server_name() 
      << " : connection configured OK");
  }


  void RtspH264Server::gstcallback_client_requests_options(
    GstRTSPClient*  client, 
    GstRTSPContext* context, 
    RtspH264Server* owner){
    CLOG_INFO("rtsp-h264-server : on-client-requests-options : static cb")
    owner->on_client_requests_options(client,context);
  }


  void
  RtspH264Server::on_client_requests_options(GstRTSPClient* client, 
                                            GstRTSPContext* context){
    bool close_incoming_connection = false;                                              
    log_info("on-client-requests-opts : begin");

    if(context->uri){
      
      if(strlen(context->uri->abspath) == 0){
        log_warn("on-client-requests-opts : CLIENT DOES NOT SPECIFY PATH!");
        close_incoming_connection = true;
      }
    }else{
      log_warn("on-client-requests-opts : WARNING! CLIENT CONFIG IS NOT "
        "CORRECT!");
      close_incoming_connection = true;
    }

    if(params_.key_auth_use()){
      log_info("on-client-requests-opts : client has to match key");
      std::string q(context->uri->query), v;
      if(!extract_key_value_from_query(q,"key",v) == false){
        log_warn("on-client-requests-ops : WARNING! APIKEY CONTROL IS ACTIVE"
          " BUT CLIENT DOES OFFERS A KEY");
        close_incoming_connection = true;
      }else if(v.compare(params_.key_auth_string()) != 0){
        log_warn(o() << "on-client-requests-ops : WARNING! CLIENT KEY [" << v 
          << "] does not match with this server key [" 
          << params_.key_auth_string() <<"]");
        close_incoming_connection = true;
      }
    }
    StreamsMap::iterator sit = streams.find(std::string(context->uri->abspath));

    if(sit == streams.end()){
      log_warn(o() << "on-client-requests-ops : WARNING! CLIENT TRIES "
        "CONNECTION TO PATH [ " << context->uri->abspath 
        << "] BUT PATH DOES NOT EXIST");
      close_incoming_connection = true;
    }

    if((sit->second->clients_get_num() >= sit->second->clients_get_max())
      &&(sit->second->clients_get_max() != 0)){
      log_warn(o() << "on-client-requests-ops : WARNING! CLIENT TRIES TO"
        "CONNECT TO [ " << context->uri->abspath << "] BUT THIS STREAM HAS "
        "ALREADY A MAXIMUM NUMBER OF CLIENTS CONNECTED. clients[" 
        << sit->second->clients_get_num() << "]");
      close_incoming_connection = true;
    }

    if(close_incoming_connection == true){
      log_warn("on-client-requests-ops : closing incoming connection");
      gst_rtsp_client_close(client);
      return;
    }


    if( (params_.max_clients() > 0 ) 
      && (connected_clients_get_total() >= params_.max_clients())){
      log_warn(o() << "on-client-requests-ops : "
        << params_.server_name() << " : WARNING! max num clients [" 
        << params_.max_clients() << "] reached. CLOSING OLDEST CONNECTION!");
      
      GstRTSPClient* oldest_client;
      oldest_client = connected_clients_get_oldest();
      if(oldest_client != nullptr){
        gst_rtsp_client_close(oldest_client);
      }
    }

    
    RtspClientInformation clientInformation;
    clientInformation.host(context->uri->host);
    clientInformation.port(context->uri->port);
    clientInformation.abs_path(context->uri->abspath);
    clientInformation.query((context->uri->query != nullptr?context->uri->query:""));
    clientInformation.gst_client(client);
    clientInformation.gst_context(context);
    


    log_info(o() << "on-client-requests-opts : attaching client to stream [" 
      << sit->second->name() << "]");
    sit->second->client_connects(clientInformation);
  }


  void RtspH264Server::gstcallback_client_requests_teardown(
    GstRTSPClient* client,
    GstRTSPContext* context,
    RtspH264Server* owner){
  
    CLOG_INFO("rtsp-h264-server : on-client-requests-teardown : static cb")
    owner->on_client_requests_teardown(client,context);
  }



  void 
  RtspH264Server::on_client_requests_teardown(
                                    GstRTSPClient* client,
                                    GstRTSPContext* context){
    CLOG_INFO("rtsp-h264-server : on-client-requests-teardown : begin")
    
    //search context
    
    RtspH264StreamPublisher* pub 
      = stream_get_by_path(context->uri->abspath);
    if(pub == nullptr){
      log_warn(o() << "on-client-requests-teardown : client disconnected but no "
        "streamer was found at this server for path ["
        << context->uri->abspath << "]");
      return;
    }
    RtspClientInformation clientInfo;
    if(pub->client_get(client,clientInfo)){
      if(cb_on_client_disconnect){
        log_debug(3,"rtsp-h264-server : on-client-requests-teardown : "
        "client found. invoking callback");
        cb_on_client_disconnect(clientInfo);
      }
    }
    log_eval(pub->client_disconnects(client),
      o() << "on-client-requests-teardown : client disconnected for [" 
        << context->uri->abspath << "]",
      o() << "on-client-requests-teardown : client disconnection not found "
        "for [" << context->uri->abspath << "]");
    return;
  }

  void RtspH264Server::gstcallback_session_cleanup(RtspH264Server* owner, 
      gboolean ignored){
    CLOG_WARN("rtsp-h264-server : static-session-cleanup")
    owner->on_session_cleanup(ignored);
  }

  void RtspH264Server::on_session_cleanup(bool ignored){
    log_warn("session-cleanup");
    GstRTSPSessionPool* pool;
    int num;
    pool = gst_rtsp_server_get_session_pool(handlers.rtsp_server);
    num = gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);
    if(num > 0){
      log_warn(o() << "session-cleanup : number of cleaned sessions = " << num);
    }
  }

}
}
}