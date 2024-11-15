#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>



namespace coyot3{
namespace av{
namespace rtsp{

  //"static" internal
  bool GST_IS_INITIALIZED = false;





  RtspH264Server::RtspH264Server(const std::string& name)
  : ModuleBase(name)
  ,config_()
  ,handlers()
  ,params_()
  ,streams()
  {
    class_name("h264-rtsp-server");
    log_info("constructor ");
    CLOG_INFO("BORRAR ÉSTO... SOLO ES PARA SABER QUE LLEGO AQUÍ")
    add_task_init(std::bind(&RtspH264Server::task_init_,this));
    add_task_start(std::bind(&RtspH264Server::task_start_,this));
    add_task_pause(std::bind(&RtspH264Server::task_pause_,this));
    add_task_stop(std::bind(&RtspH264Server::task_stop_,this));
    add_task_end(std::bind(&RtspH264Server::task_end_,this));

  }

  RtspH264Server::~RtspH264Server(){
    log_warn("destructor");
    End(true);
  }

  bool RtspH264Server::set_configuration(const RtspServerConfig& config)
  {
    config_ = config;
    params_.server_name(config_.server_name());
    params_.server_port(config_.server_port());
    params_.base_path(config_.base_path());
    params_.ssl_on(config_.ssl_on());
    params_.file_ssl_ca(config_.file_ssl_ca());
    params_.file_ssl_cert(config_.file_ssl_cert());
    params_.file_ssl_key(config_.file_ssl_key());
    params_.auth_global_use(config_.auth_global_use());
    params_.auth_user(config_.auth_user());
    params_.auth_pass(config_.auth_pass());
    params_.auth_credentials_use(config_.auth_credentials_use());
    params_.auth_client_credentials_file(config_.auth_client_credentials_file());
    params_.debug_level(config_.debug_level());
    params_.key_auth_use(config_.key_auth_use());
    params_.key_auth_string(config_.key_auth_string());
    params_.key_auth_autogen(config_.key_auth_autogen());
    params_.max_clients(config_.max_clients());
    name(params_.server_name());

    handlers.server_port = config.server_port();
    return true;
  }

  bool 
  RtspH264Server::task_init_(){
    //crear los servidores.
    log_info("init : creating publishers");
    return publishers_create_();
  }


  
  bool 
  RtspH264Server::task_start_(){
    bool fops = true;
    log_info("start : begin");
    if(GST_IS_INITIALIZED){
      log_info("start : gst is initialized");
    }else{
      log_info("start : initializing gst");
      gst_init(nullptr,nullptr);
      GST_IS_INITIALIZED = true;
    }

    if((fops &= create_rtsp_server_()) == false){
      log_warn("start : creation of the rtsp-server FAILED");
    }else if((fops &= publishers_attach_()) == false){
      log_warn("start : attach of streamers FAILED");
    }else if((fops &= publishers_init_()) == false){
      log_warn("start : initialization of streamers FAILED");
    }else if((fops &= publishers_start_()) == false){
      log_warn("start : start of streamers FAILED");
    }else if((fops &= server_main_loop_start_()) == false){
      log_warn("start : error starting gst-mainloop");
    }
    CLOG_EVALUATION(fops,
    "rtsp-h264-server : start",
    "DONE OK",
    "FAILED")
    return fops;
  }

  bool
  RtspH264Server::server_main_loop_start_(){
    
    new std::thread([&](){
      handlers.gstreamer_main_loop = g_main_loop_new(nullptr,false);
      g_main_loop_run(handlers.gstreamer_main_loop);
      g_main_loop_unref(handlers.gstreamer_main_loop);
    });
    
    return true;
  }
  
  bool 
  RtspH264Server::task_pause_(){
    
    log_info("pause : pausing publishers");
    return publishers_pause_();
  }
  bool 
  RtspH264Server::task_stop_(){
    log_info("stop : stopping publishers");
    return publishers_stop_();
    
  }
  bool 
  RtspH264Server::task_end_(){
    log_warn("end : ending publisherd");
    return publishers_end_(true);
    
  }





  bool RtspH264Server::create_rtsp_server_(){
    bool fops = true;

    handlers.rtsp_server           = gst_rtsp_server_new();
    fops &= (handlers.rtsp_server != nullptr);
    log_eval(fops
      ,"creating rtsp server instance : ok"
      ,"creating rtsp server instance : GST-ERROR! server not created"
      );
    if(!fops)return false;

    log_info(o() << "start-create-rtsp-server : setting server "
      "port to [" << handlers.server_port << "]");
    gst_rtsp_server_set_service(
      handlers.rtsp_server, 
      std::to_string(handlers.server_port).c_str());

    handlers.rtsp_server_source_id 
      = gst_rtsp_server_attach(handlers.rtsp_server,nullptr);

    log_eval((handlers.rtsp_server_source_id != 0)
      ,"start-create-rtsp-server : attaching server to global context : ok"
      ,"start-create-rtsp-server : attaching server to global context : "
        "GST-ERROR! server not created");
    if(handlers.rtsp_server_source_id == 0)return false;
    //
    g_signal_connect(
      handlers.rtsp_server,
      "client-connected",
      G_CALLBACK(RtspH264Server::gstcallback_on_client_connect), this);
    
    g_timeout_add_seconds(
      2,
      (GSourceFunc)RtspH264Server::gstcallback_session_cleanup,
      this);
    return fops;
  }


  bool
  RtspH264Server::destroy_rtsp_server_(){
    log_warn("not yet implemented");
    return false;
  }





}
}
}