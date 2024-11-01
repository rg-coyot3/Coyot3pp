#include <Coyot3pp/H264RtspServer/RtspH264StreamPublisher.hpp>

namespace ct = coyot3::tools;
namespace ci = coyot3::av::image;
namespace cm = coyot3::mod;

namespace coyot3{
namespace av{
namespace rtsp{


  RtspH264StreamPublisher::RtspH264StreamPublisher(const std::string& name)
  :ModuleBase(name)
  , config_()
  , handlers()
  , params_()
  , clients()
  , source_image(nullptr)
  , stream_image(){
    class_name("rtsp-h264-serv");
    log_info("constructor");
    conf_task_init_(std::bind(&RtspH264StreamPublisher::task_init,this));
    conf_task_start_(std::bind(&RtspH264StreamPublisher::task_start,this));
  }

  RtspH264StreamPublisher::~RtspH264StreamPublisher(){
    log_warn("destructor");
    End(true);
  }



  bool 
  RtspH264StreamPublisher::set_configuration(const RtspStreamConfig& config){
    bool allgood = true, thisop;
    config_ = config;
    name(config_.stream_name());
    params_.name(config_.stream_name());
    params_.path(config_.path());
    params_.width(config_.width());
    params_.height(config_.height());
    params_.production_interval(1000/config_.update_frequence());
    
    params_.pipeline(ct::CytStringSetStringify(config_.gst_pipeline_chain()));
    allgood &= thisop = (params_.pipeline().size() != 0);
    log_eval(thisop, 
      o() << "set-config : ok for pipeline ((" << params_.pipeline() << "))",
      "set-config : ERROR! PIPELINE NOT SET!!! THIS IS NOT GOING TO WORK!!!");
    params_.path(config_.path());
    log_eval(thisop, 
      o() << "set-config : ok for path ((" << params_.path() << "))",
      "set-config : ERROR! PATH NOT SET!!! THIS IS NOT GOING TO WORK!!!");
    allgood = (params_.path().size() != 0);

    return true;
  }

  bool        
  RtspH264StreamPublisher::task_init(){
    bool opres = true;
    log_info("init : begin");
    opres = (handlers.owner != nullptr);
    log_eval(opres, 
      "init : owner check OK",
      "init : OWNER IS NOT SET!");
    
    opres = init_stream_thread_create_();
    log_eval(opres, 
      "init : creating stream thread : OK",
      "init : creating stream thread : NOT CREATED!");

    return opres;
  }


  bool        
  RtspH264StreamPublisher::task_start(){
    bool opres = true;

    if((opres &= start_rtsp_server_add_url_()) == false){
      log_warn("start : error creating/linking server!");
    }else if((opres &= source_init_()) == false){
      log_warn("start : warnings initilalizing source!");
    }else if((opres &= source_start_()) == false){
      log_warn("start : error/warnings starting sources!");
    }else if((opres &= start_stream_thread_launch_()) == false){
      log_warn("start : error launching stream thread!");
    }
    
    if(opres == true){
      log_info("start : done OK ");
    }else{
      log_warn("start : ERROR! ");
    }
    
    return opres;
  }




  // ops
  bool 
  RtspH264StreamPublisher::start_rtsp_server_add_url_(){
    
    log_info(o() << "start-rtsp-server-add-url- : " 
      << params_.name() << " : for path [" << params_.path() << "] begin");
      
    bool opres       = true;

        log_debug(3,o() << "start-rtsp-server-add-url- : "
          << params_.name() << "getting mount points");
    handlers.mounts  = gst_rtsp_server_get_mount_points(handlers.owner);

        log_debug(3,o() << "start-rtsp-server-add-url- : "
          << params_.name() << "creating factory");
    handlers.factory = gst_rtsp_media_factory_new();

        log_debug(3,o() << "start-rtsp-server-add-url- : "
          << params_.name() << "setting pipeline for this factory");
    gst_rtsp_media_factory_set_launch(
      handlers.factory,
      params_.pipeline().c_str());

        log_debug(3,o() << "start-rtsp-server-add-url- : "
          << params_.name() << "setting retransmission time to zero");
    gst_rtsp_media_factory_set_retransmission_time(
      handlers.factory, 
      0*GST_MSECOND);
    
        log_debug(3,o() << "start-rtsp-server-add-url- : " 
          << params_.name() << "connecting 'media-configure' callback");
    handlers.media_connection = 
      g_signal_connect(
          handlers.factory,
          "media-configure", 
          (GCallback)RtspH264StreamPublisher::static_factory_media_configure,
          //handlers.gst_application_alt);
          this);

    opres = (handlers.media_connection != 0);
    CLOG_EVALUATION(opres,
      "rtsp-h264-stream : start-rtsp-server-add-url- : " << params_.name() 
      , "connected OK."
      , "ERROR CONNECTING RTSP CALLBACKS!")
    
          log_debug(3,"start-rtsp-server-add-url- : setting factory as shared");
    gst_rtsp_media_factory_set_shared(handlers.factory, true);
    
          log_debug(3,o() <<"start-rtsp-server-add-url- : "
          "setting factory path to [" << params_.path() << "]");
    gst_rtsp_mount_points_add_factory(handlers.mounts,
                                      config_.path().c_str(),
                                      handlers.factory);
          
          log_debug(3,"start-rtsp-server-add-url- : cleaning allocations for "
            "mounts");
    g_object_unref(handlers.mounts);
    
          log_debug(3,"start-rtsp-server-add-url- : setting zero-latency "
            "property");
    gst_rtsp_media_factory_set_latency(handlers.factory,0);

    log_info(o() << "start-rtsp-server-add-url-" << params_.name() 
      << " : for path [" << params_.path() << "] : DONE");
    return opres;
  }



  bool 
  RtspH264StreamPublisher::source_init_(){
    log_info("source-init-");
    if(source_image== nullptr){
      log_info("source-init- : ERROR! SOURCE IMAGE NOT SET!");
      return true;
    }
    return source_image->Init();
    return true;
  }



  bool 
  RtspH264StreamPublisher::source_start_(){
    log_info("source-start-");
    if(!source_image){
      log_info("source-start- : ERROR! SOURCE IMAGE NOT SET!");
    }
    if(source_image->initialized()){
      return source_image->Start();
    }
    return true;
  }

  bool
  RtspH264StreamPublisher::source_stop_(){
    log_info("stop-source-");
    if(!source_image){
          log_info("source-stop- : ERROR! SOURCE IMAGE NOT SET!");
          return false;
    }
    return source_image->Stop();
  }


  bool 
  RtspH264StreamPublisher::init_stream_thread_create_(){
    handlers.stream_thread = new(std::nothrow) coyot3::tools::WorkerThread(
      std::bind(&RtspH264StreamPublisher::stream_image_publish,this)
      ,"stream-publisher"
    );
    log_eval(handlers.stream_thread != nullptr,
    o () << "rtsp-h264-stream : init-stream-thread-create- : " << params_.name() 
      << "worker created OK",
    o () << "rtsp-h264-stream : init-stream-thread-create- : " << params_.name() 
      << "ERROR CREATING WORKER! NO MEM?");

    return (handlers.stream_thread != nullptr);
  }

  bool 
  RtspH264StreamPublisher::start_stream_thread_launch_(){
    log_info(o() << "start-stream-thread-launch- : starting with iteration "
      "interval of [" << params_.production_interval() << "] msecs");
    return handlers.stream_thread->start(params_.production_interval());
  }


  bool 
  RtspH264StreamPublisher::stop_stream_thread_(){
    if(handlers.stream_thread == nullptr) return false;
    return handlers.stream_thread->stop();
  }

  bool 
  RtspH264StreamPublisher::end_stream_thread_kill_(){
    log_warn("end-stream-thread-kill- : removing");
    if(handlers.stream_thread == nullptr)return false;
    delete handlers.stream_thread;
    return true;
  }




  
  bool RtspH264StreamPublisher::client_get(const GstRTSPClient* ptr,
                                      RtspClientInformation& cli){
    ClientsMap::iterator it = clients.find(ptr);
    if(it == clients.end()){
      return false;  
    }
    cli = it->second;
    return true;
  }

  int RtspH264StreamPublisher::clients_get_num(){
    return static_cast<int>(clients.size());
  }
  int RtspH264StreamPublisher::clients_get_max(){
    return params_.max_clients();
  }
  
  bool RtspH264StreamPublisher::client_connects(const RtspClientInformation& cli){
    bool exists = false;

    ClientsMap::iterator it = clients.find(cli.gst_client());
    
    if(it != clients.end()){
      log_warn("client-connects : gst-client seems already signed up. updating "
        "stored client information.");
      it->second = cli;
    }else{
      log_info("client-connects : gst-client connected");
      clients.insert(std::make_pair(cli.gst_client(),cli));
    }

    if(clients.size() != 0){
      //CLOG_INFO("to-delete : client connects : gst-app-alt = " << handlers.gst_application_alt)
      handlers.gst_application = handlers.gst_application_alt;
    }

    return !exists;
  }


  bool RtspH264StreamPublisher::client_disconnects(const GstRTSPClient* ptr){
    ClientsMap::iterator it = clients.find(ptr);
    if(it == clients.end()){
      log_warn("client-disconnects : gst-client not found at this server");
      return false;
    }
    log_info("client-disconnects : ");
    it->second.disconnection_ts(ct::get_current_timestamp());
    /*aquí ya veré si incluyo otra información*/
    return true;
  }

  bool RtspH264StreamPublisher::stream_processing_activate_(){
    log_info("stream-processing-activate- : activating");
    if(!handlers.stream_thread->isRunning()){
      handlers.stream_thread->start();
    }
    return true;
  }
  bool RtspH264StreamPublisher::stream_processing_deactivate_(){
    log_info("stream-processing-deactivate- : deactivating");
    if(handlers.stream_thread->isRunning()){
      handlers.stream_thread->stop();
    }
    return true;
  }

}
}
}