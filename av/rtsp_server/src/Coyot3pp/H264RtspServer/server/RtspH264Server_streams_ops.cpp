#include <Coyot3pp/H264RtspServer/RtspH264Server.hpp>



namespace coyot3{
namespace av{
namespace rtsp{

  
  RtspH264StreamPublisher* 
  RtspH264Server::stream_get_by_path(const std::string& name){
    StreamsMap::iterator it = streams.find(name);
    if(it == streams.end()){
      return nullptr;
    }
    return it->second;
  }
  RtspH264StreamPublisher* 
  RtspH264Server::stream_get_by_name(const std::string& name){
    StreamsMap::iterator it = streams.find(name);
    for(it = streams.begin();it != streams.end(); ++it){
      if(name.compare(it->second->name()) == 0){
        return it->second;
      }
    }
    return nullptr;
  }

  bool RtspH264Server::stream_create(const RtspStreamConfig& config){
    if(streams.find(config.path())!= streams.end()){
      log_warn(o() << "stream-create : path [" << config.path() 
      << "] already exists");
      return false;
    }
    if(stream_get_by_name(config.stream_name())!= nullptr){
      log_warn(o() << "stream-create : name [" 
        << config.stream_name() << "] already exists!. Yep my friend! "
        "you gotta look at the config!");
      return false;
    }
    RtspH264StreamPublisher* newStream;
    newStream = new(std::nothrow) RtspH264StreamPublisher();
    if(!newStream){
      log_err(o() << "stream-create : name [" 
      << config.stream_name() << "] : OUFFF!!! dirty stuff... no memory to"
      " allocate this new server?.");
      return false;
    }
    if(!newStream->set_configuration(config)){
      log_warn(o() << "stream-create : name ["
      << config.stream_name() << "] : streamer was created but guess what..."
      " the config parameters were wrong");
      delete newStream;
      return false;
    }
    streams.insert(std::make_pair(config.path(),newStream));
    log_info(o() << "stream-create : name ["
    << config.stream_name() << "] : for path [" << config.path() << "] "
    "is prepared.");
    return true;
  }

  bool RtspH264Server::publishers_create_(){
    bool opres = true;
    size_t oks = 0;

    CLOG_INFO("rtsp-h264-server : publishers-create- : begin")
    size_t numPreviewedStreams = 0;
    oks = config_.streams_collection().forEach(
      [&](const RtspStreamConfig& sourceConf){
        bool strcop;
        if(sourceConf.is_active() == false){
          CLOG_INFO("rtsp-h264-server : publishers-create- : [" <<
            sourceConf.stream_name() << "] is INACTIVE")
          return true;
        }
        ++numPreviewedStreams;
        if(!stream_create(sourceConf)){
          CLOG_WARN("rtsp-h264-server : publishers-create- : " << name() << 
          " : for source [" << sourceConf.stream_name() << " /p:" << 
          sourceConf.path() << "], there were errors setting up!")
          return false;
        }
      return true;
    });
    
    log_eval(oks == numPreviewedStreams
    ,o() << "publishers-create- : OK for all [" << numPreviewedStreams 
      << "] streams"
    ,o() << "publishers-create- NOT ALL STREAMS WERE CREATED. Only created [" 
      << streams.size() << "] of [" << numPreviewedStreams << "]");
    return (oks == numPreviewedStreams);
  }


  bool RtspH264Server::publishers_attach_(){
    log_info("start-attach-publishers- : attaching owner");
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      log_info(o() << "start-attach-publishers- : attaching owner "
      "for [" <<it->second->name() << "]");
      it->second->set_owner(handlers.rtsp_server);
    }
    return true;
  }

  bool RtspH264Server::publishers_init_(){
    bool allgood = true;
    size_t oks = 0;

    CLOG_INFO("rtsp-h264-server : publishers-init- : initializing publishers")
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      bool thisop;
      CLOG_INFO("rtsp-h264-server : publishers-init- : initializing "
      "publisher for [" <<it->second->name() << "]")
      allgood &= thisop = it->second->Init();
      if(thisop==false){
        log_warn(o() << "rtsp-h264-server : publishers-init- : initialization not " 
          "ok for " << it->second->name());
      }
    }
    log_eval(allgood,
      "rtsp-h264-server : publishers-init- : ok",
      "rtsp-h264-server : publishers-init- : there were errors "
        "starting publishers.");
    return allgood;
  }


  bool RtspH264Server::publishers_start_(){
    bool allgood = true;
    size_t oks = 0;

    log_info("publishers-start- : starting publishers");
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      bool thisop;
      log_info(o() << "publishers-start- : starting publisher for [" 
        <<it->second->name() << "]");
      allgood &= thisop = it->second->Start();
      if(thisop==false){
        log_warn(o() << "publishers-start- : start not ok for " 
          << it->second->name());
      }
    }
    log_eval(allgood,
      "rtsp-h264-server : publishers-start- : ok",
      "rtsp-h264-server : publishers-start- : there were errors starting "
        "publishers.");
    return allgood;
  }


  bool RtspH264Server::publishers_pause_(){
    bool allgood = true;
    size_t oks = 0;

    log_info("start-publishers-pause- : pausing publishers");
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      bool thisop;
      log_info(o() << "publishers-pause- : pausing publisher for [" 
        << it->second->name() << "]");
      allgood &= thisop = it->second->Pause();
      if(thisop==false){
        log_warn(o() << "publishers-pause- : pause not ok for: " 
          << it->second->name());
      }
    }
    log_eval(allgood,
      "rtsp-h264-server : publishers-pause- : ok",
      "rtsp-h264-server : publishers-pause- : there were errors "
        "pausing publishers.");
    return allgood;
  }

  bool RtspH264Server::publishers_stop_(){
    bool allgood = true;
    size_t oks = 0;

    log_info("publishers-stop- : stopping publishers");
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      bool thisop;
      log_info(o() << "publishers-stop- : stopping publisher for [" 
        <<it->second->name() << "]");
      allgood &= thisop = it->second->Stop();
      if(thisop==false){
        log_warn(o() << "publishers-stop- : stopping not ok for " 
          << it->second->name());
      }
    }
    log_eval(allgood,
      "rtsp-h264-server : publishers-stop- : ok",
      "rtsp-h264-server : publishers-stop- : there were errors stopping "
        "publishers.");
    return allgood;
  }

  bool RtspH264Server::publishers_end_(bool force){
    bool allgood = true;
    size_t oks = 0;

    log_info("publishers-end- : ending publishers");
    StreamsMap::iterator it;
    for(it = streams.begin();it != streams.end();++it){
      bool thisop;
      log_info(o() << "publishers-end- : ending publisher for [" 
        << it->second->name() << "]");
      allgood &= thisop = it->second->End(force);
      if(thisop==false){
        log_warn(o() << "publishers-end- : ending not ok for " 
          << it->second->name());
      }
    }
    log_eval(allgood,
      "rtsp-h264-server : publishers-end- : ok",
      "rtsp-h264-server : publishers-end- : there were errors ending "
        "publishers.");

    return allgood;
  }



}
}
}
