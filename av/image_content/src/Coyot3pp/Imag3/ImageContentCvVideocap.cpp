#include <Coyot3pp/Imag3/ImageContent/ImageContentCvVideocap.hpp>

namespace coyot3{
namespace av{
namespace image{

  CYT3MACRO_model_class_definitions(
    ImageContentCvVideocapData
    ,
    , ( )
    , ( )
      , source              , std::string     , ""
      , input_min_interval  , int             , 500
  )


  ImageContentCvVideocap::ImageContentCvVideocap(const std::string& name, const std::string& src)
  :ImageContent(name)
  ,wth(nullptr)
  ,cvcap(nullptr)
  ,source_(src){
    class_name("imgc-cvvideocap");
    log_info(o() << "constructor : conf src=" << source());
    params_.name(name);

    add_task_start(std::bind(&ImageContentCvVideocap::task_start,this));
    add_task_stop(std::bind(&ImageContentCvVideocap::task_stop,this));
  }

  ImageContentCvVideocap::~ImageContentCvVideocap(){}


  bool ImageContentCvVideocap::task_start(){
    
    log_info("start : begin");
    if(wth != nullptr){
      log_warn("start : trying to "
      "launch to an already launched instance? this command will not proceed");
      return false;
    }
    wth = new(std::nothrow) coyot3::tools::WorkerThread(
      std::bind(&ImageContentCvVideocap::task,this),
      params_.name());
    CLOG_EVALUATION(wth!= nullptr,
    "imc-cvvideocap : start : " << params_.name(),
    "created worker thread",
    "ERROR CREATING WORKER THREAD! NO MEM?")
    if(!wth)return false;
    return wth->start(connection_retry_interval());
  }
  bool ImageContentCvVideocap::task_stop(){
    CLOG_EVALUATION(wth != nullptr,
    "imc-cvvideocap : stop : " << params_.name() << " : stopping worker thread",
    "done ok",
    "worker thread is not started?")
    if(!wth)return false;
    is_active_ = false;
    wth->stop();
    delete wth;
    log_info("task-stop : releasing capt");
    wth = nullptr;

    log_info("stop : releasing done");
    return true;
  }


  void ImageContentCvVideocap::task(){
    if(source().size() == 0){
      log_info("imc-cvvideocap : task : opening default source");
      cvcap = new cv::VideoCapture(0);
    }else{
      log_info(o() << "imc-cvvideocap : task : opening source [" 
        << source() << "]");
      cvcap = new cv::VideoCapture(source());
    }
    if(cvcap->isOpened() == false){
      log_warn("imc-cvvideocap : task error opening source! ");
      return;
    }
    is_active_ = true;
    while(is_active_){

      {//mutex lock
        std::lock_guard guard(mtx_source_);
        try{
          cvcap->read(image_source);
          //params_.width(image_source.cols);
          //params_.height(image_source.rows);
        }catch(const cv::Exception& e){
          log_warn(o() << "imc-cvvideocap : task : error "
          "acquiring image. Source lost? [" << source_ << "] : except(" 
          << e.what() << ")");
          log_warn("imc-cvvideocap : task : auto-relaunch");
          is_active_ = false;
        }catch(...){
          log_warn(o() << "imc-cvvideocap : task : error "
          "acquiring image. Source lost? [" << source_ << "]. auto-relaunch");
          is_active_ = false;
        }
      }

      log_debug(5,o() << "imc-cvvideocap : task : acquired image (" 
        << params_.width() << "," << params_.height() << ")");
      if(params_.show_preview()){
        try{
          cv::resizeWindow(params_.name(), params_.width(),params_.height());
          cv::imshow(params_.name(),image_source);
          cv::waitKey(1);
        }catch(const cv::Exception& e){
          log_warn(o() << "imc-cvvideocap : task : error showing preview! "
            "exitting cycle. except(" << e.what() << ")");
          is_active_ = false;
        }catch(...){
          log_warn("imc-cvvideocap : task : error showing preview! exitting "
            "cycle.");
          is_active_ = false;
        }
        
        
      }
      usleep(update_interval()* 1000);
    }
    delete cvcap;
    cvcap = nullptr;
    log_warn("imc-cvvideocap : task : loop exit.");
  }

  std::string ImageContentCvVideocap::source()const{
    return source_;
  }
  std::string ImageContentCvVideocap::source(const std::string& src){
    return (source_ = src);
  }
  int64_t     ImageContentCvVideocap::connection_retry_interval() const{
    return connection_retry_interval_;
  }
  int64_t     ImageContentCvVideocap::connection_retry_interval(int64_t r){
    return (connection_retry_interval_ = r);
  }
  int64_t     ImageContentCvVideocap::update_interval() const{
    return params_.update_interval();
  }
  int64_t     ImageContentCvVideocap::update_interval(int64_t r){
    return (params_.update_interval() = r);
  }

}
}
}