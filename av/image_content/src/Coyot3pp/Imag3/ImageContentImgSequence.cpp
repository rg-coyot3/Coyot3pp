#include <Coyot3pp/Imag3/ImageContent/ImageContentImgSequence.hpp>


namespace cm = coyot3::mod;
namespace ct = coyot3::tools;


namespace coyot3{
namespace av{
namespace image{

  ImageContentImgSequence::ImageContentImgSequence(const std::string& name)
  :ImageContent(name)
  ,params_imgsec_()
  ,wth_(nullptr)
  ,just_launched_(false){
    class_name("imc-imgsec");
    log_info("constructor");
    add_task_init(std::bind(&ImageContentImgSequence::task_init,this),true);
    add_task_init(std::bind(&ImageContentImgSequence::task_start,this));
    add_task_pause(std::bind(&ImageContentImgSequence::task_pause,this));
    add_task_stop(std::bind(&ImageContentImgSequence::task_stop,this));
    add_task_end(std::bind(&ImageContentImgSequence::task_end,this));

  }
  ImageContentImgSequence::~ImageContentImgSequence(){
    CLOG_WARN("image-sequence-content : destructor")
    End(true);
  }

  bool 
  ImageContentImgSequence::set_configuration(
    const ImageContentImgSequenceParams& conf){
      params_imgsec_ = conf;
    return true;
  }
  
  ImageContentImgSequenceParams& ImageContentImgSequence::params_imgsec(){
    return params_imgsec_;
  }


  bool ImageContentImgSequence::task_init(){
    log_info("init : begin");
    wth_ = new(std::nothrow) ct::WorkerThread(
      std::bind(&ImageContentImgSequence::image_sequence_task,this),"imc");
    if(!wth_){
      log_err("init : fatal error creating worker thread");
      return false;
    }
    params_imgsec().current_index(0);
    wth_->setInterval(params_imgsec_.interval());
    log_info("init : end");
    return true;
  }
  bool ImageContentImgSequence::task_start(){
    log_info("start : begin");
    wth_->start();
    log_info("start : end");
    return true;
    
  }
  bool ImageContentImgSequence::task_pause(){
    log_info("pause : begin");
    wth_->stop();
    log_info("pause : end");
    return true;
  }
  bool ImageContentImgSequence::task_stop(){
    log_info("stop : begin");
    wth_->stop();
    log_info("stop : end");
    return true;
  }
  bool ImageContentImgSequence::task_end(){
    log_info("end : begin");
    if(wth_ != nullptr){
      log_info("end : clearing worker thread");
      wth_->stop();
      delete wth_;
      wth_ = nullptr;
    }
    log_info("ended");
    return true;
  }

  bool ImageContentImgSequence::next(){
    params_imgsec_.current_index()++;
    if(params_imgsec_.current_index() >= params_imgsec_.sequence().size()){
      params_imgsec_.current_index(0);
    }
    if((wth_ != nullptr) && (state() != cm::ec::CytModuleState::STOPPED)){
      wth_->triggerNow();
    }
    return true;
  }
  bool ImageContentImgSequence::previous(){
    params_imgsec_.current_index()--;
    if(params_imgsec_.current_index() < 0 ){
      params_imgsec_.current_index(params_imgsec_.sequence().size() - 1);
    }
    if((wth_ != nullptr) && (state() != cm::ec::CytModuleState::STOPPED)){
      wth_->triggerNow();
    }
    return true;
  }



  void ImageContentImgSequence::calculate_next_index(){
    CLOG_DEBUG(7,"imc-image-sequence : calc-next-index- :")

    std::string buffer;
    buffer = params_.base_path() + params_imgsec_.sequence()[
        params_imgsec_.current_index()]
        .path();
    CLOG_DEBUG(5,"imc-image-sequence : calculate next index : ")

    
    if(just_launched_ == true){
      bool fe = ct::file_exists(
        params_imgsec_.sequence()[
          params_imgsec_.current_index()]
          .path());
      
      just_launched_ = false;
      if(fe == true)return;
    }
    int init_index = params_imgsec_.current_index();
    int numerrs = 0;
    bool allgood = false;

    while(allgood == false){
      switch(params_imgsec_.mode()){
        case ec::ImageContentImgSequenceMode::SEQUENCE_NORMAL:
          CLOG_DEBUG(5,"imc-image-sequence : calculate next index : seq-normal")
          ((params_imgsec_.current_index() + 1)>=params_imgsec_.sequence().size())?
            params_imgsec_.current_index(0):
            params_imgsec_.current_index(params_imgsec_.current_index() +1);
          break;
        case ec::ImageContentImgSequenceMode::SEQUENCE_INVERSE:
          CLOG_DEBUG(5,"imc-image-sequence : calculate next index : seq-inv")
          ((params_imgsec_.current_index() - 1)< 0)?
            params_imgsec_.current_index(params_imgsec_.sequence().size() -1):
            params_imgsec_.current_index(params_imgsec_.current_index() - 1);
          break;
        case ec::ImageContentImgSequenceMode::RANDOM:
          CLOG_DEBUG(5,"imc-image-sequence : calculate next index : random")
          params_imgsec_.current_index(
            ct::generate_natural_number(0,params_imgsec_.sequence().size()));
          break;
        case ec::ImageContentImgSequenceMode::STATIC:
          CLOG_DEBUG(5,"imc-image-sequence : calculate next index : static")
          break;
        default:
          CLOG_DEBUG(5,"imc-image-sequence : calculate next index : "
          "UNHANDLED MODE!")
          
      }
      if(init_index == params_imgsec_.current_index()){
        CLOG_DEBUG(3,"imc-image-sequence : calculate next index : current index"
        "is same as precedent (" << init_index << ")")
      }
      buffer = params_.base_path() + params_imgsec_.sequence()[
        params_imgsec_.current_index()]
        .path();
      if(ct::file_exists(buffer) == false){
        CLOG_WARN("imc-image-sequence : calculate next index : "
          "unable to find file [" << buffer << "] ! ignoring!")
        ++numerrs;
      }else{
        allgood = true;
      }

      if(numerrs>3){
        CLOG_WARN("imc-image-sequence : calculate next index : "
          "too many errors. leaving iteration with precedent index [" 
          << init_index << "]")
        params_imgsec_.current_index(init_index);
      }
    }
    current_file_path_ = buffer;
    CLOG_DEBUG(5,"imc-image-sequence : calculate next index : set index [" 
    << params_imgsec_.current_index() << "] targeted [" 
    << current_file_path_ << "]")
  }
  void ImageContentImgSequence::image_sequence_task(){
    
    calculate_next_index();
    image_source = cv::imread(current_file_path_);
    if(image_source.empty()){
      CLOG_WARN("imc-image-sequence : task : error loading image [" 
      << current_file_path_ << "] ignoring iteration")
    }else{
      CLOG_DEBUG(3,"imc-image-sequence : task : set image [" 
      << current_file_path_ << "]")
    }
  }




//- misc begin;
  


//- misc end;

  CYT3MACRO_enum_class_definitions(
    ImageContentImgSequenceMode
    , 
      , SEQUENCE_NORMAL
      , RANDOM
      , SEQUENCE_INVERSE
      , STATIC
  )



  CYT3MACRO_model_class_definitions(
    ImageContentImgSequenceParamsImage
    , 
    , ( )
    , ( )
    , name      , std::string       , ""
    , path      , std::string       , ""
    , interval  , int               , -1
  )

  CYT3MACRO_model_class_set_stack_definitions(
    ImageContentImgSequenceParamsImage,100)

  CYT3MACRO_model_class_definitions(
    ImageContentImgSequenceParams
    , 
    , ( )
    , ( )
    , base_path      , std::string                              , ""
    , interval       , int                                      , 5000
    , sequence       , ImageContentImgSequenceParamsImageStack  , 
    , current_index  , int                                      , 0
    , mode           , ec::ImageContentImgSequenceMode          , ec::ImageContentImgSequenceMode::SEQUENCE
  )


  CYT3MACRO_model_class_serializable_json_definitions(
    ImageContentImgSequenceParamsImage
    , 
    , 
    , ( )
    , ( )
    , name            , "name"            , 
    , path            , "path"            , 
    , interval        , "interval"        ,
  )
  CYT3MACRO_model_class_set_stack_serializable_json_definitions(
    ImageContentImgSequenceParamsImage)

  CYT3MACRO_model_class_serializable_json_definitions(
    ImageContentImgSequenceParams
    , 
    , 
    , ( sequence , "sequence" , ImageContentImgSequenceParamsImageStack)
    , ( )
    , base_path       , "base_path"    , 
    , interval         , "interval"    ,
  )


}
}
}
