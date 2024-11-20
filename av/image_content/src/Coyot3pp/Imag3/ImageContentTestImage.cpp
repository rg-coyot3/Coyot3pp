#include <Coyot3pp/Imag3/ImageContent/ImageContentTestImage.hpp>

namespace coyot3{
namespace av{
namespace image{

    CYT3MACRO_model_class_definitions(
    IcTestParams
    , 
    , ( )
    , ( )
    , width       , int       , 400
    , height      , int       , 400
    , red         , int       , 0
    , green       , int       , 0
    , blue        , int       , 0
    , interval    , int       , 100
    , last_it_ts  , int64_t   , 0
    , offset      , double    , 0
  )

  ImageContentTestImage::ImageContentTestImage()
  :test_image_params()
  ,wth(nullptr){
    class_name("imc-testimg");
    add_task_init(std::bind(&ImageContentTestImage::task_init,this),true);
    add_task_start(std::bind(&ImageContentTestImage::task_start,this));
    add_task_stop(std::bind(&ImageContentTestImage::task_stop,this));
  }

  ImageContentTestImage::~ImageContentTestImage(){

  }


  bool ImageContentTestImage::task_init(){
    
    params_.width(test_image_params.width());
    params_.height(test_image_params.height());
    params_.name("Test Image");
    img_source_initialize(params_.width(),params_.height());
    return true;

  }
  bool ImageContentTestImage::task_start(){
    log_info("start : begin");
    if(wth)return true;
    wth = new coyot3::tools::WorkerThread(
      std::bind(&ImageContentTestImage::task, this)
      ,"test-img"
    );
    wth->start(test_image_params.interval());
    params_.name("test-image");
    test_image_params.last_it_ts(coyot3::tools::get_current_timestamp());
    log_info(o() << "start : dimensions=" 
      << params_.width() << "," << params_.height() << "done");
    return true;
  }
  bool ImageContentTestImage::task_stop(){
    if(!wth)return true;
    wth->stop();
    delete wth;
    wth = nullptr;

    return true;
  }


  void ImageContentTestImage::task(){
    int64_t now = coyot3::tools::get_current_timestamp();
    test_image_params.offset(test_image_params.offset() + (now - test_image_params.last_it_ts()));
    test_image_params.last_it_ts(now);
    (test_image_params.offset()>5000?test_image_params.offset(test_image_params.offset() - 5000):test_image_params.offset());
    test_image_params.red()   = static_cast<int>(255.0 * (sin(M_2_PI * ( test_image_params.offset() / 5000.0)))/ 2.0);
    test_image_params.green() = static_cast<int>(255.0 * (sin(M_PI * (test_image_params.offset() / 5000.0)))/ 2.0);
    test_image_params.blue()  = static_cast<int>(255.0 * (sin(M_2_PI * ( 4 * test_image_params.offset() / 5000.0)))/ 2.0);
    log_debug(5,o() << "vsi-test : task : offset(" << test_image_params.offset() 
      << ") sin(" << ((1.0 + sin(M_2_PI * ( test_image_params.offset() / 5000.0)))/ 2.0) << ")"
        "(r:" << test_image_params.red() 
      << ",g:" << test_image_params.green() 
      << ",b:" << test_image_params.blue() 
      <<  ")");
    image_source.setTo(cv::Scalar(test_image_params.red(),test_image_params.green(),test_image_params.blue()));
    if(params_.show_preview()){
      if(preview_window_created_ == false){
        cv::imshow(params_.name() + "testsource",image_source);
        preview_window_created_ = true;  
      }
      cv::resizeWindow(params_.name() + "testsource", test_image_params.width(),test_image_params.height());
      cv::imshow(params_.name() + "testsource",image_source);
      cv::waitKey(1);
    }
  }


}
}
}