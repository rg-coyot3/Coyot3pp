#pragma once 

#include "ImageContent.hpp"

namespace coyot3{
namespace av{
namespace image{

  CYT3MACRO_model_class_declarations(
    ImageContentCvVideocapData
    ,
    , ( )
    , ( )
      , source              , std::string     , ""
      , input_min_interval  , int             , 500
  )

  class ImageContentCvVideocap 
  :public ImageContent{
    public:

    ImageContentCvVideocap(const std::string& name, const std::string& source);
    virtual ~ImageContentCvVideocap();

    



    std::string             source()const;
    std::string             source(const std::string& src);
    int64_t                 connection_retry_interval() const;
    int64_t                 connection_retry_interval(int64_t r);
    int64_t                 update_interval() const;
    int64_t                 update_interval(int64_t r);
    

    protected:

      bool task_start();
      bool task_stop();


      coyot3::tools::WorkerThread* wth;

      cv::VideoCapture*            cvcap;

      std::string                  source_;
      bool                         is_active_;
      int64_t                      connection_retry_interval_;
      int64_t                      update_interval_;

      void                         task();
  }; 

}
}
}