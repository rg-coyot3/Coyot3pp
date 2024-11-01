#pragma once

#include "ImageContent.hpp"

namespace coyot3{
namespace av{
namespace image{

  CYT3MACRO_model_class_declarations(
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

  class ImageContentTestImage
  :public ImageContent{

    public:

      ImageContentTestImage();
      virtual ~ImageContentTestImage();

      IcTestParams test_image_params;

    protected:
      bool task_init();
      bool task_start();
      bool task_stop();
      coyot3::tools::WorkerThread* wth;
      
      void task();
      



    private:



  };



}
}
}