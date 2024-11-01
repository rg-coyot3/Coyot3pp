#pragma once

#include "ImageContent.hpp"

#include <opencv4/opencv2/objdetect.hpp>


namespace coyot3{
namespace av{
namespace image{



  class ImageContentQrCode 
  : public ImageContent{
    public : 
      ImageContentQrCode();
      ~ImageContentQrCode();

    bool prepare_output() override;

    protected:
      bool task_start();
      bool task_stop();
      bool get_image_from_display();


  };


}
}
}