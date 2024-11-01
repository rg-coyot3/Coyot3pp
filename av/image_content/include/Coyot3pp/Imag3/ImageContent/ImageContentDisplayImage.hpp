#pragma once 

#include "ImageContent.hpp"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdint>
#include <cstring>
#include <vector>

namespace coyot3{
namespace av{
namespace image{

  class ImageContentDisplayImage 
  : public ImageContent{
    public : 
      ImageContentDisplayImage(const std::string& name = std::string());
      ~ImageContentDisplayImage();

    bool prepare_output() override;

    protected:

      bool task_start();
      bool task_stop();

      bool get_image_from_display();
      
      std::vector<uint8_t> pixels_;
      int                  bits_per_pixel_;
      Display*             display_;
      Window               window_root_;
      XWindowAttributes    window_attributes_;

  };

}
}
}