#pragma once



#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/background_segm.hpp> /// for motion detector

#include <mutex>

#include <Coyot3pp/Cor3/Coyot3.hpp>

namespace coyot3{
namespace wrappers{
namespace av{

CYT3MACRO_enum_class_declarations(
  MetadataBandFormat
  ,
    , HORIZONTAL = 0
    , VERTICAL = 1
)

CYT3MACRO_enum_class_declarations(
  MetadataBandMode
  ,
    , SIN_AUTOGEN
    , SIN_AUTOGEN_THREADED
    , EXTERNAL_VALUE
)

CYT3MACRO_enum_class_declarations(
  MetadataBandShiftDirection
  ,
    , LEFT_TO_RIGHT
    , RIGHT_TO_LEFT
    , TOP_TO_DOWN
    , DOWN_TO_TOP
    , STATIC
)


class VideoStreamMetadataBand{
  public:
    enum class Format{
      HORIZONTAL = 0,
      VERTICAL = 1
    };
    static const char* FormatToString(Format f);

    enum class Mode{
      SIN_AUTOGEN = 0,
      SIN_AUTOGEN_THREADED = 1,
      EXTERNAL_VALUE = 2
    };
    
    static const char* ModeToString(Mode m);

    enum class ShiftDirection{
      LEFT_TO_RIGHT = 0,
      RIGHT_TO_LEFT = 1,
      TOP_TO_DOWN = 2,
      DOWN_TO_TOP = 3,
      STATIC = 4
    };
    static const char* ShiftDirectionToString(ShiftDirection s);

    VideoStreamMetadataBand();
    VideoStreamMetadataBand(int h,int w);
    virtual ~VideoStreamMetadataBand();

    double value();
    double value(double value);

    /**
     * in pixels per 10-milliseconds => if == 1 each 10 milliseconds will 
     *  translate to 1 pixel shift
     */
    double updateSpeed();
    double updateSpeed(double s);

    ShiftDirection shiftDirection();
    ShiftDirection shiftDirection(ShiftDirection s);

    cv::Mat& getImageRef();
    cv::Mat  getImageCopy();

    std::mutex& getMutexRef();

    bool     setDimensions(int w,int h);

    bool autogenThreadStart();
    bool autogenThreadStop();

    bool refreshImage();
    bool addMark(const std::string& m,int markSize = 4);

    bool autoMark(bool activation,int64_t interval,const std::string& mark = std::string(),int markSize = 0);
  protected:
    void _reset_values();
    bool generate_background_black_();
    bool product_regen_();
    /**
     * 0 <= q <= 1
     */
    bool add_colored_line_(double q,int r= 48,int g= 58,int b= 107);
    bool shift_image_(int desp);
    bool add_mark_point_();
    bool add_mark_line_(int percent);
    bool add_mark_(const std::string& m,int markSize = 4);


    double value_;
    double value_sec_;
    double speed_;

    int    width_;
    int    height_;
    Format format_;
    Mode   mode_;
    ShiftDirection shift_direction_;


    cv::Mat image_product_;
    cv::Mat background_base_;
    int64_t last_update_ts_;
    
    int64_t threaded_update_period_;
    coyot3::tools::ControlThread* cth_;


    bool        tmp_auto_mark_active_;
    int64_t     tmp_auto_mark_last_mark_ts_;
    int64_t     tmp_auto_mark_interval_;
    std::string tmp_auto_mark_marker_;
    int         tmp_auto_mark_marker_size_;



    std::mutex mtx_;

  private:
    void _initialize_values(); //only at constructor
};

}
}
}

std::ostream& operator<<(std::ostream& o,coyot3::wrappers::av::VideoStreamMetadataBand::Format f);
std::ostream& operator<<(std::ostream& o,coyot3::wrappers::av::VideoStreamMetadataBand::Mode f);
std::ostream& operator<<(std::ostream& o,coyot3::wrappers::av::VideoStreamMetadataBand::ShiftDirection f);
