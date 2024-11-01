#pragma once



#include "RtspServerConfigObject.hpp"
#include "RtspServerModelsCommon.hpp"
#include <Coyot3pp/Cor3/ModuleBase.hpp>


#include <Coyot3pp/Imag3/ImageContent/ImageContent.hpp>





namespace coyot3{
namespace av{
namespace rtsp{

  

  CYT3MACRO_model_class_declarations(
    RtspH264PublisherParams
    ,
    , ( )
    , ( )
      , name                        , std::string         , ""
      , id                          , int                 , 0
      
      , pipeline                    , std::string         , ""
      , path                        , std::string         , ""

      , text_contents_show          , bool                , false
      , text_size_scale             , double              , 1.0
      , width                       , int                 , 300
      , height                      , int                 , 200
      
      , max_clients                 , int                 , 0


      , compress_ratio              , double              , 1.0
      , cv_color_conversion         , int                 , -1
      , quality                     , int                 , 1

      , production_interval         , int                 , 100

  ) 

  
  CYT3MACRO_model_class_declarations(
    RtspClientInformation
    ,
    , ( int64_t client_connection_time_total())
    , ( )
      , host              , std::string     , ""
      , port              , uint32_t        , 0
      , uri               , std::string     , ""
      , abs_path          , std::string     , ""
      , query             , std::string     , ""
      , key               , std::string     , ""
      , connection_ts     , int64_t         , 0
      , disconnection_ts  , int64_t         , 0
      
      , gst_client    , GstRTSPClient*      , nullptr
      , gst_context   , GstRTSPContext*     , nullptr
  )

  CYT3MACRO_model_class_set_mapped_declarations(RtspClientInformation,
                                                gst_client)


  class RtspH264StreamPublisher
  : public coyot3::mod::ModuleBase{
    

    public:
      typedef coyot3::av::image::ImageContent ImageContent;
      typedef std::map<const GstRTSPClient*, RtspClientInformation>  ClientsMap;



      struct Handlers{
        GstRTSPServer*  owner;
        GstAppSrc*      gst_application;
        GstAppSrc*      gst_application_alt;
        

        GstRTSPMountPoints*   mounts;
        GstRTSPMediaFactory*  factory;

        gulong                media_connection;
        GstClockTime          gst_timestamp;

        std::string           gst_type;
        std::string           gst_format;

        coyot3::tools::WorkerThread* stream_thread;

        


          Handlers();
          ~Handlers();

      };


                   RtspH264StreamPublisher(const std::string& name = std::string());
      virtual     ~RtspH264StreamPublisher();

      bool        set_configuration(const RtspStreamConfig& config);
      bool        set_owner(GstRTSPServer* owner);







      bool                image_source_set(ImageContent* src);
      bool                image_source_set(ImageContent& src);
      ImageContent*       image_source_detach();


      bool                  client_connects(const RtspClientInformation& cli);
      bool                  client_disconnects(const GstRTSPClient* ptr);
      bool                  client_get(const GstRTSPClient* ptr,
                                      RtspClientInformation& cli);
      int                   clients_get_num();
      int                   clients_get_max();

      //
      int                 id() const;
      int                 id(int i);
      
      bool                oldest_client_obtain(RtspClientInformation& cli);
      

    protected:

      RtspStreamConfig         config_;
      Handlers                    handlers;
      RtspH264PublisherParams       params_;
      
      ClientsMap                  clients;

      ImageContent*               source_image;
      cv::Mat                     stream_image;

      bool        task_init();
      bool        task_start();


      bool source_init_();
      bool source_start_();
      bool source_stop_();

      void stream_image_publish();


      bool start_rtsp_server_add_url_();
     

      
      bool init_stream_thread_create_();
      bool start_stream_thread_launch_();
      bool stop_stream_thread_();
      bool end_stream_thread_kill_();

      bool stream_processing_activate_();
      bool stream_processing_deactivate_();



      


      bool gst_buffer_publisher();

      void factory_media_configure(
            GstRTSPMediaFactory*      factory
            ,GstRTSPMedia*            media);



    private:



    public: //statics

      static void static_factory_media_configure(
              GstRTSPMediaFactory*      factory
              ,GstRTSPMedia*            media
              ,RtspH264StreamPublisher* publisher);
  };



}
}
}