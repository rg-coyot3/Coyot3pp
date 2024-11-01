#pragma once



#include <Coyot3pp/Cor3/ModuleBase.hpp>


#include "RtspServerModelsCommon.hpp"
#include "RtspServerConfigObject.hpp"
#include "RtspH264StreamPublisher.hpp"



namespace coyot3{
namespace av{
namespace rtsp{
  

   CYT3MACRO_model_class_declarations(
    RtspServerParams
      , 
      , ( )
      , ( )
        , server_name                   , std::string             , ""
        , server_port                   , int                     , 8554
        , base_path                     , std::string             , "/tmp"
        , ssl_on                        , bool                    , false
        , file_ssl_ca                   , std::string             , ""
        , file_ssl_cert                 , std::string             , ""
        , file_ssl_key                  , std::string             , ""
        , auth_global_use               , bool                    , false
        , auth_user                     , std::string             , ""
        , auth_pass                     , std::string             , ""
        , auth_credentials_use          , bool                    , false
        , auth_client_credentials_file  , std::string             , ""
        , debug_level                   , int                     , 1
        , key_auth_use                  , bool                    , false
        , key_auth_string               , std::string             , "12345ABCDEF"
        , key_auth_autogen              , bool                    , false
        , max_clients                   , int                     , 0

        , ts_start                      , int64_t                     , 0
        , ts_end                        , int64_t                     , 0

        , current_key                   , std::string                 , ""
  );

  
  class RtspH264Server
  :public coyot3::mod::ModuleBase{
    public:

    typedef std::map<std::string, RtspH264StreamPublisher*> StreamsMap;

    struct ServerGstStruct{
      GMainLoop*                    gstreamer_main_loop;
      coyot3::tools::WorkerThread*  gstloop_thread;

      GstRTSPServer*                rtsp_server;
      guint                         rtsp_server_source_id;
      int                           server_port;

      ServerGstStruct();
      ~ServerGstStruct();
    };



    
      RtspH264Server(const std::string& name = std::string());
      virtual ~RtspH264Server();

      bool    set_configuration(const RtspServerConfig& config);




      
      void set_cb_on_client_connected(std::function<void(RtspClientInformation)> cb);
      void set_cb_on_client_disconnected(std::function<void(RtspClientInformation)> cb);

      bool                      stream_create(const RtspStreamConfig& config);
      RtspH264StreamPublisher*  stream_get_by_name(const std::string& name);
      RtspH264StreamPublisher*  stream_get_by_path(const std::string& name);

      RtspServerParams&   params();
      RtspServerParams    params() const;


    protected:
      
      RtspServerConfig               config_;
      RtspServerParams               params_;

      ServerGstStruct                handlers;
      
      StreamsMap                     streams;
      
      /**
       * @brief If there's any custom streamer to be attached, it must be 
       *        created with 'stream-create' and push it BEFORE the Init call.
       * 
       * @return true 
       * @return false 
       */
      bool task_init_();
      bool task_start_();
      bool task_pause_();
      bool task_stop_();
      bool task_end_();

      

      int                            connected_clients_get_total();
      GstRTSPClient*                 connected_clients_get_oldest();

      
      std::function<void(RtspClientInformation)> cb_on_client_connects;
      std::function<void(RtspClientInformation)> cb_on_client_disconnect;

      void on_client_connects(GstRTSPServer* server, GstRTSPClient* client);
      void on_client_requests_options(GstRTSPClient* client, 
                                      GstRTSPContext* context);
      void on_client_requests_teardown(GstRTSPClient* client, 
                                      GstRTSPContext* context);
      void on_session_cleanup(bool ignored);


      

      bool create_rtsp_server_();
      bool destroy_rtsp_server_();
      

      bool publishers_create_();
      bool publishers_attach_();
      bool publishers_init_();
      bool publishers_start_();
      bool publishers_pause_();
      bool publishers_stop_();
      bool publishers_end_(bool force);
      bool server_main_loop_start_();
      bool server_main_loop_stop_();

    private:
      std::string server_name;

    public: //statics

    static void gstcallback_on_client_connect(
      GstRTSPServer* server, 
      GstRTSPClient* client, 
      RtspH264Server* owner);
    

    static void gstcallback_session_cleanup(
      RtspH264Server* owner, 
      gboolean ignored
    );
    static void gstcallback_client_requests_options(
      GstRTSPClient* client, 
      GstRTSPContext* context, 
      RtspH264Server* owner);

    static void gstcallback_client_requests_teardown(
      GstRTSPClient* client,
      GstRTSPContext* context,
      RtspH264Server* owner);

    
    static const std::string pipeline_preset_ull;
    static const std::string pipeline_preset_tst;
    static const std::string pipeline_preset_dsk;

    // friend void RtspH264Server::gstcallback_session_cleanup(
    //   RtspH264Server* owner, 
    //   gboolean ignored
    // );
    // friend void RtspH264Server::gstcallback_client_requests_options(
    //   GstRTSPClient* client, 
    //   GstRTSPContext* context, 
    //   RtspH264Server* owner);

    // friend void RtspH264Server::gstcallback_client_requests_teardown(
    //   GstRTSPClient* client,
    //   GstRTSPContext* context,
    //   RtspH264Server* owner);

  };

}
}
}