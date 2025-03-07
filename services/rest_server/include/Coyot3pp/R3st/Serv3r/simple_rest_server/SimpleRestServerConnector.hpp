#pragma once

#include "SimpleRestServerConnectorModels.hpp"
#include <crow.h>



namespace coyot3::communication::rest{



  class SimpleRestApiServer{
    public:
      SimpleRestApiServer(const std::string& ownerName);
      virtual ~SimpleRestApiServer();
      
      bool set_config(const RestServerConnectorConfig& conf);

      bool register_post_route(const std::string& route, RestPostApiCallback cb);
      bool register_post_route(const std::string& route, RestJsonPostApiCallback cb);
      bool register_rest_route();//to-do


      bool Init();
      bool Start();
      bool Stop();
      bool End();

    protected:
      std::string owner;
      RestServerConnectorConfig config;
      crow::SimpleApp* appPtr;

      RestPostApiCallbackInfoMappedSet        raw_callbacks;
      RestPostJsonApiCallbackInfoMappedSet    json_callbacks;
      ApiMethodClientTraceMappedSetJsIO       traces;

      std::thread*                            app_th_;


      crow::response  crow_callback_post_request_raw_(const crow::request& req);
      int  crow_callback_post_request_json_(const std::string& method,
                                            const std::string& req, 
                                            std::string& res);



      void trace_activity(const std::string& cli,
                          const std::string& met, 
                          int res, 
                          const std::string& i, 
                          const std::string& o);
      

    private:



  };


}