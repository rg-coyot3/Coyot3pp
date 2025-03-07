#include <Coyot3pp/R3st/Serv3r/simple_rest_server/SimpleRestServerConnectorModels.hpp>




namespace coyot3::communication::rest{
// config
  CYT3MACRO_model_class_definitions(
    RestServerConnectorConfig
    ,
    , ( )
    , ( )
      , port          , int                       , 
      , ipbind        , std::string               , 
  )
    CYT3MACRO_model_class_serializable_json_definitions(
      RestServerConnectorConfig
      , 
      , 
      , ( )
      , ( )
        , port      , "port"    , 
        , ipbind    , "ipbind"  , 
    )


  //callback infos
  CYT3MACRO_model_class_definitions_no_opsoverload(
    RestPostApiCallbackInfo
    , 
    , ( 
      RestPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    CYT3MACRO_model_class_set_mapped_definitions(RestPostApiCallbackInfo,method)

    CYT3MACRO_model_class_definitions_no_opsoverload(
    RestPostJsonApiCallbackInfo
    , 
    , ( 
      RestJsonPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    CYT3MACRO_model_class_set_mapped_definitions(RestPostJsonApiCallbackInfo, method)


  //
  CYT3MACRO_model_class_definitions(
    ApiMethodInformation
    , 
    , ( )
    , ( )
      , method        , std::string       , 
      , ts_last_invoke, int64_t           , 0
      , last_req      , std::string       , 
      , last_res      , std::string       , 
      , num_success   , int64_t           , 0
      , num_err       , int64_t           , 0
  )

    CYT3MACRO_model_class_definitions(
      ApiMethodTrace
      , 
      , ( )
      , ( )
        , method        , std::string       , 
        , ts            , int64_t           , 
        , req           , std::string       , 
        , res           , std::string       , 
        , ret_code      , int               , 
    )
      CYT3MACRO_model_class_serializable_json_definitions(
        ApiMethodTrace
        ,
        , 
        , ( )
        , ( )
          , method        , "method"        ,
          , ts            , "ts"            ,
          , req           , "req"           ,
          , res           , "res"           ,
          , ret_code      , "ret_code"      ,
      )

      CYT3MACRO_model_class_set_stack_definitions(ApiMethodTrace, 200)
      CYT3MACRO_model_class_set_stack_serializable_json_definitions(ApiMethodTrace)


      CYT3MACRO_model_class_definitions(
        ApiMethodTraceStat
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace) )
        , ( )
          , num_invokations       , int64_t           , 
      )
        CYT3MACRO_model_class_serializable_json_definitions(
          ApiMethodTraceStat
          , ApiMethodTrace
          ,
          , ( )
          , ( )
            , num_invokations     , "num_invokations"   , 
        )

      CYT3MACRO_model_class_set_mapped_definitions(ApiMethodTraceStat, method)
      CYT3MACRO_model_class_set_mapped_serializable_json_definitions(ApiMethodTraceStat, method)


      CYT3MACRO_model_class_definitions(
        ApiMethodClientTrace
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace))
        , ( )
          , client        , std::string                 ,  
          , stack         , ApiMethodTraceStack         , 
          , map           , ApiMethodTraceStatMappedSet , 
          , total_requests, int64_t                     , 
      )

      CYT3MACRO_model_class_serializable_json_definitions(
        ApiMethodClientTrace
        , ApiMethodTrace
        , 
        , ( 
              stack       , "stack"     , ApiMethodTraceStack
            , map         , "map"       , ApiMethodTraceStatMappedSet
        )
        , ( )
          , client        , "client"    , 
      )

      CYT3MACRO_model_class_set_mapped_definitions(ApiMethodClientTrace, client)
      CYT3MACRO_model_class_set_mapped_serializable_json_definitions(ApiMethodClientTrace, client)
      
      
      
    //impl

    bool RestPostApiCallbackInfo::operator==(const RestPostApiCallbackInfo& o) const{
      return (
        (method() == o.method())
      );
    }
    bool RestPostJsonApiCallbackInfo::operator==(const RestPostJsonApiCallbackInfo& o) const{
      return (
        (method() == o.method())
      );
    }

    RestPostApiCallbackInfo& 
    RestPostApiCallbackInfo::operator=(const RestPostApiCallbackInfo& o){
      method(o.method());
      callback = o.callback;
      return *this;
    }
    RestPostJsonApiCallbackInfo& 
    RestPostJsonApiCallbackInfo::operator=(const RestPostJsonApiCallbackInfo& o){
      method(o.method());
      callback = o.callback;
      return *this;
    }




    bool ApiMethodTraceStat::update_stat(const ApiMethodTrace& trace){
      ApiMethodTrace::operator=(trace);
      num_invokations(num_invokations()+1);
      return true;
    }

    bool ApiMethodClientTrace::update_stat(const ApiMethodTrace& trace){
      ApiMethodTrace::operator=(trace);
      stack().push_back(trace);
      if(map().is_member(trace.method())== false){
        ApiMethodTraceStat stat;
        static_cast<ApiMethodTrace>(stat) = trace;
      }
      map().get(trace.method()).update_stat(trace);
      total_requests(total_requests()+1);
      return true;
    }


  
}