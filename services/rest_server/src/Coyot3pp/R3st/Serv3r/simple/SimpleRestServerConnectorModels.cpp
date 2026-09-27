#include <Coyot3pp/R3st/Serv3r/simple_rest_server/SimpleRestServerConnectorModels.hpp>




namespace coyot3::communication::rest{
// config
  COYOT3PP_MODEL_CLASS_DEFINITIONS(
    RestServerConnectorConfig
    ,
    , ( )
    , ( )
      , port          , int                       , 
      , ipbind        , std::string               , 
  )
    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
      RestServerConnectorConfig
      , 
      , 
      , ( )
      , ( )
        , port      , "port"    , 
        , ipbind    , "ipbind"  , 
    )


  //callback infos
  COYOT3PP_MODEL_CLASS_DEFINITIONS_no_opsoverload(
    RestPostApiCallbackInfo
    , 
    , ( 
      RestPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(RestPostApiCallbackInfo,method)

    COYOT3PP_MODEL_CLASS_DEFINITIONS_no_opsoverload(
    RestPostJsonApiCallbackInfo
    , 
    , ( 
      RestJsonPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(RestPostJsonApiCallbackInfo, method)


  //
  COYOT3PP_MODEL_CLASS_DEFINITIONS(
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

    COYOT3PP_MODEL_CLASS_DEFINITIONS(
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
      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
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

      COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(ApiMethodTrace, 200)
      COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DEFINITIONS(ApiMethodTrace)


      COYOT3PP_MODEL_CLASS_DEFINITIONS(
        ApiMethodTraceStat
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace) )
        , ( )
          , num_invokations       , int64_t           , 
      )
        COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
          ApiMethodTraceStat
          , ApiMethodTrace
          ,
          , ( )
          , ( )
            , num_invokations     , "num_invokations"   , 
        )

      COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(ApiMethodTraceStat, method)
      COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DEFINITIONS(ApiMethodTraceStat, method)


      COYOT3PP_MODEL_CLASS_DEFINITIONS(
        ApiMethodClientTrace
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace))
        , ( )
          , client        , std::string                 ,  
          , stack         , ApiMethodTraceStack         , 
          , map           , ApiMethodTraceStatMappedSet , 
          , total_requests, int64_t                     , 
      )

      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
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

      COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(ApiMethodClientTrace, client)
      COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DEFINITIONS(ApiMethodClientTrace, client)
      
      
      
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