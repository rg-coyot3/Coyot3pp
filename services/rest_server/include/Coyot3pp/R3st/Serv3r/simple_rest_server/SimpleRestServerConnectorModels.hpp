#pragma once
#include <Coyot3pp/Cor3/Coyot3.hpp>


namespace coyot3::communication::rest{



  typedef std::function<int(const std::string&, const std::string& , std::string&)> RestPostApiCallback;
  typedef std::function<int(const std::string&, const Json::Value& , Json::Value&)> RestJsonPostApiCallback;

  //config
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    RestServerConnectorConfig
    ,
    , ( )
    , ( )
      , port          , int            , 
      , ipbind        , std::string    , 
  )
    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
      RestServerConnectorConfig
      , 
      , 
      , ( )
      , ( )
        , port      , "port"    , 
        , ipbind    , "ipbind"  , 
    )

  //callback infos
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    RestPostApiCallbackInfo
    , 
    , ( 
      RestPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(RestPostApiCallbackInfo,method)

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    RestPostJsonApiCallbackInfo
    , 
    , ( 
      RestJsonPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(RestPostJsonApiCallbackInfo, method)
    

  //stats
  COYOT3PP_MODEL_CLASS_DECLARATIONS(
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

    COYOT3PP_MODEL_CLASS_DECLARATIONS(
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
      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
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

      COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(ApiMethodTrace, 200)
      COYOT3PP_MODEL_CLASS_SET_STACK_SERIALIZABLE_JSON_DECLARATIONS(ApiMethodTrace)


      COYOT3PP_MODEL_CLASS_DECLARATIONS(
        ApiMethodTraceStat
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace) )
        , ( )
          , num_invokations       , int64_t           , 0
      )
        COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
          ApiMethodTraceStat
          , ApiMethodTrace
          ,
          , ( )
          , ( )
            , num_invokations     , "num_invokations"   , 
        )

      COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(ApiMethodTraceStat, method)
      COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DECLARATIONS(ApiMethodTraceStat, method)


      COYOT3PP_MODEL_CLASS_DECLARATIONS(
        ApiMethodClientTrace
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace))
        , ( )
          , client        , std::string                 ,  
          , stack         , ApiMethodTraceStack         , 
          , map           , ApiMethodTraceStatMappedSet , 
          , total_requests, int64_t                     , 
      )

      COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
        ApiMethodClientTrace
        , ApiMethodTrace
        , 
        , ( 
              stack       , "stack"     , ApiMethodTraceStack
            , map         , "map"       , ApiMethodTraceStatMappedSet
        )
        , ( )
          , client        , "client"          , 
          , total_requests, "total_requests"  , 
      )

      COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(ApiMethodClientTrace, client)
      COYOT3PP_MODEL_CLASS_SET_MAPPED_SERIALIZABLE_JSON_DECLARATIONS(ApiMethodClientTrace, client)
      
      
      
    



  
}