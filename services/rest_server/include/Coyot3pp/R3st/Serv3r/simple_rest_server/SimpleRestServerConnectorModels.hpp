#pragma once
#include <Coyot3pp/Cor3/Coyot3.hpp>


namespace coyot3::communication::rest{



  typedef std::function<int(const std::string&, const std::string& , std::string&)> RestPostApiCallback;
  typedef std::function<int(const std::string&, const Json::Value& , Json::Value&)> RestJsonPostApiCallback;

  //config
  CYT3MACRO_model_class_declarations(
    RestServerConnectorConfig
    ,
    , ( )
    , ( )
      , port          , int            , 
      , ipbind        , std::string    , 
  )
    CYT3MACRO_model_class_serializable_json_declarations(
      RestServerConnectorConfig
      , 
      , 
      , ( )
      , ( )
        , port      , "port"    , 
        , ipbind    , "ipbind"  , 
    )

  //callback infos
  CYT3MACRO_model_class_declarations(
    RestPostApiCallbackInfo
    , 
    , ( 
      RestPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    CYT3MACRO_model_class_set_mapped_declarations(RestPostApiCallbackInfo,method)

  CYT3MACRO_model_class_declarations(
    RestPostJsonApiCallbackInfo
    , 
    , ( 
      RestJsonPostApiCallback callback 
    )
    , ( )
    , method        , std::string     , 
    
  )
    CYT3MACRO_model_class_set_mapped_declarations(RestPostJsonApiCallbackInfo, method)
    

  //stats
  CYT3MACRO_model_class_declarations(
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

    CYT3MACRO_model_class_declarations(
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
      CYT3MACRO_model_class_serializable_json_declarations(
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

      CYT3MACRO_model_class_set_stack_declarations(ApiMethodTrace, 200)
      CYT3MACRO_model_class_set_stack_serializable_json_declarations(ApiMethodTrace)


      CYT3MACRO_model_class_declarations(
        ApiMethodTraceStat
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace) )
        , ( )
          , num_invokations       , int64_t           , 0
      )
        CYT3MACRO_model_class_serializable_json_declarations(
          ApiMethodTraceStat
          , ApiMethodTrace
          ,
          , ( )
          , ( )
            , num_invokations     , "num_invokations"   , 
        )

      CYT3MACRO_model_class_set_mapped_declarations(ApiMethodTraceStat, method)
      CYT3MACRO_model_class_set_mapped_serializable_json_declarations(ApiMethodTraceStat, method)


      CYT3MACRO_model_class_declarations(
        ApiMethodClientTrace
        , ApiMethodTrace
        , ( bool update_stat(const ApiMethodTrace& trace))
        , ( )
          , client        , std::string                 ,  
          , stack         , ApiMethodTraceStack         , 
          , map           , ApiMethodTraceStatMappedSet , 
          , total_requests, int64_t                     , 
      )

      CYT3MACRO_model_class_serializable_json_declarations(
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

      CYT3MACRO_model_class_set_mapped_declarations(ApiMethodClientTrace, client)
      CYT3MACRO_model_class_set_mapped_serializable_json_declarations(ApiMethodClientTrace, client)
      
      
      
    



  
}