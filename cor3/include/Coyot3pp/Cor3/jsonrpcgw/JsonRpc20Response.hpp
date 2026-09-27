#pragma once

#include "../Coyot3.hpp"

namespace coyot3::tools{

  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    JsonRpc20ResponseError
    ,
    , ( 
      bool is_valid() const,
      bool is_error() const,
      bool is_result() const,
      static constexpr const int  ERR_PARSE            = -32700,
      static constexpr const int  ERR_INVALID_REQUEST  = -32600,
      static constexpr const int  ERR_METHOD_NOT_FOUND = -32601,
      static constexpr const int  ERR_INTERNAL_ERROR   = -32603,
    )
    , ( )
      , id              , Json::Value , Json::nullValue
      , error_code      , int64_t     , 
      , error_message   , std::string , 
      , error_data      , Json::Value , Json::nullValue
  )

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
      JsonRpc20ResponseError
      , 
      , ( activePrePostJsonization )
      , ( )
        , 
    )




}