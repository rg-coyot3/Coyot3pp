#pragma once

#include "../Coyot3.hpp"

namespace coyot3::tools{

  struct jsonrpc_id_type{
    enum class idtype{
      INVALID,VOID,NULLT,INT,UINT,FLOAT,STRING
    };
    jsonrpc_id_type();
    jsonrpc_id_type(int64_t i);
    jsonrpc_id_type(uint64_t i);
    jsonrpc_id_type(double i);
    jsonrpc_id_type(const std::string& i);
    jsonrpc_id_type(const Json::Value& i);
    jsonrpc_id_type(const jsonrpc_id_type& o);

    jsonrpc_id_type& operator=(const jsonrpc_id_type& o);
    jsonrpc_id_type& operator=(int64_t i);
    jsonrpc_id_type& operator=(uint64_t i);
    jsonrpc_id_type& operator=(double i);
    jsonrpc_id_type& operator=(const std::string& i);
    jsonrpc_id_type& operator=(const Json::Value& i);
    
    bool             operator==(const jsonrpc_id_type& o) const;
    bool             operator==(int64_t i) const;
    bool             operator==(uint64_t i) const;
    bool             operator==(double i) const;
    bool             operator==(const std::string& i) const;
    bool             operator==(const Json::Value& i) const;

    bool             is_null() const;
    bool             is_set() const;
    bool             is_valid() const;
    bool             clear();
    protected:
      idtype  type_;
      bool    valid_;
      int64_t id_int_;
      uint64_t id_uint_;
      double   id_double_;
      std::string id_string_;
  };

  Json::Value& operator<<(Json::Value& d, const jsonrpc_id_type& s);


  class JsonRpc20Request{
    public:
      JsonRpc20Request();
      JsonRpc20Request(const JsonRpc20Request& o);
      virtual ~JsonRpc20Request();

      JsonRpc20Request& operator=(const JsonRpc20Request& o);



    protected:
      std::string   method_;

    private:


  };


  COYOT3PP_MODEL_CLASS_DECLARATIONS(
    JsonRpc20Request
    ,
    , ( 
      Json::Value& id(int64_t i),
      Json::Value& id(const std::string& i),
      Json::Value  param(const std::string& properyName)
    )
    , (

    )
    , method        , std::string     , ""
    , params        , Json::Value     , Json::nullValue
    , id            , Json::Value     , Json::nullValue
  )

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DECLARATIONS(
      JsonRpc20Request
      ,
      , activePrePostJsonization
      , ( )
      , ( )
        , method      , "method"      , 
        , params      , "params"      , 
        
    )

  

  Json::Value& JsonRpc20Request::id(int64_t i){
    id_ = static_cast<Json::LargestInt>(i);
    return id_;
  }
  Json::Value& JsonRpc20Request::id(const std::string& i){
    id_ = i;
    return id_;
  }
  Json::Value JsonRpc20Request::param(const std::string& propertyName){
    if(params_.isMember(propertyName) == false)return Json::nullValue;
    return params_[propertyName];
  }

  void JsonRpc20RequestJsIO::to_json_post_process(Json::Value& js) const{
    js["jsonrpc"] = "2.0";
    if(params_["id"].type() == Json::nullValue)return;
    js["id"] = params_["id"];
  }
  bool JsonRpc20RequestJsIO::from_json_pre_process(const Json::Value& js){
    if(js.isMember("id")){
      id_=js["id"];
      Json::ValueType idtype = id_.type();
      if((idtype != Json::intValue) && (idtype != Json::stringValue)) return false;
    }
    if(js.isMember("jsonrpc") == false)return false;
    if(js["jsonrpc"].type() != Json::stringValue) return false;
    if(js["jsonrpc"].asString().compare("2.0"))return false;
  }

}