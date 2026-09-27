#include <Coyot3pp/Cor3/jsonrpcgw/JsonRpc20Request.hpp>


namespace coyot3::tools{



  jsonrpc_id_type::jsonrpc_id_type()
  :type_(idtype::VOID)
  ,valid_(true)
  {

  }

  jsonrpc_id_type::jsonrpc_id_type(int64_t i)
  :type_(idtype::INT)
  ,valid_(true)
  ,id_int_(i)
  {
    
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(int64_t i){
    id_int_ = i;
    type_ = idtype::INT;
    return *this;
  }



  jsonrpc_id_type::jsonrpc_id_type(uint64_t i)
  :type_(idtype::UINT)
  ,valid_(true)
  ,id_uint_(i)
  {
    
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(uint64_t i){
    id_uint_ = i;
    type_ = idtype::UINT;
    return *this;
  }
  jsonrpc_id_type::jsonrpc_id_type(double i)
  :type_(idtype::FLOAT)
  ,valid_(true)
  ,id_double_(i)
  {
    
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(double i){
    id_double_ = i;
    type_ = idtype::FLOAT;
    return *this;
  }


  jsonrpc_id_type::jsonrpc_id_type(const std::string& i)
  :type_(idtype::STRING)
  ,valid_(true)
  ,id_string_(i)
  {
    
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(const std::string& i){
    id_string_= i;
    type_ = idtype::STRING;
    return *this;
  }

  jsonrpc_id_type::jsonrpc_id_type(const Json::Value& i)
  {
    *this = i;
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(const Json::Value& i){
    valid_ = true;
    switch (i.type())
    {
    case Json::nullValue:
      type_ = idtype::NULLT;
      break;
    case Json::intValue:
      if(i.isDouble()){
        type_ = idtype::FLOAT;
        id_double_ = i.asDouble();
      }else{
        type_ = idtype::INT;
        id_int_ = i.asLargestInt();
      }
      break;
    case Json::uintValue:
      type_ = idtype::UINT;
      id_uint_ = i.asLargestUInt();
      break;
    case Json::stringValue:
      type_ = idtype::STRING;
      id_string_ = i.asString();
      break;
    
    default:
      type_ = idtype::INVALID;
      valid_= false;
      break;
    }
    return *this;
  }
  jsonrpc_id_type::jsonrpc_id_type(const jsonrpc_id_type& o)
  :type_(o.type_)
  ,valid_(o.valid_)
  ,id_int_(o.id_int_)
  ,id_uint_(o.id_uint_)
  ,id_string_(o.id_string_){
    
  }
  jsonrpc_id_type& jsonrpc_id_type::operator=(const jsonrpc_id_type& o){
    type_=o.type_;
    valid_=o.valid_;
    id_int_ = o.id_int_;
    id_uint_ = o.id_uint_;
    id_string_ = o.id_string_;
    return *this;
  }

  bool jsonrpc_id_type::operator==(const jsonrpc_id_type& o) const{
    if((valid_ != o.valid_) || (type_ != o.type_))return false;
    switch (type_)
    {
    case idtype::INVALID:
    case idtype::VOID:
    case idtype::NULLT:
      return true;
      break;
    case idtype::INT:
      return id_int_ == o.id_int_;
      break;
    case idtype::UINT:
      return id_uint_ == o.id_uint_;
      break;
    case idtype::FLOAT:
      return id_double_ == o.id_double_;
      break;
    case idtype::STRING:
      return (id_string_.compare(o.id_string_) == 0);
    default:
      return false;
      break;
    }
    return false;
  }
  bool jsonrpc_id_type::is_null() const{
    return type_ == idtype::NULLT;
  }
  bool jsonrpc_id_type::is_set() const{
    return ((type_ != idtype::INVALID) && (type_ != idtype::VOID));
  }

  COYOT3PP_MODEL_CLASS_DEFINITIONS(
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

    COYOT3PP_MODEL_CLASS_SERIALIZABLE_JSON_DEFINITIONS(
      JsonRpc20Request
      ,
      , activePrePostJsonization
      , ( )
      , ( )
        , method      , "method"      , 
        , params      , "params"      , 
        
    )



}