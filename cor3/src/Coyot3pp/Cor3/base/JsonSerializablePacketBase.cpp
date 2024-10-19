/**
* 
* 
*   Dev/Maintain : Ricardo GONZALEZ ALMEIDA
* 
* 
*/
#include <Coyot3pp/Cor3/JsonSerializablePacketBase.hpp>

namespace coyot3{
namespace tools{

JsonSerializablePacketBase::JsonSerializablePacketBase()
: parsed_valid_(false)
, deserialization_source_()
{

}

JsonSerializablePacketBase::~JsonSerializablePacketBase()
{
    
}
std::string JsonSerializablePacketBase::deserialization_src() const{
  return deserialization_source_;
}

bool JsonSerializablePacketBase::from_stream(const uint8_t* payload,size_t payload_size)
{
    std::string s;
    for(size_t n = 0;n<payload_size;++n)
    {
      s.push_back((char)(payload[n]));
    }
    return (parsed_valid_ = from_rjson_string(s));
}

bool JsonSerializablePacketBase::from_rjson_string(const std::string& source)
{
  rapidjson::Document doc;
  if(source.size() == 0)return false;
  doc.Parse(source.c_str());
  
  return (parsed_valid_ = from_json(doc));

}

bool JsonSerializablePacketBase::from_json_string(const std::string& source){
  if(source.size()== 0)return false;
  Json::Reader reader;
  Json::Value  json;
  deserialization_source_ = source;
  if(!reader.parse(source,json))
  {
    parsed_valid_ = false;
    return false;
  }
  return (parsed_valid_ = from_json(json));
}
bool JsonSerializablePacketBase::from_json(const Json::Value& source)
{

  return (parsed_valid_ = false);
}

bool JsonSerializablePacketBase::from_json(const rapidjson::Value& source){
  return (parsed_valid_ = false);
}
ByteStream JsonSerializablePacketBase::to_stream() const
{
    ByteStream bs;
    std::string buffer = to_string();
    for(char c : buffer)
    {
      bs.push_back((uint8_t)c);
    }
    return bs;
}
std::string JsonSerializablePacketBase::to_string() const
{
    std::stringstream ss;
    ss << to_json();
    return ss.str();
    
}
Json::Value  JsonSerializablePacketBase::to_json() const{
  return Json::Value();
}

bool JsonSerializablePacketBase::ok() const{
  return parsed_valid_;
}
rapidjson::Writer<rapidjson::StringBuffer> JsonSerializablePacketBase::to_rjson() const{
  return rapidjson::Writer<rapidjson::StringBuffer>();
}

}
}