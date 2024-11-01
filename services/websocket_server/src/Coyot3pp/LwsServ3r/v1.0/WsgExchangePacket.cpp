#include <Coyot3pp/LwsServ3r/v1.0/WsgExchangePacket.hpp>



namespace coyot3{
namespace services{
namespace websocket{


const char* WsgExchangePacket::JsField::type = "type";
const char* WsgExchangePacket::JsField::timestamp = "timestamp";
const char* WsgExchangePacket::JsField::payload = "payload";

WsgExchangePacket::WsgExchangePacket()
:type("default")
,payload("")
,_isValid(true)
{
  timestamp = coyot3::tools::getCurrentTimestamp();
}
WsgExchangePacket::WsgExchangePacket(const uint8_t* d,size_t s)
{
  fromJson(d,s);
}
WsgExchangePacket::WsgExchangePacket(const Json::Value& s)
{
  fromJson(s);
}
WsgExchangePacket::WsgExchangePacket(const std::string& s)
{
  fromJson(s);
}
WsgExchangePacket::~WsgExchangePacket(){

}
bool WsgExchangePacket::isValid()
{
  return _isValid;
}
Json::Value WsgExchangePacket::toJson() const {
  Json::Value json;
  json[JsField::type] = type;
  json[JsField::timestamp] = static_cast<Json::LargestInt>(timestamp);
  json[JsField::payload] = payload;
  return json;
}

bool WsgExchangePacket::fromJson(const uint8_t* d,size_t s)
{
  Json::Value json;
  Json::Reader r;
  
  if(!r.parse((const char*)d,(const char*)(d+s),json))
  {
    return (_isValid = false);
  }
  return fromJson(json);
}
bool WsgExchangePacket::fromJson(const std::string& js)
{
  Json::Value json;
  Json::Reader r;
  if(!r.parse(js,json))
  {
    return (_isValid = false);
  }
  return fromJson(json);
}
bool WsgExchangePacket::fromJson(const Json::Value& s){
  if( !s.isMember(JsField::type)
    ||!s.isMember(JsField::timestamp)
    ||!s.isMember(JsField::payload))
  {
    return false;
  }
  try{
    type = s[JsField::type].asString();
    timestamp = s[JsField::timestamp].asLargestInt();
    payload = s[JsField::payload];
  }catch(...)
  {
    return false;
  }
  return true;
}


}
}
}