#pragma once
#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <jsoncpp/json/json.h>
#include <string>
namespace coyot3{
namespace services{
namespace websocket{



struct WsgExchangePacket{
      struct JsField {
        static const char* type;
        static const char* timestamp;
        static const char* payload;
      };
      WsgExchangePacket();
      WsgExchangePacket(const uint8_t* d,size_t s);
      WsgExchangePacket(const std::string& s);
      WsgExchangePacket(const Json::Value& s);

      template<typename T>
      WsgExchangePacket(const std::string& t = std::string("default"),T p = T())
      : type(t)
      , payload(p)
      , _isValid(true)
      {
        timestamp = coyot3::tools::getCurrentTimestamp();
      }
      
      ~WsgExchangePacket();

      bool isValid();

      std::string type;
      int64_t     timestamp;
      Json::Value payload;

      Json::Value toJson() const;
      bool fromJson(const uint8_t* d,size_t s);
      bool fromJson(const std::string& js);
      bool fromJson(const Json::Value& js);
    protected:
      bool _isValid;  
};


}
}
}