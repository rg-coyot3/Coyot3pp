

#pragma once


#include <string>
#include <ostream>
#include <sstream>
#include <jsoncpp/json/json.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

namespace coyot3{
namespace tools{

typedef std::basic_string<uint8_t> ByteStream;

class JsonSerializablePacketBase {
  public:
                              JsonSerializablePacketBase();
  virtual                     ~JsonSerializablePacketBase();

  virtual bool                                         from_stream(const uint8_t* payload,size_t payload_size);
  
  
  bool    from_json_string(const std::string& source);
  bool    from_rjson_string(const std::string& source);
  
  virtual bool                                         from_json(const Json::Value& source);
  virtual bool                                         from_json(const rapidjson::Value& doc);
  virtual std::string                                  to_string() const;
  virtual ByteStream                                   to_stream() const;
  virtual Json::Value                                  to_json() const;
  virtual rapidjson::Writer<rapidjson::StringBuffer>   to_rjson() const;

  bool                      ok() const;
  std::string               deserialization_src() const;
  protected:

    bool parsed_valid_;
    std::string deserialization_source_;

};

}//eons tools
}//eons coyot3
