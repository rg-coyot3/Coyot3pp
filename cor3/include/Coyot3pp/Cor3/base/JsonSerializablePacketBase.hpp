

#pragma once


#include <string>
#include <ostream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include "cyt_swiss_knife_tools.hpp"

// #include <rapidjson/document.h>
// #include <rapidjson/writer.h>
// #include <rapidjson/stringbuffer.h>

namespace coyot3{
namespace tools{

typedef std::basic_string<uint8_t> ByteStream;

class JsonSerializablePacketBase {
  public:
                              JsonSerializablePacketBase();
  virtual                     ~JsonSerializablePacketBase();

  virtual bool                                         from_json_stream(const uint8_t* payload,size_t payload_size);
  
  bool    json_from_file(const std::string& file_path);
  bool    json_to_file(const std::string& file_path);
  bool    from_json_string(const std::string& source);
  //bool    from_rjson_string(const std::string& source);
  
  virtual bool                                         from_json(const Json::Value& source);
  //virtual bool                                         from_json(const rapidjson::Value& doc);
  virtual std::string                                  to_json_string() const;
  virtual ByteStream                                   to_json_stream() const;
  virtual Json::Value                                  to_json() const;
  
  //virtual rapidjson::Writer<rapidjson::StringBuffer>   to_rjson() const;

  bool                      ok() const;
  std::string               deserialization_src() const;
  protected:

    bool parsed_valid_;
    std::string deserialization_source_;

};

}//eons tools
}//eons coyot3
