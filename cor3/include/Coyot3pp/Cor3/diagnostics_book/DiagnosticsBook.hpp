#pragma once

#include "../Coyot3.hpp"
#include "../JsonSerializablePacketBase.hpp"

namespace coyot3{
namespace tools{

class DiagnosticsBookItem{
  public:
  
    typedef std::map<std::string,std::string> KeyValueMap;
    typedef std::map<std::string,DiagnosticsBookItem> DdsSiblingsMap;
    enum class DiagnosticLevel{
      OK = 1,
      WARN = 2,
      ERROR = 3,
      DATA_TIMEOUT = 4,
      UNAVAILABLE = 0,
      UNSET = -1,
      ERROR_DATA_SOURCE = -2,
      DESERIALIZATION_ERROR = -3

    };
    static const char* DiagnosticLevelToString(DiagnosticLevel l);
    DiagnosticLevel DiagnosticLevelFromString(const std::string& i);

    DiagnosticsBookItem();
    DiagnosticsBookItem(const std::string& t);
    DiagnosticsBookItem(const DiagnosticsBookItem& o);
    virtual ~DiagnosticsBookItem();

    

    int64_t         id() const;
    int64_t         id(int64_t i);
    
    DiagnosticLevel level() const;
    DiagnosticLevel level(DiagnosticLevel l);

    std::string name() const;
    std::string name(const std::string& n);

    std::string tag() const;
    std::string tag(const std::string& t);

    std::string message() const;
    std::string message(const std::string& m);

    std::string hardid() const;
    std::string hardid(const std::string& h);

    std::vector<std::string> valuesKeys();
    std::string value(const std::string& k) const;
    std::string value(const std::string& k,const std::string& v);

    int64_t     update_ts() const;
    int64_t     update_now();

    DiagnosticsBookItem& operator= (const DiagnosticsBookItem& o);
    bool                  operator==(const DiagnosticsBookItem& o);

    bool                  child_get(const std::string& n, DdsSiblingsMap::iterator& i);
    bool                  child_set(const DiagnosticsBookItem& d);


    bool search(int64_t itemId,DiagnosticsBookItem& dest) const;
    bool search(const std::string& itemName,DiagnosticsBookItem& dest) const;
    bool searchTag(const std::string& itemTag,DiagnosticsBookItem& dest) const;

    void copy_standalone_item(const DiagnosticsBookItem& dest);


  protected:
    int64_t                           id_;
    DiagnosticLevel                   level_;
    std::string                       name_;
    std::string                       tag_;
    std::string                       message_;
    std::string                       hardid_;
    KeyValueMap                       values_;
    int64_t                           update_ts_;
    DdsSiblingsMap                    children_;

  private:




};

class DiagnosticsBook
: public DiagnosticsBookItem{
  public:
    DiagnosticsBook();
    DiagnosticsBook(const DiagnosticsBookItem& o);
    virtual ~DiagnosticsBook();



  protected:


  private:



};

//json de-serialization

  class DiagnosticsBookItemJsIO 
: public DiagnosticsBookItem
, public coyot3::tools::JsonSerializablePacketBase
{
  public:

    struct JsField{
      static const char* id;
      static const char* level;
      static const char* level_desc;
      static const char* name;
      static const char* tag;
      static const char* message;
      static const char* hardware_id;
      static const char* update_ts;
      static const char* values;
      static const char* values_key;
      static const char* values_value;
      static const char* components;
    };

    
    DiagnosticsBookItemJsIO();
    DiagnosticsBookItemJsIO(const DiagnosticsBookItem& s);
    virtual ~DiagnosticsBookItemJsIO();

    virtual bool from_json(const Json::Value& source) override;
            
    virtual Json::Value to_json() const override;
            Json::Value to_json_with_MRM() const;

        
  protected:

  private:

};




}
}