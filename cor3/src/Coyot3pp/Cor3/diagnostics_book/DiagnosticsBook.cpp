#include <Coyot3pp/Cor3/diagnostics_book/DiagnosticsBook.hpp>


namespace coyot3{
namespace tools{



const char* DiagnosticsBookItem::DiagnosticLevelToString(DiagnosticLevel l)
{
  switch(l)
  {
    case DiagnosticLevel::OK: return "OK";break;
    case DiagnosticLevel::WARN: return "WARN";break;
    case DiagnosticLevel::ERROR: return "ERROR";break;
    case DiagnosticLevel::DATA_TIMEOUT: return "DATA_TIMEOUT";break;
    case DiagnosticLevel::UNAVAILABLE: return "UNAVAILABLE";break;
    case DiagnosticLevel::UNSET: return "UNSET";break;
    case DiagnosticLevel::DESERIALIZATION_ERROR: return "DESERIALIZATION_ERROR";break;
    case DiagnosticLevel::ERROR_DATA_SOURCE: 
    default:
      return "ERROR_DATA_SOURCE";
      break;
  }
}

DiagnosticsBookItem::DiagnosticLevel DiagnosticsBookItem::DiagnosticLevelFromString(const std::string& i)
{
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::OK))))return DiagnosticLevel::OK;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::WARN))))return DiagnosticLevel::WARN;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::ERROR))))return DiagnosticLevel::ERROR;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::UNAVAILABLE))))return DiagnosticLevel::UNAVAILABLE;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::DATA_TIMEOUT))))return DiagnosticLevel::DATA_TIMEOUT;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::UNSET))))return DiagnosticLevel::UNSET;
  if(!(i.compare(DiagnosticLevelToString(DiagnosticLevel::DESERIALIZATION_ERROR))))return DiagnosticLevel::DESERIALIZATION_ERROR;
  return DiagnosticLevel::ERROR_DATA_SOURCE;
}

DiagnosticsBookItem::DiagnosticsBookItem()
:id_(0)
,level_(DiagnosticLevel::UNSET)
,name_() 
,tag_()
,message_()
,hardid_()
,values_()
,update_ts_(0)
,children_(){

}
DiagnosticsBookItem::DiagnosticsBookItem(const std::string& t)
:id_(0)
,level_(DiagnosticLevel::UNSET)
,name_() 
,tag_(t)
,message_()
,hardid_()
,values_()
,update_ts_(0)
,children_(){
  
}


DiagnosticsBookItem::~DiagnosticsBookItem(){
  children_.clear();
}

DiagnosticsBookItem::DiagnosticsBookItem(const DiagnosticsBookItem& o)
{
  *this = o;
}

DiagnosticsBookItem& DiagnosticsBookItem::operator=(const DiagnosticsBookItem& o){
  id_ = o.id_;
  level_ = o.level_;
  name_ = o.name_;
  tag_ = o.tag_;
  message_ = o.message_;
  hardid_ = o.hardid_;
  values_ = o.values_;
  update_ts_ = o.update_ts_;
  children_ = o.children_;
  return *this;
}
void DiagnosticsBookItem::copy_standalone_item(const DiagnosticsBookItem& dest)
{
  id(dest.id());
  level(dest.level());
  name(dest.name());
  tag(dest.tag());
  message(dest.message());
  hardid(dest.hardid());
  values_ = dest.values_;
  update_ts_ = dest.update_ts();
}

bool DiagnosticsBookItem::operator==(const DiagnosticsBookItem& o)
{
  return (
      (id_ == o.id_)
    ||(level_ == o.level_)
    ||(name_ == o.name_)
    ||(tag_ == o.tag_)
    ||(message_ == o.message_)
    ||(hardid_ == o.hardid_)
    ||(values_ == o.values_)
    ||(update_ts_ == o.update_ts_)
  );
}

int64_t DiagnosticsBookItem::id() const{
  return id_;
}
int64_t DiagnosticsBookItem::id(int64_t i){
  id_ = i;
  return id_;
}


DiagnosticsBookItem::DiagnosticLevel DiagnosticsBookItem::level() const
{
  return level_;
}
DiagnosticsBookItem::DiagnosticLevel DiagnosticsBookItem::level(DiagnosticLevel l)
{
  level_ = l;
  return level_;
}

std::string DiagnosticsBookItem::name() const
{
  return name_;
}
std::string DiagnosticsBookItem::name(const std::string& n)
{
  name_ = n;
  return name_;
}

std::string DiagnosticsBookItem::tag() const
{
  return tag_;
}
std::string DiagnosticsBookItem::tag(const std::string& t)
{
  tag_ = t;
  return tag_;
}


std::string DiagnosticsBookItem::message() const
{
  return message_;
}
std::string DiagnosticsBookItem::message(const std::string& m)
{
  message_ = m;
  return message_;
}

std::string DiagnosticsBookItem::hardid() const
{
  return hardid_;
}
std::string DiagnosticsBookItem::hardid(const std::string& h)
{
  hardid_ = h;
  return hardid_;
}

std::string DiagnosticsBookItem::value(const std::string& k) const
{
  KeyValueMap::const_iterator it = values_.find(k);
  if(it == values_.end())
  {
    return "";
  }
  return it->second;
}
std::string DiagnosticsBookItem::value(const std::string& k,const std::string& v)
{
  KeyValueMap::iterator it = values_.find(k);
  if(it == values_.end())
  {
    values_.insert(std::pair<std::string,std::string>(k,v));
  }else{
    it->second = v;
  }
  return v;
}

int64_t     DiagnosticsBookItem::update_ts() const
{
  return update_ts_;
}
int64_t     DiagnosticsBookItem::update_now()
{
  update_ts_ = coyot3::tools::getCurrentTimestamp();
  return update_ts_;
}

bool DiagnosticsBookItem::child_set(const DiagnosticsBookItem& d)
{
  DdsSiblingsMap::iterator it;
  it = children_.find(d.tag());
  if(it == children_.end())
  {
    // CLOG_INFO("creating child : " << d.tag());
    children_.insert(std::pair<std::string,DiagnosticsBookItem>(d.tag(),d));
    return true;
  }
  //CLOG_INFO("updating child : " << d.tag());
  it->second = d;
  return false;  
}
bool DiagnosticsBookItem::child_get(const std::string& n,DdsSiblingsMap::iterator& i)
{
  return ( (i = children_.find(n)) != children_.end());
}

bool DiagnosticsBookItem::search(int64_t itemId,DiagnosticsBookItem& dest) const{
  DiagnosticsBookItem res;
  if(itemId == id()){
    res.copy_standalone_item(*this);
    return true;
  }
  for(const std::pair<std::string,DiagnosticsBookItem>& item : children_)
  {
    if(item.second.search(itemId,res)== true){
      return true;
    } 
  }
  return false;
}

bool DiagnosticsBookItem::search(const std::string& itemName,DiagnosticsBookItem& dest) const{
  DiagnosticsBookItem res;
  if(itemName.compare(name()) == 0){
    res.copy_standalone_item(*this);
    return true;
  }
  for(const std::pair<std::string,DiagnosticsBookItem>& item : children_)
  {
    if(item.second.search(itemName,res)== true){
      return true;
    } 
  }
  return false;
}


bool DiagnosticsBookItem::searchTag(const std::string& tagName,DiagnosticsBookItem& dest) const{
  DiagnosticsBookItem res;
  if(tagName.compare(name()) == 0){
    res.copy_standalone_item(*this);
    return true;
  }
  for(const std::pair<std::string,DiagnosticsBookItem>& item : children_)
  {
    if(item.second.search(tagName,res)== true){
      return true;
    } 
  }
  return false;
}



//
//
//
DiagnosticsBook::DiagnosticsBook()
:DiagnosticsBookItem()
{
  id_ = 0;
  level_ = DiagnosticLevel::UNSET;
  tag_ = "ROOT";
  name_ = "ROOT";
}

DiagnosticsBook::DiagnosticsBook(const DiagnosticsBookItem& o)
:DiagnosticsBookItem(o){
  
}
DiagnosticsBook::~DiagnosticsBook(){

}




//
//
//
  const char* DiagnosticsBookItemJsIO::JsField::id    = "id";
  const char* DiagnosticsBookItemJsIO::JsField::level = "level";
  const char* DiagnosticsBookItemJsIO::JsField::level_desc = "level_desc";
  const char* DiagnosticsBookItemJsIO::JsField::name = "name";
  const char* DiagnosticsBookItemJsIO::JsField::tag = "tag";
  const char* DiagnosticsBookItemJsIO::JsField::message = "message";
  const char* DiagnosticsBookItemJsIO::JsField::hardware_id = "hardware_id";
  const char* DiagnosticsBookItemJsIO::JsField::update_ts = "update_ts";
  const char* DiagnosticsBookItemJsIO::JsField::values = "values";
  const char* DiagnosticsBookItemJsIO::JsField::values_key = "key";
  const char* DiagnosticsBookItemJsIO::JsField::values_value = "value";
  const char* DiagnosticsBookItemJsIO::JsField::components = "components";

  DiagnosticsBookItemJsIO::DiagnosticsBookItemJsIO()
  :DiagnosticsBookItem()
  ,JsonSerializablePacketBase()
  {

  }

  DiagnosticsBookItemJsIO::DiagnosticsBookItemJsIO(const DiagnosticsBookItem& s)
  :DiagnosticsBookItem(s)
  ,JsonSerializablePacketBase()
  {

  }

  DiagnosticsBookItemJsIO::~DiagnosticsBookItemJsIO()
  {

  }


  Json::Value DiagnosticsBookItemJsIO::to_json() const
  {
    Json::Value js;
    js[JsField::id] = static_cast<Json::LargestInt>(id_);
    js[JsField::level] = static_cast<Json::Int>(level_);
    js[JsField::level_desc] = DiagnosticLevelToString(level_);
    js[JsField::name] = name_;
    js[JsField::hardware_id] = hardid_;
    js[JsField::update_ts] = static_cast<Json::LargestInt>(update_ts_);
    js[JsField::tag] = tag_;
    
    if(values_.size() > 0)
    {
      Json::Value jkvs;
      for(const std::pair<std::string,std::string>& kv : values_)
      {
        Json::Value jkv;
        jkv[JsField::values_key] = kv.first;
        jkv[JsField::values_value] = kv.second;
        jkvs.append(jkv);
      }
      js[JsField::values] = jkvs;
    }
    if(children_.size() > 0)
    {
      Json::Value chs;
      for(std::pair<std::string,DiagnosticsBookItem> p : children_)
      {
        // CLOG_INFO("to-json appending for [" << p.second.tag() << "]");
        chs.append(DiagnosticsBookItemJsIO(p.second).to_json());
      }
      js[JsField::components] = chs;
    }
    return js;
  }

  bool DiagnosticsBookItemJsIO::from_json(const Json::Value& source){
    bool ok = true;
    int64_t bfint;
    Json::Value bfjsn, bfjsm;
    Json::ArrayIndex i,n;
    if(coyot3::tools::json_import_value(source,JsField::tag,tag_) == true)
    if(coyot3::tools::json_import_value(source,JsField::components,bfjsn))
    if(bfjsn.type() == Json::arrayValue)
    {
      n = bfjsn.size();
      for(i = 0; i< n ; ++i){
        DiagnosticsBookItemJsIO buffer;
        if(buffer.from_json(bfjsn[n])){
          child_set(buffer);
        }else{
          CLOG_WARN("diagnostic-data-source-jsio : from-json : error importing "
            "component data");
          ok = false;
        }
      }
      coyot3::tools::json_import_value(source,JsField::id,id_);
    }
     //json basic.
    ok &= coyot3::tools::json_import_value(source,JsField::level,bfint);
    ok &= coyot3::tools::json_import_value(source,JsField::name,name_);
    ok &= coyot3::tools::json_import_value(source,JsField::hardware_id,hardid_);
    ok &= coyot3::tools::json_import_value(source,JsField::message,message_);
    
    if(ok)level_ = static_cast<DiagnosticLevel>(bfint);

    if(coyot3::tools::json_import_value(source,JsField::values,bfjsn))
    if(bfjsn.type() == Json::arrayValue){
      //no values
      n = bfjsn.size();
      for(i = 0;i < n;++i){
        std::string k,v;
        ok &= coyot3::tools::json_import_value(bfjsn[i],JsField::values_key,k);
        ok &= coyot3::tools::json_import_value(bfjsn[i],JsField::values_value,v);
        value(k,v);
      }
    }
    return ok;
  }


  // begin MFI
  Json::Value DiagnosticsBookItemJsIO::to_json_with_MRM() const
  {
    static int nb_recursive_call = 0;
    static Json::Value js_mrm;
    static Json::Value jkvs_mrm;
    Json::Value js;
    js[JsField::id]          = static_cast<Json::LargestInt>(id_);
    js[JsField::level]       = static_cast<Json::Int>(level_);
    js[JsField::level_desc]  = DiagnosticLevelToString(level_);
    js[JsField::name]        = name_;
    js[JsField::hardware_id] = hardid_;
    js[JsField::update_ts]   = static_cast<Json::LargestInt>(update_ts_);
    js[JsField::tag]         = tag_;

    if (nb_recursive_call ==0){ 
      js_mrm[JsField::id]         = static_cast<Json::LargestInt>(id_);
      js_mrm[JsField::level]      = static_cast<Json::Int>(level_);
      js_mrm[JsField::level_desc] = DiagnosticLevelToString(level_);
      js_mrm[JsField::name]       = name_;
      js_mrm[JsField::hardware_id] = hardid_;
      js_mrm[JsField::update_ts] = static_cast<Json::LargestInt>(update_ts_);
      js_mrm[JsField::tag] = tag_;
    }
    
    if(values_.size() > 0)
    {
      Json::Value jkvs;
      for(const std::pair<std::string,std::string>& kv : values_)
      {
        Json::Value jkv;
        jkv[JsField::values_key] = kv.first;
        jkv[JsField::values_value] = kv.second;
        jkvs.append(jkv);      }
      js[JsField::values] = jkvs;
    }
    if(children_.size() > 0)
    {
      Json::Value chs;
      for(std::pair<std::string,DiagnosticsBookItem> p : children_){
        p.second.level();
        
        // CLOG_INFO("to-json appending for [" << p.second.tag() << "]");
        if (static_cast<int>(p.second.level()) == 3)
        {
        Json::Value jkv;
        jkv[JsField::values_key] = p.second.name();
        jkv[JsField::values_value] = p.second.tag();
        jkvs_mrm.append(jkv);      
        }
  
        nb_recursive_call++;
        chs.append(DiagnosticsBookItemJsIO(p.second).to_json());
      }
      js[JsField::components] = chs;
    }

    js_mrm[JsField::values] = jkvs_mrm;
    return js;
  }
  //end MFI



}
}