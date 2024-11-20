#include <Coyot3pp/Cor3/dictionnary_codec/dictionnary_codec.hpp>

namespace coyot3{
namespace tools{

void SetDebugLevel(int l)
{
  CLOG_DEBUG_LEVEL_SET(l);
}
/*----- meaning --- begin -------*/
const char* StateEvaluationToString(StateEvaluation s)
{
  switch(s)
  {
    case StateEvaluation::NOT_APPLICABLE : return "NOT_APPLICABLE";break;
    case StateEvaluation::NORMAL : return "NORMAL";break;
    case StateEvaluation::WARNING : return "WARNING";break;
    case StateEvaluation::ERROR : return "ERROR";break;
    case StateEvaluation::DEBUG : return "DEBUG";break;
    case StateEvaluation::err_UNKNOWN : return "err_UNKNOWN";break;
    default:
      return "state-evaluation-decode-error";
  }

}
StateEvaluation StateEvaluationFromString(const std::string& s)
{
  if(!s.compare(StateEvaluationToString(StateEvaluation::NOT_APPLICABLE)))return StateEvaluation::NOT_APPLICABLE;
  if(!s.compare(StateEvaluationToString(StateEvaluation::NORMAL)))return StateEvaluation::NORMAL;
  if(!s.compare(StateEvaluationToString(StateEvaluation::WARNING)))return StateEvaluation::WARNING;
  if(!s.compare(StateEvaluationToString(StateEvaluation::ERROR)))return StateEvaluation::ERROR;
  if(!s.compare(StateEvaluationToString(StateEvaluation::DEBUG)))return StateEvaluation::DEBUG;
  return StateEvaluation::err_UNKNOWN;
}



const char* Meaning::JsField::value  = "value";
const char* Meaning::JsField::description  = "content";
const char* Meaning::JsField::evaluation  = "state";

Meaning::Meaning()
:value_(-1)
,description_()
,evaluation_(StateEvaluation::err_UNKNOWN)
{

}
Meaning::Meaning(const Meaning& o)
{
  *this = o;
}
Meaning::Meaning(int64_t val,StateEvaluation sev,const std::string& eva)
:value_(val)
,description_(eva)
,evaluation_(sev){

}

Meaning::~Meaning()
{

}

int64_t Meaning::value() const {return value_;}
int64_t Meaning::value(int64_t v){return (value_ = v);}

std::string Meaning::description() const {return description_;}
std::string Meaning::description(const std::string& d){return description_ = d;}

StateEvaluation Meaning::evaluation() const {return evaluation_;}
StateEvaluation Meaning::evaluation(StateEvaluation e){return evaluation_ = e;}

Meaning& Meaning::operator=(const Meaning& o)
{
  value_ = o.value_;
  description_ = o.description_;
  evaluation_ = o.evaluation_;
  return *this;
}

Json::Value Meaning::toJson(bool numericStateEval)const{
  Json::Value js;
    js[JsField::value] = static_cast<Json::LargestInt>(value_);
    if(numericStateEval == true)
    {
      js[JsField::evaluation] = static_cast<Json::LargestInt>(evaluation_);
    }else{
      js[JsField::evaluation] = StateEvaluationToString(evaluation_);
    }
    
    js[JsField::description] = description_;
  return js;
}


bool Meaning::fromJson(const Json::Value& s)
{
  Json::ValueType type;
  if(
      (!s.isMember(JsField::value))
    ||(!s.isMember(JsField::evaluation))
    ||(!s.isMember(JsField::description))
  ){
    CLOG_DEBUG(4,"meaning : from-json : members lack for input ((" << s << "))");
    return false;
  }
  try{
    value_ = static_cast<int64_t>(s[JsField::value].asLargestInt());
    
    type = s[JsField::evaluation].type();
    if(type == Json::intValue)
    {
      evaluation_ = static_cast<StateEvaluation>(s[JsField::evaluation].asLargestInt());
    }else if(type == Json::stringValue){
      evaluation_ = StateEvaluationFromString(s[JsField::evaluation].asString());
    }else{
      CLOG_ERROR("meaning : from-json : member [evaluation] is not of type string or integer");
      return false;
    }
    description_ = s[JsField::description].asString();
  }catch(const Json::Exception& e){
    CLOG_WARN("meaning : from-json : json-exception raise :e(" << e.what() << ")");
    return false;
  }catch(...){
    CLOG_WARN("meaning : from-json : exception raise : reason unknown");
    return false;
  }
  return true;
}
std::string Meaning::toString() const
{
  std::stringstream ss;
  ss << "v:" << value_ << ";e:" << evaluation_ << ";d:" << description_;
  return ss.str();
}

/*----- meaning --- end -------*/

MeaningsSet::MeaningsSet(bool is_array)
:m_meaning_set_()
,m_as_array_(is_array)
{

}
MeaningsSet::MeaningsSet(const MeaningsSet& o)
{
  *this = o;
}
MeaningsSet::~MeaningsSet()
{

}

MeaningsSet& MeaningsSet::operator=(const MeaningsSet& o)
{
  MeaningSetValueMapConstIterator ito;
  m_meaning_set_.clear();
  for(ito = o.m_meaning_set_.begin();ito!= o.m_meaning_set_.end();++ito)
  {
    m_meaning_set_.insert(*ito);
  }
  return *this;
}

size_t MeaningsSet::push(const Meaning& o)
{
  MeaningSetValueMapIterator it;
  it = m_meaning_set_.find(o.value());

  if(it != m_meaning_set_.end())
  {
    //value already included.
    return static_cast<size_t>(0);
  }
  m_meaning_set_.insert(std::make_pair(o.value(),o));
  return static_cast<size_t>(1);
}

size_t MeaningsSet::push(const MeaningsSet& o)
{
  MeaningSetValueMapConstIterator itc;
  MeaningSetValueMapIterator      itl;
  size_t resolved = 0;
  
  for(itc = o.m_meaning_set_.begin();itc != o.m_meaning_set_.end();++itc)
  {
    itl = m_meaning_set_.find(itc->second.value());

    if(itl == m_meaning_set_.end())
    {
      m_meaning_set_.insert(std::make_pair(itc->second.value(),itc->second));
      ++resolved;
    }
  }
  return resolved;
}

bool MeaningsSet::get(int64_t i,Meaning& m) const
{
  MeaningSetValueMapConstIterator it;

  it = m_meaning_set_.find(i);
  if(it == m_meaning_set_.end())
  {
    return false;
  }
  m = it->second;
  return true;
}

void MeaningsSet::clear()
{
  m_meaning_set_.clear();
}

size_t MeaningsSet::remove(int64_t i)
{
  MeaningSetValueMapIterator it;
  if((it = m_meaning_set_.find(i)) == m_meaning_set_.end())
  {
    return static_cast<int64_t>(0);
  }
  m_meaning_set_.erase(it);
  return static_cast<int64_t>(1);
}

size_t MeaningsSet::remove(const Meaning& m)
{
  MeaningSetValueMapIterator it;
  if((it = m_meaning_set_.find(m.value())) == m_meaning_set_.end())
  {
    return static_cast<int64_t>(0);
  }
  m_meaning_set_.erase(it);
  return static_cast<int64_t>(1);

}
size_t MeaningsSet::remove(const MeaningsSet& s)
{
  
  MeaningSetValueMapConstIterator itc;
  size_t resolved = 0;
  for(itc = s.m_meaning_set_.begin();itc != s.m_meaning_set_.end();++itc)
  {
    resolved+=remove(itc->second);
  }
  
  return resolved;
}

Json::Value MeaningsSet::toJson() const
{
  Json::Value js;
  for(const std::pair<int64_t,Meaning>& o: m_meaning_set_)
  {
    switch(m_as_array_)
    {
      case true:
          js.append(o.second.toJson());
        break;
      case false:
          js[std::to_string(o.second.value())] = o.second.toJson();
        break;
    }
  }
  return js;
}

bool MeaningsSet::fromJson(const Json::Value& s)
{
  bool res = true;
    CLOG_DEBUG(8,"meaning-set : from-json : parsing evaluation items from [" << s << "]");
    if(s.type() != Json::arrayValue)
    {
      CLOG_WARN("meaning-set : error , input is not an array!");
      return false;
    }
    Json::ArrayIndex sz = s.size();
    for(Json::ArrayIndex i = 0;i < sz;++i)
    {
      CLOG_DEBUG(8,"meaning-set : from-json : parsing meaning [" << i << "]");
      Meaning m;
      if(!m.fromJson(s[i]))
      {
        //error
        res = false;
        continue;
      }
      push(m);
    }
  
  return true;
}

std::string MeaningsSet::toString() const{
  std::stringstream ss;

  bool isFirst = true;
  for(const std::pair<int64_t,Meaning>& o : m_meaning_set_)
  {
    if(!isFirst) ss << " - ";
    ss << o.second.toString();
    isFirst = false;
  }
  return ss.str();
}



// --- concept item - begin ----

const char* ConceptItem::JsField::name = "name";
const char* ConceptItem::JsField::id = "id";
const char* ConceptItem::JsField::description = "description";
const char* ConceptItem::JsField::meta = "meta";
const char* ConceptItem::JsField::bit_start = "bit_start";
const char* ConceptItem::JsField::length = "length";
const char* ConceptItem::JsField::check = "check";
const char* ConceptItem::JsField::evaluation = "evaluation";

ConceptItem::ConceptItem()
:name_()
,id_()
,description_()
,meta_()
,bit_start_(0)
,length_(0)
,check_(false)
,meanings_()
{

}
ConceptItem::ConceptItem(const ConceptItem& o)
{
  *this = o;
}
ConceptItem::~ConceptItem()
{

}

ConceptItem& ConceptItem::operator=(const ConceptItem& o)
{
  name_ = o.name_;
  id_ = o.id_;
  meta_ = o.meta_;
  bit_start_ = o.bit_start_;
  length_ = o.length_;
  check_ = o.check_;

  meanings_ = o.meanings_;

  return *this;
}


bool ConceptItem::decode(int64_t value,Meaning& m) const
{
  int64_t cval = static_cast<int64_t>(0x00);
  int64_t flag = static_cast<int64_t>(0x01);
  for(int i = 0;i < length_;++i){
    if(i){
      cval<<=1;
    }
    cval|=flag;
  }
  cval<<=bit_start_;
  cval&=value;
  cval>>=bit_start_;
  int64_t extracted;
  return meanings_.get(cval,m);
}

bool ConceptItem::pushMeaning(StateEvaluation s,int64_t v,const std::string& content)
{
  Meaning m(v,s,content);
  if(meanings_.push(m)== 0)
  {
    return false;
  }
  return true;
}

Json::Value ConceptItem::toJson() const
{
  Json::Value js;
  js[JsField::name] = name_;
  js[JsField::id] = id_;
  js[JsField::description] = description_;
  js[JsField::meta] = meta_;
  js[JsField::bit_start] = static_cast<Json::Int>(bit_start_);
  js[JsField::length] = static_cast<Json::Int>(length_);
  js[JsField::check] = check_;
  
  js[JsField::evaluation] = meanings_.toJson();

  return js; 
}

std::string ConceptItem::toString() const {
  std::stringstream ss;
  ss  << "n: "   << name_ 
      << ";d: " << description_
      << ";bs: "<< bit_start_
      << ";le: "<< length_
      << ";ac: "<< (check_==true?"Y":"N")
      << ";(("  << meanings_.toString() << "));met:" << meta_;
  return ss.str();  
}

bool ConceptItem::fromJson(const Json::Value& js) {
  CLOG_DEBUG(8,"concept-item : from-json : parsing item ((" << js << "))");
  if( 
      (!js.isMember(JsField::name))
    ||(!js.isMember(JsField::id))
    ||(!js.isMember(JsField::description))
    ||(!js.isMember(JsField::bit_start))
    ||(!js.isMember(JsField::length))
    ||(!js.isMember(JsField::check))
    ||(!js.isMember(JsField::evaluation))
  ){
    std::stringstream ss;
    ss << "CONCEPT-ITEM : from-json : required keys not present ["
      << JsField::name << ","
      << JsField::id << ","
      << JsField::description << ","
      << JsField::meta << ","
      << JsField::bit_start << ","
      << JsField::length << ","
      << JsField::check << ","
      << JsField::evaluation << "] "
      "for input [" << js << "]";
    CLOG_ERROR(ss.str())
    throw std::runtime_error(ss.str());
  }

  try{

  
    name_ = js[JsField::name].asString();
    id_ = js[JsField::id].asString();
    description_ = js[JsField::description].asString();
    meta_ = js[JsField::meta].asString();
    bit_start_ = js[JsField::bit_start].asInt();
    length_ = js[JsField::length].asInt();
    check_ = js[JsField::check].asBool();

    
  }catch(const Json::Exception& e){
    std::stringstream ss;
    ss << "CONCEPT-ITEM : from-json : jsoncpp-exception throwed while parsing (" 
      << e.what() << ")";
    CLOG_ERROR(ss.str());
    throw std::runtime_error(ss.str());
    return false;
  }catch(...){
    std::stringstream ss;
    ss << "CONCEPT-ITEM : from-json : jsoncpp-exception throwed while parsing (" 
      "unknown-reason)";
    CLOG_ERROR(ss.str());
    throw std::runtime_error(ss.str());
    return false;
  }
  
  if(js.isMember(JsField::meta) == true)
  {
    switch(js[JsField::meta].type())
    {
      
      case Json::stringValue:
        meta_ = js[JsField::meta].asString();
        break;
      default:
        meta_ = "-meta-content-not-parsed-bcs-only-strings-are-accepted";
    }
  }
  CLOG_DEBUG(8,"concept-item : from-json : parsed basics, now parsing meanings ((" << js[JsField::evaluation] << "))");
  return meanings_.fromJson(js[JsField::evaluation]);
}

ConceptItemsSet::ConceptItemsSet()
:citems_(){
}
ConceptItemsSet::ConceptItemsSet(const ConceptItemsSet& o){
  *this = o;
}
ConceptItemsSet::~ConceptItemsSet(){

}

ConceptItemsSet& ConceptItemsSet::operator=(const ConceptItemsSet& o)
{
  citems_ = o.citems_;
  return *this;
}


Json::Value ConceptItemsSet::toJson() const
{
  Json::Value js(Json::arrayValue);
  
  for(const std::pair<std::string,ConceptItem>& p : citems_)
  {
    js.append(p.second.toJson());
  }
  return js;
}
bool        ConceptItemsSet::fromJson(const Json::Value& js)
{
  if(js.type() != Json::arrayValue)
  {
    CLOG_ERROR("CONCEPT-ITEM-SET : from-json : item is not an array");
    throw std::runtime_error("CONCEPT-ITEM-SET : from-json : item is not an array");
    return false;
  }
  
  Json::ArrayIndex i, s = js.size();
  CLOG_DEBUG(8,"concept-item : from-json : parsing [" << s << "] items");

  bool res = true;
  for( i = 0;i < s; ++i)
  {
    CLOG_DEBUG(8,"concept-item-set : from-json : concept-item parsing item [" << i << "]");

    ConceptItem c;
    try{
      if(!c.fromJson(js[i]))
      {
        CLOG_ERROR("CONCEPT-ITEM-SET : from-json : error obtaining item [" 
          << static_cast<int>(i) << "]");
        res = false;
      }
      if(citems_.find(c.id_) != citems_.end())
      {
        CLOG_WARN("CONCEPT-ITEM-SET : from-json : item [" << c.id_ << "] already exists!");
        throw "CONCEPT-ITEM-SET : from-json : repeated items in dictionnary section";
      }else{
        citems_.insert(std::make_pair(c.id_,c));
        CLOG_INFO("concept-items-set : from-json : concept-item parsed item [" << i << "] item [" << c.id_ << "]");

      }
      
    }catch(const Json::Exception& e){
      CLOG_ERROR("concept-item-set : from-json : exception parsing item (" 
        << e.what() << ") for ((" << js[i] << "))");
      res = false;
    }catch(...){
      CLOG_ERROR("concept-item-set : from-json : exception parsing item (" 
        "unknown-exception) for ((" << js[i] << "))");
      res = false;
    }
    
  }
  return res;
}

std::string ConceptItemsSet::toString()const
{
  std::stringstream ss;

  bool isFirst = true;
  for(const std::pair<std::string,ConceptItem>& p : citems_)
  {
    if(!isFirst)
    {
      ss << " , ";
    }
    ss << p.second.toString();
  }
  return ss.str();
}

std::string ConceptItemsSet::getKeys() const
{
  std::stringstream ss;

  bool isFirst = true;
  for(const std::pair<std::string,ConceptItem>& p : citems_)
  {
    if(isFirst == false)
    {
      ss << ",";
    }
    ss << p.first;
    isFirst = false;
  }
  return ss.str();

}
size_t ConceptItemsSet::size() const{
  return citems_.size();
}


bool ConceptItemsSet::get(const std::string& id,ConceptItem& destination) const
{
  concept_item_set_const_iterator i = citems_.find(id);
  if(i == citems_.end())
  {
    return false;
  }
  destination = i->second;
  return true;
}
// --- concept item - end ----


// --- concept evaluation - begin ---

const char* ConceptEvaluation::JsField::id = "id";
const char* ConceptEvaluation::JsField::name = "name";
const char* ConceptEvaluation::JsField::ev = "ev";
const char* ConceptEvaluation::JsField::desc = "desc";


ConceptEvaluation::ConceptEvaluation()
:id_()
,name_()
,ev_(StateEvaluation::err_UNKNOWN)
,desc_(){

}
ConceptEvaluation::ConceptEvaluation(const ConceptEvaluation& o)
{
  *this = o;
}
ConceptEvaluation::ConceptEvaluation(const std::string& i
                                    ,const std::string& n)
:id_(i)
,name_(n)
,ev_(StateEvaluation::err_UNKNOWN)
,desc_(CYTOOLS_DICT_EVAL_NOT_INITIALIZED)
{

}
ConceptEvaluation::ConceptEvaluation(const std::string& i
                                    ,const std::string& n
                                    , StateEvaluation e
                                    ,const std::string& d)
:id_(i)
,name_(n)
,ev_(e)
,desc_(d){

}
ConceptEvaluation::~ConceptEvaluation()
{

}


ConceptEvaluation& ConceptEvaluation::operator=(const ConceptEvaluation& o)
{
  id_   = o.id_;
  name_ = o.name_;
  ev_   = o.ev_;
  desc_ = o.desc_;

  return *this;
}

Json::Value ConceptEvaluation::toJson(bool numericStateEval) const{
  Json::Value js;
  js[JsField::id] = id_;
  js[JsField::name] = name_;
  if(numericStateEval == true)
  {
    js[JsField::ev] = static_cast<Json::Int>(ev_);
  }else{
    js[JsField::ev] = StateEvaluationToString(ev_);
  }
  
  js[JsField::desc] = desc_;
  return js;
}
bool ConceptEvaluation::fromJson(const Json::Value& js){
  if(
      (!(js.isMember(JsField::id)))
      ||(!(js.isMember(JsField::name)))
      ||(!(js.isMember(JsField::ev)))
      ||(!(js.isMember(JsField::desc)))
  ){
    CLOG_WARN("concept-evaluation : from-json : error obtaining members ["
      << JsField::id << ", "
      << JsField::name << ", "
      << JsField::ev << ", "
      << JsField::desc << "] from (( " <<js  << " ))");
      return false;

    return false;
  }
  try{
    id_ = js[JsField::id].asString();
    name_ = js[JsField::name].asString();
    switch(js[JsField::ev].type()){
      case Json::stringValue:
        ev_ = StateEvaluationFromString(js[JsField::ev].asString());
        break;
      case Json::intValue:
        ev_ = static_cast<StateEvaluation>(js[JsField::ev].asInt());
        break;
      default:
        CLOG_ERROR("concept-evaluation : from-json : error parsing parameter EV!");
    }
    desc_ = js[JsField::desc].asString();
  }catch(const Json::Exception& e){
    std::stringstream ss;
    ss << "concept-evaluation : from-json : json exception (" << e.what() << 
      ") : while parsing ((" << js << "))";
    CLOG_ERROR(ss.str());
    return false;
  }
  return true;
}
std::string ConceptEvaluation::toString() const
{
  std::stringstream ss;

  ss << "id: " << id_ << ", name: " << name_ << ", val:" << ev_ << ", " << desc_;
  return ss.str();
}




bool  ConceptEvaluation::set(const std::string& i,const std::string& n)
{
  id_ = i;
  name_ = n;
  return true;
}
bool  ConceptEvaluation::set(const Meaning& m)
{
  ev_ = m.evaluation_;
  desc_ = m.description_;
  return true;
}









ConceptEvaluationSet::ConceptEvaluationSet()
:cevalset_()
{

}
ConceptEvaluationSet::ConceptEvaluationSet(const ConceptEvaluationSet& o){
  *this = o;
}
ConceptEvaluationSet::~ConceptEvaluationSet()
{

}

ConceptEvaluationSet& ConceptEvaluationSet::operator=(const ConceptEvaluationSet& o)
{
  cevalset_ = o.cevalset_;
  return *this;
}

Json::Value ConceptEvaluationSet::toJson(bool numericStateEval) const{
  Json::Value js(Json::arrayValue);

  for( const concept_eval_set_pair_c& p: cevalset_)
  {
    js.append(p.second.toJson(numericStateEval));
  }
  return js;
}

std::string ConceptEvaluationSet::getKeys() const{
  std::stringstream ss;
  bool isFirst = false;
  for( const concept_eval_set_pair_c& p: cevalset_)
  {
    if(!isFirst)
    {
      ss << ",";
    }
    ss << p.first;
    isFirst = false;
  }
  return ss.str();
}
size_t ConceptEvaluationSet::size() const{
  return cevalset_.size();
}


bool ConceptEvaluationSet::fromJson(const Json::Value& js){
  Json::ArrayIndex i,s = js.size();
  bool allgood = true;
  for(i = 0;i < s ; ++i)
  {
    ConceptEvaluation c;
    if(!c.fromJson(js[i]))
    {
      allgood = false;
      
    }
    if(push(c) == 0)
    {
      CLOG_ERROR("concept-evaluation-set : from-json : inserting concept-evaluation. duplicated ID?");
      allgood = false;
    }
  }
  return allgood;
}

size_t ConceptEvaluationSet::push(const ConceptEvaluation& o)
{
  concept_eval_set_iterator it;
  it = cevalset_.find(o.id_);
  if(it != cevalset_.end())
  {
    return static_cast<size_t>(0);
  }
  
  cevalset_.insert(std::make_pair(o.id_,o));
  return static_cast<size_t>(1);
}
size_t ConceptEvaluationSet::push(const ConceptEvaluationSet& o)
{
  size_t res = 0;
  for(const std::pair<std::string,ConceptEvaluation>& c : o.cevalset_)
  {
    res+=push(c.second);
  }
  return res;
}
size_t ConceptEvaluationSet::push(const std::string& id,const std::string& name)
{
  concept_eval_set_iterator it;
  it = cevalset_.find(id);
  if(it == cevalset_.end())
  {
    return static_cast<size_t>(0);
  }
  ConceptEvaluation c(id,name);
  cevalset_.insert(std::make_pair(c.id_,c));
  return static_cast<size_t>(1);
}

bool   ConceptEvaluationSet::update(const std::string& id,StateEvaluation s,const std::string& desc)
{
  concept_eval_set_iterator it;
  it = cevalset_.find(id);
  if(it == cevalset_.end())
  {
    return false;
  }
  it->second.ev_ = s;
  it->second.desc_ = desc;
  return true;
}
bool   ConceptEvaluationSet::update(const ConceptItem& i,int64_t sourceValue)
{
  concept_eval_set_iterator it;
  it = cevalset_.find(i.id_);
  if(it == cevalset_.end())
  {
    return false;
  }
  Meaning m;
  i.decode(sourceValue,m);
  it->second.ev_ = m.evaluation();
  it->second.desc_ = m.description();
  return true;
}
size_t ConceptEvaluationSet::update(const ConceptItemsSet& i,int64_t sourceValue)
{
  ConceptItemsSet::concept_item_set_const_iterator iter;
  size_t res=0;
  for(iter = i.citems_.begin();iter != i.citems_.end();++iter)
  {
    res+=update(iter->second,sourceValue);
  }
  return res;
}


bool ConceptEvaluationSet::get(const std::string& id,ConceptEvaluation& eval) const
{
  concept_eval_set_const_iterator it;
  it = cevalset_.find(id);
  if(it == cevalset_.end())
  {
    return false;
  }
  eval = it->second;
  return true;
}
// --- concept evaluation - end ---

// --- evaluation group - begin - 
const char* EvalResult::JsField::content = "content";
const char* EvalResult::JsField::state = "state";

EvalResult::EvalResult()
:content(CYTOOLS_DICT_EVALRESULT_NOT_INITIALIZED)
,state(StateEvaluation::err_UNKNOWN){
  
}
EvalResult::EvalResult(const EvalResult& o)
{
  *this = o;
}
EvalResult::~EvalResult()
{

}
EvalResult& EvalResult::operator=(const EvalResult& o)
{
  content = o.content;
  state = o.state;
  return *this;
}

Json::Value EvalResult::toJson(bool numericStateEval)const{
  Json::Value js;

  js[JsField::content] = content;
  if(numericStateEval == false)
  {
    js[JsField::state] = StateEvaluationToString(state);
  }else{
    js[JsField::state] = static_cast<Json::Int>(state);
  }
  return js;
}
bool EvalResult::fromJson(const Json::Value& js)
{
  Json::ValueType stateType;
  if(
    (!js.isMember(JsField::content))
    ||(!js.isMember(JsField::state))
  ){
    CLOG_ERROR("evaluation-group : EVAL : from-json : error obtaining all required fields ["
      << JsField::content << "," << JsField::state << "]");
    return false;
  }
  try{

    content = js[JsField::content].asString();
    stateType = js[JsField::state].type();
    if(stateType == Json::stringValue)
    {
      state = StateEvaluationFromString(js[JsField::state].asString());
    }else if(stateType == Json::intValue){
      state = static_cast<StateEvaluation>(js[JsField::state].asInt());
    }else{
      CLOG_ERROR("")
    }

  }catch(const Json::Exception& e)
  {
    CLOG_ERROR("EVALUATION-GROUP : EVAL : from-json : error parsing data (" << e.what() << ")");
    return false;
  }catch(...){
    CLOG_ERROR("EVALUATION-GROUP : EVAL : from-json : error parsing data ( unknown-reason )");
    return false;
  }
  return true;
}
std::string EvalResult::toString() const{
  std::stringstream ss;
  ss << "state:" << state << ",content: " << content;
  return ss.str();
}


const char* EvaluationGroupCalculationResult::JsField::id = "id";
const char* EvaluationGroupCalculationResult::JsField::name = "name";
const char* EvaluationGroupCalculationResult::JsField::timestamp = "timestamp";
const char* EvaluationGroupCalculationResult::JsField::description = "description";
const char* EvaluationGroupCalculationResult::JsField::result = "result";
const char* EvaluationGroupCalculationResult::JsField::items = "items";


EvaluationGroupCalculationResult::EvaluationGroupCalculationResult()
:id()
,name()
,description()
,result()
,items()
{
  timestamp = coyot3::tools::getCurrentTimestamp();
}
EvaluationGroupCalculationResult::EvaluationGroupCalculationResult(const EvaluationGroupCalculationResult& o)
{
  *this = o;
}
EvaluationGroupCalculationResult::~EvaluationGroupCalculationResult(){}

EvaluationGroupCalculationResult& EvaluationGroupCalculationResult::operator=(const EvaluationGroupCalculationResult& o)
{
  id = o.id;
  name = o.name;
  timestamp = o.timestamp;
  description = o.description;
  result = o.result;
  items = o.items;
  return *this;
}

Json::Value EvaluationGroupCalculationResult::toJson(bool numericStateEval)const
{
  Json::Value js;

  js[JsField::id] = id;
  js[JsField::name] = name;
  js[JsField::timestamp] = static_cast<Json::LargestInt>(timestamp);
  js[JsField::description] = description;
  js[JsField::result] = result.toJson(numericStateEval);
  js[JsField::items] = Json::Value(Json::arrayValue);

  for(const ConceptEvaluation& c : items)
  {
    js[JsField::items].append(c.toJson(numericStateEval));
  }
  
  
  return js;
}

bool EvaluationGroupCalculationResult::fromJson(const Json::Value& js)
{
  if(
    (!js.isMember(JsField::id))
    ||(!js.isMember(JsField::name))
    ||(!js.isMember(JsField::timestamp))
    ||(!js.isMember(JsField::result))
    ||(!js.isMember(JsField::items))
  ){
    CLOG_ERROR("evaluation-group-calc-result : from-json : all required fields were not found ["
      << JsField::id << ","
      << JsField::name << ","
      << JsField::result << "," 
      << JsField::items << "]"
    );
    return false;
  }
  if(js[JsField::items].type() != Json::arrayValue)
  {
    CLOG_ERROR("evaluation-group-calc-result : from-json : items property is not an array");
    return false;
  }
  try{
    id = js[JsField::id].asString();
    name = js[JsField::name].asString();
    result.fromJson(js[JsField::result]);
  }catch(const Json::Exception& e){
    CLOG_ERROR("evaluation-group-calc-result : from-json : error parsing input (" << e.what() << ")");
    return false;
  }catch(...){
    CLOG_ERROR("evaluation-group-calc-result : from-json : error parsing input ( unknown-reason )");
    return false;
  }
  Json::ArrayIndex i,s = js[JsField::items].size();
  items.clear();
  for(i = 0;i < s ;++i)
  {
    ConceptEvaluation c;
    if(!c.fromJson(js[JsField::items][i]))
    {
      CLOG_ERROR("evaluation-group-calc-result : from-json : found an invalid item");
      return false;
    }
    items.push_back(c);
  }
  if(js.isMember(JsField::description))
  {
    description = js[JsField::description].asString();
  }
  return true;
}



size_t EvaluationGroupCalculationResult::push(const ConceptEvaluation& c)
{
  items.push_back(c);
  return items.size();
}
 


const char* EvaluationGroup::JsField::id = "id";
const char* EvaluationGroup::JsField::name = "name";
const char* EvaluationGroup::JsField::description = "description";
const char* EvaluationGroup::JsField::evaluations = "evaluations";
const char* EvaluationGroup::JsField::all_ok = "all_ok";
const char* EvaluationGroup::JsField::all_down = "all_down";
const char* EvaluationGroup::JsField::warnings = "warnings";
const char* EvaluationGroup::JsField::items = "items";

EvaluationGroup::EvaluationGroup()
:id()
,name()
,description()
{

}
EvaluationGroup::EvaluationGroup(const EvaluationGroup& o){
  *this = o;
}
EvaluationGroup::~EvaluationGroup(){

}

EvaluationGroup& EvaluationGroup::operator=(const EvaluationGroup& o)
{
  id = o.id;
  name = o.name;
  description = o.description;
  identifiers = o.identifiers;
  meaning_all_ok = o.meaning_all_ok;
  meaning_all_down = o.meaning_all_down;
  meaning_warnings = o.meaning_warnings;
  return *this;
}

Json::Value EvaluationGroup::toJson() const{
  Json::Value js;

  js[JsField::id] = id;
  js[JsField::name] = name;
  js[JsField::description] = description;
  js[JsField::evaluations][JsField::all_ok] = meaning_all_ok.toJson();
  js[JsField::evaluations][JsField::all_down] = meaning_all_down.toJson();
  js[JsField::evaluations][JsField::warnings] = meaning_warnings.toJson();
  Json::Value items(Json::arrayValue);

  for(const std::string& id : identifiers)
  {
    items.append(id);
  }
  js[JsField::items] = items;

  return js;
}

bool EvaluationGroup::fromJson(const Json::Value& js)
{
  if(
      (!js.isMember(JsField::id))
    ||(!js.isMember(JsField::name))
    ||(!js.isMember(JsField::description))
    ||(!js.isMember(JsField::items))
    ||(!js.isMember(JsField::evaluations))
  ){
    CLOG_ERROR("evaluation-group : from-json : source does not contain all required keys ["
      << JsField::id << ","
      << JsField::name << ","
      << JsField::description << ","
      << JsField::evaluations << "]"
      << JsField::items << ","
    );
    return false;
  }
  Json::Value e = js[JsField::evaluations];
  if(
      (!e.isMember(JsField::all_ok))
    ||(!e.isMember(JsField::all_down))
    ||(!e.isMember(JsField::warnings))
  ){
    CLOG_ERROR("evaluation-group : from-json : source evaluations does not contain all required keys [" 
      << JsField::all_ok << ","
      << JsField::all_down << ","
      << JsField::warnings << "]");
    return false;
  }

  if(
      (!meaning_all_ok.fromJson(e[JsField::all_ok]))
    ||(!meaning_all_down.fromJson(e[JsField::all_down]))
    ||(!meaning_warnings.fromJson(e[JsField::warnings]))
  ){
    CLOG_ERROR("evaluation-group : from-json : error parsing evaluations");
    return false;
  }

  try{
    id = js[JsField::id].asString();
    name = js[JsField::name].asString();
    description = js[JsField::description].asString();
    if(js[JsField::items].type() != Json::arrayValue)
    {
      CLOG_ERROR("evaluation-group : from-json : error parsing source ( items must be an array of strings )");
      return false;  
    }
    Json::ArrayIndex i = js[JsField::items].size();
    identifiers.clear();
    for(i = 0; i < js[JsField::items].size() ; ++i)
    {
      identifiers.push_back(js[JsField::items][i].asString());
    }

  }catch(const Json::Exception& e)
  {
    CLOG_ERROR("evaluation-group : from-json : error parsing source (" << e.what() << ")");
    return false;
  }catch(...){
    CLOG_ERROR("evaluation-group : from-json : error parsing source ( unknown-reason )");
    return false;
  }

  return true;
}

std::string EvaluationGroup::toString() const
{
  std::stringstream ss;

  ss << "id:" << id
     << ";name:" << name
     << ";ids(";
  bool isFirst = true;
  for(const std::string& i : identifiers)
  {
    if(!isFirst)
    {
      ss << ",";
    }
    ss << i;
  }
  ss << ");desc:" << description;
  return ss.str();
}


bool EvaluationGroup::calculate(const CevalSetMap& source,EvaluationGroupCalculationResult& result) const
{
  bool allok    = true;
  bool allwrong = true;
  ConceptEvaluation eval;

  std::string sectionId;
  std::string itemId;
  std::vector<std::string> secitemid;
  
  CevalSetMapConstIterator it;
  
  for(const std::string& id : identifiers)
  {
    if(coyot3::tools::stringSplit(id,".",secitemid) != 2)
    {
      CLOG_WARN("evaluation-group : calculate : error extracting section-id and item-id from [" << id << "]");
      return false;
    }
    sectionId = secitemid[0];
    itemId = secitemid[1];

    if((it = source.find(sectionId)) == source.end())
    {
      CLOG_WARN("evaluation-group : calculate : error obtaining evaluation of section [" << sectionId << "]");
      return false;
    }
    
    if(!it->second.get(itemId,eval))
    {
      CLOG_WARN("evaluation-group : calculate : error obtaining evaluation for [" 
        << id << "],(" << sectionId << "," << itemId << "), unable to find at [" << it->second.getKeys() << "]");
      continue;
    }
    result.push(eval);
    if(eval.ev_ != StateEvaluation::NORMAL)
    {
      allok = false;
    }else{
      allwrong = false;
    }
    
  }

  
  result.id = id;
  result.name = name;
  result.description = description;
  if(allok == true){

    result.result = meaning_all_ok;
  }else if(allwrong == true){
    result.result = meaning_all_down;
  }else{
    result.result = meaning_warnings;
  }
  return true;
}


EvaluationGroupsResult::EvaluationGroupsResult()
:evg_results()
{

}
EvaluationGroupsResult::EvaluationGroupsResult(const EvaluationGroupsResult& o)
{
  *this = o;
}
EvaluationGroupsResult::~EvaluationGroupsResult()
{

}

EvaluationGroupsResult& EvaluationGroupsResult::operator=(const EvaluationGroupsResult& o)
{
  
  evg_results = o.evg_results;
  return *this;
}

Json::Value EvaluationGroupsResult::toJson(bool numericStateEval) const
{
  Json::Value js = Json::Value(Json::arrayValue);
  for(const std::pair<std::string,EvaluationGroupCalculationResult>& r : evg_results)
  {
    js.append(r.second.toJson(numericStateEval));
  }
  return js;
}
bool EvaluationGroupsResult::fromJson(const Json::Value& js)
{
  if(js.type() != Json::arrayValue)
  {
    CLOG_ERROR("evaluation-groups-result : from-json : input is not an array");
    return false;
  }
  Json::ArrayIndex i,s = js.size();
  evg_results.clear();
  for(i = 0;i < s;++i){
    EvaluationGroupCalculationResult r;
    if(!r.fromJson(js[i]))
    {
      CLOG_ERROR("evaluation-groups-result : from-json : error deserializing item [" << i << "]");
      return false;
    }
    evg_results.insert(std::make_pair(r.id,r));
  }
  return true;
}
void EvaluationGroupsResult::clear()
{
  evg_results.clear();
}

size_t EvaluationGroupsResult::push(const EvaluationGroupCalculationResult& c)
{
  

  if(evg_results.find(c.id) != evg_results.end())
  {
    return 0;
  }
  evg_results.insert(std::make_pair(c.id,c));
  return evg_results.size();
}

// --- evaluation group - end - 



const char* DictionnarySection::JsField::id = "id";
const char* DictionnarySection::JsField::name = "name";
const char* DictionnarySection::JsField::description = "description";
const char* DictionnarySection::JsField::items = "items";

DictionnarySection::DictionnarySection()
:id_()
,name_()
,description_()
,ciset_(){

}
DictionnarySection::DictionnarySection(const DictionnarySection& o)
{
  *this = o;
}
DictionnarySection::~DictionnarySection(){

}
DictionnarySection& DictionnarySection::operator=(const DictionnarySection& o)
{
  id_ = o.id_;
  name_ = o.name_;
  description_ = o.description_;
  ciset_ = o.ciset_;

  return *this;
}

Json::Value DictionnarySection::toJson() const {
  Json::Value js;

  js[JsField::id] = id_;
  js[JsField::name] = name_;
  js[JsField::description] = description_;
  js[JsField::items] = ciset_.toJson();

  return js;
}


bool DictionnarySection::fromJson(const Json::Value& js)
{
  bool res = true;
  if(
      (!js.isMember(JsField::id))
    ||(!js.isMember(JsField::name))
    ||(!js.isMember(JsField::description))
    ||(!js.isMember(JsField::items))
  ){
    CLOG_ERROR("dictionnary-section : from-json : input does not contain all required fields ["
      << JsField::id << ","
      << JsField::name << ","
      << JsField::description << ","
      << JsField::items << "]");
    return false;
  }

  try{
    id_ = js[JsField::id].asString();
    name_ = js[JsField::name].asString();
    description_ = js[JsField::description].asString();

  }catch(const Json::Exception& e){
    CLOG_ERROR("dictionnary-section : from-json : error parsing input (" << e.what() << ")");
    return false;
  }catch(...){
    CLOG_ERROR("dictionnary-section : from-json : error parsing input ( unknown-reason )");
    return false;
  }
  CLOG_DEBUG(8,"dictionnary-section : from-json : parsed id, name,description, now parsing items ((" << js[JsField::items] << "))");
  if(!ciset_.fromJson(js[JsField::items]))
  {
    CLOG_ERROR("dictionnary-section : from-json : error parsing [items]");
    res = false;
  }else{
    CLOG_INFO("dictionnary-section : from-json : parsed [" << ciset_.size() << "] items(" << ciset_.getKeys() <<  ")");
  }
  
  return res;
}


std::string DictionnarySection::toString()const {
  std::stringstream ss;
  ss << "id: " << id_
     << ";name: " << name_
     << ";concepts:[" << ciset_.toString() << "]:"
     << description_;
  return ss.str();
}



ConceptEvaluationSet DictionnarySection::evaluate(int64_t input) const
{
  ConceptEvaluationSet evaluation;
  //CLOG_WARN("concept-evaluation-set : evaluate : evaluating [" << ciset_.citems_.size() << "] items");
  for(const std::pair<std::string,ConceptItem>& c : ciset_.citems_)
  {
    std::string id;

    id = c.second.id_;

    c.second.name_;
    Meaning meaning;
    if(c.second.check_ == false)
    {
      CLOG_DEBUG(3,"concept-evaluation-set : evaluate : ignoring item [" << c.second.id_ << "]");
      continue;
    }
    if(!c.second.decode(input,meaning))
    {
      CLOG_WARN("concept-evaluation-set : evaluate : error evaluating input-value[" << input << "] at [" << c.second.id_);
      ConceptEvaluation ceval(c.second.id_,c.second.name_);
      evaluation.push(ceval);
    }else{
      //CLOG_WARN("concept-evaluation-set : evaluate : evaluated input-value[" << input << "] at [" << c.second.id_ << "]");
      ConceptEvaluation ceval(c.second.id_,c.second.name_,meaning.evaluation_,meaning.description_);
      if(!evaluation.push(ceval))
      {
        CLOG_WARN("concept-evaluation-set : evaluate : error pushing concept-evaluation [" << ceval.toString() << "]");
      }
    }
  }
  //CLOG_WARN("concept-evaluation-set : evaluate : evaluated [" << evaluation.getKeys() << "]");
  return evaluation;
}
ConceptEvaluation DictionnarySection::evaluateId(const std::string& id,int64_t value) const
{
  
  ConceptItem citem;
  Meaning     meaning;
  
  if(ciset_.get(id,citem) == false)
  {
    CLOG_WARN("dictionnary-section : evaluate-id : id [" << id << "] not found");
    return ConceptEvaluation();
  }
  citem.decode(value,meaning);
  ConceptEvaluation ceval(id,citem.name_);
  ceval.set(meaning);
  return ceval;
}


bool DictionnarySection::get(const std::string& id,ConceptItem& destination) const
{
  return ciset_.get(id,destination);
}

// system-state-detail-dictionnary : 
const char* StatesDictionnary::JsField::title = "title";
const char* StatesDictionnary::JsField::evaluation_groups = "evaluation_groups";
const char* StatesDictionnary::JsField::sections = "sections";


StatesDictionnary::StatesDictionnary()
:title()
,evaluatedGroups()
,sections()
{

}
StatesDictionnary::StatesDictionnary(const StatesDictionnary& o)
{
  *this = o;
}
StatesDictionnary::~StatesDictionnary()
{
  
}

StatesDictionnary& StatesDictionnary::operator=(const StatesDictionnary& o)
{
  title = o.title;
  evaluatedGroups = o.evaluatedGroups;
  sections = o.sections;
  return *this;
}

Json::Value StatesDictionnary::dictionnaryJsonize() const{
  Json::Value js;
  js[JsField::title] = title;
  js[JsField::evaluation_groups] = Json::Value(Json::arrayValue);
  js[JsField::sections] = Json::Value(Json::arrayValue);

  std::map<std::string,EvaluationGroup>::const_iterator itg;
  std::map<std::string,DictionnarySection>::const_iterator its;

  for(itg = evaluatedGroups.begin();itg != evaluatedGroups.end();++itg)
  {
    js[JsField::evaluation_groups].append(itg->second.toJson());
  }
  for(its = sections.begin() ; its != sections.end();++its)
  {
    js[JsField::sections].append(its->second.toJson());
  }
  return js;
}

bool StatesDictionnary::dictionnaryFromJson(const Json::Value& js)
{
  if(
      (!js.isMember(JsField::title))
    ||(!js.isMember(JsField::evaluation_groups))
    ||(!js.isMember(JsField::sections))
  ){
    CLOG_ERROR("states-dictionnary : dict-from-json : not found all required fields ["
      <<JsField::evaluation_groups << ","
      << JsField::sections << "]");
    return false;
  }
  title = js[JsField::title].asString();
  if(
    (js[JsField::evaluation_groups].type() != Json::arrayValue) 
    || (js[JsField::sections].type() != Json::arrayValue)
  ){
    CLOG_ERROR("states-dictionnary : dict-from-json : not found all required fields are not arrays");
    return false;
  }
  Json::ArrayIndex i,s = js[JsField::evaluation_groups].size();
  evaluatedGroups.clear();
  std::map<std::string,EvaluationGroup>::iterator ite;
  for(i = 0;i < s;++i)
  {
    EvaluationGroup g;
    try{
      if(!g.fromJson(js[JsField::evaluation_groups][i]))
      {
        CLOG_ERROR("states-dictionnary : dict-from-json : error parsing group ((" << js[JsField::evaluation_groups][i] << "))");
        return false;
      }
    }catch(const Json::Exception& e){
      CLOG_ERROR("states-dictionnary : dict-from-json : exception parsing group "
        "(" << e.what() <<") parsing group ((" << js[JsField::evaluation_groups][i] <<"))");
      exit(1);
      return false;
    }catch(...){
      CLOG_ERROR("states-dictionnary : dict-from-json : exception parsing group "
        "(unknown-exception) parsing group ((" << js[JsField::evaluation_groups][i] <<"))");
      exit(1);
      return false;
    }
    ite = evaluatedGroups.find(g.id);
    if(ite != evaluatedGroups.end())
    {
      CLOG_ERROR("states-dictionnary : dict-from-json : item group [" << g.id << "] already exists!");
      return false;
    }
    CLOG_INFO("states-dictionaty : dict-from-json : pushing group item [" << g.id << "]");
    evaluatedGroups.insert(std::make_pair(g.id,g));
  }

  s = js[JsField::sections].size();
  std::map<std::string,DictionnarySection>::iterator its;
  for(i = 0;i < s;++i)
  {
    DictionnarySection d;
    try{

    
      if(!d.fromJson(js[JsField::sections][i]))
      {
        CLOG_ERROR("states-dictionnary : dict-from-json : error parsing section ((" << js[JsField::sections][i] << "))");
        return false;
      }
    }catch(const Json::Exception& e){
      CLOG_ERROR("states-dictionnary : dict-from-json : exception parsing (" << e.what() << ") from section ((" << i << "))");
      exit(1);
      return false;
    }catch(...){
      CLOG_ERROR("states-dictionnary : dict-from-json : exception parsing (unknown-exception) from section ((" << i << "))");
      exit(1);
      return false;
    }
    its = sections.find(d.id_);
    if(its != sections.end())
    {
      CLOG_ERROR("states-dictionnary : dict-from-json : section [" << d.id_ << "] already exists!");
      return false;
    }
    CLOG_INFO("states-dictionnary : fict-from-json : recorded section [" << d.id_ << "]");
    sections.insert(std::make_pair(d.id_,d));
  }

  return true;
}

bool StatesDictionnary::make_report_group_indexed(int64_t value,const std::string& sectionId,EvaluationGroupsResult& destination)
{
  
  std::map<std::string,DictionnarySection>::iterator its;
  destination.clear();
  CevalSetMap         cevalmap;
  CevalSetMapIterator cevalmapiter;
  
  ConceptEvaluation   ceval;
  
  std::map<std::string,DictionnarySection>::const_iterator it;
  if((it = sections.find(sectionId))== sections.end())
  {
    CLOG_WARN("states-dictionnary : make report group indexed : section [" 
      << sectionId << "] has not been found in this dictionnary");
    return false;
  }
  cevalmap.insert(std::make_pair(sectionId,it->second.evaluate(value)));
  
  for(const std::pair<std::string,ConceptEvaluationSet>& i : cevalmap)
  {
    CLOG_DEBUG(5,"states-dictionnary : make-report-group-indexed : " 
      << cevalmap.size() << " evaluated groups. for [" << i.first << "], there are [" << i.second.size() << "] solutions.");
  }

  for(const std::pair<std::string,EvaluationGroup>& s : evaluatedGroups)
  {
    EvaluationGroupCalculationResult result;

    s.second.calculate(cevalmap,result);
    destination.push(result);
  }
  return true;
}
bool StatesDictionnary::make_report_section(int64_t value,const std::string& sectionId,ConceptEvaluationSet& cevalset)
{

  std::map<std::string,DictionnarySection>::const_iterator cit;
  if((cit = sections.find(sectionId)) == sections.end())
  {
    CLOG_WARN("states-dictionnary : make-report-section : section [" << sectionId << "] not found");
    return false;
  }
  cevalset = cit->second.evaluate(value);
  return true;
}
bool StatesDictionnary::make_report_sections_all(int64_t value,CevalSetMap& cevalsetmap)
{
  cevalsetmap.clear();
  for(const std::pair<std::string,DictionnarySection>& secpair : sections)
  {
    ConceptEvaluationSet cevalset;
    make_report_section(value,secpair.first,cevalset);
    cevalsetmap.insert(std::make_pair(secpair.first,cevalset));
  }
  return true;
}

std::string StatesDictionnary::toStringLittle()
{
  std::stringstream ss;
  ss << "Dict: " << title
     << ";number of sections: " << sections.size()
     << ";number of groups: " << evaluatedGroups.size();
  return ss.str();
}

}
}



std::ostream& operator << (std::ostream& o, coyot3::tools::StateEvaluation s)
{
  return (o << coyot3::tools::StateEvaluationToString(s));
}