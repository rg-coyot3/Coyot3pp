#include <Coyot3pp/Cor3/base/cyt_value.hpp>


namespace coyot3{
namespace tools{

  const Value* Value::NULL_Value = nullptr;
  const char* Value::TypeToString(Type t)
  {
    switch(t)
    {
      case Type::NULLTYPE:return "NULLTYPE";break;
      case Type::BOOLEAN:return "BOOLEAN";break;
      case Type::INTEGER:return "INTEGER";break;
      case Type::UNSIGNED:return "UNSIGNED";break;
      case Type::FLOAT:return "FLOAT";break;
      case Type::STRING:return "STRING";break;
      case Type::err_type_:
      default:
        return "err_type";
    }
  }

  value_float_t Value::toNumber(const std::string& s){
    value_float_t r = 0.0;
    int base = 0;
    bool flo = false;
    for(char c : s)
    {
      if(c == ' ')
      {
        if(base==0)
          continue;
        else{
          return r;
        }
      }
      
      if(c >= '0' && c <= '9')
      {
        if(flo == false)
        {
          r+= static_cast<int>(c - '0');
          base++;
        }else{
          base*=10;
          r+= static_cast<value_float_t>(c - '0')/base;
        }
        
        continue;
      }
      //not a number... is it a decimal separator?
      if(c == '.' || c == ','){
        if(flo == false)
        {
          base = 1;
          flo = true;
          continue;
        }else{
          return r;
        }
      }
      //not a number, decimal separator or initial space.
      return r;
      
    }
    //never arrives ere but with an empty string or one space.
    return r;
  }

  Value::Value()
  :name_()
  ,name_f_(false)
  ,type_(Type::NULLTYPE)
  ,val_bool_(false)
  ,val_int_(0)
  ,val_uint_(0)
  ,val_flo_(0.0)
  ,val_str_(){
    
  }
  Value::Value(const Value& o)
  {
    *this = o;
  }
  Value::Value(bool o)
  :name_()
  ,name_f_(false)
  ,type_(Type::BOOLEAN)
  ,val_bool_(o)
  {

  }
  Value::Value(int o)
  :name_()
  ,name_f_(false)
  ,type_(Type::INTEGER)
  ,val_int_(static_cast<value_int_t>(o))
  {

  }


  Value::Value(value_int_t o)
  :name_()
  ,name_f_(false)
  ,type_(Type::INTEGER)
  ,val_int_(o)
  {

  }
  Value::Value(uint o)
  :name_()
  ,name_f_(false)
  ,type_(Type::UNSIGNED)
  ,val_uint_(static_cast<value_uint_t>(o))
  {

  }

  Value::Value(value_uint_t o)
  :name_()
  ,name_f_(false)
  ,type_(Type::UNSIGNED)
  ,val_uint_(o)
  {

  }
  Value::Value(float o)
  :name_()
  ,name_f_(false)
  ,type_(Type::FLOAT)
  ,val_flo_(static_cast<value_float_t>(o))
  {

  }
  Value::Value(double o)
  :name_()
  ,name_f_(false)
  ,type_(Type::FLOAT)
  ,val_flo_(static_cast<value_float_t>(o))
  {

  }
  Value::Value(value_float_t o)
  :name_()
  ,name_f_(false)
  ,type_(Type::FLOAT)
  ,val_flo_(o)
  {

  }
  Value::Value(const char* o)
  :name_()
  ,name_f_(false)
  ,type_(Type::STRING)
  {
    val_str_.str(std::string(o));  
  }
  Value::Value(const std::string& o)
  :name_()
  ,name_f_(false)
  ,type_(Type::STRING)
  ,val_str_(o)
  {

  }

  Value::Value(const Json::Value& o)
  {
    try{

    
      switch(o.type())
      {
        case Json::ValueType::nullValue:
          type_ = Type::NULLTYPE;
          break;
        case Json::ValueType::intValue:
          type_ = Type::INTEGER;
          val_int_ = o.asLargestInt();
          break;
        case Json::ValueType::uintValue:
          type_ = Type::UNSIGNED;
          val_uint_ = o.asLargestUInt();
          break;
        case Json::ValueType::realValue:
          type_ = Type::FLOAT;
          val_flo_ = o.asFloat();
          break;
        case Json::ValueType::stringValue:
          type_ = Type::STRING;
          val_str_ << o.asString();
          break;
        case Json::ValueType::booleanValue:
          type_ = Type::BOOLEAN;
          val_flo_ = o.asBool();
          break;  
        case Json::ValueType::arrayValue:
          {
            type_ = Type::ARRAY;
            Json::ArrayIndex index;
            Json::ArrayIndex length = o.size();
            for(index = 0; index < length;++index){
              val_varlist_.push_back(Value(o[index]));
            }
          }
          break;
        case Json::ValueType::objectValue:
          {
            type_ = Type::OBJECT;
            Json::Value::Members mnames = o.getMemberNames();
            for(std::string n:mnames){
              Value nv = o[n];
              val_varlist_.push_back(nv);
              val_varlist_.back().name(n);
            }
          }
          break;
        default:
          std::cout << "::coyot3::tools::Value::constructor(json-value) : "
            "unknown type!!! set to nulltype" << std::endl;
          type_ = Type::NULLTYPE;
      }
    }catch(const Json::Exception& e){
      std::cout << "::coyot3::tools::Value::constructor(json-value) : "
        "exception : " << e.what() << std::endl;
    }catch(...){
      std::cout << "::coyot3::tools::Value::constructor(json-value) : "
        "exception : unknown-exception" << std::endl;
    }
  }

  Value::~Value(){
    _clear_val_varlist();
  }


  Value& Value::append(const Value& o){
    
    if(type_ != Type::ARRAY){
      type_ = Type::ARRAY;
      val_varlist_.clear();
    }
    val_varlist_.push_back(o);
    return *this;
  }

  bool            Value::isNull() const
  {
    return (type_ == Type::NULLTYPE);
  }
  bool            Value::isBool() const
  {
    return (type_ == Type::BOOLEAN);
  }
  bool            Value::isInt() const
  {
    return (type_ == Type::INTEGER);
  }
  bool            Value::isUInt() const
  {
    return (type_ == Type::UNSIGNED); 
  }
  bool            Value::isFloat() const
  {
    return (type_ == Type::FLOAT);
  }
  bool            Value::isString() const 
  {
    return (type_ == Type::STRING);
  }
  bool            Value::isArray() const
  {
    return (type_ == Type::ARRAY);
  }
  bool            Value::isObject() const
  {
    return (type_ == Type::OBJECT);
  }

  
  Value& Value::operator=(const Value& o)
  {
    type_ = o.type_;
    val_bool_ = o.val_bool_;
    val_int_ = o.val_int_;
    val_uint_ = o.val_uint_;
    val_flo_ = o.val_flo_;
    val_str_.clear();
    val_str_ << o.val_str_.str();
    if(o.name_f_ == true){
      name_ = o.name_;
      name_f_ = o.name_f_;
          // std::cout << "copia " << std::endl;

    }
    // name_ = o.name_;
    // name_f_ = o.name_f_;

    _clear_val_varlist();
    for(const Value& ptr_: o.val_varlist_)
    {
      val_varlist_.push_back(ptr_);
    }
    return *this;
  }
  Value& Value::operator=(bool o)
  {
    name_.clear();
    type_ = Type::BOOLEAN;
    val_bool_ = o;
    return *this;
  }
  Value& Value::operator=(int o){
    return Value::operator=(value_int_t(o));
  }
  Value& Value::operator=(value_int_t o)
  {
    type_=Type::INTEGER;
    val_int_ = o;
    return *this;
  }
  Value& Value::operator=(value_uint_t o)
  {
   
    type_=Type::UNSIGNED;
    val_uint_ = o;
    return *this;
  }
  Value& Value::operator=(double o)
  {
    type_ = Type::FLOAT;
    val_flo_ = static_cast<value_float_t>(o);
    return *this;
  }
  Value& Value::operator=(value_float_t o)
  {
    
    type_ = Type::FLOAT;
    val_flo_ = o;
    return *this;
  }
  Value& Value::operator=(const char* o){
    
    type_ = Type::STRING;
    val_str_.str(std::string(o));
    return *this;
  }
  Value& Value::operator=(const std::string& o)
  {
    
    type_ = Type::STRING;
    val_str_.str(o);
    return *this;
  }

  Value& Value::operator=(const Json::Value& o)
  {
    *this = Value(o);
    return *this;
  }

  const Value::Type Value::type() const{
    return type_;
  }

  std::string Value::name() const
  {
    return name_;
  }
  bool Value::has_tag() const{
    return name_f_;
  }

  std::string Value::name(const std::string& n)
  {
    // std::cout << "set-name :" << n << std::endl;
    name_ = n;
    name_f_ = (name_.size() != 0);
    // std::cout << "name-f [" << name_f_ << "]" << std::endl;
    return name_;
  }

  void Value::toNull(){
    type_ = Type::NULLTYPE;
  }
  bool Value::asBool() const{
    
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        return false;
        break;
      case Value::Type::BOOLEAN:
        return val_bool_;
        break;
      case Value::Type::INTEGER:
        return (val_int_ != 0);
        break;
      case Value::Type::UNSIGNED:
        return (val_uint_ != 0);
        break;
      case Value::Type::FLOAT:
        return (val_flo_ != 0.0);
        break;
      case Value::Type::STRING:
        return (val_str_.str().size() != 0);
        break;
    }
    return false;
  }
  bool Value::asBool(bool& d) const{
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        d = false;
        break;
      case Value::Type::BOOLEAN:
        d = val_bool_;
        break;
      case Value::Type::INTEGER:
        d = (val_int_ != 0);
        break;
      case Value::Type::UNSIGNED:
        d = (val_uint_ != 0);
        break;
      case Value::Type::FLOAT:
        d = (val_flo_ != 0.0);
        break;
      case Value::Type::STRING:
        d = (val_str_.str().size() != 0);
        break;
      default:
        return false;
    }
    return true;
  }

  bool Value::toBool() {
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        val_bool_ = false;
        break;
      case Value::Type::BOOLEAN:
        val_bool_ = val_bool_;
        break;
      case Value::Type::INTEGER:
        val_bool_ = (val_int_ != 0);
        break;
      case Value::Type::UNSIGNED:
        val_bool_ = (val_uint_ != 0);
        break;
      case Value::Type::FLOAT:
        val_bool_ = (val_flo_ != 0.0);
        break;
      case Value::Type::STRING:
        val_bool_ = (val_str_.str().size() != 0);
        break;
    }
    type_ = Type::BOOLEAN;
    return val_bool_;
  }

  value_int_t Value::asInt() const{
    
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        return static_cast<value_int_t>(0);
        break;
      case Value::Type::BOOLEAN:
        return static_cast<value_int_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        return val_int_;
        break;
      case Value::Type::UNSIGNED:
        return static_cast<value_int_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        return static_cast<value_int_t>(val_flo_);
        break;
      case Value::Type::STRING:
        return static_cast<value_int_t>(Value::toNumber(val_str_.str()));
        break;
      case Value::Type::ARRAY:
      case Value::Type::OBJECT:
        return static_cast<value_int_t>(val_varlist_.size());
        break;
    }
    return static_cast<value_int_t>(0);
  }
  bool Value::asInt(value_int_t& d) const{
    
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        d = static_cast<value_int_t>(0);
        break;
      case Value::Type::BOOLEAN:
        d = static_cast<value_int_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        d = val_int_;
        break;
      case Value::Type::UNSIGNED:
        d = static_cast<value_int_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        d = static_cast<value_int_t>(val_flo_);
        break;
      case Value::Type::STRING:
        d = static_cast<value_int_t>(Value::toNumber(val_str_.str()));
        break;
      case Value::Type::ARRAY:
      case Value::Type::OBJECT:
        d = static_cast<value_int_t>(val_varlist_.size());
        break;
      default:
        return false;
    }
    return true;
  }
  value_int_t Value::toInt(){
    name_.clear();
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        val_int_ = static_cast<value_int_t>(0);
        break;
      case Value::Type::BOOLEAN:
        val_int_ = static_cast<value_int_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        val_int_ =  val_int_;
        break;
      case Value::Type::UNSIGNED:
        val_int_ =  static_cast<value_int_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        val_int_ = static_cast<value_int_t>(val_flo_);
        break;
      case Value::Type::STRING:
        val_int_ = static_cast<value_int_t>(Value::toNumber(val_str_.str()));
        break;
      case Value::Type::ARRAY:
      case Value::Type::OBJECT:
        val_int_ = static_cast<value_int_t>(val_varlist_.size());
        break;
    }
    type_ = Type::INTEGER;
    return val_int_;
  }
  value_uint_t Value::asUInt() const{
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        return static_cast<value_uint_t>(0);
        break;
      case Value::Type::BOOLEAN:
        return static_cast<value_uint_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        return static_cast<value_uint_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        return val_uint_;
        break;
      case Value::Type::FLOAT:
        return static_cast<value_uint_t>(val_flo_);
        break;
      case Value::Type::STRING:
        return static_cast<value_uint_t>(Value::toNumber(val_str_.str()));
        break;
    }
    return static_cast<value_uint_t>(0);
  }

  bool Value::asUInt(value_uint_t& d) const{
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        d = static_cast<value_uint_t>(0);
        break;
      case Value::Type::BOOLEAN:
        d = static_cast<value_uint_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        d = static_cast<value_uint_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        d = val_uint_;
        break;
      case Value::Type::FLOAT:
        d = static_cast<value_uint_t>(val_flo_);
        break;
      case Value::Type::STRING:
        d = static_cast<value_uint_t>(Value::toNumber(val_str_.str()));
        break;
      default:
        return false;
    }
    return true;
  }

  bool            Value::set(bool& v) const{
    if(isNull() || isObject() || isArray())return false;
    v = asBool();
    return true;
  }
  bool            Value::set(value_int_t& v) const{
    if(isNull() || isObject() || isArray())return false;
    v = asInt();
    return true;
  }
  bool            Value::set(value_uint_t& v) const{
    if(isNull() || isObject() || isArray())return false;
    v = asUInt();
    return true;
  }
  bool            Value::set(std::string& v) const{
    if(isNull() || isObject() || isArray())return false;
    v = asString();
    return true;
  }
  bool            Value::set(value_float_t& v) const{
    if(isNull() || isObject() || isArray())return false;
    v = asFloat();
    return true;
  }

  value_uint_t Value::toUInt(){
    name_.clear();
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        val_uint_ = static_cast<value_uint_t>(0);
        break;
      case Value::Type::BOOLEAN:
        val_uint_ = static_cast<value_uint_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        val_uint_ = static_cast<value_uint_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        val_uint_ = val_uint_;
        break;
      case Value::Type::FLOAT:
        val_uint_ = static_cast<value_uint_t>(val_flo_);
        break;
      case Value::Type::STRING:
        val_uint_ = static_cast<value_uint_t>(Value::toNumber(val_str_.str()));
        break;
    }
    type_=Type::UNSIGNED;
    return val_uint_;
  }
  value_float_t Value::asFloat() const{
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        return static_cast<value_float_t>(0);
        break;
      case Value::Type::BOOLEAN:
        return static_cast<value_float_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        return static_cast<value_float_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        return static_cast<value_float_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        return val_flo_;
        break;
      case Value::Type::STRING:
        return Value::toNumber(val_str_.str());
        break;
    }
    return static_cast<value_float_t>(0.0);
  }

  bool Value::asFloat(value_float_t& d) const{

    switch(type_)
    {
      case Value::Type::NULLTYPE:
        d = static_cast<value_float_t>(0);
        break;
      case Value::Type::BOOLEAN:
        d = static_cast<value_float_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        d = static_cast<value_float_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        d = static_cast<value_float_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        d = val_flo_;
        break;
      case Value::Type::STRING:
        d = Value::toNumber(val_str_.str());
        break;
      default:
        return false;
    }
    return true;
  }
  value_float_t Value::toFloat() {
    name_.clear();
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        val_flo_ = static_cast<value_float_t>(0);
        break;
      case Value::Type::BOOLEAN:
        val_flo_ = static_cast<value_float_t>((val_bool_ == false?0:1));
        break;
      case Value::Type::INTEGER:
        val_flo_ = static_cast<value_float_t>(val_int_);
        break;
      case Value::Type::UNSIGNED:
        val_flo_ = static_cast<value_float_t>(val_uint_);
        break;
      case Value::Type::FLOAT:
        val_flo_ = val_flo_;
        break;
      case Value::Type::STRING:
        val_flo_ = Value::toNumber(val_str_.str());
        break;
    }
    type_ = Type::FLOAT;
    return val_flo_;
  }

  std::string Value::asString() const{

    switch(type_)
    {
      case Value::Type::NULLTYPE:
        return "null";
        break;
      case Value::Type::BOOLEAN:
        return (val_bool_ == true?"true":"false");
        break;
      case Value::Type::INTEGER:
        return std::to_string(val_int_);
        break;
      case Value::Type::UNSIGNED:
        return std::to_string(val_uint_);
        break;
      case Value::Type::FLOAT:
        return std::to_string(val_flo_);
        break;
      case Value::Type::STRING:
        return val_str_.str();
        break;
    }
    return "";
  }

  bool Value::asString(std::string& d) const{
    
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        d = "null";
        break;
      case Value::Type::BOOLEAN:
        d = (val_bool_ == true?"true":"false");
        break;
      case Value::Type::INTEGER:
        d = std::to_string(val_int_);
        break;
      case Value::Type::UNSIGNED:
        d = std::to_string(val_uint_);
        break;
      case Value::Type::FLOAT:
        d = std::to_string(val_flo_);
        break;
      case Value::Type::STRING:
        d = val_str_.str();
        break;
      case Value::Type::ARRAY:
      {
        bool isFirst = true;
        std::stringstream sstr;
        for(const Value& v : val_varlist_){
          if(isFirst)isFirst = false; else sstr << ",";
          sstr << v.str();
        }
        d = sstr.str();
      }
      case Value::Type::OBJECT:
      {
        bool isFirst = true;
        std::stringstream sstr;
        for(const Value& v : val_varlist_){

          if(isFirst)isFirst = false; else sstr << ",";
          sstr << v.name() << "=" << v.str();
        }
        d = sstr.str();
      }
      default:
        return false;
    }
    return true;
  }

  std::string Value::toString(){
    if(type_==Value::Type::STRING){
      return val_str_.str();
    }
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        val_str_ << "null";
        break;
      case Value::Type::BOOLEAN:
        val_str_ << (val_bool_ == true?"true":"false");
        break;
      case Value::Type::INTEGER:
        val_str_ << std::to_string(val_int_);
        break;
      case Value::Type::UNSIGNED:
        val_str_ << std::to_string(val_uint_);
        break;
      case Value::Type::FLOAT:
        val_str_ << std::to_string(val_flo_);
        break;
      case Value::Type::OBJECT:
      {
        bool isFirst = true;
        for(const Value& pt : val_varlist_)
        {
          if(isFirst)isFirst = false;else val_str_<<",";
          val_str_<< pt.name() << "=";
          val_str_<< pt.asString();
        }
      }
      case Value::Type::ARRAY:
      {
        bool isFirst = true;
        for(const Value& pt : val_varlist_)
        {
          if(isFirst)isFirst = false;else val_str_<<",";
          val_str_<< pt.asString();
        }
        break;
      }
    }
    type_= Type::STRING;
    return val_str_.str();
  }

  Value Value::operator+(const Value& o) const
  {
    switch(o.type()){
       case Type::NULLTYPE:
        return Value();
        break;
      case Type::BOOLEAN:
        return Value(static_cast<bool>(val_bool_ & o.asBool()));
        break;
      case Type::INTEGER:
        return Value(val_int_ + o.asInt());
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + o.asUInt());
        break;
      case Type::FLOAT:
        return Value(val_flo_ + o.asFloat());
        break;
      case Type::STRING:
        return Value(val_str_.str() + o.asString());
        break;
    }
    return Value();
  }
  Value Value::operator+(bool o) const
  {
    switch(type_){
      case Type::NULLTYPE:
        return Value();     
        break;
      case Type::BOOLEAN:
        return Value(val_bool_ || o);
        break;
      case Type::INTEGER:
        return Value(val_int_ + static_cast<value_int_t>(o));
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + static_cast<value_uint_t>(o));
        break;
      case Type::FLOAT:
        return Value(val_flo_ + static_cast<value_float_t>(o));
        break;
      case Type::STRING:
        return Value(val_str_.str() + (o == true?"true":"false"));
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator+(bool) : type not managed" << std::endl;
    return Value();
  }
  Value Value::operator+(value_int_t o) const
  {
    switch(type_){
      case Type::NULLTYPE:
        return Value();     
        break;
      case Type::BOOLEAN:
        return Value(val_bool_ || (o != 0));
        
        break;
      case Type::INTEGER:
        return Value(val_int_ + o);
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + static_cast<value_uint_t>(o) );
        break;
      case Type::FLOAT:
        return Value(val_flo_ + static_cast<value_float_t>(o));
        break;
      case Type::STRING:
        return Value(val_str_.str() + std::to_string(o));
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator+(int) : type not managed" << std::endl;
    return Value();
  }
  Value Value::operator+(value_uint_t o) const
  {
    switch(type_){
      case Type::NULLTYPE:
        return Value();     
        break;
      case Type::BOOLEAN:
        return Value(val_bool_ || (o != 0));
        break;
      case Type::INTEGER:
        return Value(val_int_ + static_cast<value_int_t>(o));
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + o);
        
        break;
      case Type::FLOAT:
        return Value(val_flo_ + static_cast<value_float_t>(o));
        break;
      case Type::STRING:
        return Value(val_str_.str() + std::to_string(o));
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator+(uint) : type not managed" << std::endl;
    return Value();
  }
  Value Value::operator+(value_float_t o) const
  {
    switch(type_){
      case Type::NULLTYPE:
        return Value();     
        break;
      case Type::BOOLEAN:
        return Value(val_bool_ || (o != 0.0));
        break;
      case Type::INTEGER:
        return Value(val_int_ + static_cast<value_int_t>(o));
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + o);
        break;
      case Type::FLOAT:
        return Value(val_flo_ + o);
        break;
      case Type::STRING:
        return Value(val_str_.str() + std::to_string(o));   
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator+(float) : type not managed" << std::endl;
    return *this;
  }


  Value Value::operator+(const std::string& o) const
  {
    value_float_t buff;

    switch(type_){
      case Type::NULLTYPE:
        return Value();     
        break;
      case Type::BOOLEAN:
        return Value(val_bool_ || (Value::toNumber(o) != 0));
        break;
      case Type::INTEGER:
        return Value(val_int_ + static_cast<value_int_t>(Value::toNumber(o)));
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ + static_cast<value_uint_t>(Value::toNumber(o)));
        break;
      case Type::FLOAT:
        return Value(val_flo_ + Value::toNumber(o));
        break;
      case Type::STRING:
        return Value(val_str_.str() + o);
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator+(integer) : type not managed" << std::endl;
    return *this;
  }

  Value Value::operator-(const Value& o) const
  {
    
    switch(type_){
      case Type::NULLTYPE:
        return Value();
        break;
      case Type::BOOLEAN:
        if(o.asBool() == true)
        {
          return Value(false);
        }
        return Value(val_bool_);
        break;
      case Type::INTEGER:
        return Value(val_int_ - o.asInt());
        break;
      case Type::UNSIGNED:
        return Value(val_uint_ - o.asUInt());
        break;
      case Type::FLOAT:
        return Value(val_flo_ - o.asFloat());
        break;
      case Type::STRING:
        return Value(val_str_.str().substr(0,val_str_.str().size() - o.asString().size()));
        break;
    }
    // std::cout << "::coyot3::tools::Value::operator-(Value) : type not managed" << std::endl;
    return Value();
  }


Value& Value::operator+=(const Value& o){
    return *this = (*this + o);
}
Value& Value::operator-=(const Value& o){
  return *this = (*this - o);
}

Json::Value Value::to_json() const
{
  Json::Value d;
  Json::Value r;
  
  
  switch(type_)
  { 
    case coyot3::tools::Value::Type::BOOLEAN:
      d = asBool();
      break;
    case coyot3::tools::Value::Type::INTEGER:
      d = static_cast<Json::LargestInt>(asInt());
      break;
    case coyot3::tools::Value::Type::UNSIGNED:
      d = static_cast<Json::LargestUInt>(asUInt());
      break;
    case coyot3::tools::Value::Type::FLOAT:
      d = static_cast<double>(asFloat());
      break;
    case coyot3::tools::Value::Type::STRING:
      d = asString();
      break;    
    case coyot3::tools::Value::Type::ARRAY:
      d = Json::Value(Json::arrayValue);
      for(Value vp : val_varlist_){
        d.append(vp.to_json());
      }
      break;
    case coyot3::tools::Value::Type::OBJECT:
    {
      std::cout << "debug - toJson: object" << std::endl;
      int unk_ = 0;
      for(const Value& vp : val_varlist_){
        std::string n = vp.name();
        if(n.size() == 0)
        {
          n = std::string("unknown_") + std::to_string(unk_++);
        }
        d[n] = vp.to_json();
      }

      break;
    }
    case coyot3::tools::Value::Type::NULLTYPE:
    default:
      d = Json::Value();
  }


  return d;
}




  bool Value::from_json(const Json::Value& o){
    try{

    
      switch(o.type())
      {
        case Json::ValueType::nullValue:
          type_ = Type::NULLTYPE;
          break;
        case Json::ValueType::intValue:
          type_ = Type::INTEGER;
          val_int_ = o.asLargestInt();
          break;
        case Json::ValueType::uintValue:
          type_ = Type::UNSIGNED;
          val_uint_ = o.asLargestUInt();
          break;
        case Json::ValueType::realValue:
          type_ = Type::FLOAT;
          val_flo_ = o.asFloat();
          break;
        case Json::ValueType::stringValue:
          type_ = Type::STRING;
          val_str_ << o.asString();
          break;
        case Json::ValueType::booleanValue:
          type_ = Type::BOOLEAN;
          val_flo_ = o.asBool();
          break;  
        case Json::ValueType::arrayValue:
          {
            type_ = Type::ARRAY;
            Json::ArrayIndex index;
            Json::ArrayIndex length = o.size();
            for(index = 0; index < length;++index)
            {
              val_varlist_.push_back(new Value(o[index]));
            }
          }
          break;
        case Json::ValueType::objectValue:
          {
            type_ = Type::OBJECT;
            Json::Value::Members mnames = o.getMemberNames();
            for(std::string n:mnames)
            {
              Value nv = o[n];
              val_varlist_.push_back(nv);
              val_varlist_.back().name(n);
            }
          }
          break;
        default:
          std::cout << "::coyot3::tools::Value::constructor(json-value) : "
            "unknown type!!! set to nulltype" << std::endl;
          type_ = Type::NULLTYPE;
      }
    }catch(const Json::Exception& e){
      return false;
    }catch(...){
      return false;
    }
    return true;
  }


Value& Value::operator[](const std::string& n){
  return get_member(n);
}

const Value& Value::operator[](const std::string& n) const{
  return get_member(n);
}



#define CYTvalue_MAX_ARRAY_SIZE 10000
Value& Value::operator[](const std::size_t& index){
  Value p;
  std::size_t idx = (index > CYTvalue_MAX_ARRAY_SIZE?CYTvalue_MAX_ARRAY_SIZE:index);
  if(idx < val_varlist_.size())
  {
    return val_varlist_[idx];
  }else{
    std::size_t i,l = val_varlist_.size();
    for(i = (l+1-idx);i<idx;++i)
    {
      val_varlist_.push_back(Value());
    }
    
  }
  return val_varlist_.back();
}

const Value& Value::operator[](std::size_t index) const{
  
  std::size_t idx = (index > CYTvalue_MAX_ARRAY_SIZE?CYTvalue_MAX_ARRAY_SIZE:index);
  if(idx < val_varlist_.size())
  {
    return val_varlist_[idx];
  }else{
    std::size_t i,l = val_varlist_.size();
    for(i = (l+1-idx);i<idx;++i)
    {
      val_varlist_.push_back(Value());
    }
    
  }
  return val_varlist_.back();
}



std::size_t Value::size() const {
  if(type_ != Type::ARRAY && type_ != Type::OBJECT)return -1;
  return val_varlist_.size();
}

bool Value::has_member(const std::string& m) const {
  if(type_ != Type::OBJECT)
  {
    return false;
  }
  for(const Value& p : val_varlist_)
  {
    if(m.compare(p.name()) == 0)
    {
      return true;
    }
  }
  return false;
}
Value& Value::get_member(const std::string& m){
  // std::cout << "get-member" << std::endl;
  if(type_ != Type::OBJECT){
    type_ = Type::OBJECT;
    val_varlist_.clear(); 
  }
  for(Value& ref : val_varlist_)
  {
    if(m.compare(ref.name()) == 0)
    {
      return ref;
    }
  }
  val_varlist_.push_back(Value());
  val_varlist_.back().name(m);
  return val_varlist_.back();
}

const Value& Value::get_member(const std::string& m) const{
  if(type_ != Type::OBJECT){
    type_ = Type::OBJECT;
    val_varlist_.clear();
    Value v;v.name(m);
    val_varlist_.push_back(v);
    return val_varlist_.back();
  }
  for(Value& ref : val_varlist_)
  {
    if(m.compare(ref.name()) == 0)
    {
      return ref;
    }
  }
  Value p;
  p.type_ = Type::NULLTYPE;
  p.name(m);
  val_varlist_.push_back(p);
  return val_varlist_.back();
}


void Value::clear()
{
  val_varlist_.clear();
  type_ = Type::NULLTYPE;
}


int Value::_clear_val_varlist(){
  int ni = val_varlist_.size();
  val_varlist_.clear();
  return ni;
}


void Value::each(std::function<void(const Value&)> f) const{
  for(const Value& v : val_varlist_){
    f(v);
  }
}
void Value::each(std::function<void(Value&)> f){
  for(Value& v : val_varlist_){
    f(v);
  }
}


  std::string Value::str() const{
    std::stringstream sstr;
    if(name_f_ == true){
      // std::cout << "str-name-f is true" << std::endl;
      sstr << name_ << "=";
    }
    switch(type_)
    {
      case Value::Type::NULLTYPE:
        sstr << "null";
      break;
      case Value::Type::BOOLEAN:
        sstr << (val_bool_ == true?"true":"false");
      break;
      case Value::Type::INTEGER:
        sstr << std::to_string(val_int_);
      break;
      case Value::Type::UNSIGNED:
        sstr << std::to_string(val_uint_);
      break;
      case Value::Type::FLOAT:
        sstr << std::to_string(val_flo_);
      break;
      case Value::Type::STRING:
        sstr << val_str_.str();
      break;
      case Value::Type::ARRAY:
      
      {
        sstr << "[";
        bool isFirst=true;
        for(const Value& v : val_varlist_){
          if(isFirst)isFirst = false; else sstr << ",";
          sstr << v.str();
        }
        sstr << "]";
      }
      break;
      case Value::Type::OBJECT:
      {
        sstr << "{";
        bool isFirst=true;
        for(const Value& v : val_varlist_){
          if(isFirst)isFirst = false; else sstr << ",";
          sstr << v.str();
        }
        sstr << "}";
      }
      break;
      default:
        return "value-str-ERROR!";
    }
    return sstr.str();
    
  }


}//eons 
}//eons


// Json::Value& operator<<(Json::Value& d,const coyot3::tools::Value& s)
// {
//   if(s.name().size()== 0)
//   {

  
//     switch(s.type())
//     { 
//       case coyot3::tools::Value::Type::BOOLEAN:
//         d = s.asBool();
//         break;
//       case coyot3::tools::Value::Type::INTEGER:
//         d = static_cast<Json::LargestInt>(s.asInt());
//         break;
//       case coyot3::tools::Value::Type::UNSIGNED:
//         d = static_cast<Json::LargestUInt>(s.asUInt());
//         break;
//       case coyot3::tools::Value::Type::FLOAT:
//         d = static_cast<double>(s.asFloat());
//         break;
//       case coyot3::tools::Value::Type::STRING:
//         d = s.asString();
//         break;
//       case coyot3::tools::Value::Type::OBJECT:
//       case coyot3::tools::Value::Type::ARRAY:
//         {
//           std::size_t index=0,l = s.size();
//           for(index = 0;index < l;index++){
//             Json::Value b;
//             b << s[index];
//           }
//         }
//         break;
//       case coyot3::tools::Value::Type::NULLTYPE:
//       default:
//         d = Json::Value();
//     }
//   }
//   return d;
// }


std::ostream& operator<<(std::ostream& o, const coyot3::tools::Value& s){
  switch(s.type()){
    case coyot3::tools::Value::Type::INTEGER:
      return o << s.asInt();
      break;
    case coyot3::tools::Value::Type::STRING:
      return o << s.asString();
      break;
    case coyot3::tools::Value::Type::FLOAT:
      return o << s.asFloat();
      break;
    case coyot3::tools::Value::Type::UNSIGNED:
      return o << s.asUInt();
      break;
    case coyot3::tools::Value::Type::ARRAY:
    {
      o<< "[";
      bool isFirst = true;
      s.each([&](const coyot3::tools::Value& v){
        if(isFirst)isFirst=false;else o << ",";
        o << v;
      });
      return o;
    }
    break;
    case coyot3::tools::Value::Type::OBJECT:
    {
      o << "{";
      bool isFirst = true;
      s.each([&](const coyot3::tools::Value& v){
        if(isFirst)isFirst = false;else o << ",";
        o << v.name() << "=" << v;
      });
    }
    break;
    default :
      o << "coyot3-value-ERROR";
  }
  return o;
}

// std::ostream& operator<<(std::ostringstream& o, const coyot3::tools::Value& s){
//   switch(s.type()){
//     case coyot3::tools::Value::Type::INTEGER:
//       return o << s.asInt();
//       break;
//     case coyot3::tools::Value::Type::STRING:
//       return o << s.asString();
//       break;
//     case coyot3::tools::Value::Type::FLOAT:
//       return o << s.asFloat();
//       break;
//     case coyot3::tools::Value::Type::UNSIGNED:
//       return o << s.asUInt();
//       break;
//     case coyot3::tools::Value::Type::ARRAY:
//     {
//       o<< "[";
//       bool isFirst = true;
//       s.each([&](const coyot3::tools::Value& v){
//         if(isFirst)isFirst=false;else o << ",";
//         o << v;
//       });
//       return o;
//     }
//     break;
//     case coyot3::tools::Value::Type::OBJECT:
//     {
//       o << "{";
//       bool isFirst = true;
//       s.each([&](const coyot3::tools::Value& v){
//         if(isFirst)isFirst = false;else o << ",";
//         o << v.name() << "=" << v;
//       });
//     }
//     break;
//     default :
//       o << "coyot3-value-ERROR";
//   }
//   return o;
// }