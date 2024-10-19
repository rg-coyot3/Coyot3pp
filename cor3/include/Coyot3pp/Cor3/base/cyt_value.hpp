#pragma once


#include <stdio.h>
#include <cmath>
#include <limits.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <functional>





namespace coyot3{

namespace tools{
  typedef  int64_t               value_int_t;
  typedef  uint64_t              value_uint_t;
  typedef  long double           value_float_t;



  class Value;
  typedef  std::vector<Value> value_list_t;


  /**
   * @brief Wrapper to be able to work with low typed values, in the way that
   *        json does.
   * 
   */

  class Value{
    public:
      static const Value* NULL_Value;
      enum class Type{
        NULLTYPE = 0,
        BOOLEAN = 1,
        INTEGER = 2,
        UNSIGNED = 3,
        FLOAT  = 4,
        STRING = 5,

        ARRAY = 6,
        OBJECT = 7,

        err_type_ = -1
      };
      static const char* TypeToString(Type t);
      static value_float_t toNumber(const std::string& s);



      Value();
      Value(const Value& o);
      Value(bool o);
      Value(int o);
      Value(value_int_t o);
      Value(uint o);

      Value(value_uint_t o);
      Value(float o);
      Value(double o);
      Value(value_float_t o);
      Value(const char* o);
      Value(const std::string& o);
      Value(const Json::Value& o);

      virtual ~Value();


      std::string str() const;

      const Type      type() const;
      std::string     name() const;
      bool            has_tag() const;
      /**
       * @brief: transforms the item in object, automatically.
      */
      std::string     name(const std::string& n);
      void            clear();

      Value&          append(const Value& o);

      bool            asBool()   const;
      value_int_t     asInt()    const;
      value_uint_t    asUInt()   const;
      value_float_t   asFloat()  const;
      std::string     asString() const;

      bool            asBool(bool& d)             const;
      bool            asInt(value_int_t& d)     const;
      bool            asUInt(value_uint_t& d)   const;
      bool            asFloat(value_float_t& d) const;
      bool            asString(std::string& d)    const;

      bool            isNull()   const;
      bool            isBool()   const;
      bool            isInt()    const;
      bool            isUInt()   const;
      bool            isFloat()  const;
      bool            isString() const;
      bool            isArray()  const;
      bool            isObject() const;

      bool            set(bool& v) const;
      bool            set(value_int_t& v) const;
      bool            set(value_uint_t& v) const;
      bool            set(std::string& v) const;
      bool            set(value_float_t& v) const;
      template<typename T>
      bool            set(std::vector<T>& v) const{
        if(!isArray())return false;
        bool allgood = true;
        for(const Value& i : val_varlist_){
          T val;
          if(!i.set(val)){allgood = false; continue;}
          v.push_back(val); 
        }
        return allgood;
      }
      

      void            toNull();
      bool            toBool();
      value_int_t     toInt();
      value_uint_t    toUInt();
      value_float_t   toFloat();
      std::string     toString();



      Json::Value     to_json() const;
      bool            from_json(const Json::Value& o);

      std::size_t     size() const;      
      bool            has_member(const std::string& m) const;
      Value&          get_member(const std::string& m);
      const Value&    get_member(const std::string& m) const;

      Value& operator=(const Value& o);
      Value& operator=(bool o);
      Value& operator=(int o);
      Value& operator=(value_int_t o);
      Value& operator=(value_uint_t o);
      Value& operator=(double o);
      Value& operator=(value_float_t o);
      Value& operator=(const char* o);
      Value& operator=(const std::string& o);

      Value& operator=(const Json::Value& o);
      
      template<typename T>
      Value& operator=(const std::vector<T>& o){
        if(type_ != Type::ARRAY){
          type_ = Type::ARRAY;
          val_varlist_.clear();
        }
        for(const T& v : o){
          Value i = v;
          val_varlist_.push_back(v);
        }
        return *this;
      }

      template<typename T>
      Value& operator=(const std::map<std::string,T>& o){
        if(type_ != Type::OBJECT){
          type_ = Type::OBJECT;
          val_varlist_.clear();
        }
        for(const std::pair<std::string,T>& v : o){
          Value i = v.second;
          val_varlist_.push_back(i);
          val_varlist_.back().name(v.first);
        }
        return *this;
      }

      
      Value operator+(const Value& o) const;
      Value operator+(bool o) const;
      Value operator+(value_int_t o) const;
      Value operator+(value_uint_t o) const;
      Value operator+(value_float_t o) const;
      Value operator+(const std::string& o) const;

      Value& operator+=(const Value& o);

      Value operator-(const Value& o) const;
      Value& operator-=(const Value& o);

      Value& operator[](const std::string& n);
      Value& operator[](const std::size_t& index);


      const Value& operator[](const std::string& n) const;
      const Value& operator[](std::size_t index) const;

      void each(std::function<void(const Value&)> f) const;
      void each(std::function<void(Value&)> f);

    protected:
      std::string             name_;
      bool                    name_f_;

      mutable Type            type_;
      bool                    val_bool_;
      value_int_t             val_int_;
      value_uint_t            val_uint_;
      value_float_t           val_flo_;
      std::stringstream       val_str_;
      
      mutable value_list_t    val_varlist_;


      int               _clear_val_varlist();

  };


  
}
}

//Json::Value& operator<<(Json::Value& d,const coyot3::tools::Value& s);
std::ostream& operator<<(std::ostream& o, const coyot3::tools::Value& s);
//std::ostream& operator<<(std::ostringstream& o, const coyot3::tools::Value& s);
//coyot3::tools::value_int_t operator=(coyot3::tools::value_int_t d, const coyot3::tools::Value& s);