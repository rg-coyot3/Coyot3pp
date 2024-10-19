#pragma once
#include "cyt_foreach_macro_helpers.hpp"


/**
 * CYT3MACRO_enum_class_declarations 
 * CYT3MACRO_enum_class_definitions
 *    This macro declares an enum class containing the set of states as 
 *      described in its inputs.
 * 
 * It will create the following methods associated to the enum-class:
 *  - static const char* [[OwnerClass]::<name>ToString(<name> input);
 *        it returns a stringified input.
 *  - static const [[OwnerClass]::<name> [[OwnerClass]::<name>FromString(const char* input):
 *        it returns the enum state from the string
 *  - ::std::operator << overload that sends the stringification of the state
 * 
 * 
 *  Example:
 *    at .h/.hpp
 * 
 *  CYT3MACRO_enum_class_declarations 
 * 
 * 
*/






/**
 * @brief : declares an enum class as <CY_enum_class_name>. Arguments are the states. 
 *  States can contain default values. Predefines INTERNAL_ERROR(-2) 
 *  and UNKNOWN_UNSET(-1). Also declares:
 *  const char* <CY_enum_class_name>ToString(<CY_enum_class_name>)
 *  const <CCY_enum_class_name> <CY_enum_class_name>FromString(const char* s)
 * @param CY_enum_class_name : enum class name
 * @param ... each state of the enum class. Can contain the predefined Value (i.e: STATE_1 = 1, STATE_2, STATE_3)
*/
#define CYT3MACRO_enum_class_declarations(CY_enum_class_name, CY_ownerclass, ...) \
  IFE(CY_ownerclass)(namespace ec{)\
    enum class CY_enum_class_name { \
      INTERNAL_ERROR = -2,\
      UNKNOWN_OR_UNSET = -1,\
      __VA_ARGS__ \
    };\
    const char* CY_enum_class_name##ToString( CY_enum_class_name s);\
    CY_enum_class_name CY_enum_class_name##FromString( const char* s);\
    bool CY_enum_class_name##CastCheck(CY_enum_class_name s);\
    IFE(CY_ownerclass)(::std::ostream& operator<<(::std::ostream& o,const CY_enum_class_name& s);) \
    IFE(CY_ownerclass)(int& operator<<(int& o,CY_enum_class_name s);) \
    IFE(CY_ownerclass)(CY_enum_class_name& operator<<(CY_enum_class_name& o,int s);) \
    IFE(CY_ownerclass)(CY_enum_class_name& operator<<(CY_enum_class_name& o, const std::string& i);)\
  IFE(CY_ownerclass)(})\






////////////////////////////////////////////////
////////////////////////////////////////////////
    #define cyt3macro_prvdef_enum_class_tostring_case(V_CONSTANT, V_CASE)\
      case V_CONSTANT::V_CASE: return #V_CASE; break;

    #define cyt3macro_prvdef_enum_class_fromstring_case(V_CONSTANT, V_CASE)\
      if(strcmp(V_CONSTANT##ToString(V_CONSTANT::V_CASE),s) == 0){return V_CONSTANT::V_CASE;}

    #define cyt3macro_prvdef_enum_class_castcheck_case(V_CONSTANT, V_CASE)\
      case V_CONSTANT::V_CASE: 






  #define cyt3macro_enum_class_tostring_def_(CY_enum_class_name, CY_ownerclass, ...)\
    const char* IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name##ToString( CY_enum_class_name s)\
      {\
        switch(s){\
          FOR_EACH_WITH_CONSTANT(cyt3macro_prvdef_enum_class_tostring_case, \
                                  IFN(CY_ownerclass)(CY_ownerclass::)\
                                  CY_enum_class_name, __VA_ARGS__) \
          case IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::INTERNAL_ERROR: \
            return "INTERNAL_ERROR"; break; \
          case IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::UNKNOWN_OR_UNSET: \
            return "UNKNOWN_OR_UNSET"; break; \
          default: \
            return "err_" #CY_enum_class_name "_unknown_state"; \
        } \
    }

  #define cyt3macro_enum_class_fromstring_def_(CY_enum_class_name, CY_ownerclass, ...)\
  IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name \
  IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name##FromString( const char* s){\
      FOR_EACH_WITH_CONSTANT(cyt3macro_prvdef_enum_class_fromstring_case, IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name, __VA_ARGS__) \
      if(strcmp(IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name##ToString(IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::UNKNOWN_OR_UNSET),s) == 0){return IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::UNKNOWN_OR_UNSET;} \
      return IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::INTERNAL_ERROR; \
    }

  #define cyt3macro_enum_class_castcheck_def_(CY_enum_class_name, CY_ownerclass, ...)\
  \
  IFN(CY_ownerclass)(const ) bool IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name##CastCheck( IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name s){\
    switch(s){\
      FOR_EACH_WITH_CONSTANT(cyt3macro_prvdef_enum_class_castcheck_case, \
                                  IFN(CY_ownerclass)(CY_ownerclass::)\
                                  CY_enum_class_name, __VA_ARGS__) \
      case IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::UNKNOWN_OR_UNSET: \
        return true;\
        break;\
      case IFN(CY_ownerclass)(CY_ownerclass::)CY_enum_class_name::INTERNAL_ERROR: \
      default: \
        return false;\
    }\
  }



    #define cyt3macro_enum_class_opsoverloads_def_(CY_enum_class_name, CY_ownerclass)\
      ::std::ostream& operator<<(::std::ostream& o,const CY_enum_class_name& s){ \
        return (o << CY_enum_class_name##ToString(s)); }\
      int& operator<<(int& o, CY_enum_class_name s){return (o = static_cast<int>(s));} \
      CY_enum_class_name& operator<<(CY_enum_class_name& o,int s){return( o = static_cast<CY_enum_class_name>(s));} \
      CY_enum_class_name& operator<<(CY_enum_class_name& o,const std::string& i){return( o = CY_enum_class_name##FromString(i.c_str()));} \
    

  

/**
 * @brief definitions for the cyt3macro-enum-class.
 * @param CY_ownerclass : owner class. CAN BE EMPTY
 * @param CY_enum_class_name : enum type name 
 * @param ... set of values
 *  
*/
#define CYT3MACRO_enum_class_definitions(CY_enum_class_name, CY_ownerclass, ...)\
    \
    IFE(CY_ownerclass)( namespace ec{ )\
      \
      cyt3macro_enum_class_tostring_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
      \
      cyt3macro_enum_class_fromstring_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
      \
      cyt3macro_enum_class_castcheck_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
      \
      IFE(CY_ownerclass)(cyt3macro_enum_class_opsoverloads_def_(CY_enum_class_name, CY_ownerclass))\
      \
    IFE(CY_ownerclass)( } )


// #define CYT3MACRO_enum_class_definitions(CY_enum_class_name, CY_ownerclass, ...)\
//   IFE(CY_ownerclass)(namespace ec{)\
//     \
//     cyt3macro_enum_class_tostring_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
//     \
//     cyt3macro_enum_class_fromstring_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
//     \
//     cyt3macro_enum_class_castcheck_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__)\
//     \
//     \IFE(CY_ownerclass)(}) \
//     \
//     IFE(CY_ownerclass)(cyt3macro_enum_class_opsoverloads_def_(CY_enum_class_name, CY_ownerclass, __VA_ARGS__))\




///////////////////////////////////////////
///////////////////////////////////////////
  //priv macro
  #define cyt3macro_prvdef_enum_class_ostringify_unit(CY_enum_class_name) \
    std::ostream& operator<<(std::ostream& o,const CY_enum_class_name& s){ \
      return (o << CY_enum_class_name##ToString(s)); \
    } 
/**
 * @brief defines the ostream operator << overloads for each mentioned enum class.
 * 
 */
#define CYT3MACRO_enum_class_ostringify_definitions(...) \
  FOR_EACH(cyt3macro_prvdef_enum_class_ostringify_unit,__VA_ARGS__)




/**
 * @brief Declarationsfor a basic class container for an enum class.
 *  Includes basic constructor, copy constructor, virtual destructor, 
 *  operator= overload, operator== overload
 * @param V_class_name : REQUIRED : class name
 * @param CY_parent_class : OPTIONAL : parent class name
 */
#define CYT3MACRO_enum_class_basic_class_container_declarations(V_class_name,CY_parent_class) \
  V_class_name(); \
  V_class_name(const V_class_name& o); \
  virtual ~V_class_name(); \
  V_class_name& operator=(const V_class_name& o); \
  bool operator==(const V_class_name& o) const; 


/**
 * @brief Detinitions for a basic class container for an enum class
 * @param V_class_name : REQUIRED : class name to define
 * @param V_parent_class_name : OPTIONAL : parent class name. 
 */
#define CYT3MACRO_enum_class_basic_class_container_definitions(V_class_name,CY_parent_class) \
    V_class_name::V_class_name()\
      IFN(CY_parent_class) (:CY_parent_class())\
      { } \
    V_class_name::V_class_name(const V_class_name& o)\
      IFN(CY_parent_class)(:CY_parent_class(o) )\
      { }; \
    V_class_name::~V_class_name(){ } \
    V_class_name& V_class_name::operator=(const V_class_name& o){\
      IFE(CY_parent_class)(return *this;)\
      IFN(CY_parent_class)(return CY_parent_class::operator=(o);)\
    } \
    bool V_class_name::operator==(const V_class_name& o) const{\
      IFE(CY_parent_class)(return true;)\
      IFN(CY_parent_class)(return CY_parent_class::operator==(o)); \
    } 
  


