#pragma once
#include "cyt_foreach_macro_helpers.hpp"


/**
 * COYOT3PP_ENUM_CLASS_DECLARATIONS 
 * COYOT3PP_ENUM_CLASS_DEFINITIONS
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
 *  COYOT3PP_ENUM_CLASS_DECLARATIONS 
 * 
 * 
*/



    #define __coyot3pp_enum_class_priv_dec_state_pairs_unit_(P_ec_state_name, P_ec_state_value)\
      (\
        P_ec_state_name\
        IFN(P_ec_state_value)(= P_ec_state_value)\
      )

  #define __coyot3pp_enum_class_priv_dec_state_pairs_(...)\
    CHAIN_COMMA(\
      FOR_EACH_PAIR(__coyot3pp_enum_class_priv_dec_state_pairs_unit_,__VA_ARGS__)\
    )
   
    #define __coyot3pp_enum_class_prv_dec_funcparams_each_(P_f_param)\
      P_f_param ;
  #define __coyot3pp_enum_class_prv_dec_funcparams_(...)\
    FOR_EACH(__coyot3pp_enum_class_prv_dec_funcparams_each_,__VA_ARGS__)

/**
 * @brief : declares an enum class as <CY_enum_class_name>. Arguments are the states. 
 *  States can contain default values. Predefines INTERNAL_ERROR(-2) 
 *  and UNKNOWN_UNSET(-1). Also declares:
 *  const char* <CY_enum_class_name>ToString(<CY_enum_class_name>)
 *  const <CCY_enum_class_name> <CY_enum_class_name>FromString(const char* s)
 * @param CY_enum_class_name : enum class name
 * @param CY_ec_functions_decl : functions and methods declarations
 * @param ... each state of the enum class amd its value. Can contain the predefined Value (i.e: STATE_1 , 1, STATE_2, , STATE_3, 3)
*/
#define COYOT3PP_ENUM_CLASS_DECLARATIONS(P_enum_class_name, CY_additional_funcs_and_ops, ...) \
  namespace ec{\
    enum class P_enum_class_name { \
      INTERNAL_ERROR = -2,\
      UNKNOWN_OR_UNSET = -1,\
      __coyot3pp_enum_class_priv_dec_state_pairs_(__VA_ARGS__) \
    };\
    const char* P_enum_class_name##ToString( P_enum_class_name s);\
    P_enum_class_name P_enum_class_name##FromString( const char* s);\
    bool P_enum_class_name##CastCheck(P_enum_class_name s);\
    ::std::ostream& operator<<(::std::ostream& o,const P_enum_class_name& s); \
    int& operator<<(int& o,P_enum_class_name s); \
    P_enum_class_name& operator<<(P_enum_class_name& o,int s); \
    P_enum_class_name& operator<<(P_enum_class_name& o, const std::string& i);\
    IFN(CY_additional_funcs_and_ops)(__coyot3pp_enum_class_prv_dec_funcparams_(PASS_PARAMETERS(CY_additional_funcs_and_ops)))\
  }



////////////////////////////////////////////////
////////////////////////////////////////////////
    #define __coyot3pp_enum_class_prvdef_tostring_case_def_(P_enum_class_name, P_enum_class_state_name, P_ec_state_value)\
      case P_enum_class_name::P_enum_class_state_name: return #P_enum_class_state_name; break;

    #define __coyot3pp_enum_class_prvdef_fromstring_case_(P_enum_class_name, P_enum_class_state_name, P_ec_state_value)\
      if(strcmp(#P_enum_class_state_name,s) == 0){return P_enum_class_name::P_enum_class_state_name;}

    #define __coyot3pp_enum_class_prvdef_castcheck_case_(P_enum_class_name, P_enum_class_state_name, P_ec_state_value)\
      case P_enum_class_name::P_enum_class_state_name: 





  #define __coyot3pp_enum_class_prvdef_tostring_def_(P_enum_class_name, CY_additional_funcs_and_ops, ...)\
    const char* P_enum_class_name##ToString( P_enum_class_name s)\
      {\
        switch(s){\
          FOR_EACH_PAIR_WITH_CONSTANT(__coyot3pp_enum_class_prvdef_tostring_case_def_, \
                                  P_enum_class_name, __VA_ARGS__) \
          case P_enum_class_name::INTERNAL_ERROR: \
            return "INTERNAL_ERROR"; break; \
          case P_enum_class_name::UNKNOWN_OR_UNSET: \
            return "UNKNOWN_OR_UNSET"; break; \
          default: \
            return "err_" #P_enum_class_name "_unknown_state"; \
        } \
    }

  #define __coyot3pp_enum_class_prvdef_fromstring_def_(P_enum_class_name, CY_additional_funcs_and_ops, ...)\
    P_enum_class_name P_enum_class_name##FromString( const char* s){\
      FOR_EACH_PAIR_WITH_CONSTANT(__coyot3pp_enum_class_prvdef_fromstring_case_, P_enum_class_name, __VA_ARGS__) \
      if(strcmp("UNKNOWN_OR_UNSET",s) == 0){return P_enum_class_name::UNKNOWN_OR_UNSET;} \
      return P_enum_class_name::INTERNAL_ERROR; \
    }

  #define __coyot3pp_enum_class_prvdef_castcheck_def_(P_enum_class_name, CY_additional_funcs_and_ops, ...)\
  \
  bool P_enum_class_name##CastCheck( P_enum_class_name s){\
    switch(s){\
      FOR_EACH_PAIR_WITH_CONSTANT(__coyot3pp_enum_class_prvdef_castcheck_case_, P_enum_class_name, __VA_ARGS__) \
      case P_enum_class_name::UNKNOWN_OR_UNSET: \
        return true;\
        break;\
      case P_enum_class_name::INTERNAL_ERROR: \
      default: \
        return false;\
    }\
  }



    #define __coyot3pp_enum_class_enum_class_opsoverloads_def_(P_enum_class_name, CY_additional_funcs_and_ops)\
      ::std::ostream& operator<<(::std::ostream& o,const P_enum_class_name& s){ \
        return (o << P_enum_class_name##ToString(s)); }\
      int& operator<<(int& o, P_enum_class_name s){return (o = static_cast<int>(s));} \
      P_enum_class_name& operator<<(P_enum_class_name& o,int s){return( o = static_cast<P_enum_class_name>(s));} \
      P_enum_class_name& operator<<(P_enum_class_name& o,const std::string& i){return( o = P_enum_class_name##FromString(i.c_str()));} \
        

  

/**
 * @brief definitions for the cyt3macro-enum-class.
 * @param CY_additional_funcs_and_ops : owner class. CAN BE EMPTY
 * @param P_enum_class_name : enum type name 
 * @param ... set of values
 *  
*/
#define COYOT3PP_ENUM_CLASS_DEFINITIONS(P_enum_class_name, CY_additional_funcs_and_ops, ...)\
    \
    namespace ec{ \
      \
      __coyot3pp_enum_class_prvdef_tostring_def_(P_enum_class_name, CY_additional_funcs_and_ops, __VA_ARGS__)\
      \
      __coyot3pp_enum_class_prvdef_fromstring_def_(P_enum_class_name, CY_additional_funcs_and_ops, __VA_ARGS__)\
      \
      __coyot3pp_enum_class_prvdef_castcheck_def_(P_enum_class_name, CY_additional_funcs_and_ops, __VA_ARGS__)\
      \
      __coyot3pp_enum_class_enum_class_opsoverloads_def_(P_enum_class_name, CY_additional_funcs_and_ops)\
      \
    }





#define COYOT3PP_ENUM_CLASS_DECLARATIONS_AND_DEFINITIONS(P_enum_class_name, CY_additional_funcs_and_ops, ...)\
        COYOT3PP_ENUM_CLASS_DECLARATIONS(P_enum_class_name, CY_additional_funcs_and_ops, __VA_ARGS__)\
        COYOT3PP_ENUM_CLASS_DEFINITIONS(P_enum_class_name, CY_additional_funcs_and_ops, __VA_ARGS__)
  


