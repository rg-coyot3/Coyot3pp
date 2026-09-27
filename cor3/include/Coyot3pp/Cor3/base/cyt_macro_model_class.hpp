#pragma once
#include "cyt_foreach_macro_helpers.hpp"
#include "cyt_macro_enum_class.hpp"
#include <functional>


// DECLARATION BEGIN:
    #define __coyot3pp_privdec_model_class_constructor_params_decl_param_(CY_NAME, CY_TYPE, CY_DEFVALUE)\
      , const CY_TYPE& CY_NAME

  #define __coyot3pp_privdec_model_class_constructor_params_decl_(CY_class_name, CY_enumclass_deps, CY_NAME, CY_TYPE, CY_DEFVALUE , ... ) \
    CY_class_name(\
      const CY_TYPE& CY_NAME\
      IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(__coyot3pp_privdec_model_class_constructor_params_decl_param_,PASS_PARAMETERS(CY_enumclass_deps)))\
      FOR_EACH_TRIPLES(__coyot3pp_privdec_model_class_constructor_params_decl_param_,__VA_ARGS__) \
    );

  #define __coyot3pp_privdec_modelclass_declare_types_(CY_prop_name,CY_type,CY_default_value) \
     typedef CY_type CY_prop_name##_t;

  #define __coyot3pp_modelclass_properties_dec_(V_PROPNAME,V_TYPE,V_DEFVALUE) \
    IFE(V_DEFVALUE)(V_PROPNAME##_t V_PROPNAME##_;) \
    IFN(V_DEFVALUE)(V_PROPNAME##_t V_PROPNAME##_ = V_DEFVALUE;)

  #define __coyot3pp_model_class_getterssetters_dec_(V_PROPNAME, V_TYPE, V_DEFVALUE) \
    V_PROPNAME##_t  V_PROPNAME() const; \
    V_PROPNAME##_t& V_PROPNAME(); \
    V_PROPNAME##_t  V_PROPNAME(const V_PROPNAME##_t& v );

  #define __coyot3pp_model_class_enumclass_declare_types_(CY_prop_name, CY_enumclass_type, CY_default_value)\
    typedef CY_enumclass_type CY_prop_name##_t;
  
  #define __coyot3pp_model_class_enumclass_declare_getterssetters_(CY_prop_name, CY_enumclass_type, CY_default_value)\
    CY_prop_name##_t  CY_prop_name() const; \
    CY_prop_name##_t& CY_prop_name(); \
    CY_prop_name##_t  CY_prop_name(const CY_prop_name##_t& v );\
    CY_prop_name##_t  CY_prop_name(const char* v ); \
    CY_prop_name##_t  CY_prop_name(int v );

  #define __coyot3pp_model_class_enumclass_properties_dec_(CY_prop_name,CY_enumclass_type,CY_default_value) \
    IFE(CY_default_value)(CY_prop_name##_t CY_prop_name##_ = CY_prop_name##_t::UNKNOWN_OR_UNSET;) \
    IFN(CY_default_value)(CY_prop_name##_t CY_prop_name##_ = CY_default_value;)


  #define __coyot3pp_privdev_model_class_get_model_template_dec_(CY_class_name, CY_enumclass_deps, ...) \
    static std::string get_model_template();


    #define __coyot3pp_privdev_model_class_constructor_by_params_paramsseq_dec_(P_param_name, P_param_type, P_param_default)\
      P_param_name##_t P_param_name


  #define __coyot3pp_privdev_model_class_constructor_by_params_dec_(CY_enumclass_deps, ...)\
    CHAIN_COMMA(\
      FOR_EACH_TRIPLES(__coyot3pp_privdev_model_class_constructor_by_params_paramsseq_dec_,__VA_ARGS__)\
    )

      #define __coyot3pp_privdev_model_class_dec_tr_params_itr_dec_(CY_prop_name,CY_type,CY_default_value)\
          (CY_type p_##CY_prop_name)

    #define __coyot3pp_privdev_model_class_dec_tr_params_dec_(...)\
      CHAIN_COMMA(\
        FOR_EACH_TRIPLES(__coyot3pp_privdev_model_class_dec_tr_params_itr_dec_, __VA_ARGS__ ) \
      )
    
  // ps-priv : begin
  #define __coyot3pp_privdec_model_class_common_priv_dev_(CY_class_name, CY_parent_class, CY_enumclass_deps, ...) \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(__coyot3pp_model_class_enumclass_declare_types_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(__coyot3pp_privdec_modelclass_declare_types_, __VA_ARGS__) \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(__coyot3pp_model_class_enumclass_declare_getterssetters_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(__coyot3pp_model_class_getterssetters_dec_,__VA_ARGS__)\
        \
        CY_class_name & operator=(const CY_class_name & o); \
        IFN(CY_parent_class)(CY_class_name& operator=(const CY_parent_class& o);)\
        bool           operator==(const CY_class_name & o) const; \
        bool           operator!=(const CY_class_name & o) const; \
        \
        CY_class_name(); \
        CY_class_name(const CY_class_name & o); \
        \
        IFN(CY_parent_class)(CY_class_name(const CY_parent_class& o);)\
        CY_class_name(\
          __coyot3pp_privdev_model_class_dec_tr_params_dec_(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          __coyot3pp_privdev_model_class_dec_tr_params_dec_(PASS_PARAMETERS(CY_enumclass_deps))\
        );\
        \
        virtual ~CY_class_name();\
        \
        __coyot3pp_privdev_model_class_get_model_template_dec_(CY_class_name, CY_enumclass_deps, __VA_ARGS__) \
        \
      protected: \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(__coyot3pp_model_class_enumclass_properties_dec_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(__coyot3pp_modelclass_properties_dec_,__VA_ARGS__)


  #define __coyot3pp_privdec_model_class_additional_method_add_(CV_additional_method) \
    CV_additional_method;

// ps-priv : end

    #define __coyot3pp_privdev_model_class_heritage_unit_decl_(CY_parent_class)\
        public CY_parent_class

  #define __coyot3pp_privdev_model_class_heritage_decl_(...)\
    DEFER(\
      CHAIN_COMMA( \
      FOR_EACH(__coyot3pp_privdev_model_class_heritage_unit_decl_, __VA_ARGS__) \
    ))

/**
 * @brief declares a model class with default values. 
 *  <A1> = name of class
 *  a_n(1/3) = name of the property
 *  a_n(2/3) = type of the property
 *  a_n(3/3) = default value of the property
 * 
 *  for each name it will declare a protected property "<name>_"
 *    and 2 methods (getter and setter):
 *     <type> name() const
 *     <type>type name(type v)
 *  @param CY_class_name : REQUIRED : class name
 *  @param CY_parent_class : OPTIONAL : parent class name
 *  @param ...
 */
#define COYOT3PP_MODEL_CLASS_DECLARATIONS(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  class CY_class_name\
  IFN(CY_parent_class)(: public CY_parent_class)\
  { \
    public: \
    \
    FOR_EACH(__coyot3pp_privdec_model_class_additional_method_add_, PASS_PARAMETERS(CY_additional_methods)) \
    \
    __coyot3pp_privdec_model_class_common_priv_dev_(CY_class_name,  CY_parent_class, CY_enumclass_deps, __VA_ARGS__) \
    \
  };







//
//

//DECLARATION END:


//DEFINITION BEGIN:
  #define __coyot3pp_model_class_copyitem_def_(a1,a2,a3) \
    a1##_ = o.a1##_;


  // el resto de las líneas añade '&&' como prefijo
  #define __coyot3pp_model_class_eqopitem_line_def_(a1,a2,a3) \
    && (a1##_ == o.a1##_) 

  #define __coyot3pp_model_class_eqopitem_lines_def(a1,a2,a3,...)\

  // la primera línea no añade '&&'
  #define __coyot3pp_model_class_eqopitems_def_(a1,a2,a3,...) \
    (a1##_ == o.a1##_) \
    FOR_EACH_TRIPLES(__coyot3pp_model_class_eqopitem_line_def_, __VA_ARGS__)

    #define __coyot3pp_privdev_model_class_get_model_template_def_add_item_(a1,a2,a3)\
      mt += "(" #a2 ")" #a1 IFN(a3)("/default=" #a3 "/") IFE(a3)("//") "\n";

  #define __coyot3pp_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  std::string CY_class_name::get_model_template(){\
    std::string mt;\
    FOR_EACH_TRIPLES(__coyot3pp_privdev_model_class_get_model_template_def_add_item_,PASS_PARAMETERS(CY_enumclass_deps))\
    FOR_EACH_TRIPLES(__coyot3pp_privdev_model_class_get_model_template_def_add_item_,__VA_ARGS__) \
    return mt; \
  }


  #define __coyot3pp_model_class_getters_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1() const{ return a1##_;} \
    CY_class_name::a1##_t& CY_class_name::a1(){ return a1##_;} \

  #define __coyot3pp_model_class_setters_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1(const CY_class_name::a1##_t& v){ return a1##_ = v;}


  #define __coyot3pp_model_class_setters_ecextra_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1(const char* v){ \
      a1 = a2##FromString(v); \
      return a1; \
    } \
    CY_class_name::a1##_t CY_class_name::a1(int v){\
      a1 = static_cast<a2>(v);\
      return a1;\
    }
  
#define __coyot3pp_model_class_def_tr_assign_(a1,a2,a3)\
    a1##_ = p_##a1;

#define __coyot3pp_model_class_definitions_commons_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  CY_class_name::CY_class_name()\
    IFN(CY_parent_class)(:CY_parent_class()){} \
  CY_class_name::CY_class_name(const CY_class_name & o){*this = o;}\
  IFN(CY_parent_class)(CY_class_name::CY_class_name(const CY_parent_class & o):CY_parent_class(o){})\
  CY_class_name::~CY_class_name(){}\
  CY_class_name::CY_class_name(\
          __coyot3pp_privdev_model_class_dec_tr_params_dec_(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          __coyot3pp_privdev_model_class_dec_tr_params_dec_(PASS_PARAMETERS(CY_enumclass_deps))\
  ){\
    FOR_EACH_TRIPLES(__coyot3pp_model_class_def_tr_assign_,__VA_ARGS__)\
    FOR_EACH_TRIPLES(__coyot3pp_model_class_def_tr_assign_,PASS_PARAMETERS(CY_enumclass_deps))\
  }\
  bool CY_class_name::operator!=(const CY_class_name & o) const{\
    return !(*this == o); \
  } \
  FOR_EACH_TRIPLES_WITH_01_STATIC(__coyot3pp_model_class_getters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(__coyot3pp_model_class_setters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(__coyot3pp_model_class_getters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\
  FOR_EACH_TRIPLES_WITH_01_STATIC(__coyot3pp_model_class_setters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))

#define __coyot3pp_model_class_definitions_operators_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...)\
  CY_class_name & CY_class_name::operator=(const CY_class_name & o){ \
    IFN(CY_parent_class)(CY_parent_class::operator=(o);)\
    FOR_EACH_TRIPLES(__coyot3pp_model_class_copyitem_def_,__VA_ARGS__)\
    FOR_EACH_TRIPLES(__coyot3pp_model_class_copyitem_def_,PASS_PARAMETERS(CY_enumclass_deps))\
    return *this; \
  } \
  IFN(CY_parent_class)(CY_class_name & CY_class_name::operator=(const CY_parent_class& o){CY_parent_class::operator=(o);return *this;})\
  \
  __coyot3pp_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
  bool CY_class_name::operator==(const CY_class_name & o) const{\
    return (\
    IFN(CY_parent_class)( (CY_parent_class::operator==(o)) && )\
    __coyot3pp_model_class_eqopitems_def_(__VA_ARGS__)\
    );\
  }\

#define COYOT3PP_MODEL_CLASS_DEFINITIONS(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  __coyot3pp_model_class_definitions_commons_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
  __coyot3pp_model_class_definitions_operators_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)


#define COYOT3PP_MODEL_CLASS_DEFINITIONS_no_opsoverload(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...)\
  __coyot3pp_model_class_definitions_commons_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)
////////////////////////////////////
////////////////////////////////////

#define COYOT3PP_MODEL_CLASS_DECLARATIONS_AND_DEFINITIONS(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...)\
        COYOT3PP_MODEL_CLASS_DECLARATIONS(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
        COYOT3PP_MODEL_CLASS_DEFINITIONS(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)



/**
 * @brief Declares an indexed Set / Array class of <CY_class_name>, idexed by <CY_member_index_property>.
 *  Declares a map the type <CY_class_name>MappedSetMapType and a basic methods set:
 *    - insert ; remove ; update ; get ; is_member ; size
 * @param CY_class_name : REQUIRED : class name to be held in the class.
 * @param CY_class_index_property : REQUIRED : property of the class to use as index.
 */
#define COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(CY_class_name,CY_member_index_property) \
  class CY_class_name##MappedSet{ \
    public:\
      typedef std::map<CY_class_name::CY_member_index_property##_t,CY_class_name> CY_class_name##MappedSetMapType;\
    \
    CY_class_name##MappedSet(); \
    CY_class_name##MappedSet(const CY_class_name##MappedSet& o); \
    virtual ~CY_class_name##MappedSet(); \
    \
    CY_class_name##MappedSet& operator=(const CY_class_name##MappedSet& o);\
    bool                operator==(const CY_class_name##MappedSet& o) const ;\
    bool                operator!=(const CY_class_name##MappedSet& o) const ;\
    CY_class_name##MappedSet  operator+(const CY_class_name##MappedSet& o) ;\
    CY_class_name##MappedSet& operator+=(const CY_class_name##MappedSet& o);\
    CY_class_name       operator[](CY_class_name::CY_member_index_property##_t index) const;\
    \
    bool              insert(const CY_class_name & o); \
    bool              remove(CY_class_name::CY_member_index_property##_t index); \
    bool              remove(CY_class_name index); \
    bool              update(const CY_class_name & o, bool createIfDoesNotExist = false);\
    bool              is_member(CY_class_name::CY_member_index_property##_t index) const;\
    CY_class_name     get(CY_class_name::CY_member_index_property##_t index) const; \
    CY_class_name&    get(CY_class_name::CY_member_index_property##_t index); \
    std::size_t       size() const;\
    size_t            for_each(std::function<bool(const CY_class_name&)> func) const;\
    size_t            for_each(std::function<bool(CY_class_name&)> func);\
    void              clear();\
    \
    protected: \
      CY_class_name##MappedSetMapType map_;\
      mutable std::mutex map_mtx_;\
  };





  //def constructors
  #define __coyot3pp_model_class_set_mapped_def_constrsdestr_(CY_class_name) \
    CY_class_name##MappedSet::CY_class_name##MappedSet():map_(){ }\
    CY_class_name##MappedSet::CY_class_name##MappedSet(const CY_class_name##MappedSet& o){ *this = o;}\
    CY_class_name##MappedSet::~CY_class_name##MappedSet(){}

  //def op =
  #define __coyot3pp_model_class_set_mapped_def_opassign_(CY_class_name)\
    CY_class_name##MappedSet& CY_class_name##MappedSet::operator=(const CY_class_name##MappedSet& o){\
      map_ = o.map_;\
      return *this;\
    }
  //def op !=
  #define __coyot3pp_model_class_set_mapped_def_opneq_(CY_class_name)\
    bool CY_class_name##MappedSet::operator!=(const CY_class_name##MappedSet& o) const { return !( *this == o);}
  //def op ==
  #define __coyot3pp_model_class_set_mapped_def_opeq_(CY_class_name)\
    bool CY_class_name##MappedSet::operator==(const CY_class_name##MappedSet& o) const {\
      if(size() != o.size())return false;\
      CY_class_name##MappedSetMapType::const_iterator i1;\
      for(i1 = map_.begin();i1 != map_.end();i1++){\
        if(o.is_member(i1->first) == false){return false;}\
        if(i1->second != o.get(i1->first)){return false;}\
      }\
      return true;\
    }
  //def op []
  #define __coyot3pp_model_class_set_mapped_def_opbraq_(CY_class_name,CY_member_index_property)\
    CY_class_name       CY_class_name##MappedSet::operator[](CY_class_name::CY_member_index_property##_t index) const{\
      return get(index);\
    }
  //def insert
  #define __coyot3pp_model_class_set_mapped_def_insert_(CY_class_name,CY_member_index_property)\
    bool              CY_class_name##MappedSet::insert(const CY_class_name & o){\
      if(is_member(o.CY_member_index_property()) == true)return false;\
      map_.insert(std::make_pair(o.CY_member_index_property(),o));\
      return true;\
    }

  //def removes
  #define __coyot3pp_model_class_set_mapped_def_removes_(CY_class_name,CY_member_index_property) \
    bool  CY_class_name##MappedSet::remove(CY_class_name::CY_member_index_property##_t index){\
      if(is_member(index) == false){return false;}\
      map_.erase(map_.find(index));\
      return true;\
    } \
    bool  CY_class_name##MappedSet::remove(CY_class_name index){\
      return remove(index.CY_member_index_property());\
    } 

  //def is_member
  #define __coyot3pp_model_class_set_mapped_def_is_member(CY_class_name,CY_member_index_property) \
    bool CY_class_name##MappedSet::is_member(CY_class_name::CY_member_index_property##_t index) const {\
      return (map_.find(index) != map_.end());\
    }

  #define __coyot3pp_model_class_set_mapped_def_get(CY_class_name,CY_member_index_property)\
    CY_class_name     CY_class_name##MappedSet::get(CY_class_name::CY_member_index_property##_t index) const{\
      CY_class_name ret;\
      if(is_member(index)){\
        ret = map_.find(index)->second;\
      }\
      return ret;\
    } \
    CY_class_name&     CY_class_name##MappedSet::get(CY_class_name::CY_member_index_property##_t index){\
      return map_.find(index)->second;\
    } 

  #define __coyot3pp_model_class_set_mapped_def_size_(CY_class_name,CY_member_index_property)\
    std::size_t       CY_class_name##MappedSet::size() const{\
      return map_.size();\
    } 

  #define __coyot3pp_model_class_set_mapped_def_update_(CY_class_name,CY_member_index_property)\
    bool              CY_class_name##MappedSet::update(const CY_class_name & o, bool createIfDoesNotExist){\
      CY_class_name##MappedSetMapType::iterator i1 = map_.find(o.CY_member_index_property());\
      if(i1 == map_.end()){\
        if(createIfDoesNotExist == false)return false;\
        else map_[o.CY_member_index_property()] = o;\
        return true;\
      }\
      i1->second = o;\
      return true;\
    }
  
  #define __coyot3pp_model_class_set_mapped_def_clear_(CY_class_name,CY_member_index_property)\
    void              CY_class_name##MappedSet::clear(){\
      map_.clear();\
    }

  #define __coyot3pp_model_class_set_mapped_def_foreachs_(CY_class_name,CY_member_index_property)\
      size_t CY_class_name##MappedSet::for_each(std::function<bool(const CY_class_name&)> func) const{\
        if(size() == 0)return 0;\
        CY_class_name##MappedSetMapType::const_iterator it;\
        size_t res = 0;\
        for(it = map_.begin();it != map_.end();++it){\
          if(func(it->second) == true) ++res;\
        }\
        return res;\
      }\
      size_t CY_class_name##MappedSet::for_each(std::function<bool(CY_class_name&)> func){\
        if(size() == 0)return 0;\
        CY_class_name##MappedSetMapType::iterator it;\
        size_t res = 0;\
        for(it = map_.begin();it != map_.end();++it){\
          if(func(it->second) == true) ++res;\
        }\
        return res;\
      }

/**
 * @brief model set definitions
 * @param CY_class_name : REQUIRED : base class name
 * @param CY_member_index_property : REQUIRED : base class index property
 * 
 */
#define COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(CY_class_name,CY_member_index_property) \
  __coyot3pp_model_class_set_mapped_def_constrsdestr_(CY_class_name)\
  \
  __coyot3pp_model_class_set_mapped_def_opassign_(CY_class_name)\
  \
  __coyot3pp_model_class_set_mapped_def_opeq_(CY_class_name)\
  \
  __coyot3pp_model_class_set_mapped_def_opneq_(CY_class_name)\
  \
  __coyot3pp_model_class_set_mapped_def_opbraq_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_insert_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_removes_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_is_member(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_get(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_size_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_update_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_clear_(CY_class_name,CY_member_index_property)\
  \
  __coyot3pp_model_class_set_mapped_def_foreachs_(CY_class_name,CY_member_index_property)


#define COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS_AND_DEFINITIONS(CY_class_name,CY_member_index_property)\
        COYOT3PP_MODEL_CLASS_SET_MAPPED_DECLARATIONS(CY_class_name,CY_member_index_property)\
        COYOT3PP_MODEL_CLASS_SET_MAPPED_DEFINITIONS(CY_class_name,CY_member_index_property)

//'Droid Sans Mono', 'monospace', monospace




#define COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(CY_class_name, CY_default_max_size)\
  class CY_class_name##Stack {\
    public: \
      typedef std::vector<CY_class_name> CY_class_name##StackType;\
      typedef std::vector<CY_class_name>::iterator CY_class_name##StackIterator;\
      typedef std::vector<CY_class_name>::const_iterator CY_class_name##StackConstIterator;\
      CY_class_name##Stack();\
      CY_class_name##Stack(const CY_class_name##Stack& o);\
      virtual ~CY_class_name##Stack();\
      std::size_t  size()            const;\
      std::size_t  max_size()        const;\
      std::size_t  max_size(std::size_t v);\
      \
      bool    push(const CY_class_name & o);\
      bool    push(const CY_class_name##Stack& o);\
      bool    push_back(const CY_class_name& o);\
      bool    push_back(const CY_class_name##Stack& o);\
      bool    push_front(const CY_class_name& o);\
      bool    push_front(const CY_class_name##Stack& o);\
      \
      bool    pop_front();\
      bool    pop_back();\
      \
      bool    remove(const CY_class_name & o);\
      bool    remove(std::size_t pos, std::size_t length=1);\
      std::size_t remove_if(std::function<bool(CY_class_name&)> func);\
      \
      void    clear();\
      CY_class_name##Stack& operator=(const CY_class_name##Stack& o);\
      bool                  operator==(const CY_class_name##Stack& o) const;\
      bool                  operator!=(const CY_class_name##Stack& o) const;\
      \
      CY_class_name&        at(int index);\
      const CY_class_name&  at(int index) const;\
      CY_class_name&        operator[](std::size_t index);\
      const CY_class_name&  operator[](std::size_t index) const;\
      \
      std::size_t forEach(std::function<bool(CY_class_name &)> func);\
      std::size_t forEach(std::function<bool(const CY_class_name &)> func) const;\
      std::size_t for_each(std::function<bool(CY_class_name &)> func);\
      std::size_t for_each(std::function<bool(const CY_class_name &)> func) const;\
      \
    protected:\
      std::vector<CY_class_name>                stack_;\
      mutable std::mutex                        stack_mtx_;\
      std::size_t                               default_max_size_;\
  };





    #define __coyot3pp_model_class_def_set_stack_opeq_(CY_class_name) \
      CY_class_name##Stack& CY_class_name##Stack::operator=(const CY_class_name##Stack& o){\
        stack_ = o.stack_;\
        return *this;\
      } \
      bool CY_class_name##Stack::operator==(const CY_class_name##Stack& o) const{ \
        if(size() != o.size())return false;\
        for(const CY_class_name & iloc : stack_){ \
          bool eq = false; \
          for(const CY_class_name & iext : o.stack_){ \
            if(iloc == iext){ \
              eq = true;\
            } \
          }\
          if(eq == false){return false;}\
        }\
        return true;\
      }\
      bool CY_class_name##Stack::operator!=(const CY_class_name##Stack& o) const{ \
        return !(*this == o);\
      }


      

      #define __coyot3pp_model_class_def_set_stack_foreachs_(CY_class_name) \
        std::size_t CY_class_name##Stack::forEach(std::function<bool(CY_class_name &)> func){return for_each(func);} \
        std::size_t CY_class_name##Stack::for_each(std::function<bool(CY_class_name &)> func){ \
          std::lock_guard<std::mutex> g(stack_mtx_);\
          CY_class_name##StackType::iterator i; \
          std::size_t ok = 0; \
          for(i=stack_.begin();i != stack_.end(); ++i){ \
            if(func(*i)==true)++ok; \
          } \
          return ok; \
        } \
        std::size_t CY_class_name##Stack::forEach(std::function<bool(const CY_class_name &)> func)const{return for_each(func);} \
        std::size_t CY_class_name##Stack::for_each(std::function<bool(const CY_class_name &)> func)const{ \
          std::lock_guard<std::mutex> g(stack_mtx_);\
          CY_class_name##StackType::const_iterator i; \
          std::size_t ok = 0; \
          for(i=stack_.begin();i != stack_.end(); ++i){ \
            if(func(*i)==true) ++ok; \
          } \
          return ok; \
        } 

      #define __coyot3pp_model_class_def_set_stack_remove_(CY_class_name)\
        bool CY_class_name##Stack::remove(const CY_class_name& o){ \
          std::lock_guard<std::mutex> g(stack_mtx_);\
          CY_class_name##StackType::iterator i,rem = stack_.end(); \
          for(i=stack_.begin();i != stack_.end(); ++i){ \
            if(*i == o)rem = i; \
          } \
          if(rem ==stack_.end())return false;\
          stack_.erase(rem);\
          return true;\
        }\
        bool    CY_class_name##Stack::remove(std::size_t pos, std::size_t length){\
          std::lock_guard<std::mutex> g(stack_mtx_);\
          if((pos + length) >= stack_.size()) return false; \
          stack_.erase(stack_.begin() + pos, stack_.begin() + pos + length); \
          return true; \
        }\
        std::size_t CY_class_name##Stack::remove_if(std::function<bool(CY_class_name&)> func){\
            std::vector<CY_class_name##StackIterator> dels;\
            for(CY_class_name##StackIterator i = stack_.begin();\
                i != stack_.end(); i++){\
              if(func(*i) == true)dels.push_back(i);\
            }\
            std::size_t r = dels.size();\
            while(dels.size() > 0){stack_.erase(dels.back());dels.pop_back();}\
            return r;\
        }



    #define __coyot3pp_model_class_def_set_stack_push_(CY_class_name) \
      bool    CY_class_name##Stack::push(const CY_class_name & o){ return push_back(o); }\
      bool    CY_class_name##Stack::push_back(const CY_class_name & o){ \
        std::lock_guard<std::mutex> g(stack_mtx_);\
        stack_.push_back(o); \
        if(stack_.size()>= default_max_size_){ \
          stack_.erase(stack_.begin()); \
        }\
        return true;\
      }\
      bool    CY_class_name##Stack::push(const CY_class_name##Stack& o){ return push_back(o);}\
      bool    CY_class_name##Stack::push_back(const CY_class_name##Stack& o){ \
        return o.forEach([&](const CY_class_name & item){\
          push_back(item);\
          return true;\
        }) == o.size();\
      }\
      bool CY_class_name##Stack::push_front(const CY_class_name& o){ \
        std::lock_guard<std::mutex> g(stack_mtx_);\
        stack_.insert(stack_.begin(),o);\
        if(default_max_size_ == 0)return true;\
        if(stack_.size() <= default_max_size_)return true;\
        stack_.pop_back();\
        return false;\
      } \
      bool CY_class_name##Stack::push_front(const CY_class_name##Stack& o){ \
        return o.for_each([&](const CY_class_name& el){\
          return push_front(el);\
        }) == o.size();\
      }

    #define __coyot3pp_model_class_def_set_stack_shorts_(CY_class_name) \
      std::size_t   CY_class_name##Stack::size() const{return stack_.size();} \
      std::size_t   CY_class_name##Stack::max_size() const{return default_max_size_;} \
      std::size_t   CY_class_name##Stack::max_size(std::size_t v){return (default_max_size_ = v);} \
      void          CY_class_name##Stack::clear(){stack_.clear();} 

    #define __coyot3pp_model_class_def_set_stack_constrdestr_(CY_class_name, CY_default_max_size) \
      CY_class_name##Stack::CY_class_name##Stack():stack_(){\
      IFE(CY_default_max_size)(default_max_size_ = (size_t)-1;)\
      IFN(CY_default_max_size)(default_max_size_ = CY_default_max_size;)}\
      CY_class_name##Stack::CY_class_name##Stack(const CY_class_name##Stack& o){*this = o;} \
      CY_class_name##Stack::~CY_class_name##Stack(){} 


    #define __coyot3pp_model_class_def_set_stack_ats_(CY_class_name)\
    CY_class_name& CY_class_name##Stack::at(int index){\
      return stack_[index];\
    }\
    const CY_class_name& CY_class_name##Stack::at(int index) const{\
      return stack_[index];\
    }\
    CY_class_name& CY_class_name##Stack::operator[](std::size_t index){\
      return stack_[index];\
    }\
    const CY_class_name& CY_class_name##Stack::operator[](std::size_t index) const{\
      return stack_[index];\
    }



  #define COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(CY_class_name, CY_default_max_size) \
    \
    __coyot3pp_model_class_def_set_stack_constrdestr_(CY_class_name, CY_default_max_size) \
    \
    __coyot3pp_model_class_def_set_stack_shorts_(CY_class_name) \
    \
    __coyot3pp_model_class_def_set_stack_opeq_(CY_class_name) \
    \
    __coyot3pp_model_class_def_set_stack_foreachs_(CY_class_name) \
    \
    __coyot3pp_model_class_def_set_stack_push_(CY_class_name)\
    \
    __coyot3pp_model_class_def_set_stack_remove_(CY_class_name)\
    \
    __coyot3pp_model_class_def_set_stack_ats_(CY_class_name)\


#define COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS_AND_DEFINITIONS(CY_class_name, CY_default_max_size)\
        COYOT3PP_MODEL_CLASS_SET_STACK_DECLARATIONS(CY_class_name, CY_default_max_size)\
        COYOT3PP_MODEL_CLASS_SET_STACK_DEFINITIONS(CY_class_name, CY_default_max_size)





#define COYOT3PP_MODEL_CLASSES_IMPORT_EXPORT_DECLARATIONS(CY_class_left, CY_class_right, CY_params, ...)\
  namespace CY_class_left##Equivalences##Cy_class_right{\
    bool CY_class_left##To##CY_class_right(const CY_class_left& source, CY_class_right& destination);\
    bool CY_class_right##To##CY_class_left(const CY_class_right& source, CY_class_left& destination);\
  }



  #define __coyot3pp_model_classes_import_export_a2b_def(param_left,param_right)\
    destination.param_right() = source.param_left();

  #define __coyot3pp_model_classes_import_export_b2a_def(param_left,param_right)\
    destination.param_left() = source.param_right();

#define COYOT3PP_MODEL_CLASSES_IMPORT_EXPORT_DEFINITIONS(CY_class_left, CY_class_right, CY_params, ...)\
  namespace CY_class_left##Equivalences##Cy_class_right{\
    bool CY_class_left##To##CY_class_right(const CY_class_left& source, CY_class_right& destination){\
      FOR_EACH_PAIR(__coyot3pp_model_class_import_export_a2b_def,__VA_ARGS)\
    }\
    bool CY_class_right##To##CY_class_left(const CY_class_right& source, CY_class_left& destination){\
      FOR_EACH_PAIR(__coyot3pp_model_class_import_export_b2a_def,__VA_ARGS)\
    }\
  }

  