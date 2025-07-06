#pragma once
#include "cyt_foreach_macro_helpers.hpp"
#include "cyt_macro_enum_class.hpp"
#include <functional>


// DECLARATION BEGIN:
    #define cyt3macro_privdec_model_class_constructor_params_decl_param_(CY_NAME, CY_TYPE, CY_DEFVALUE)\
      , const CY_TYPE& CY_NAME

  #define cyt3macro_privdec_model_class_constructor_params_decl_(CY_class_name, CY_enumclass_deps, CY_NAME, CY_TYPE, CY_DEFVALUE , ... ) \
    CY_class_name(\
      const CY_TYPE& CY_NAME\
      IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(cyt3macro_privdec_model_class_constructor_params_decl_param_,PASS_PARAMETERS(CY_enumclass_deps)))\
      FOR_EACH_TRIPLES(cyt3macro_privdec_model_class_constructor_params_decl_param_,__VA_ARGS__) \
    );

  #define cytemacro_privdec_modelclass_declare_types_(CY_prop_name,CY_type,CY_default_value) \
     typedef CY_type CY_prop_name##_t;

  #define cytemacro_modelclass_properties_dec_(V_PROPNAME,V_TYPE,V_DEFVALUE) \
    IFE(V_DEFVALUE)(V_PROPNAME##_t V_PROPNAME##_;) \
    IFN(V_DEFVALUE)(V_PROPNAME##_t V_PROPNAME##_ = V_DEFVALUE;)

  #define cytemacro_model_class_getterssetters_dec_(V_PROPNAME, V_TYPE, V_DEFVALUE) \
    V_PROPNAME##_t  V_PROPNAME() const; \
    V_PROPNAME##_t& V_PROPNAME(); \
    V_PROPNAME##_t  V_PROPNAME(const V_PROPNAME##_t& v );

  #define cyt3macro_model_class_enumclass_declare_types_(CY_prop_name, CY_enumclass_type, CY_default_value)\
    typedef CY_enumclass_type CY_prop_name##_t;
  
  #define cyt3macro_model_class_enumclass_declare_getterssetters_(CY_prop_name, CY_enumclass_type, CY_default_value)\
    CY_prop_name##_t  CY_prop_name() const; \
    CY_prop_name##_t& CY_prop_name(); \
    CY_prop_name##_t  CY_prop_name(const CY_prop_name##_t& v );\
    CY_prop_name##_t  CY_prop_name(const char* v ); \
    CY_prop_name##_t  CY_prop_name(int v );

  #define cytemacro_model_class_enumclass_properties_dec_(CY_prop_name,CY_enumclass_type,CY_default_value) \
    IFE(CY_default_value)(CY_prop_name##_t CY_prop_name##_ = CY_prop_name##_t::UNKNOWN_OR_UNSET;) \
    IFN(CY_default_value)(CY_prop_name##_t CY_prop_name##_ = CY_default_value;)

  #define cyt3macro_privdev_model_class_get_model_template_dec_(CY_class_name, CY_enumclass_deps, ...) \
    static std::string get_model_template();

      #define cyt3macro_privdev_model_class_dec_constr_params_itr_(CY_prop_name,CY_type,CY_default_value)\
        (const CY_type& p_##CY_prop_name)

    #define cyt3macro_privdev_model_class_dec_constr_params(...)\
      CHAIN_COMMA(\
        FOR_EACH_TRIPLES(cyt3macro_privdev_model_class_dec_constr_params_itr_, __VA_ARGS__ ) \
      )

      #define cyt3macro_privdev_model_class_dec_tr_params_itr_(CY_prop_name,CY_type,CY_default_value)\
        (CY_type p_##CY_prop_name)

    #define cyt3macro_privdev_model_class_dec_tr_params(...)\
      CHAIN_COMMA(\
        FOR_EACH_TRIPLES(cyt3macro_privdev_model_class_dec_tr_params_itr_, __VA_ARGS__ ) \
      )

  // ps-priv : begin
  #define cytemacro_privdec_model_class_common_priv_dev_(CY_class_name, CY_parent_class, CY_enumclass_deps, ...) \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(cyt3macro_model_class_enumclass_declare_types_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(cytemacro_privdec_modelclass_declare_types_, __VA_ARGS__) \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(cyt3macro_model_class_enumclass_declare_getterssetters_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(cytemacro_model_class_getterssetters_dec_,__VA_ARGS__)\
        \
        CY_class_name & operator=(const CY_class_name & o); \
        bool            operator==(const CY_class_name & o) const; \
        bool            operator!=(const CY_class_name & o) const; \
        \
        CY_class_name(); \
        CY_class_name(const CY_class_name & o); \
        IFN(CY_parent_class)(CY_class_name(const CY_parent_class & o);)\
        CY_class_name(\
          cyt3macro_privdev_model_class_dec_tr_params(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          cyt3macro_privdev_model_class_dec_tr_params(PASS_PARAMETERS(CY_enumclass_deps))\
        );\
        \
        virtual ~CY_class_name();\
        \
        cyt3macro_privdev_model_class_get_model_template_dec_(CY_class_name, CY_enumclass_deps, __VA_ARGS__) \
        \
      protected: \
        \
        IFN(CY_enumclass_deps)(FOR_EACH_TRIPLES(cytemacro_model_class_enumclass_properties_dec_,PASS_PARAMETERS(CY_enumclass_deps)))\
        \
        FOR_EACH_TRIPLES(cytemacro_modelclass_properties_dec_,__VA_ARGS__)


  #define cyt3macro_privdec_model_class_additional_method_add_(CV_additional_method) \
    CV_additional_method;






// ps-priv : end

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
#define CYT3MACRO_model_class_declarations(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  class CY_class_name\
  IFN(CY_parent_class)(: public CY_parent_class)\
  { \
    public: \
    \
    FOR_EACH(cyt3macro_privdec_model_class_additional_method_add_, PASS_PARAMETERS(CY_additional_methods)) \
    \
    cytemacro_privdec_model_class_common_priv_dev_(CY_class_name, CY_parent_class, CY_enumclass_deps, __VA_ARGS__) \
    \
  };







//
//

//DECLARATION END:


//DEFINITION BEGIN:
  #define cyt3macro_model_class_copyitem_def_(a1,a2,a3) \
    a1##_ = o.a1##_;


  // el resto de las líneas añade '&&' como prefijo
  #define cyt3macro_model_class_eqopitem_line_def_(a1,a2,a3) \
    && (a1##_ == o.a1##_) 

  #define cyt3macro_model_class_eqopitem_lines_def(a1,a2,a3,...)\

  // la primera línea no añade '&&'
  #define cyt3macro_model_class_eqopitems_def_(a1,a2,a3,...) \
    (a1##_ == o.a1##_) \
    FOR_EACH_TRIPLES(cyt3macro_model_class_eqopitem_line_def_, __VA_ARGS__)

    #define cyt3macro_privdev_model_class_get_model_template_def_add_item_(a1,a2,a3)\
      mt += "(" #a2 ")" #a1 IFN(a3)("/default=" #a3 "/") IFE(a3)("//") "\n";

  #define cyt3macro_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  std::string CY_class_name::get_model_template(){\
    std::string mt;\
    FOR_EACH_TRIPLES(cyt3macro_privdev_model_class_get_model_template_def_add_item_,PASS_PARAMETERS(CY_enumclass_deps))\
    FOR_EACH_TRIPLES(cyt3macro_privdev_model_class_get_model_template_def_add_item_,__VA_ARGS__) \
    return mt; \
  }


  #define cyt3macro_model_class_getters_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1() const{ return a1##_;} \
    CY_class_name::a1##_t& CY_class_name::a1(){ return a1##_;} \

  #define cyt3macro_model_class_setters_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1(const CY_class_name::a1##_t& v){ return a1##_ = v;}


  #define cyt3macro_model_class_setters_ecextra_def_(CY_class_name,a1,a2,a3)\
    CY_class_name::a1##_t CY_class_name::a1(const char* v){ \
      a1 = a2##FromString(v); \
      return a1; \
    } \
    CY_class_name::a1##_t CY_class_name::a1(int v){\
      a1 = static_cast<a2>(v);\
      return a1;\
    }


  #define cyt3macro_model_class_def_tr_assign_(a1,a2,a3)\
    a1##_ = p_##a1;



/* constructor por parámetros, extraído para evitar el problema de asignación
   const ptr* sobre ptr*



*/

#define CYT3MACRO_model_class_definitions(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  CY_class_name::CY_class_name()\
    IFN(CY_parent_class)(:CY_parent_class()){} \
  CY_class_name::~CY_class_name(){}\
  IFN(CY_parent_class)(CY_class_name::CY_class_name(const CY_parent_class & o):CY_parent_class(o){}) \
  CY_class_name::CY_class_name(const CY_class_name & o){*this = o;}\
  CY_class_name::CY_class_name(\
          cyt3macro_privdev_model_class_dec_tr_params(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          cyt3macro_privdev_model_class_dec_tr_params(PASS_PARAMETERS(CY_enumclass_deps))\
  ){\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,__VA_ARGS__)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,PASS_PARAMETERS(CY_enumclass_deps))\
  }\
  CY_class_name & CY_class_name::operator=(const CY_class_name & o){ \
    IFN(CY_parent_class)(CY_parent_class::operator=(o);)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_copyitem_def_,__VA_ARGS__)\
    return *this; \
  } \
  \
  cyt3macro_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
  bool CY_class_name::operator==(const CY_class_name & o) const{\
    return (\
    IFN(CY_parent_class)( (CY_parent_class::operator==(o)) && )\
    cyt3macro_model_class_eqopitems_def_(__VA_ARGS__)\
    );\
  }\
  bool CY_class_name::operator!=(const CY_class_name & o) const{\
    return !(*this == o); \
  } \
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\


#define CYT3MACRO_model_class_definitions_no_eq_overload(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  CY_class_name::CY_class_name()\
    IFN(CY_parent_class)(:CY_parent_class()){} \
  CY_class_name::~CY_class_name(){}\
  IFN(CY_parent_class)(CY_class_name::CY_class_name(const CY_parent_class & o):CY_parent_class(o){}) \
  CY_class_name::CY_class_name(const CY_class_name & o){*this = o;}\
  CY_class_name::CY_class_name(\
          cyt3macro_privdev_model_class_dec_tr_params(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          cyt3macro_privdev_model_class_dec_tr_params(PASS_PARAMETERS(CY_enumclass_deps))\
  ){\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,__VA_ARGS__)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,PASS_PARAMETERS(CY_enumclass_deps))\
  }\
  CY_class_name & CY_class_name::operator=(const CY_class_name & o){ \
    IFN(CY_parent_class)(CY_parent_class::operator=(o);)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_copyitem_def_,__VA_ARGS__)\
    return *this; \
  } \
  \
  cyt3macro_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
  bool CY_class_name::operator!=(const CY_class_name & o) const{\
    return !(*this == o); \
  } \
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\

  #define CYT3MACRO_model_class_definitions_no_opsoverload(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, ...) \
  CY_class_name::CY_class_name()\
    IFN(CY_parent_class)(:CY_parent_class()){} \
  CY_class_name::~CY_class_name(){}\
  IFN(CY_parent_class)(CY_class_name::CY_class_name(const CY_parent_class & o):CY_parent_class(o){}) \
  CY_class_name::CY_class_name(const CY_class_name & o){*this = o;}\
  CY_class_name::CY_class_name(\
          cyt3macro_privdev_model_class_dec_tr_params(__VA_ARGS__)\
          IFN(__VA_ARGS__)(IFN(PASS_PARAMETERS(CY_enumclass_deps))(COMMA()))\
          cyt3macro_privdev_model_class_dec_tr_params(PASS_PARAMETERS(CY_enumclass_deps))\
  ){\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,__VA_ARGS__)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_def_tr_assign_,PASS_PARAMETERS(CY_enumclass_deps))\
  }\
  \
  cyt3macro_privdev_model_class_get_model_template_def_(CY_class_name, CY_parent_class, CY_additional_methods, CY_enumclass_deps, __VA_ARGS__)\
  bool CY_class_name::operator!=(const CY_class_name & o) const{\
    return !(*this == o); \
  } \
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,__VA_ARGS__)\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_getters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\
  FOR_EACH_TRIPLES_WITH_01_STATIC(cyt3macro_model_class_setters_def_,CY_class_name,PASS_PARAMETERS(CY_enumclass_deps))\


////////////////////////////////////
////////////////////////////////////






/**
 * @brief Declares an indexed Set / Array class of <CY_class_name>, idexed by <CY_member_index_property>.
 *  Declares a map the type <CY_class_name>MappedSetMapType and a basic methods set:
 *    - insert ; remove ; update ; get ; is_member ; size
 * @param CY_class_name : REQUIRED : class name to be held in the class.
 * @param CY_class_index_property : REQUIRED : property of the class to use as index.
 */
#define CYT3MACRO_model_class_set_mapped_declarations(CY_class_name,CY_member_index_property) \
  class CY_class_name##MappedSet{ \
    public:\
      typedef std::map<CY_class_name::CY_member_index_property##_t,CY_class_name> CY_class_name##MappedSetMapType;\
      typedef std::pair<CY_class_name::CY_member_index_property##_t,CY_class_name> CY_class_name##Pair;\
    \
    CY_class_name##MappedSet(); \
    CY_class_name##MappedSet(const CY_class_name##MappedSet& o); \
    CY_class_name##MappedSet(const std::initializer_list<CY_class_name>& _init);\
    virtual ~CY_class_name##MappedSet();\
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
    bool              update(const CY_class_name & o, bool force = true);\
    bool              is_member(CY_class_name::CY_member_index_property##_t index) const;\
    CY_class_name&    first();\
    CY_class_name     first() const;\
    CY_class_name     get(CY_class_name::CY_member_index_property##_t index) const; \
    CY_class_name&    get(CY_class_name::CY_member_index_property##_t index); \
    std::size_t       size() const;\
    std::size_t       for_each(std::function<bool(const CY_class_name&)> func) const;\
    std::size_t       for_each(std::function<bool(CY_class_name&)> func);\
    std::size_t       for_each_member(std::function<bool(CY_class_name::CY_member_index_property##_t)> func) const;\
    void              clear();\
    \
    protected: \
      CY_class_name##MappedSetMapType map_;\
      mutable std::mutex              mtx_;\
  };





  //def constructors
  #define cyt3macro_model_class_set_mapped_def_constrsdestr_(CY_class_name) \
    CY_class_name##MappedSet::CY_class_name##MappedSet():map_(){ }\
    CY_class_name##MappedSet::CY_class_name##MappedSet(const CY_class_name##MappedSet& o){ *this = o;}\
    CY_class_name##MappedSet::~CY_class_name##MappedSet(){}\
    CY_class_name##MappedSet::CY_class_name##MappedSet(const std::initializer_list<CY_class_name>& _init){\
      for(const CY_class_name& p : _init){\
        insert(p);\
      }\
    }

  //def op =
  #define cyt3macro_model_class_set_mapped_def_opassign_(CY_class_name)\
    CY_class_name##MappedSet& CY_class_name##MappedSet::operator=(const CY_class_name##MappedSet& o){\
      std::lock_guard<std::mutex> guard(mtx_);\
      map_ = o.map_;\
      return *this;\
    }
  //def op !=
  #define cyt3macro_model_class_set_mapped_def_opneq_(CY_class_name)\
    bool CY_class_name##MappedSet::operator!=(const CY_class_name##MappedSet& o) const { return !( *this == o);}
  //def op ==
  #define cyt3macro_model_class_set_mapped_def_opeq_(CY_class_name)\
    bool CY_class_name##MappedSet::operator==(const CY_class_name##MappedSet& o) const {\
      std::lock_guard<std::mutex> guard(mtx_);\
      if(size() != o.size())return false;\
      CY_class_name##MappedSetMapType::const_iterator i1;\
      for(i1 = map_.begin();i1 != map_.end();i1++){\
        if(o.is_member(i1->first) == false){return false;}\
        if(i1->second != o.get(i1->first)){return false;}\
      }\
      return true;\
    }
  //def op []
  #define cyt3macro_model_class_set_mapped_def_opbraq_(CY_class_name,CY_member_index_property)\
    CY_class_name       CY_class_name##MappedSet::operator[](CY_class_name::CY_member_index_property##_t index) const{\
      return get(index);\
    }
  //def insert
  #define cyt3macro_model_class_set_mapped_def_insert_(CY_class_name,CY_member_index_property)\
    bool              CY_class_name##MappedSet::insert(const CY_class_name & o){\
      std::lock_guard<std::mutex> guard(mtx_);\
      if(is_member(o.CY_member_index_property()) == true)return false;\
      std::lock_guard<std::mutex> guard(mtx_);\
      map_.insert(std::make_pair(o.CY_member_index_property(),o));\
      return true;\
    }

  //def removes
  #define cyt3macro_model_class_set_mapped_def_removes_(CY_class_name,CY_member_index_property) \
    bool  CY_class_name##MappedSet::remove(CY_class_name::CY_member_index_property##_t index){\
      std::lock_guard<std::mutex> guard(mtx_);\
      if(is_member(index) == false){return false;}\
      map_.erase(map_.find(index));\
      return true;\
    } \
    bool  CY_class_name##MappedSet::remove(CY_class_name index){\
      std::lock_guard<std::mutex> guard(mtx_);\
      return remove(index.CY_member_index_property());\
    } 

  //def is_member
  #define cyt3macro_model_class_set_mapped_def_is_member(CY_class_name,CY_member_index_property) \
    bool CY_class_name##MappedSet::is_member(CY_class_name::CY_member_index_property##_t index) const {\
      std::lock_guard<std::mutex> guard(mtx_);\
      return (map_.find(index) != map_.end());\
    }

  #define cyt3macro_model_class_set_mapped_def_get(CY_class_name,CY_member_index_property)\
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

  #define cyt3macro_model_class_set_mapped_def_size_(CY_class_name,CY_member_index_property)\
    std::size_t       CY_class_name##MappedSet::size() const{\
      return map_.size();\
    }\
    CY_class_name& CY_class_name##MappedSet::first(){\
      return map_.begin()->second;\
    }\
    CY_class_name CY_class_name##MappedSet::first() const{\
      return map_.begin()->second;\
    }

  #define cyt3macro_model_class_set_mapped_def_clear_(CY_class_name,CY_member_index_property)\
    void CY_class_name##MappedSet::clear(){\
      std::lock_guard<std::mutex> guard(mtx_);\
      map_.clear();\
    }

  #define cyt3macro_model_class_set_mapped_def_update_(CY_class_name,CY_member_index_property)\
    bool              CY_class_name##MappedSet::update(const CY_class_name & o, bool force){\
      std::lock_guard<std::mutex> guard(mtx_);\
      CY_class_name##MappedSetMapType::iterator i1 = map_.find(o.CY_member_index_property());\
      if(i1 == map_.end())if(force == true)return insert(o);else return false;\
      i1->second = o;\
      return true;\
    }

  #define cyt3macro_model_class_set_mapped_def_foreachs_(CY_class_name,CY_member_index_property)\
      std::size_t CY_class_name##MappedSet::for_each(std::function<bool(const CY_class_name&)> func) const{\
        std::lock_guard<std::mutex> guard(mtx_);\
        if(size() == 0)return 0;\
        CY_class_name##MappedSetMapType::const_iterator it;\
        std::size_t res = 0;\
        for(it = map_.begin();it != map_.end();++it){\
          if(func(it->second) == true) ++res;\
        }\
        return res;\
      }\
      std::size_t CY_class_name##MappedSet::for_each(std::function<bool(CY_class_name&)> func){\
        std::lock_guard<std::mutex> guard(mtx_);\
        if(size() == 0)return 0;\
        CY_class_name##MappedSetMapType::iterator it;\
        std::size_t res = 0;\
        for(it = map_.begin();it != map_.end();++it){\
          if(func(it->second) == true) ++res;\
        }\
        return res;\
      }\
      std::size_t CY_class_name##MappedSet::for_each_member(std::function<bool(CY_class_name::CY_member_index_property##_t)> func) const{\
        if(size() == 0)return 0;\
        CY_class_name##MappedSetMapType::const_iterator it;\
        std::size_t res = 0;\
        for(it = map_.begin();it != map_.end();++it){\
          if(func(it->first) == true) ++res;\
        }\
        return res;\
      }

/**
 * @brief model set definitions
 * @param CY_class_name : REQUIRED : base class name
 * @param CY_member_index_property : REQUIRED : base class index property
 * 
 */
#define CYT3MACRO_model_class_set_mapped_definitions(CY_class_name,CY_member_index_property) \
  cyt3macro_model_class_set_mapped_def_constrsdestr_(CY_class_name)\
  \
  cyt3macro_model_class_set_mapped_def_opassign_(CY_class_name)\
  \
  cyt3macro_model_class_set_mapped_def_opeq_(CY_class_name)\
  \
  cyt3macro_model_class_set_mapped_def_opneq_(CY_class_name)\
  \
  cyt3macro_model_class_set_mapped_def_opbraq_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_insert_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_removes_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_is_member(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_get(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_size_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_clear_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_update_(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_def_foreachs_(CY_class_name,CY_member_index_property)




//'Droid Sans Mono', 'monospace', monospace






#define CYT3MACRO_model_class_set_stack_declarations(CY_class_name, CY_default_max_size)\
  class CY_class_name##Stack {\
    public: \
      typedef std::vector<CY_class_name> CY_class_name##StackType;\
      CY_class_name##Stack();\
      CY_class_name##Stack(const std::initializer_list<CY_class_name>& init);\
      CY_class_name##Stack(const CY_class_name##Stack& o);\
      virtual ~CY_class_name##Stack();\
      std::size_t  size()            const;\
      void         resize(size_t s);\
      std::size_t  max_size()        const;\
      std::size_t  max_size(std::size_t v);\
      bool    push_front(const CY_class_name & o);\
      bool    push_front(const CY_class_name##Stack& o);\
      bool    push(const CY_class_name & o);\
      bool    push(const CY_class_name##Stack& o);\
      bool    push_back(const CY_class_name & o);\
      bool    push_back(const CY_class_name##Stack& o);\
      CY_class_name front() const;\
      CY_class_name back() const;\
      void    clear();\
      CY_class_name##Stack& operator=(const CY_class_name##Stack& o);\
      bool                  operator==(const CY_class_name##Stack& o) const;\
      bool                  operator!=(const CY_class_name##Stack& o) const;\
      CY_class_name&        operator[](int index);\
      const CY_class_name&        operator[](int index) const;\
      std::size_t forEach(std::function<bool(CY_class_name &)> func);\
      std::size_t forEach(std::function<bool(const CY_class_name &)> func) const;\
      std::size_t for_each(std::function<bool(CY_class_name &)> func);\
      std::size_t for_each(std::function<bool(const CY_class_name &)> func) const;\
      std::size_t for_each_and_remove(std::function<bool(CY_class_name &)> func);\
      CY_class_name& at(int index);\
      const CY_class_name& at(int index) const;\
    protected:\
      std::vector<CY_class_name>                stack_;\
      mutable std::mutex                        stack_mtx_;\
      std::size_t                              default_max_size_;\
  };





    #define cyt3macro_model_class_def_set_stack_opeq_(CY_class_name) \
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


      

      #define cyt3macro_model_class_def_set_stack_foreachs_(CY_class_name) \
        std::size_t CY_class_name##Stack::forEach(std::function<bool(CY_class_name &)> func){\
          return for_each(func);\
        }\
        std::size_t CY_class_name##Stack::for_each(std::function<bool(CY_class_name &)> func){ \
          CY_class_name##StackType::iterator i; \
          std::lock_guard<std::mutex> guard(stack_mtx_);\
          std::size_t ok = 0; \
          for(i=stack_.begin();i != stack_.end(); ++i){ \
            if(func(*i)==true)++ok; \
          } \
          return ok; \
        } \
        std::size_t CY_class_name##Stack::forEach(std::function<bool(const CY_class_name &)> func)const{\
          return forEach(func);\
        }\
        std::size_t CY_class_name##Stack::for_each(std::function<bool(const CY_class_name &)> func)const{\
          CY_class_name##StackType::const_iterator i; \
          std::size_t ok = 0; \
          std::lock_guard<std::mutex> guard(stack_mtx_);\
          for(i=stack_.begin();i != stack_.end(); ++i){ \
            if(func(*i)==true) ++ok; \
          } \
          return ok; \
        }\
        std::size_t CY_class_name##Stack::for_each_and_remove(std::function<bool(CY_class_name &)> func){\
          if(stack_.size() == 0)return 0;\
          std::lock_guard<std::mutex> guard(stack_mtx_);\
          CY_class_name##StackType::iterator i = stack_.begin();\
          std::size_t s = 0;\
          do{\
            if(func(*i) == true){s++;i++;}\
            else{stack_.erase(i++);}\
          }while(i != stack_.end());\
          return s;\
        }




    #define cyt3macro_model_class_def_set_stack_push_(CY_class_name) \
      bool    CY_class_name##Stack::push_front(const CY_class_name & o){ \
        stack_.insert(stack_.begin(), o);\
        if(stack_.size()>= default_max_size_){ \
          stack_.erase(stack_.begin()); \
        }\
        return true;\
      }\
      bool    CY_class_name##Stack::push_front(const CY_class_name##Stack& o){ \
        size_t result = o.for_each([&](const CY_class_name & item){ \
          stack_.insert(stack_.begin(), item); \
          if(stack_.size()>= default_max_size_){ \
            stack_.erase(stack_.begin()); \
          }\
          return true;\
        });\
        return result == o.size();\
      }\
      bool    CY_class_name##Stack::push(const CY_class_name & o){ \
        return push_back(o);\
      }\
      bool    CY_class_name##Stack::push_back(const CY_class_name & o){ \
        stack_.push_back(o); \
        if(stack_.size()>= default_max_size_){ \
          stack_.erase(stack_.begin()); \
        }\
        return true;\
      }\
      bool    CY_class_name##Stack::push(const CY_class_name##Stack& o){ \
        return push_back(o);\
      }\
      bool    CY_class_name##Stack::push_back(const CY_class_name##Stack& o){ \
        o.for_each([&](const CY_class_name & item){\
          push_back(item);\
          return true;\
        });\
        return true;\
      }

    #define cyt3macro_model_class_def_set_stack_shorts_(CY_class_name) \
      std::size_t   CY_class_name##Stack::size() const{return stack_.size();} \
      void          CY_class_name##Stack::resize(std::size_t sz){stack_.resize(sz);}\
      std::size_t   CY_class_name##Stack::max_size() const{return default_max_size_;} \
      std::size_t   CY_class_name##Stack::max_size(std::size_t v){return (default_max_size_ = v);} \
      void          CY_class_name##Stack::clear(){stack_.clear();} \
      CY_class_name CY_class_name##Stack::front() const{return stack_.front();} \
      CY_class_name CY_class_name##Stack::back() const{return stack_.back();} \

    #define cyt3macro_model_class_def_set_stack_constrdestr_(CY_class_name, CY_default_max_size) \
      CY_class_name##Stack::CY_class_name##Stack():stack_(){\
      IFE(CY_default_max_size)(default_max_size_ = (size_t)-1;)\
      IFN(CY_default_max_size)(default_max_size_ = CY_default_max_size;)}\
      CY_class_name##Stack::CY_class_name##Stack(const CY_class_name##Stack& o){*this = o;} \
      CY_class_name##Stack::~CY_class_name##Stack(){}\
      CY_class_name##Stack::CY_class_name##Stack(const std::initializer_list<CY_class_name>& init)\
      {for(const CY_class_name& i : init)stack_.push_back(i);}\


    #define cyt3macro_model_class_def_set_stack_ats_(CY_class_name)\
    CY_class_name& CY_class_name##Stack::at(int index){\
      return stack_[index];\
    }\
    const CY_class_name& CY_class_name##Stack::at(int index) const{\
      return stack_[index];\
    }\
    CY_class_name& CY_class_name##Stack::operator[](int index){\
      return stack_[index];\
    }\
    const CY_class_name& CY_class_name##Stack::operator[](int index) const{\
      return stack_[index];\
    }



  #define CYT3MACRO_model_class_set_stack_definitions(CY_class_name, CY_default_max_size) \
    \
    cyt3macro_model_class_def_set_stack_constrdestr_(CY_class_name, CY_default_max_size) \
    \
    cyt3macro_model_class_def_set_stack_shorts_(CY_class_name) \
    \
    cyt3macro_model_class_def_set_stack_opeq_(CY_class_name) \
    \
    cyt3macro_model_class_def_set_stack_foreachs_(CY_class_name) \
    \
    cyt3macro_model_class_def_set_stack_push_(CY_class_name)\
    \
    cyt3macro_model_class_def_set_stack_ats_(CY_class_name)




//
// PORT
//

  

#define CYT3MACRO_model_class_port_declarations(CY_target_class, CY_source_class, CY_func_to_target_extra , CY_func_to_source_extra, ...)\
  struct CY_target_class##Port{\
    IFN(CY_func_to_source_extra)(static bool CY_func_to_source_extra(const CY_target_class& i COMMA() CY_source_class& o);)\
    IFN(CY_func_to_target_extra)(static bool CY_func_to_target_extra(const CY_source_class& i COMMA() CY_target_class& o);)\
    static bool To##CY_source_class(const CY_target_class& i COMMA() CY_source_class& o);\
    static bool To##CY_target_class(const CY_source_class& i COMMA() CY_target_class& o);\
  };



  #define cyt3macro_model_class_port_def_tosou_item_(cy_prop_target, cy_prop_destination)\
    o.cy_prop_destination = i.cy_prop_target ;

  #define cyt3macro_model_class_port_def_totar_item_(cy_prop_target, cy_prop_destination)\
    o.cy_prop_target = i.cy_prop_destination;

#define CYT3MACRO_model_class_port_definitions(CY_target_class, CY_source_class, CY_func_to_target_extra , CY_func_to_source_extra, ...)\
  bool CY_target_class##Port::To##CY_source_class(const CY_target_class& i COMMA() CY_source_class& o){\
      FOR_EACH_PAIR(cyt3macro_model_class_port_def_tosou_item_, __VA_ARGS__)\
      IFN(CY_func_to_source_extra)(return CY_func_to_source_extra(i COMMA() o);)\
    IFE(CY_func_to_source_extra)(return true;)\
  }\
  bool CY_target_class##Port::To##CY_target_class(const CY_source_class& i COMMA() CY_target_class& o){\
      FOR_EACH_PAIR(cyt3macro_model_class_port_def_totar_item_, __VA_ARGS__)\
      IFN(CY_func_to_target_extra)(return CY_func_to_target_extra(i COMMA() o);)\
      IFE(CY_func_to_target_extra)(return true;)\
  }




/**
 * @brief ease my way to "transport" contents from one object to other. will create:\
 *  * Cyt3Port<class-left>To<class-right>(const class-left& src, class-right& dest);
 *  * Cyt3Port<class-left>To<class-right>(const class-right& src, class-left& dest);
 * @param CY_class_left : class type 1
 * @param CY_class_right : class type 2
 * @param CY_namespace : not required, the namespace where it will be included
 * @param CY_param_a2b_extras : not required, extra function name when transporting left to right
 * @param CY_param_b2a_extras : not required, extra function name when transporting right to left
 * @param ... : pairs of equivalences
 */
#define CYT3MACRO_model_classes_import_export_declarations(CY_class_left, CY_class_right, CY_namespace, CY_param_a2b_extras, CY_param_b2a_extras, ...)\
  IFN(CY_namespace)(namespace CY_namespace{)\
    bool Cyt3Port##CY_class_left##To##CY_class_right(const CY_class_left& source COMMA() CY_class_right& destination);\
    bool Cyt3Port##CY_class_right##To##CY_class_left(const CY_class_right& source COMMA() CY_class_left& destination);\
    IFN(CY_param_a2b_extras)(bool CY_param_a2b_extras(const CY_class_left& source COMMA() CY_class_right& destination);)\
    IFN(CY_param_b2a_extras)(bool CY_param_b2a_extras(const CY_class_right& source COMMA() CY_class_left& destination);)\
  IFN(CY_namespace)(})
  
  #define cyt3macro_model_classes_import_export_a2b_def(param_left,param_right)\
    destination.param_right = source.param_left;

  #define cyt3macro_model_classes_import_export_b2a_def(param_left,param_right)\
    destination.param_left = source.param_right;

#define CYT3MACRO_model_classes_import_export_definitions(CY_class_left, CY_class_right, CY_namespace, CY_param_a2b_extras, CY_param_b2a_extras, ...)\
  IFN(CY_namespace)(namespace CY_namespace{)\
    bool Cyt3Port##CY_class_left##To##CY_class_right(const CY_class_left& source, CY_class_right& destination){\
      FOR_EACH_PAIR(cyt3macro_model_classes_import_export_a2b_def,__VA_ARGS__)\
      IFE(CY_param_a2b_extras)(return true;)\
      IFN(CY_param_a2b_extras)(return CY_param_a2b_extras(source,destination);)\
    }\
    bool Cyt3Port##CY_class_right##To##CY_class_left(const CY_class_right& source, CY_class_left& destination){\
      FOR_EACH_PAIR(cyt3macro_model_classes_import_export_b2a_def,__VA_ARGS__)\
      IFE(CY_param_b2a_extras)(return true;)\
      IFN(CY_param_a2b_extras)(return CY_param_b2a_extras(source,destination);)\
    }\
  IFN(CY_namespace)(})
