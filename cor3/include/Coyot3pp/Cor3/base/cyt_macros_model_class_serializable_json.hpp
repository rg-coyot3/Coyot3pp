#pragma once

#include "cyt_foreach_macro_helpers.hpp"
#include <mutex>



  #define cyt3macro_model_class_serializable_json_field_dec_(CY_prop_source,CY_json_tag,CY_prop_jscast_type)\
    static const char* CY_prop_source;


  #define cyt3macro_model_class_serializable_json_field_derived_dec_(...)\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_field_dec_,__VA_ARGS__)

/**
 * @brief 
 * @param CY_props_jsio : described in set of triples, the properties that have JsIO derived classes.
 *   i.e. (propj1,ClassName1,"prop_j_1",propj2,ClassName2,"prop_j_2")
 * @param CY_props_enum_classes: (prope1,EnumClass1."prop_e_1",prope2,EnumClass2,"prop_e_2")
 * @param ... described in sets of triples, [ ]
 */
#define CYT3MACRO_model_class_serializable_json_declarations(CY_class_name, \
  CY_jsio_parent_class, \
  CY_jsonizations_postfuncs_suffix, \
  CY_props_jsios,\
  CY_props_enum_classes,\
  ...) \
  class CY_class_name##JsIO \
    :public coyot3::tools::JsonSerializablePacketBase \
    ,public CY_class_name{\
    public:\
      struct JsFields{ \
        FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_field_dec_,PASS_PARAMETERS(CY_props_jsios))\
        FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_field_dec_,PASS_PARAMETERS(CY_props_enum_classes))\
        FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_field_dec_,__VA_ARGS__)\
      };\
      \
      CY_class_name##JsIO(); \
      CY_class_name##JsIO(const CY_class_name & o); \
      virtual ~CY_class_name##JsIO();\
      IFN(CY_jsonizations_postfuncs_suffix)(virtual Json::Value to_json_post_process(Json::Value js) const;)\
      IFN(CY_jsonizations_postfuncs_suffix)(virtual bool from_json_pre_process(const Json::Value& source);)\
      \
      virtual Json::Value   to_json() const override;\
      virtual bool          from_json(const Json::Value& source) override;\
      static std::string    get_serialization_model_template(int l__ = 0);\
      std::string    check_input_model(const Json::Value& source, int l__ = 0);\
      protected:\
        static const char* version_;\
  };\
  Json::Value&  operator<<(::Json::Value& o,const CY_class_name & i);\
  bool          operator<<(CY_class_name & o, const std::string& i);\
  bool          operator<<(CY_class_name & o, const Json::Value& i);\
  std::ostream& operator<<(::std::ostream& o, const CY_class_name & i);



// removed rapidjson support
//      virtual bool          from_json(const rapidjson::Value& source) override;





  #define cytemacro_model_class_serializable_json_def_json_field_(CY_class_name, CY_prop_source, CY_json_tag, CY_prop_type) \
    const char* CY_class_name::JsFields::CY_prop_source = CY_json_tag;


  #define cyt3macro_model_class_serializable_json_def_tojson_jsioitem_(CY_prop_source, CY_json_tag, CY_jsio_type)\
    js[JsFields::CY_prop_source] = CY_jsio_type##JsIO(CY_prop_source##_).to_json();

  #define cyt3macro_model_class_json_serializable_def_tojson_enumclass_(CY_prop_source, CY_json_tag, CY_jsio_type)\
    js[JsFields::CY_prop_source] = static_cast<int>(CY_prop_source##_);


      
  // constructors
  #define cyt3macro_model_class_serializable_json_def_constructors_(CY_class_name)\
    CY_class_name##JsIO::CY_class_name##JsIO()\
    :JsonSerializablePacketBase() \
    ,CY_class_name(){}\
    CY_class_name##JsIO::CY_class_name##JsIO(const CY_class_name & o)\
      :JsonSerializablePacketBase() \
      ,CY_class_name(o){} \
    CY_class_name##JsIO::~CY_class_name##JsIO(){} \



    #define cyt3macro_model_class_json_serializable_def_tojson_enumclass_item_(CY_prop_source,CY_json_tag,CY_enum_class) \
      js[JsFields::CY_prop_source] = CY_enum_class##ToString(CY_prop_source##_); 

    #define cyt3macro_model_class_json_serializable_def_tojson_jsio_item_(CY_prop_source,CY_json_tag,CY_class_name) \
      js[JsFields::CY_prop_source] = CY_class_name##JsIO(CY_prop_source##_).to_json(); 

    #define cyt3macro_model_class_json_serializable_def_tojson_item_(CY_prop_source, CY_json_tag,CY_prop_type) \
      js[JsFields::CY_prop_source] = \
          IFE(CY_prop_type)(coyot3::tools::as_json( CY_prop_source##_ );) \
          IFN(CY_prop_type)(coyot3::tools::as_json( static_cast<CY_prop_type>(CY_prop_source##_) );) \

  #define cyt3macro_model_class_serializable_json_def_to_json_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,...)\
    Json::Value CY_class_name##JsIO::to_json() const{\
      Json::Value js;\
      IFN(CY_jsio_parent)(js = CY_jsio_parent##JsIO(*this).to_json();)\
      FOR_EACH_TRIPLES(cyt3macro_model_class_json_serializable_def_tojson_item_,__VA_ARGS__) \
      IFN(CY_props_jsios)(FOR_EACH_TRIPLES(cyt3macro_model_class_json_serializable_def_tojson_jsio_item_,PASS_PARAMETERS(CY_props_jsios))) \
      IFN(CY_props_enum_classes)(FOR_EACH_TRIPLES(cyt3macro_model_class_json_serializable_def_tojson_enumclass_item_,PASS_PARAMETERS(CY_props_enum_classes))) \
      IFE(CY_jsonizations_postfuncs_suffix)(return js;)\
      IFN(CY_jsonizations_postfuncs_suffix)(return to_json_post_process(js);)\
    }


    #define cyt3macro_model_class_serializable_json_def_fromjson_enumclass_item_(CY_prop_source, CY_json_tag,CY_enum_class)\
      std::string CY_prop_source##_buffer_;\
      res &= coyot3::tools::json_import_value(source,JsFields::CY_prop_source,CY_prop_source##_buffer_);\
      if(res==true)CY_prop_source##_ = CY_enum_class##FromString(CY_prop_source##_buffer_.c_str());

    #define cyt3macro_model_class_serializable_json_def_fromjson_jsio_item_(CY_prop_source, CY_json_tag, CY_class_name) \
      if(source.isMember(JsFields::CY_prop_source) == false){\
        res &= false;\
      }else{\
        CY_class_name##JsIO CY_prop_source##_jsio_buffer; \
        res &= CY_prop_source##_jsio_buffer.from_json(source[JsFields::CY_prop_source]);\
        CY_prop_source##_ = CY_prop_source##_jsio_buffer;\
      }

    #define cyt3macro_model_class_serializable_json_def_fromjson_item_(CY_prop_source, CY_json_tag, CY_prop_jscast_type) \
      IFE(CY_prop_jscast_type)(res &= coyot3::tools::json_import_value(source, CY_json_tag, CY_prop_source##_);) \
      IFN(CY_prop_jscast_type)(CY_prop_jscast_type CY_prop_source##__buffer; res &= coyot3::tools::json_import_value(source, CY_json_tag, CY_prop_source##__buffer); CY_prop_source##_ = static_cast<CY_prop_source##_t>(CY_prop_source##__buffer);) 


  #define cyt3macro_model_class_serializable_json_def_fromjson_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,...)\
    bool CY_class_name##JsIO::from_json(const Json::Value& source){\
      bool res = true;\
      IFN(CY_jsonizations_postfuncs_suffix)(res &= from_json_pre_process(source);)\
      IFN(CY_jsio_parent)(CY_jsio_parent##JsIO buffer; res &= buffer.from_json(source);CY_jsio_parent::operator=(buffer);)\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_fromjson_item_,__VA_ARGS__) \
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_fromjson_jsio_item_, PASS_PARAMETERS(CY_props_jsios)) \
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_fromjson_enumclass_item_, PASS_PARAMETERS(CY_props_enum_classes)) \
      return res;\
    }



    #define cyt3macro_model_class_serializable_json_def_modelcheck_enumclass_item_(CY_prop_source, CY_json_tag, CY_enum_class)\
      if(source.isMember(JsFields::CY_prop_source) == false){\
       sstr << coyot3::tools::indentation(l__) << #CY_json_tag " : !!! NOT PRESENT !!!" << std::endl;\
      }else{\
        std::string b;\
        if(source[ JsFields::CY_prop_source ].isString() == false){\
          sstr << coyot3::tools::indentation(l__) << #CY_json_tag " : !!! TYPE ERROR !!!" << std::endl;\
        }else{\
          sstr << coyot3::tools::indentation(l__) << #CY_json_tag " : ok [" << source[ JsFields::CY_prop_source ] << "]" << std::endl;\
        }\
      }

    #define cyt3macro_model_class_serializable_json_def_modelcheck_jsio_item_(CY_prop_source, CY_json_tag, CY_jsio_class)\
      if(source.isMember(JsFields::CY_prop_source) == false){\
        sstr << coyot3::tools::indentation(l__) << #CY_json_tag " : !!! NOT PRESENT !!!" << std::endl;\
      }else{\
        CY_jsio_class##JsIO CY_prop_source##_jsio_buffer;\
        sstr << coyot3::tools::indentation(l__) << #CY_json_tag " : " << std::endl;\
        sstr << CY_prop_source##_jsio_buffer.check_input_model(source[JsFields::CY_prop_source], l__ + 2);\
      }


    #define cyt3macro_model_class_serializable_json_def_modelcheck_item_(CY_prop_source, CY_json_tag, CY_prop_jscast_type)\
      if(source.isMember(JsFields::CY_prop_source) == false){\
        sstr << coyot3::tools::indentation(l__) << #CY_json_tag  " : !!! NOT PRESENT !!!" << std::endl;\
      }else{\
        IFN(CY_prop_jscast_type)(CY_prop_jscast_type b;)\
        IFE(CY_prop_jscast_type)(CY_prop_source##_t b;)\
        if(coyot3::tools::json_import_value(source, CY_json_tag, b) == false){\
          sstr << coyot3::tools::indentation(l__) <<  #CY_json_tag " : !!! TYPE ERROR !!!" << std::endl;\
        }else{\
          sstr << coyot3::tools::indentation(l__) <<  #CY_json_tag " : ok [" << b << "]" << std::endl;\
        }\
      }\

  #define cyt3macro_model_class_serializable_json_def_modelcheck_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,...)\
    std::string CY_class_name##JsIO::check_input_model(const Json::Value& source, int l__){\
      std::stringstream sstr;\
      sstr << coyot3::tools::indentation(l__) << ": " #CY_class_name "JsIO  : checking : " << std::endl;\
      IFN(CY_jsio_parent)(CY_jsio_parent##JsIO buffer; sstr << buffer.check_input_model(source, l__ + 2);)\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_modelcheck_enumclass_item_, PASS_PARAMETERS(CY_props_enum_classes))\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_modelcheck_jsio_item_, PASS_PARAMETERS(CY_props_jsios))\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_modelcheck_item_ , __VA_ARGS__)\
      return sstr.str();\
    }



    #define cyt3macro_model_class_serializable_json_def_fromrjson_item_(CY_prop_source, CY_json_tag, CY_prop_jscast_type) \
      IFE(CY_prop_jscast_type)(res &= coyot3::tools::rjson_import_value(source, CY_json_tag, CY_prop_source##_);) \
      IFN(CY_prop_jscast_type)(CY_prop_jscast_type CY_prop_source##__buffer; res &= coyot3::tools::rjson_import_value(source, CY_json_tag, CY_prop_source##__buffer); CY_prop_source##_ = static_cast<CY_prop_source##_t>(CY_prop_source##__buffer);) 

    #define cyt3macro_model_class_serializable_json_def_fromrjson_jsio_item_(CY_prop_source, CY_json_tag, CY_class_name) \
      if(coyot3::tools::rjson_has_member(source,JsFields::CY_prop_source) == false){\
        res &= false;\
      }else{\
        CY_class_name##JsIO CY_prop_source##_jsio_buffer; \
        res &= CY_prop_source##_jsio_buffer.from_json(source[JsFields::CY_prop_source]);\
        CY_prop_source##_ = CY_prop_source##_jsio_buffer;\
      }


    #define cyt3macro_model_class_serializable_json_def_fromrjson_enumclass_item_(CY_prop_source, CY_json_tag,CY_enum_class)\
      std::string CY_prop_source##_buffer_;\
      res &= coyot3::tools::rjson_import_value(source,JsFields::CY_prop_source,CY_prop_source##_buffer_);\
      if(res==true)CY_prop_source##_ = CY_enum_class##FromString(CY_prop_source##_buffer_.c_str());



  #define cyt3macro_model_class_serializable_json_def_operator_ostream_(CY_class_name)\
    std::ostream& operator<<(::std::ostream& o, const CY_class_name & i){\
      return (o << CY_class_name##JsIO(i).to_json());\
    }

  #define cyt3macro_model_class_serializable_json_def_operator_json_(CY_class_name)\
    Json::Value& operator<<(::Json::Value& o, const CY_class_name & i){\
      return (o = CY_class_name##JsIO(i).to_json());\
    }

  #define cyt3macro_model_class_serializable_json_def_operator_classname_(CY_class_name)\
    bool operator<<(CY_class_name & o, const Json::Value& i){\
      bool res;CY_class_name##JsIO buffer(o); \
      res = buffer.from_json(i);\
      o = buffer; \
      return res;\
    }


    #define cyt3macro_model_class_serializable_json_def_getmodel_item_(CY_prop_source, CY_json_tag, CY_class_name)\
      ret += coyot3::tools::indentation(l__) + #CY_json_tag " => " #CY_prop_source "\n";\

    #define cyt3macro_model_class_serializable_json_def_getmodel_jsioitem_(CY_prop_source, CY_json_tag, CY_class_name)\
      ret += coyot3::tools::indentation(l__) + #CY_json_tag " => " #CY_prop_source "\n";\
      ret += CY_class_name##JsIO::get_serialization_model_template(l__+1);\

  #define cyt3macro_model_class_serializable_json_def_operator_stdstring_(CY_class_name)\
    bool operator<<(CY_class_name & o, const std::string& i){\
      bool res;CY_class_name##JsIO buffer(o); \
      res = buffer.from_json_string(i);\
      o = buffer; \
      return res;\
    }

  #define cyt3macro_model_class_serializable_json_def_getmodel_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,...)\
    std::string CY_class_name##JsIO::get_serialization_model_template(int l__){\
      std::string ret; \
      IFN(CY_jsio_parent)(ret = coyot3::tools::indentation(l__) + CY_jsio_parent##JsIO::get_serialization_model_template(l__);)\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_getmodel_jsioitem_, PASS_PARAMETERS(CY_props_jsios));\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_getmodel_item_, PASS_PARAMETERS(CY_props_enum_classes));\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_json_def_getmodel_item_, __VA_ARGS__);\
      return ret;\
    }

/**
 * @brief 
 * @param CY_class_name : REQUIRED 
 * @param CY_jsonizations_postfuncs_suffix : if present (any character or string) will require for the user to implement to_json_post_process and from_json_pre_process
 * @param CY_props_jsios : OPTIONAL : if not defined, at least there must be parentheses.
 * @param CY_props_enum_classes : OPTIONAL : if not defined, at least there must be parentheses.
 * @param ... : REQUIRED : in triples
 */
#define CYT3MACRO_model_class_serializable_json_definitions(CY_class_name, \
                                                            CY_jsio_parent, \
                                                            CY_jsonizations_postfuncs_suffix, \
                                                            CY_props_jsios,\
                                                            CY_props_enum_classes,\
                                                            ...) \
  \
  IFN(CY_props_jsios)(FOR_EACH_TRIPLES_WITH_01_STATIC(cytemacro_model_class_serializable_json_def_json_field_, CY_class_name##JsIO, PASS_PARAMETERS(CY_props_jsios))) \
  \
  FOR_EACH_TRIPLES_WITH_01_STATIC(cytemacro_model_class_serializable_json_def_json_field_, CY_class_name##JsIO, PASS_PARAMETERS(CY_props_enum_classes)) \
  \
  FOR_EACH_TRIPLES_WITH_01_STATIC(cytemacro_model_class_serializable_json_def_json_field_, CY_class_name##JsIO, __VA_ARGS__) \
  \
  cyt3macro_model_class_serializable_json_def_constructors_(CY_class_name)\
  \
  cyt3macro_model_class_serializable_json_def_to_json_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,__VA_ARGS__);\
  \
  cyt3macro_model_class_serializable_json_def_fromjson_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,__VA_ARGS__);\
  \
  cyt3macro_model_class_serializable_json_def_getmodel_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,__VA_ARGS__)\
  \
  cyt3macro_model_class_serializable_json_def_modelcheck_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios, CY_props_enum_classes, __VA_ARGS__)\
  \
  cyt3macro_model_class_serializable_json_def_operator_ostream_(CY_class_name)\
  \
  cyt3macro_model_class_serializable_json_def_operator_json_(CY_class_name)\
  \
  cyt3macro_model_class_serializable_json_def_operator_classname_(CY_class_name)\
  \
  cyt3macro_model_class_serializable_json_def_operator_stdstring_(CY_class_name)\
  
// REMOVED RAPIDJSON SUPPORT
//  cyt3macro_model_class_serializable_json_def_fromrapidjson_(CY_class_name, CY_jsio_parent, CY_jsonizations_postfuncs_suffix, CY_props_jsios,CY_props_enum_classes,__VA_ARGS__);
  


///
///
/// JsIO class json serializable for MappedSet derivations
///
///   serializables and deserializables as arrays
///
///

#define CYT3MACRO_model_class_set_mapped_serializable_json_declarations(CY_class_name,CY_member_index_property)\
  class CY_class_name##MappedSetJsIO \
    : public coyot3::tools::JsonSerializablePacketBase \
    , public CY_class_name##MappedSet {\
    public:\
      CY_class_name##MappedSetJsIO();\
      CY_class_name##MappedSetJsIO(const CY_class_name##MappedSet& o);\
      virtual ~CY_class_name##MappedSetJsIO();\
      \
      virtual Json::Value to_json() const override;\
      virtual bool        from_json(const Json::Value& source) override;\
      static std::string get_serialization_model_template(int l__ = 0);\
      std::string check_input_model(const Json::Value& source, int l__ = 0);\
  };\
  Json::Value   operator<<(::Json::Value& o, const CY_class_name##MappedSet& i);\
  bool          operator<<(CY_class_name & o, const Json::Value& i);\
  std::ostream& operator<<(::std::ostream& o, const CY_class_name##MappedSet& i);

// REMOVED RAPIDJSON SUPPORT
//      virtual bool        from_json(const rapidjson::Value& source) override;

  #define cyt3macro_model_class_set_mapped_serializable_json_def_constrdestr_(CY_class_name,CY_member_index_property)\
    CY_class_name##MappedSetJsIO::CY_class_name##MappedSetJsIO()\
    :JsonSerializablePacketBase()\
    ,CY_class_name##MappedSet(){}\
    CY_class_name##MappedSetJsIO::CY_class_name##MappedSetJsIO(const CY_class_name##MappedSet& o)\
    :JsonSerializablePacketBase()\
    ,CY_class_name##MappedSet(o){}\
    CY_class_name##MappedSetJsIO::~CY_class_name##MappedSetJsIO(){}

  #define cyt3macro_model_class_set_mapped_serializable_json_def_to_json_(CY_class_name,CY_member_index_property)\
    Json::Value CY_class_name##MappedSetJsIO::to_json() const{\
      Json::Value js(Json::arrayValue);\
      CY_class_name##MappedSetMapType::const_iterator i1;\
      for(i1 = map_.begin();i1 != map_.end(); ++i1){\
        js.append(CY_class_name##JsIO(i1->second).to_json());\
      }\
      return js; \
    }

  #define cyt3macro_model_class_set_mapped_serializable_json_def_from_json_(CY_class_name,CY_member_index_property) \
    bool CY_class_name##MappedSetJsIO::from_json(const Json::Value& source){\
      bool res = true;\
      if(source.isArray() == false)return false;\
      Json::Value::ArrayIndex i, s = source.size();\
      for(i = 0; i < s; ++i){\
        CY_class_name##JsIO buffer;\
        bool bres = true; \
        res &= bres = buffer.from_json(source[i]); \
        if(bres == true){ \
          res &= insert(buffer);\
        }\
      }\
      return res;\
    }
  
    #define cyt3macro_model_class_set_mapped_serializable_def_check_model_(CY_class_name, CY_member_index_property)\
    std::string CY_class_name##MappedSetJsIO::check_input_model(const Json::Value& source, int l__){\
      std::stringstream sstr;\
      sstr << coyot3::tools::indentation(l__) << "[" #CY_class_name "MappedSetJsIO (" #CY_member_index_property ")]" << std::endl;\
      if(source.isArray() == false){\
        sstr << coyot3::tools::indentation(l__ + 2) << " !!! IS NOT AN ARRAY !!!" << std::endl;\
        return sstr.str();\
      }\
      Json::Value::ArrayIndex i, s = source.size();\
      for(i = 0; i < s; ++i){\
        CY_class_name##JsIO buffer;\
        sstr << buffer.check_input_model(source[i], l__ + 2);\
      }\
      return sstr.str();\
    }


  #define cyt3macro_model_class_set_mapped_serializable_def_operator_ostream_(CY_class_name)\
    std::ostream& operator<<(::std::ostream& o, const CY_class_name##MappedSet& i){\
      return (o << CY_class_name##MappedSetJsIO(i).to_json());\
    }

  #define cyt3macro_model_class_set_mapped_serializable_def_operator_classname_(CY_class_name)\
    bool operator<<(CY_class_name##MappedSet& o, const Json::Value& i){\
      CY_class_name##MappedSetJsIO b; \
      bool res = b.from_json(i); \
      o = b; \
      return res; \
    }
  
  #define cyt3macro_model_class_set_mapped_serializable_def_operator_json_(CY_class_name)\
    Json::Value operator<<(::Json::Value& o, const CY_class_name##MappedSet& i){\
      return (o = CY_class_name##MappedSetJsIO(i).to_json());\
    }

  #define cyt3macro_model_class_set_mapped_serializable_def_sermodeltemp(CY_class_name,CY_member_index_property)\
    std::string CY_class_name##MappedSetJsIO::get_serialization_model_template(int l__){\
      std::string ret;\
      ret = coyot3::tools::indentation(l__) + "[" #CY_class_name "MappedSetJsIO (" #CY_member_index_property ")]\n";\
      ret+= CY_class_name##JsIO::get_serialization_model_template(l__+1);\
      return ret;\
    }

#define CYT3MACRO_model_class_set_mapped_serializable_json_definitions(CY_class_name,CY_member_index_property) \
  \
  cyt3macro_model_class_set_mapped_serializable_json_def_constrdestr_(CY_class_name,CY_member_index_property) \
  \
  cyt3macro_model_class_set_mapped_serializable_json_def_to_json_(CY_class_name,CY_member_index_property) \
  \
  cyt3macro_model_class_set_mapped_serializable_json_def_from_json_(CY_class_name,CY_member_index_property) \
  \
  cyt3macro_model_class_set_mapped_serializable_def_operator_json_(CY_class_name) \
  \
  cyt3macro_model_class_set_mapped_serializable_def_operator_classname_(CY_class_name) \
  \
  cyt3macro_model_class_set_mapped_serializable_def_operator_ostream_(CY_class_name)\
  \
  cyt3macro_model_class_set_mapped_serializable_def_sermodeltemp(CY_class_name,CY_member_index_property)\
  \
  cyt3macro_model_class_set_mapped_serializable_def_check_model_(CY_class_name, CY_member_index_property)

  
//REMOVED RAPIDJSON SUPPORT
//  cyt3macro_model_class_set_mapped_serializable_json_def_from_rjson_(CY_class_name,CY_member_index_property) 
  




///
/// stack -jsio
///



#define CYT3MACRO_model_class_set_stack_serializable_json_declarations(CY_class_name) \
  class CY_class_name##StackJsIO \
  : public coyot3::tools::JsonSerializablePacketBase\
  , public CY_class_name##Stack {\
    public: \
      CY_class_name##StackJsIO();\
      CY_class_name##StackJsIO(const CY_class_name##Stack& o);\
      virtual ~CY_class_name##StackJsIO();\
      \
      Json::Value to_json() const;\
      bool        from_json(const Json::Value& source);\
      \
      static std::string get_serialization_model_template(int l__ = 0);\
      std::string check_input_model(const Json::Value& source, int l__ = 0);\
  };\
  Json::Value   operator<<(::Json::Value& o, const CY_class_name##Stack& i);\
  bool          operator<<(CY_class_name##Stack& o, const Json::Value& i);\
  std::ostream& operator<<(::std::ostream& o, const CY_class_name##Stack& i);

// REMOVED RAPIDJSON SUPPORT
//      bool        from_json(const rapidjson::Value& source);


  #define cyt3macro_model_class_set_stack_def_serializable_json_tojson_(CY_class_name) \
    Json::Value CY_class_name##StackJsIO::to_json() const {\
      Json::Value js(Json::arrayValue);\
      for(const CY_class_name& item : stack_){\
        js.append(CY_class_name##JsIO(item).to_json());\
      }\
      return js;\
    }

  #define cyt3macro_model_class_set_stack_def_serializable_json_fromjson_(CY_class_name) \
    bool CY_class_name##StackJsIO::from_json(const Json::Value& source){ \
      bool res = true; \
      if(!source.isArray())return false; \
      Json::ArrayIndex idx, sz = source.size(); \
      CY_class_name##JsIO buffer; \
      std::lock_guard guard(stack_mtx_);\
      for(idx = 0; idx < sz; ++idx){ \
        res &= buffer.from_json(source[idx]);\
        stack_.push_back(buffer); \
      } \
      return res; \
    }


    #define cyt3macro_model_class_set_stack_serializable_def_check_model_(CY_class_name)\
    std::string CY_class_name##StackJsIO::check_input_model(const Json::Value& source, int l__){\
      std::stringstream sstr;\
      sstr << coyot3::tools::indentation(l__) << "[" #CY_class_name "StackJsIO" << std::endl;\
      if(source.isArray() == false){\
        sstr << coyot3::tools::indentation(l__ + 2) << " !!! IS NOT AN ARRAY !!!" << std::endl;\
        return sstr.str();\
      }\
      Json::Value::ArrayIndex i, s = source.size();\
      for(i = 0; i < s; ++i){\
        CY_class_name##JsIO buffer;\
        sstr << buffer.check_input_model(source[i], l__ + 2);\
      }\
      return sstr.str();\
    }


  #define cyt3macro_model_class_set_stack_def_serializable_json_operators_(CY_class_name) \
    Json::Value   operator<<(::Json::Value& o, const CY_class_name##Stack& i){\
      return o = CY_class_name##StackJsIO(i).to_json(); \
    }\
    bool          operator<<(CY_class_name##Stack& o, const Json::Value& i){\
      bool res;\
      CY_class_name##StackJsIO buffer; \
      if((res=buffer.from_json(i))==true)o = buffer; \
      return res;\
    }\
    std::ostream& operator<<(::std::ostream& o, const CY_class_name##Stack& i){\
      return (o << CY_class_name##StackJsIO(i).to_json());\
    };

  #define cyt3macro_model_class_set_stack_def_serializable_json_sermodetemplate_(CY_class_name)\
    std::string CY_class_name##StackJsIO::get_serialization_model_template(int l__){\
      std::string ret;\
      ret = coyot3::tools::indentation(l__) + "[" #CY_class_name "StackJsIO]\n";\
      ret+= CY_class_name##JsIO::get_serialization_model_template(l__+1);\
      return ret;\
    }


#define CYT3MACRO_model_class_set_stack_serializable_json_definitions(CY_class_name) \
  CY_class_name##StackJsIO::CY_class_name##StackJsIO(){}\
  CY_class_name##StackJsIO::CY_class_name##StackJsIO(const CY_class_name##Stack& o):CY_class_name##Stack(o){}\
  CY_class_name##StackJsIO::~CY_class_name##StackJsIO(){}\
  \
  cyt3macro_model_class_set_stack_def_serializable_json_tojson_(CY_class_name) \
  \
  cyt3macro_model_class_set_stack_def_serializable_json_fromjson_(CY_class_name) \
  \
  cyt3macro_model_class_set_stack_def_serializable_json_operators_(CY_class_name) \
  \
  cyt3macro_model_class_set_stack_def_serializable_json_sermodetemplate_(CY_class_name) \
  \
  cyt3macro_model_class_set_stack_serializable_def_check_model_(CY_class_name)



// REMOVED RAPIDJSON SUPPORT
//  cyt3macro_model_class_set_stack_def_serializable_json_fromrjson_(CY_class_name)