#pragma once

#include "cyt_macro_model_class.hpp"

/**
 * @brief 
 * 
 *  TRIPLES: property name, header, cast
 * 
 */
#define CYT3MACRO_model_class_serializable_csv_declarations(CY_class_name, CY_enumclass_deps, ...)\
  class CY_class_name##CsvIO: public CY_class_name{\
    public:\
      enum class Conf{\
        size\
      };\
      static const char* ConfToString(Conf s);\
      static       bool  SerializerCheck();\
      \
      CY_class_name##CsvIO(const char separator = ';');\
      CY_class_name##CsvIO(const CY_class_name& o, const char separator=';');\
      virtual ~CY_class_name##CsvIO();\
      \
      std::string get_headers() const;\
      virtual std::string to_csv_string() const; \
      virtual bool        from_csv_string(const std::string& input);\
    protected:\
      char        col_separator_ = ';';\
  \
  };



    #define cytemacro_model_class_serializable_csv_def_constructors_(CY_class_name, CY_enumclass_deps, ...)\
      CY_class_name##CsvIO::CY_class_name##CsvIO(const char separator):CY_class_name(),col_separator_(separator){}\
      CY_class_name##CsvIO::CY_class_name##CsvIO(const CY_class_name& o, const char separator):CY_class_name(o),col_separator_(separator){}\
      CY_class_name##CsvIO::~CY_class_name##CsvIO(){}

      #define cytemacro_model_class_serializable_csv_def_to_csv_string_item_(cy_prop_name, cy_prop_header, cy_prop_cast)\
        IFE(cy_prop_cast)(col_separator_ << cy_prop_name();)\
        IFN(cy_prop_cast)(col_separator_ << static_cast<cy_prop_name##_t>(cy_prop_name());)

    #define cytemacro_model_class_serializable_csv_def_to_csv_string(CY_class_name, CY_enumclass_deps, ...)\
      std::string CY_class_name##CsvIO::to_csv_string() const{\
        std::stringstream sstr;\
        sstr << \
        FOR_EACH_TRIPLES(cytemacro_model_class_serializable_csv_def_to_csv_string_item_, __VA_ARGS__)\
        ;\
        return sstr.str().substr(1);\
      }
    
      #define cytemacro_model_class_serializable_csv_def_get_headers_item_(cy_prop_name, cy_prop_header, cy_prop_cast)\
        << col_separator_ << ##cy_prop_header

    #define cytemacro_model_class_serializable_csv_def_get_headers(CY_class_name, CY_enumclass_deps, ...)\
      std::string CY_class_name##CsvIO::get_headers() const{\
        std::stringstream sstr;\
        sstr \
        FOR_EACH_TRIPLES(cytemacro_model_class_serializable_csv_def_get_headers_item_, __VA_ARGS__)\
        ;\
        return sstr.str().substr(1);\
      }


      #define cytemacro_model_class_serialziable_csv_def_from_csv_item_(cy_prop_name, cy_prop_header, cy_prop_cast)\
        {\
          if(index>=length)return false;\
          IFE(cy_prop_cast)(cy_prop_name##_t buffer;)\
          IFN(cy_prop_cast)(cy_prop_cast buffer;)\
          Json::Value v(arr[index]);\
          res &= coyot3::tools::json_import_value(v,"",buffer);\
          cy_prop_name(buffer);\
          ++index;\
        }

    #define cytemacro_model_class_serialziable_csv_def_from_csv(CY_class_name, CY_enumclass_deps, ...)\
      bool CY_class_name##CsvIO::from_csv_string(const std::string& input){\
        std::vector<std::string> arr;\
        std::size_t index = 0, length = coyot3::tools::string_split(input,col_separator_,arr);\
        bool res = true;\
        FOR_EACH_TRIPLES(cytemacro_model_class_serialziable_csv_def_from_csv_item_,__VA_ARGS__)\
        return res && (index == length);\
      }

  #define CYT3MACRO_model_class_serializable_csv_definitions(CY_class_name, CY_enumclass_deps, ...)\
    \
    cytemacro_model_class_serializable_csv_def_constructors_(CY_class_name, CY_enumclass_deps, __VA_ARGS__)\
    \
    cytemacro_model_class_serializable_csv_def_to_csv_string(CY_class_name, CY_enumclass_deps, __VA_ARGS__)\
    \
    cytemacro_model_class_serializable_csv_def_get_headers(CY_class_name, CY_enumclass_deps, __VA_ARGS__)\
    \
    cytemacro_model_class_serialziable_csv_def_from_csv(CY_class_name, CY_enumclass_deps, __VA_ARGS__)




  #define CYT3MACRO_model_class_set_stack_serializable_csv_declarations(CY_class_name)\
    class CY_class_name##StackCsvIO:public CY_class_name##CsvIO{\
      public:\
        CY_class_name##StackCsvIO();\
        CY_class_name##StackCsvIO(const CY_class_name##StackCsvIO& o);\
        virtual ~CY_class_name##StackCsvIO();\
        \
        virtual std::string&  to_csv_string() const override;\
        virtual bool          from_csv_string(const std::string& input, bool with_headers = true);\
      protected:\
        CY_class_name##Stack stack_;\
        std::mutex           mtx_;\
    };\





  #define CYT3MACRO_model_class_set_stack_serializable_csv_definitions(CY_class_name)\
    CY_class_name##StackCsvIO::CY_class_name##StackCsvIO():CY_class_name##CsvIO(){}\
    CY_class_name##StackCsvIO::CY_class_name##StackCsvIO(const CY_class_name##StackCsvIO& o):CY_class_name##CsvIO(),stack_(o){}\
    CY_class_name##StackCsvIO::~CY_class_name##StackCsvIO(){}\
    \
    std::string CY_class_name##StackCsvIO::to_csv_string() const{\
      std::stringstream sstr;\
      sstr << get_headers();\
      stack.for_each([&](const CY_class_name& i){\
        CY_class_name##CsvIO c(i);\
        sstr << std::endl << c.to_csv_string();\
        return true;\
      });\
      return sstr.str();\
    }\
    \
    bool CY_class_name##StackCsvIO::from_csv_string(const std::string& input, bool with_headers){\
      std::stringstream sstr{input};\
      std::string line;\
      bool first_line = true, all_good = true;\
      while(std::getline(sstr,line,'\n')){\
        if((first_line == true) && (with_headers == true))continue;\
        first_line = false;\
        CY_class_name##CsvIO o;\
        all_good &= o.from_csv_line(line);\
        stack_.push_back(o);\
      }\
      return all_good;\
    }



