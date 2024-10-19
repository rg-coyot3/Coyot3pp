#pragma once

#include "cyt_macro_model_class.hpp"


#define CYT3MACRO_model_class_serializable_csv(CY_class_name, CY_parent_class, CY_enumclass_deps, ...)\
  class CY_class_name##CsvIO: public CY_class_name{\
    public:\
      enum class Conf{\
        size\
      };\
      static const char* ConfToString(Conf s);\
      static       bool  SerializerCheck();\
      \
      CY_class_name##CsvIO();\
      CY_class_name##CsvIO(const CY_class_name& o);\
      ~CY_class_name##CsvIO();\
      \
      std::string get_headers();\
      std::string to_csv_line();\
    protected:\
      \
  \
  };



