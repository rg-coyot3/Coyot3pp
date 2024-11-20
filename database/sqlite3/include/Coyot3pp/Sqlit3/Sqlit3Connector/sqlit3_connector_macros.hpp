#pragma once

#include <Coyot3pp/Cor3/base/cyt_foreach_macro_helpers.hpp>
#include <Coyot3pp/Cor3/base/cyt_macro_enum_class.hpp>



  #define cyt3macro_model_class_serializable_sqlit3io_coltypes_decs_(cy_var_name,cy_var_type,cy_col_name)\
    static constexpr const char* cy_var_name = cy_var_type;

  #define cyt3macro_model_class_serializable_sqlit3io_coltypesEn_decs_(cy_var_name,cy_var_type,cy_col_name)\
    DataType cy_var_name;

  #define cyt3macro_model_class_serializable_sqlit3io_colnames_decs_(cy_var_name,cy_var_type,cy_col_name)\
    static constexpr const char* cy_var_name = cy_col_name;

  #define cyt3macro_model_class_serializable_sqlit3io_colactiv_decs_(cy_var_name,cy_var_type,cy_col_name)\
    bool cy_var_name = true;




#define CYT3MACRO_model_class_serializable_sqlit3_declarations(CY_class_name, CY_options , ...)\
\
class CY_class_name##Sqlit3IO : public coyot3::ddbb::sqlite::Sqlit3Connector{\
  public:\
    enum class DataType{INTEGER,REAL,NUMERIC,TEXT,BLOB,NULL_T};\
      struct ColType{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3io_coltypes_decs_, __VA_ARGS__)\
    };\
    struct ColsTypeE{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3io_coltypesEn_decs_, __VA_ARGS__)\
    };\
    struct ColName{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3io_colnames_decs_, __VA_ARGS__)\
    };\
    struct ColsFlags{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3io_colactiv_decs_, __VA_ARGS__)\
    };\
    CY_class_name##Sqlit3IO();\
    virtual ~CY_class_name##Sqlit3IO();\
    \
    CY_class_name##Stack stack;\
    CY_class_name##Stack query_stack;\
    \
    bool  select_table_items(std::string& err, const std::string& where = std::string());\
    bool  select_table_items(std::string& err);\
    bool  insertion_field_activation(const std::string& colName) const;\
    bool  insertion_field_activation(const std::string& colName,bool acivation);\
    std::string table_name(const std::string& name);\
    std::string table_name() const;\
    bool  insert_table_items(std::string& err);\
    bool  check_create_table(std::string& err);\
    bool  volatile_stack() const;\
    bool  volatile_stack(bool v);\
  protected:\
    std::string             tablename_;\
    CY_class_name##Stack    stack_exch_;\
    std::mutex              mtx_obj_;\
    \
    ColsFlags               cols_w_act_;\
    ColsFlags               cols_w_str_;\
    ColsTypeE               cols_ttr_;\
    bool                    cols_params_calculated_;\
    bool                    volatile_stack_;\
    \
    void                    conf_insertion_types_();\
    bool                    conf_datatype_matx_();\
    \
    std::string             query_iq_prefix_;\
    void                    calc_iq_string_prefix_();\
    \
    std::string             query_sq_prefix_;\
    void                    calc_sq_string_prefix_();\
    \
    std::string             query_cr_table_;\
    void                    calc_cr_query();\
  public:\
    static std::string query_create_table(const std::string& tableName);\
    static std::string query_alter_table(const std::string& tableName);\
    static constexpr const char* INTEGER = "INTEGER";\
    static constexpr const char* NUMERIC = "NUMERIC";\
    static constexpr const char* REAL = "REAL";\
    static constexpr const char* TEXT = "TEXT";\
    static constexpr const char* BLOB = "BLOB";\
    static constexpr const char* NULL_T = "NULL";\
    static constexpr const char* NOT_NULL = "NOT NULL";\
    static constexpr const char* AUTOINCREMENT = "AUTOINCREMENT";\
};

#define cyt3macro_model_class_serializable_sqlit3_def_constr_destr_(CY_class_name, ...)\
  CY_class_name##Sqlit3IO::CY_class_name##Sqlit3IO()\
  :Sqlit3Connector(#CY_class_name "Sqlit3IO-inst")\
  ,stack()\
  ,query_stack()\
  ,tablename_()\
  ,stack_exch_()\
  ,cols_w_act_()\
  ,cols_w_str_()\
  ,cols_params_calculated_()\
  ,volatile_stack_(true)\
  ,query_iq_prefix_()\
  ,query_sq_prefix_()\
  ,query_cr_table_()\
  {\
    conf_datatype_matx_();\
    calc_iq_string_prefix_();\
    conf_insertion_types_();\
  }\
  \
  CY_class_name##Sqlit3IO::~CY_class_name##Sqlit3IO(){End(true);}


    #define cyt3macro_model_class_serializable_sqlit3_def_supp_calc_str_w_(cy_var_name,cy_var_type,cy_col_nw_ame)\
      (std::string(cy_var_type).find("TEXT") != std::string::npos ? cols_w_str_.cy_var_name = true : cols_w_str_.cy_var_name = false);

  #define cyt3macro_model_class_serializable_sqlit3_def_supp_(CY_class_name, ...)\
    void CY_class_name##Sqlit3IO::conf_insertion_types_(){\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_supp_calc_str_w_, __VA_ARGS__)\
      \
      cols_params_calculated_ = true;\
    }


// HELPERS

    #define cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_01(cy_var_name,cy_var_type,cy_col_name)\
      if(colName.compare(ColName::cy_var_name) == 0) found = (true | (cols_w_act_.cy_var_name = activation));
    
    #define cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_02(cy_var_name,cy_var_type,cy_col_name)\
      if(colName.compare(ColName::cy_var_name) == 0) found = true;

  /**
   * insertion-field-activation : activates or deactivates the insertion of 
   *  some item at the database when inserting new elements.
   * 
   */

        #define cyt3macro_model_class_serializable_sqlit3_def_insert_colname_it_(cy_var_name,cy_var_type,cy_col_name)\
        if(cols_w_act_.cy_var_name == true){if(firstItem==false)sstr << ",";else firstItem=false; sstr << " " cy_col_name ;}

    #define cyt3macro_model_class_serializable_sqlit3_def_insert_colname_(cy_var_name,cy_var_type,cy_col_name,...)\
      bool firstItem=true;\
      if(cols_w_act_.cy_var_name == true){firstItem=false;sstr << cy_col_name ;}\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_insert_colname_it_,__VA_ARGS__)


      #define cyt3macro_model_class_serializable_sqlit3_def_select_colname_iter_(cy_var_name,cy_var_type,cy_col_name)\
      sstr << ", " cy_col_name;

    #define cyt3macro_model_class_serializable_sqlit3_def_select_colname_(cy_var_name,cy_var_type,cy_col_name,...)\
    sstr << cy_col_name;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_select_colname_iter_,__VA_ARGS__)\
      
  #define cyt3macro_model_class_serializable_sqlit3_def_helpers_(CY_class_name, ...)\
  void CY_class_name##Sqlit3IO::calc_iq_string_prefix_(){\
    std::stringstream sstr;\
    sstr << "INSERT INTO '" << tablename_ << "' (";\
    cyt3macro_model_class_serializable_sqlit3_def_insert_colname_(__VA_ARGS__)\
    sstr << ") VALUES ";\
    query_iq_prefix_ = sstr.str();\
  }\
  void CY_class_name##Sqlit3IO::calc_sq_string_prefix_(){\
    std::stringstream sstr;\
    sstr << "SELECT ";\
    cyt3macro_model_class_serializable_sqlit3_def_select_colname_(__VA_ARGS__)\
    sstr << " FROM " << tablename_;\
    query_sq_prefix_ = sstr.str();\
  }\
  bool CY_class_name##Sqlit3IO::insertion_field_activation(const std::string& colName) const{\
    bool found = false;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_02,__VA_ARGS__)\
    return found;\
  }\
  bool CY_class_name##Sqlit3IO::insertion_field_activation(const std::string& colName, bool activation){\
    bool found = false;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_01,__VA_ARGS__)\
    calc_iq_string_prefix_();\
    return found;\
  }\
  std::string  CY_class_name##Sqlit3IO::table_name(const std::string& name){\
    tablename_ = name;\
    calc_iq_string_prefix_();\
    calc_sq_string_prefix_();\
    return tablename_;\
  }\
  std::string CY_class_name##Sqlit3IO::table_name() const{\
    return tablename_;\
  }\
  bool CY_class_name##Sqlit3IO::volatile_stack() const{return volatile_stack_;}\
  bool CY_class_name##Sqlit3IO::volatile_stack(bool v){return (volatile_stack_ = v);}


// DATA TYPE MATRIX CONFIGURATION

    #define cyt3macro_model_class_serializable_sqlit3_def_conf_item_(cy_var_name,cy_var_type,cy_col_name)\
      {\
        bool lf_=false;\
        if(std::string(cy_var_type).find(INTEGER) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::INTEGER));\
        if(std::string(cy_var_type).find(TEXT) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::TEXT));\
        if(std::string(cy_var_type).find(BLOB) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::BLOB));\
        if(std::string(cy_var_type).find(NUMERIC) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::NUMERIC));\
        if(std::string(cy_var_type).find(REAL) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::REAL));\
        done_ok&=lf_;\
        \
        if(std::string(cy_var_type).find(AUTOINCREMENT) != std::string::npos)cols_w_act_.cy_var_name = false;\
      }

  #define cyt3macro_model_class_serializable_sqlit3_def_conf_(CY_class_name, ...)\
  bool CY_class_name##Sqlit3IO::conf_datatype_matx_(){\
    bool done_ok=true;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_conf_item_, __VA_ARGS__)\
    return done_ok;\
  }


  //INSERT ITEMS
      #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_iter_(cy_var_name,cy_var_type,cy_col_name)\
      if(cols_w_act_.cy_var_name == true){\
        if(firstItem==false)sstr << ", ";else firstItem=false;\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
        sstr << item.cy_var_name();\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
      }  

    #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_it_(cy_var_name,cy_var_type,cy_col_name, ...)\
    if(cols_w_act_.cy_var_name == true){\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      sstr << item.cy_var_name();\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      firstItem=false;\
    }\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_insert_items_iter_,__VA_ARGS__)


  #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_(CY_class_name, ...)\
  bool CY_class_name##Sqlit3IO::insert_table_items(std::string& err){\
    std::stringstream sstr;\
    sstr << query_iq_prefix_;\
    int numItem = 0;\
    if(stack.size() == 0)return true;\
    std::lock_guard guard(mtx_obj_);\
    stack.for_each([&](const CY_class_name& item){\
     if(numItem != 0)sstr  << ", ";\
     else            sstr  << " ";\
     sstr << "( ";\
     bool firstItem=true;\
     cyt3macro_model_class_serializable_sqlit3_def_insert_items_it_(__VA_ARGS__)\
     sstr << ")";\
     numItem++;\
     return true;\
    });\
    sstr << ";";\
    bool result = make_query(sstr.str(),err);\
    if(result == false){CLOG_WARN("cyt3macro-qsqlite-io : " \
      #CY_class_name " : error : query[" << sstr.str() << "]")\
      return false;\
    }\
    if(volatile_stack_ == true)stack.clear();\
    return result;\
  }

// SELECT QUERY

    #define cyt3macro_model_class_serializable_sqlit3_def_query_items_recv_(cy_var_name,cy_var_type,cy_col_name)\
      if(coyot3::ddbb::sqlite::sqlit3_get_col(q,index++,buffer.cy_var_name()) == false){\
        log_warn(o() << "select-table-items : error reading (col=" << (index-1) << "[" #cy_var_name "]");\
        cleanread = false;\
      }

  #define cyt3macro_model_class_serializable_sqlit3_def_query_items_(CY_class_name, ...)\
  bool CY_class_name##Sqlit3IO::select_table_items(std::string& err,const std::string& where){\
    std::stringstream sstr;\
    std::lock_guard<std::mutex> guard(mtx_obj_);\
    sstr << query_sq_prefix_;\
    if(where.size() != 0){\
      sstr << " WHERE " << where;\
    }\
    sstr << ";";\
    sqlite3_stmt* q;\
    bool result;\
    result = make_query(sstr.str(), q, err);\
    if(result == false)return false;\
    bool dosteps = true;\
    int linesread = 0;\
    bool cleanread = true;\
    query_stack.clear();\
    while(dosteps == true){\
      int rc = sqlite3_step(q);\
      switch(rc){\
        case SQLITE_DONE:\
          dosteps = false;\
        break;\
        case SQLITE_ROW:\
        {\
          int index = 0;\
          CY_class_name buffer;\
          FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_query_items_recv_,__VA_ARGS__)\
          query_stack.push_back(buffer);\
        }\
        break;\
        case SQLITE_ERROR:\
        {\
          std::stringstream sstrerr;\
          sstrerr << "select-table-items : error acquiring row [" << linesread << "];error=";\
          reset_statement(q,err);\
          sstrerr << err;\
          err = sstrerr.str();\
          cleanread = false;\
          dosteps = false;\
        }\
        break;\
        default:\
        {\
          std::stringstream sstrerr;\
          sstrerr << "select-table-items : error acquiring row [" << linesread << "];error code = " << rc;\
          dosteps = false;\
          cleanread = false;\
        }\
      }\
    }\
    log_debug(5,o() << "select-table-items : read [" << linesread << "] lines.");\
    sqlite3_finalize(q);\
    return cleanread;\
  }    

// CHECK TABLE


      #define cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_it_(cy_var_name,cy_var_type,cy_col_name)\
      sstr << ", `" cy_col_name "` " cy_var_type; 
    
    #define cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_(cy_var_name,cy_var_type,cy_col_name,...)\
    sstr << " `" cy_col_name "` " cy_var_type;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_it_,__VA_ARGS__)

  #define cyt3macro_model_class_serializable_sqlit3_def_check_table(CY_class_name, ...)\
  bool  CY_class_name##Sqlit3IO::check_create_table(std::string& err){\
    std::stringstream sstr;\
    sstr << "CREATE TABLE IF NOT EXISTS `" << tablename_ << "` (";\
    cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_(__VA_ARGS__)\
    sstr << ");";\
    bool result = make_query(sstr.str(),err);\
    return result;\
  }

//

#define CYT3MACRO_model_class_serializable_sqlit3_definitions(CY_class_name,CY_options, ...)\
\
cyt3macro_model_class_serializable_sqlit3_def_constr_destr_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_supp_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_helpers_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_conf_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_insert_items_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_query_items_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_sqlit3_def_check_table(CY_class_name, __VA_ARGS__)\
