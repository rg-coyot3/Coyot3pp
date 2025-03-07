#pragma once

#include <Coyot3pp/Cor3/base/cyt_foreach_macro_helpers.hpp>
#include <Coyot3pp/Cor3/base/cyt_macro_enum_class.hpp>



  #define cyt3macro_model_class_serializable_sqlit3io_coltypes_decs_(cy_var_name,cy_col_name,cy_var_type)\
    static constexpr const char* cy_var_name = cy_var_type;

  #define cyt3macro_model_class_serializable_sqlit3io_coltypesEn_decs_(cy_var_name,cy_col_name,cy_var_type)\
    DataType cy_var_name;

  #define cyt3macro_model_class_serializable_sqlit3io_colnames_decs_(cy_var_name,cy_col_name,cy_var_type)\
    static constexpr const char* cy_var_name = cy_col_name;

  #define cyt3macro_model_class_serializable_sqlit3io_colactiv_decs_(cy_var_name,cy_col_name,cy_var_type)\
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
    CY_class_name##Stack stack_output;\
    CY_class_name##Stack stack_input;\
    \
    bool  get_table_items(std::string& err, const std::string& where);\
    bool  get_table_items(const std::string& where);\
    bool  insertion_field_activation(const std::string& colName) const;\
    bool  insertion_field_activation(const std::string& colName,bool acivation);\
    size_t push_item(const CY_class_name& i);\
    std::string table_name(const std::string& name);\
    std::string table_name() const;\
    bool  insert_table_items();\
    bool  insert_table_items(std::string& err);\
    bool  check_create_table();\
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


// DEFINITIONS

#define cyt3macro_model_class_serializable_sqlit3_def_constr_destr_(CY_class_name, ...)\
  CY_class_name##Sqlit3IO::CY_class_name##Sqlit3IO()\
  :Sqlit3Connector(#CY_class_name "Sqlit3IO-inst")\
  ,stack_output()\
  ,stack_input()\
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
    class_name(#CY_class_name "Sqlit3IO");\
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

    #define cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_01(cy_var_name,cy_col_name,cy_var_type)\
      if(colName.compare(ColName::cy_var_name) == 0) found = (true | (cols_w_act_.cy_var_name = activation));
    
    #define cyt3macro_model_class_serializable_sqlit3_def_helpers_gnapa_02(cy_var_name,cy_col_name,cy_var_type)\
      if(colName.compare(ColName::cy_var_name) == 0) found = true;

  /**
   * insertion-field-activation : activates or deactivates the insertion of 
   *  some item at the database when inserting new elements.
   * 
   */

        #define cyt3macro_model_class_serializable_sqlit3_def_insert_colname_it_(cy_var_name,cy_col_name,cy_var_type)\
        if(cols_w_act_.cy_var_name == true){if(firstItem==false)sstr << ",";else firstItem=false; sstr << " " cy_col_name ;}

    #define cyt3macro_model_class_serializable_sqlit3_def_insert_colname_(cy_var_name,cy_col_name,cy_var_type,...)\
      bool firstItem=true;\
      if(cols_w_act_.cy_var_name == true){firstItem=false;sstr << cy_col_name ;}\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_insert_colname_it_,__VA_ARGS__)


      #define cyt3macro_model_class_serializable_sqlit3_def_select_colname_iter_(cy_var_name,cy_col_name,cy_var_type)\
      sstr << ", " cy_col_name;

    #define cyt3macro_model_class_serializable_sqlit3_def_select_colname_(cy_var_name,cy_col_name,cy_var_type,...)\
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
  bool CY_class_name##Sqlit3IO::volatile_stack(bool v){return (volatile_stack_ = v);}\
  size_t CY_class_name##Sqlit3IO::push_item(const CY_class_name& i){\
    stack_output.push_back(i);\
    return stack_output.size();\
  }


// DATA TYPE MATRIX CONFIGURATION

    #define cyt3macro_model_class_serializable_sqlit3_def_conf_item_(cy_var_name,cy_col_name,cy_var_type)\
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
      #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_iter_(cy_var_name,cy_col_name,cy_var_type)\
      if(cols_w_act_.cy_var_name == true){\
        if(firstItem==false)sstr << ", ";else firstItem=false;\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
        sstr << item.cy_var_name();\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
      }  

    #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_it_(cy_var_name,cy_col_name,cy_var_type, ...)\
    if(cols_w_act_.cy_var_name == true){\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      sstr << item.cy_var_name();\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      firstItem=false;\
    }\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_insert_items_iter_,__VA_ARGS__)


  #define cyt3macro_model_class_serializable_sqlit3_def_insert_items_(CY_class_name, ...)\
  bool CY_class_name##Sqlit3IO::insert_table_items(){\
    std::string err;\
    bool result = insert_table_items(err);\
    if(result == false)log_warn(o() << "insert-table-items : error inserting : err=" << err);\
    return result;\
  }\
  bool CY_class_name##Sqlit3IO::insert_table_items(std::string& err){\
    std::stringstream sstr;\
    sstr << query_iq_prefix_;\
    int numItem = 0;\
    if(stack_output.size() == 0)return true;\
    std::lock_guard guard(mtx_obj_);\
    stack_exch_ = stack_output;\
    if(volatile_stack_ == true)stack_output.clear();\
    stack_exch_.for_each([&](const CY_class_name& item){\
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
      #CY_class_name " : error=" << err << ": query[" << sstr.str() << "]")\
      return false;\
    }\
    return result;\
  }

// SELECT QUERY

    #define cyt3macro_model_class_serializable_sqlit3_def_query_items_recv_(cy_var_name,cy_col_name,cy_var_type)\
      if(coyot3::ddbb::sqlite::sqlit3_get_col(q,index++,buffer.cy_var_name()) == false){\
        log_warn(o() << "select-table-items : error reading (col=" << (index-1) << "[" #cy_var_name "]");\
        cleanread = false;\
      }

  #define cyt3macro_model_class_serializable_sqlit3_def_query_items_(CY_class_name, ...)\
  bool CY_class_name##Sqlit3IO::get_table_items(std::string& err,const std::string& where){\
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
    stack_input.clear();\
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
          stack_input.push_back(buffer);\
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
  }\
  bool CY_class_name##Sqlit3IO::get_table_items(const std::string& where){\
    std::string err;\
    bool res = get_table_items(err,where);\
    if(res == false){\
      log_warn( o() << "select-table-items : error : err=" << err << "; select * where (" << where << ")");\
    }\
    return res;\
  }

// CHECK TABLE


      #define cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_it_(cy_var_name,cy_col_name,cy_var_type)\
      sstr << ", `" cy_col_name "` " cy_var_type; 
    
    #define cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_(cy_var_name,cy_col_name,cy_var_type,...)\
    sstr << " `" cy_col_name "` " cy_var_type;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqlit3_def_check_table_elems_it_,__VA_ARGS__)

  #define cyt3macro_model_class_serializable_sqlit3_def_check_table(CY_class_name, ...)\
  bool  CY_class_name##Sqlit3IO::check_create_table(){\
    std::string err;\
    if(check_create_table(err) == false){\
      log_warn(o() << "check-create-table : error checking table : reason=" << err);\
      return false;\
    }\
    return true;\
  }\
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
cyt3macro_model_class_serializable_sqlit3_def_check_table(CY_class_name, __VA_ARGS__) 


//AUTOINSERT

#define CYT3MACRO_model_class_serializable_sqlit3_autoinsert_declarations(CY_class_name)\
class CY_class_name##Sqlit3AutoIO : public CY_class_name##Sqlit3IO{\
  public:\
    CY_class_name##Sqlit3AutoIO();\
    virtual ~CY_class_name##Sqlit3AutoIO();\
    \
    int64_t insertion_interval() const;\
    int64_t insertion_interval(int64_t i);\
    \
    void    on_error_callback_set(std::function<bool()> cb);\
    \
  protected:\
    int64_t interval_;\
    coyot3::tools::ControlThread* cth_;\
    std::function<bool()> on_error_cb_;\
    \
    void insertion_task_();\
    \
    bool task_start_();\
    bool task_stop_();\
    \
};








  #define cytemacro_model_class_serializable_sqlit3_autoinsert_constrdestr_def_(CY_class_name)\
    CY_class_name##Sqlit3AutoIO::CY_class_name##Sqlit3AutoIO()\
    :CY_class_name##Sqlit3IO()\
    ,interval_(10000)\
    ,cth_(nullptr)\
    {\
      class_name(#CY_class_name "SqliteAutoIO-instance");\
      add_task_start(std::bind(&CY_class_name##Sqlit3AutoIO::task_start_,this));\
      add_task_stop(std::bind(&CY_class_name##Sqlit3AutoIO::task_stop_,this), true);\
    }\
    \
    CY_class_name##Sqlit3AutoIO::~CY_class_name##Sqlit3AutoIO(){\
    }


  #define cytemacro_model_class_serializable_sqlit3_autoinsert_pubmethods_def_(CY_class_name)\
    int64_t CY_class_name##Sqlit3AutoIO::insertion_interval() const{\
      return interval_;\
    }\
    int64_t CY_class_name##Sqlit3AutoIO::insertion_interval(int64_t i){\
      return interval_ = i;\
    }\
    void CY_class_name##Sqlit3AutoIO::on_error_callback_set(std::function<bool()> cb){\
      on_error_cb_=cb;\
    }

  #define cytemacro_model_class_serializable_sqlit3_autoinsert_startstop_def_(CY_class_name)\
    bool CY_class_name##Sqlit3AutoIO::task_start_(){\
      cth_ = new(std::nothrow) coyot3::tools::ControlThread(std::bind(&CY_class_name##Sqlit3AutoIO::insertion_task_,this),#CY_class_name "SqliteAutoIO-inst-th");\
      if(cth_ == nullptr){\
        log_err("task-start- : error creating control-thread! no mem?");\
        return false;\
      }\
      cth_->setInterval(interval_);\
      log_info("task-start- : starting auto-insertion thread");\
      cth_->start();\
      return check_create_table();\
    }\
    \
    bool CY_class_name##Sqlit3AutoIO::task_stop_(){\
      log_info("task-stop- : stopping control thread");\
      cth_->stop();\
      delete cth_;\
      cth_=nullptr;\
      return true;\
    }

  #define cytemacro_model_class_serializable_sqlit3_autoinsert_insertion_def_(CY_class_name)\
    void CY_class_name##Sqlit3AutoIO::insertion_task_(){\
      if(insert_table_items() == false){\
        if(on_error_cb_)on_error_cb_();\
      }\
    }
    


#define CYT3MACRO_model_class_serializable_sqlit3_autoinsert_definitions(CY_class_name)\
  \
  cytemacro_model_class_serializable_sqlit3_autoinsert_constrdestr_def_(CY_class_name)\
  \
  cytemacro_model_class_serializable_sqlit3_autoinsert_pubmethods_def_(CY_class_name)\
  \
  cytemacro_model_class_serializable_sqlit3_autoinsert_startstop_def_(CY_class_name)\
  \
  cytemacro_model_class_serializable_sqlit3_autoinsert_insertion_def_(CY_class_name)


