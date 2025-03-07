#pragma once


#include <Coyot3pp/Cor3/base/cyt_foreach_macro_helpers.hpp>
#include <Coyot3pp/Cor3/base/cyt_macro_enum_class.hpp>
#include <functional>


 /**
  *     CY_class_name ==> CY_class_name##QSqliteIO
  *     options ( ) 
  *     
  *     triples : 
  *       property-id , type , field-name
  */



  #define cyt3macro_model_class_serializable_sqliteio_coltypes_decs_(cy_var_name,cy_col_name,cy_var_type)\
    static constexpr const char* cy_var_name = cy_var_type;

  #define cyt3macro_model_class_serializable_sqliteio_coltypesEn_decs_(cy_var_name,cy_col_name,cy_var_type)\
    DataType cy_var_name;


  #define cyt3macro_model_class_serializable_sqliteio_colnames_decs_(cy_var_name,cy_col_name,cy_var_type)\
    static constexpr const char* cy_var_name = cy_col_name;

  #define cyt3macro_model_class_serializable_sqliteio_colactiv_decs_(cy_var_name,cy_col_name,cy_var_type)\
    bool cy_var_name = true;


/**
 * triples : field name , field type , column name requires the 
 *  CY_class_name_STACK to be defined
 */

#define CYT3MACRO_model_class_serializable_qsqlite_declarations(CY_class_name, CY_options , ...)\
class CY_class_name##QSqliteIO : public coyot3::ddbb::sqlite::QSqlit3Connector{\
  public:\
    enum class DataType{INTEGER,NUMERIC,TEXT,BLOB,NULL_T};\
    struct ColType{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqliteio_coltypes_decs_, __VA_ARGS__)\
    };\
    struct ColsTypeE{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqliteio_coltypesEn_decs_, __VA_ARGS__)\
    };\
    struct ColName{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqliteio_colnames_decs_, __VA_ARGS__)\
    };\
    struct ColsFlags{\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_sqliteio_colactiv_decs_, __VA_ARGS__)\
    };\
    \
    CY_class_name##QSqliteIO(QObject* parent = nullptr);\
    virtual ~CY_class_name##QSqliteIO();\
    \
    CY_class_name##Stack stack;\
    CY_class_name##Stack query_stack;\
    \
    bool              select_table_items(std::string& err,const std::string& where = std::string());\
    bool              insert_table_items(std::string& err);\
    bool              update_table_items(std::string& err);\
    bool              update_table_item(const CY_class_name& item, std::string& err);\
    bool              delete_table_items(std::string& err);\
    bool              delete_table_item(const CY_class_name& item, std::string& err);\
    bool              insertion_field_activation(const std::string& colName,bool activation);\
    std::string       table_name(const std::string& name);\
    std::string       table_name() const;\
    bool              check_create_table(std::string& err);\
    bool              volatile_stack() const;\
    bool              volatile_stack(bool v);\
  protected:\
    std::string             tablename_;\
    CY_class_name##Stack    stack_exch_;\
    std::mutex              mtx_obj_;\
    \
    ColsFlags               cols_w_act_;\
    ColsFlags               cols_w_str_;\
    ColsTypeE               cols_ttr_;\
    std::string             primary_key_field_;\
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
    static constexpr const char* TEXT = "TEXT";\
    static constexpr const char* BLOB = "BLOB";\
    static constexpr const char* NULL_T = "NULL";\
    static constexpr const char* NOT_NULL = "NOT NULL";\
    static constexpr const char* AUTOINCREMENT = "AUTOINCREMENT";\
    static constexpr const char* PRIMARY_KEY = "PRIMARY KEY";\
    static bool extract(const QVariant& source,bool& destination);\
    static bool extract(const QVariant& source,int& destination);\
    static bool extract(const QVariant& source,uint& destination);\
    static bool extract(const QVariant& source,int64_t& destination);\
    static bool extract(const QVariant& source,uint64_t& destination);\
    static bool extract(const QVariant& source,double& destination);\
    static bool extract(const QVariant& source,std::string& destination);\
};





  #define cyt3macro_model_class_serializable_qsqlite_def_constr_destr_(CY_class_name, ...)\
    CY_class_name##QSqliteIO::CY_class_name##QSqliteIO(QObject* parent)\
    :QSqlit3Connector(parent)\
    ,stack()\
    ,query_stack()\
    ,tablename_()\
    ,stack_exch_()\
    ,cols_w_act_()\
    ,cols_w_str_()\
    ,primary_key_field_()\
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
    CY_class_name##QSqliteIO::~CY_class_name##QSqliteIO(){}


    #define cyt3macro_model_class_serializable_qsqlite_def_supp_calc_str_w_(cy_var_name,cy_col_name,cy_var_type)\
      (std::string(cy_var_type).find("TEXT") != std::string::npos ? cols_w_str_.cy_var_name = true : cols_w_str_.cy_var_name = false);

  #define cyt3macro_model_class_serializable_qsqlite_def_supp_(CY_class_name, ...)\
    void CY_class_name##QSqliteIO::conf_insertion_types_(){\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_supp_calc_str_w_, __VA_ARGS__)\
      \
      cols_params_calculated_ = true;\
    }

    
      #define cyt3macro_model_class_serializable_qsqlite_def_insert_colname_it_(cy_var_name,cy_col_name,cy_var_type)\
        if(cols_w_act_.cy_var_name == true){if(firstItem==false)sstr << ",";else firstItem=false; sstr << " " cy_col_name ;}

    #define cyt3macro_model_class_serializable_qsqlite_def_insert_colname_(cy_var_name,cy_col_name,cy_var_type,...)\
      bool firstItem=true;\
      if(cols_w_act_.cy_var_name == true){firstItem=false;sstr << cy_col_name ;}\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_insert_colname_it_,__VA_ARGS__)




    #define cyt3macro_model_class_serializable_qsqlite_def_helpers_gnapa_01(cy_var_name,cy_col_name,cy_var_type)\
      if(colName.compare(ColName::cy_var_name) == 0) found = (true | (cols_w_act_.cy_var_name = activation));

  /**
   * insertion-field-activation : activates or deactivates the insertion of 
   *  some item at the database when inserting new elements.
   * 
   */

      #define cyt3macro_model_class_serializable_qsqlite_def_select_colname_iter_(cy_var_name,cy_col_name,cy_var_type)\
      sstr << ", " cy_col_name;

    #define cyt3macro_model_class_serializable_qsqlite_def_select_colname_(cy_var_name,cy_col_name,cy_var_type,...)\
    sstr << cy_col_name;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_select_colname_iter_,__VA_ARGS__)\
      



  #define cyt3macro_model_class_serializable_qsqlite_def_helpers_(CY_class_name, ...)\
  void CY_class_name##QSqliteIO::calc_iq_string_prefix_(){\
    std::stringstream sstr;\
    sstr << "INSERT INTO '" << tablename_ << "' (";\
    cyt3macro_model_class_serializable_qsqlite_def_insert_colname_(__VA_ARGS__)\
    sstr << ") VALUES ";\
    query_iq_prefix_ = sstr.str();\
  }\
  void CY_class_name##QSqliteIO::calc_sq_string_prefix_(){\
    std::stringstream sstr;\
    sstr << "SELECT ";\
    cyt3macro_model_class_serializable_qsqlite_def_select_colname_(__VA_ARGS__)\
    sstr << " FROM " << tablename_;\
    query_sq_prefix_ = sstr.str();\
  }\
  bool CY_class_name##QSqliteIO::insertion_field_activation(const std::string& colName, bool activation){\
    bool found = false;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_helpers_gnapa_01,__VA_ARGS__)\
    calc_iq_string_prefix_();\
    return found;\
  }\
  std::string  CY_class_name##QSqliteIO::table_name(const std::string& name){\
    tablename_ = name;\
    calc_iq_string_prefix_();\
    calc_sq_string_prefix_();\
    return tablename_;\
  }\
  std::string CY_class_name##QSqliteIO::table_name() const{\
    return tablename_;\
  }\
  bool CY_class_name##QSqliteIO::volatile_stack() const{return volatile_stack_;}\
  bool CY_class_name##QSqliteIO::volatile_stack(bool v){return (volatile_stack_ = v);}


    #define cyt3macro_model_class_serializable_qsqlite_def_conf_item_(cy_var_name,cy_col_name,cy_var_type)\
      {\
        bool lf_=false;\
        if(std::string(cy_var_type).find(INTEGER) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::INTEGER));\
        if(std::string(cy_var_type).find(TEXT) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::TEXT));\
        if(std::string(cy_var_type).find(BLOB) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::BLOB));\
        if(std::string(cy_var_type).find(NUMERIC) != std::string::npos)lf_ = (true | static_cast<int>(cols_ttr_.cy_var_name = DataType::NUMERIC));\
        done_ok&=lf_;\
        \
        if(std::string(cy_var_type).find(AUTOINCREMENT) != std::string::npos)cols_w_act_.cy_var_name = false;\
        if(std::string(cy_var_type).find(PRIMARY_KEY) != std::string::npos)primary_key_field_ = cy_col_name;\
      }

  #define cyt3macro_model_class_serializable_qsqlite_def_conf_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::conf_datatype_matx_(){\
    bool done_ok=true;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_conf_item_, __VA_ARGS__)\
    return done_ok;\
  }



  //INSERT ITEMS
      #define cyt3macro_model_class_serializable_qsqlite_def_insert_items_iter_(cy_var_name,cy_col_name,cy_var_type)\
      if(cols_w_act_.cy_var_name == true){\
        if(firstItem==false)sstr << ", ";else firstItem=false;\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
        sstr << item.cy_var_name();\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
      }  

    #define cyt3macro_model_class_serializable_qsqlite_def_insert_items_it_(cy_var_name,cy_col_name,cy_var_type, ...)\
    if(cols_w_act_.cy_var_name == true){\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      sstr << item.cy_var_name();\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      firstItem=false;\
    }\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_insert_items_iter_,__VA_ARGS__)


  #define cyt3macro_model_class_serializable_qsqlite_def_insert_items_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::insert_table_items(std::string& err){\
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
     cyt3macro_model_class_serializable_qsqlite_def_insert_items_it_(__VA_ARGS__)\
     sstr << ")";\
     numItem++;\
     return true;\
    });\
    sstr << ";";\
    QString queryString = QString::fromStdString(sstr.str());\
    bool result = makeInsertionQuery(queryString);\
    if(result == false){CLOG_WARN("cyt3macro-qsqlite-io : " \
      #CY_class_name " : error : query[" << sstr.str() << "]")\
      return false;\
    }\
    if(volatile_stack_ == true)stack.clear();\
    return result;\
  }

  //DELETE ITEM
    #define cytemacro_model_class_serializable_qsqlite_def_delete_item_from_pk_field_value_(cy_var_name,cy_col_name,cy_var_type)\
        if(std::string(cy_var_type).find(PRIMARY_KEY) != std::string::npos){\
          if(cols_w_str_.cy_var_name == true) sstr << "'";\
          sstr << i.cy_var_name();\
          if(cols_w_str_.cy_var_name == true) sstr << "'";\
        }  

  #define citemacro_model_class_serializable_qsqlite_def_delete_item_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::delete_table_item(const CY_class_name& i, std::string& err){\
    if(primary_key_field_.size() == 0){err = "error : this table has no primary key field;"; return false;}\
    std::stringstream sstr;\
    sstr << "DELETE FROM " << tablename_ << " WHERE " << primary_key_field_ << " = ";\
    FOR_EACH_TRIPLES(cytemacro_model_class_serializable_qsqlite_def_delete_item_from_pk_field_value_,__VA_ARGS__)\
    sstr << ";";\
    bool result = makeQuery(QString::fromStdString(sstr.str()),err);\
    if(result==false){CLOG_WARN("cyt3macro-qsqlite-io : error = " << err << " : query[" << sstr.str() << "]")}\
    return result;\
  }

  #define citemacro_model_class_serializable_qsqlite_def_delete_items_stack_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::delete_table_items(std::string& err){\
    std::size_t s = stack.size();\
    std::size_t ok = \
    stack.for_each([&](const CY_class_name& i){\
     std::string errItem;\
     if(delete_table_item(i,errItem) == false){err+=errItem; err+=":"; return false;}\
     return true;\
    });\
    if((s == ok) && (volatile_stack_ == true))stack.clear();\
    return s == ok;\
  }

    

  //UPDATE ITEM
      #define cytemacro_model_class_serializable_qsqlite_def_update_item_field_each_(cy_var_name,cy_col_name,cy_var_type)\
      if(cols_w_act_.cy_var_name == true){\
        if(firstItem==false)sstr << ", ";else firstItem=false;\
        sstr << cy_col_name << " = ";\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
        sstr << i.cy_var_name();\
        if(cols_w_str_.cy_var_name == true) sstr << "'";\
      }  

    #define cytemacro_model_class_serializable_qsqlite_def_update_item_field_first_(cy_var_name,cy_col_name,cy_var_type, ...)\
    if(cols_w_act_.cy_var_name == true){\
      sstr << cy_col_name << " = ";\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      sstr << i.cy_var_name();\
      if(cols_w_str_.cy_var_name == true) sstr << "'";\
      firstItem=false;\
    }\
    FOR_EACH_TRIPLES(cytemacro_model_class_serializable_qsqlite_def_update_item_field_each_,__VA_ARGS__)

    #define cytemacro_model_class_serializable_qsqlite_def_update_item_field_write_pk(cy_var_name, cy_col_name, cy_var_type)\
    if(std::string(cy_var_type).find("PRIMARY KEY") != std::string::npos) sstr << i.cy_var_name();\

  #define cyt3macro_model_class_serializable_qsqlite_def_update_item_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::update_table_item(const CY_class_name& i, std::string& err){\
    if(primary_key_field_.size() == 0){err = "error : this table has no primary key field;"; return false;}\
    std::stringstream sstr;\
    sstr << "UPDATE " << tablename_ << " SET ";\
    bool firstItem = true;\
    cytemacro_model_class_serializable_qsqlite_def_update_item_field_first_(__VA_ARGS__)\
    sstr << " WHERE " << primary_key_field_ << " = ";\
    FOR_EACH_TRIPLES(cytemacro_model_class_serializable_qsqlite_def_update_item_field_write_pk, __VA_ARGS__)\
    sstr << ";";\
    bool result; result = makeQuery(QString::fromStdString(sstr.str()), err);\
    if(result == false){CLOG_WARN("cyt3macro-qsqlite-io : error = " << err << " : query[" << sstr.str() << "]")}\
    return result;\
  }


  #define cytemacro_model_class_serializable_qsqlite_def_update_items_stack_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::update_table_items(std::string& err){\
    std::size_t s = stack.size();\
    std::size_t ok = \
    stack.for_each([&](const CY_class_name& item){\
      std::string errItem;\
      if(update_table_item(item,errItem) == false){err+=errItem; err+=":"; return false;}\
      return true;\
    });\
    if((s == ok) && (volatile_stack_ == true))stack.clear();\
    return s == ok;\
  }



    #define cyt3macro_model_class_serializable_qsqlite_def_query_items_ctl_(cy_var_name,cy_col_name,cy_var_type)\
    int index_##cy_var_name = q.record().indexOf(cy_col_name);\
    if(index_##cy_var_name ==  -1){err += "COLUMN '" cy_col_name "' DOES NOT EXIST IN TABLE;";result = false;}

    #define cyt3macro_model_class_serializable_qsqlite_def_query_items_recv_(cy_var_name,cy_col_name,cy_var_type)\
      result &= extract(q.value(index_##cy_var_name),buffer.cy_var_name());

  #define cyt3macro_model_class_serializable_qsqlite_def_query_items_(CY_class_name, ...)\
  bool CY_class_name##QSqliteIO::select_table_items(std::string& err,const std::string& where){\
    std::stringstream sstr;\
    std::lock_guard<std::mutex> guard(mtx_obj_);\
    sstr << query_sq_prefix_;\
    if(where.size() != 0){\
      sstr << " WHERE " << where;\
    }\
    sstr << ";";\
    QSqlQuery q;\
    bool result;\
    result = makeSelectQuery(QString::fromStdString(sstr.str()), q, err);\
    if(result == false)return false;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_query_items_ctl_,__VA_ARGS__)\
    if(result == false)return false;\
    query_stack.clear();\
    while(q.next()){\
      CY_class_name buffer;\
      FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_query_items_recv_,__VA_ARGS__)\
      query_stack.push_back(buffer);\
    }\
    return result;\
  }    

  #define cyt3macro_model_class_serializable_qsqlite_def_extracts_(CY_class_name)\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,bool& destination){\
    destination = source.toBool();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,int& destination){\
    destination = source.toInt();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,uint& destination){\
    destination = source.toUInt();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,int64_t& destination){\
    destination = source.toLongLong();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,uint64_t& destination){\
    destination = source.toULongLong();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,double& destination){\
    destination = source.toDouble();return true;\
  }\
  bool CY_class_name##QSqliteIO::extract(const QVariant& source,std::string& destination){\
    destination = source.toString().toStdString();return true;\
  }

      #define cyt3macro_model_class_serializable_qsqlite_def_check_table_elems_it_(cy_var_name,cy_col_name,cy_var_type)\
      sstr << ", `" cy_col_name "` " cy_var_type; 
    
    #define cyt3macro_model_class_serializable_qsqlite_def_check_table_elems_(cy_var_name,cy_col_name,cy_var_type,...)\
    sstr << " `" cy_col_name "` " cy_var_type;\
    FOR_EACH_TRIPLES(cyt3macro_model_class_serializable_qsqlite_def_check_table_elems_it_,__VA_ARGS__)

  #define cyt3macro_model_class_serializable_qsqlite_def_check_table(CY_class_name, ...)\
  bool  CY_class_name##QSqliteIO::check_create_table(std::string& err){\
    std::stringstream sstr;\
    sstr << "CREATE TABLE IF NOT EXISTS `" << tablename_ << "` (";\
    cyt3macro_model_class_serializable_qsqlite_def_check_table_elems_(__VA_ARGS__)\
    sstr << ");";\
    bool result = makeCreateTableQuery(QString::fromStdString(sstr.str()),err);\
    return result;\
  }



#define CYT3MACRO_model_class_serializable_qsqlite_definitions(CY_class_name,CY_options, ...)\
\
cyt3macro_model_class_serializable_qsqlite_def_update_item_(CY_class_name, __VA_ARGS__)\
\
cytemacro_model_class_serializable_qsqlite_def_update_items_stack_(CY_class_name, __VA_ARGS__)\
\
citemacro_model_class_serializable_qsqlite_def_delete_item_(CY_class_name, __VA_ARGS__)\
\
citemacro_model_class_serializable_qsqlite_def_delete_items_stack_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_constr_destr_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_supp_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_helpers_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_conf_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_insert_items_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_query_items_(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_check_table(CY_class_name, __VA_ARGS__)\
\
cyt3macro_model_class_serializable_qsqlite_def_extracts_(CY_class_name)




