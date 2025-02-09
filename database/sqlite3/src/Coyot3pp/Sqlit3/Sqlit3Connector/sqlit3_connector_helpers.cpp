#include <Coyot3pp/Sqlit3/Sqlit3Connector/sqlit3_connector_helpers.hpp>

// SQLITE_INTEGER, SQLITE_FLOAT, SQLITE_TEXT, SQLITE_BLOB, or SQLITE_NULL
namespace coyot3{
namespace ddbb{
namespace sqlite{
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, bool& d){
    CLOG_INFO("borrar : getting bool 0")
    if(sqlite3_column_type(stmt,c) != SQLITE_INTEGER)return false;
    CLOG_INFO("borrar : getting bool")
    d = static_cast<bool>(sqlite3_column_int(stmt,c));
    return true;
  }

  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, int& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_INTEGER)return false;
    d = sqlite3_column_int(stmt,c);
    return true;
  }
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, int64_t& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_INTEGER)return false;
    d = static_cast<int64_t>(sqlite3_column_int64(stmt,c));
    return true;
  }

  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, uint& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_INTEGER)return false;
    d = static_cast<uint>(sqlite3_column_int64(stmt,c));
    return true;
  }
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, uint64_t& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_INTEGER)return false;
    d = static_cast<uint64_t>(sqlite3_column_int64(stmt,c));
    return true;
  }
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, std::string& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_TEXT)return false;
    const unsigned char* pl = sqlite3_column_text(stmt,c);
    int ps = sqlite3_column_bytes(stmt,c);
    char* buffer = new char[ps];
    memcpy(buffer,pl,static_cast<size_t>(ps));
    buffer[ps] = '\0';
    d= buffer;
    d = quote_decode(d);
    delete buffer;
    return true;
  }
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, double& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_FLOAT)return false;
    d = static_cast<double>(sqlite3_column_double(stmt,c));
    return true;
  }
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, std::vector<uint8_t>& d){
    if(sqlite3_column_type(stmt,c) != SQLITE_BLOB)return false;
    const void* pl = sqlite3_column_blob(stmt,c);
    int ps = sqlite3_column_bytes(stmt,c);
    d.clear();
    for(int i=0;i<ps;++i){
      d.push_back(((uint8_t*)pl)[i]);
    }
    return true;
  }



  std::string quote_encode(const std::string& input){
    std::size_t p=0;
    std::string r = input;
    while((p = r.find_first_of("'",p)) != std::string::npos){
      r.replace(p,1,"&lsquo;");
      p++;
    }
    return r;
  }
  std::string quote_decode(const std::string& input){
    std::size_t p=0;
    std::string r = input;
    while((p = r.find_first_of("&lsquo;")) != std::string::npos){
      r.replace(p,7,"'");
      p++;
    }
    return r;
  }
}
}
}