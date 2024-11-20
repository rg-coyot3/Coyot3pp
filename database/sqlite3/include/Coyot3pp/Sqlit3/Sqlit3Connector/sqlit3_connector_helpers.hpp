#pragma once
#include <Coyot3pp/Cor3/Coyot3.hpp>

#include <sqlite3.h>

namespace coyot3{
namespace ddbb{
namespace sqlite{
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, bool& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, int& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, int64_t& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, uint& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, int64_t& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, uint64_t& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, std::string& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, double& d);
  bool sqlit3_get_col(sqlite3_stmt* stmt, int c, std::vector<uint8_t>& d);
  
  
}
}
}