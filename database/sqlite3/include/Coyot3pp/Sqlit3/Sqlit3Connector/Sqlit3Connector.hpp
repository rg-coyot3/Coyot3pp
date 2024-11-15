#pragma once

#include <Coyot3pp/Cor3/Coyot3.hpp>
#include <Coyot3pp/Cor3/ModuleBase.hpp>
#include <sqlite3.h>

namespace coyot3{
namespace ddbb{
namespace sqlite{

  class Sqlit3Connector : coyot3::mod::ModuleBase{
    public:

      Sqlit3Connector(const std::string& name = std::string());
      virtual ~Sqlit3Connector();


      std::string database_name() const;
      std::string database_name(const std::string& tn);

      bool make_query(const std::string& q);
      bool make_query(const std::string& q, std::string& err);

      bool master_attach(Sqlit3Connector* connector);


    protected:

      
      bool start();
      bool stop();
      

      std::string   database_name_;
      std::string   table_name_;

      sqlite3*          db_own_;
      Sqlit3Connector*  connector_ext_;

      std::mutex mtx_db_;


      
      bool _open_database();
      bool _close_database();



      static std::string error_code_string(int rc);

  };


}
}
}