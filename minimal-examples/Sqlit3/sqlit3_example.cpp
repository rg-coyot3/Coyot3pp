#include <Coyot3pp/Sqlit3/Sqlit3Connector/Sqlit3Connector.hpp>



int main(int argc, char** argv){
  coyot3::ddbb::sqlite::Sqlit3Connector dbconnector("any name");

  CLOG_INFO("created database connector.")
  dbconnector.database_name("example.sqlite");
  CLOG_INFO("database name set")
  if(!dbconnector.Start()){
    CLOG_ERROR("error opening database")
    exit(0);
  }
  CLOG_INFO("checking table creation")
  std::string err;
  bool res = dbconnector.make_query("CREATE TABLE IF NOT EXISTS example_table ("
    "id INTEGER PRIMARY KEY, "
    "name TEXT, "
    "description TEXT);", err);
  if(res == false){
    CLOG_ERROR("error checking table creation : [" << err << "]")
    exit(1);
  }
  // res = dbconnector.make_query("INSERT INTO example_table (name, description) VALUES "
  //       "( 'Ricardo GONZALEZ' , 'con el primer apellido' )"
  //       ", ( 'Ricardo ALMEIDA' , 'con el segundo apellido' );",err);
  // if(res == false){
  //   CLOG_ERROR("error inserting data at the table: " << err)
  //   exit(1);
  // }

  sqlite3_stmt* statement;
  CLOG_INFO("db ptr " << statement)
  res = dbconnector.make_query("SELECT id, name, description FROM example_table;",statement,err);
  CLOG_INFO("db ptr " << statement << "; err=" << err)
  if(res == false){
    CLOG_ERROR("error making select query : " << err);
    exit(1);
  }else{
    int id;
    std::string n,d;
    CLOG_INFO(" ---- listing results:")
    int rows = 0;
    bool doloop = true;
    while(doloop){
      int rc = sqlite3_step(statement);
      CLOG_INFO("reading : row = " << rows << "; rc = " << rc)

      switch(rc){
        case SQLITE_DONE:
          doloop = false;
          break;
        case SQLITE_ROW:
          CLOG_INFO("ended!!")
          
          if(coyot3::ddbb::sqlite::sqlit3_get_col(statement,0,id) == false){CLOG_WARN("ERROR OBTAINING COL [0]")};
          if(coyot3::ddbb::sqlite::sqlit3_get_col(statement,1,n) == false){CLOG_WARN("ERROR OBTAINING COL [1]")};
          if(coyot3::ddbb::sqlite::sqlit3_get_col(statement,2,d) == false){CLOG_WARN("ERROR OBTAINING COL [2]")};

        break;
        case SQLITE_ERROR:
          //dbconnector.reset_statement(statement,err);
          CLOG_WARN("sqlite error, statement reset. last-err=" << err);
          
        break;
        default: 
          CLOG_WARN("unknown error")
      }
      
        CLOG_INFO("line obtained (" << rows << ")[" << id << ", " << n << ", " << d << "]")
        rows++;
        if(rows > 1000){doloop = false;}
        
    }
    sqlite3_finalize(statement);
  }

  if(!dbconnector.Stop()){
    CLOG_ERROR("error closing database")
    exit(1);
  }
  CLOG_INFO("closing database")
  return 0;

}