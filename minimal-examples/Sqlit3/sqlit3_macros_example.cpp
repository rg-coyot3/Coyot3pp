#include "sqlit3_macro_classes.hpp"




int main(int argc, char** argv){
  SimpleClassSqlit3IO example;

  int doloop = true;
  example.database_name("example_database2.sqlite");
  example.table_name("example_table");
  example.volatile_stack(true);
  example.modlog_verbosity(10);

  CLOG_INFO("opening database")
  if(!example.Start()){
    CLOG_ERROR("error initializing database")
    exit(1);
  }

  if(!example.Start()){
    CLOG_ERROR("error opening database")
    exit(1);
  }

  std::string err;
  CLOG_INFO("checking table creation")
  if(example.check_create_table(err) == false){
    CLOG_ERROR("error checking the table : " << err)
    exit(1);
  }

  std::thread* thr = new std::thread([&]{
    CLOG_INFO("thread database read start")
    while(doloop){
      if(example.select_table_items("") == false){
        CLOG_ERROR("error getting all items")
      }else{
        int n = 0;
        example.query_stack.for_each([&](const SimpleClass& item){
          CLOG_INFO(" items read " << n++ << ": " << item.to_string())
          return true;
        });
      }
      sleep(1);
    }
  });

  int ni = 0;
  sleep(1);
  CLOG_INFO("writing database")
  while(doloop){
    SimpleClass newItem;
    newItem.name(coyot3::tools::generate_string_aphanumeric(10));
    newItem.surname(coyot3::tools::generate_string_aphanumeric(20));
    newItem.age(coyot3::tools::generate_natural_number(0,80));
    newItem.height(coyot3::tools::generate_real_number(0,2));
    CLOG_INFO(" - inserting item : " << newItem.to_string())
    example.push_item(newItem);
    
    sleep(1);
    if(ni++ > 10)doloop = false;
    if(!(ni%2 == 0))if(!example.insert_table_items(err)){
      CLOG_WARN(" - error inserting items = " << err)
    }
  }
  CLOG_INFO(" - exiting loop")
  thr->join();
  CLOG_INFO(" - ending")
  


}