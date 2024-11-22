#include "sqlit3_macro_classes.hpp"


SimpleClass generate_item(){
  SimpleClass newItem;
  newItem.name(coyot3::tools::generate_string_aphanumeric(10));
  newItem.surname(coyot3::tools::generate_string_aphanumeric(20));
  newItem.age(coyot3::tools::generate_natural_number(0,80));
  newItem.height(coyot3::tools::generate_real_number(0,2));
  return newItem;
}

bool show_items(SimpleClassSqlit3IO& c){
  int n = 0;
  CLOG_INFO("---- show items of table [" << c.table_name() << "]: BEGIN")
  c.stack_input.for_each([&](const SimpleClass& item){
    CLOG_INFO(" items read " << n++ << ": " << item.to_string())
    return true;
  });
  CLOG_INFO("---- show items of table [" << c.table_name() << "]: END")
  return true;
}

bool configure_connector(SimpleClassSqlit3AutoIO& c, 
          coyot3::ddbb::sqlite::Sqlit3Connector& master,
          const std::string& tablename,
          int64_t interval){
  CLOG_INFO("configuring table [" << c.table_name() 
    << "] connector to master [" << master.database_name() << "]")
  c.table_name(tablename);      
  c.insertion_interval(interval);
  c.master_attach(&master);
  
  return true;
}

int main(int argc, char** argv){
  coyot3::ddbb::sqlite::Sqlit3Connector master;

  master.database_name("example_multiple_tables.sqlite");
  
  SimpleClassSqlit3AutoIO example1,example2,example3;
  CLOG_INFO(" configuring instances")
  configure_connector(example1,master,"table_ex_1",5000);
  configure_connector(example2,master,"table_ex_2",5000);
  configure_connector(example3,master,"table_ex_3",5000);

  CLOG_INFO("Starting master");
  master.Start();
  
  if( !example1.Start()
    ||!example2.Start()
    ||!example3.Start()
  ){  
    CLOG_ERROR("error startig the connectors")
  }
  bool doloop = true;
  std::thread* thr = new std::thread([&]{
    CLOG_INFO("thread database read start")
    while(doloop){
      if(example1.get_table_items("") == false){
        CLOG_ERROR("error getting all items")
      }else{
        show_items(example1);        
      }
      if(example2.get_table_items("") == false){
        CLOG_ERROR("error getting all items")
      }else{
        show_items(example2);        
      }
      if(example2.get_table_items("") == false){
        CLOG_ERROR("error getting all items")
      }else{
        show_items(example3);        
      }
      sleep(6);
    }
  });



  int ni = 0;
  sleep(1);
  CLOG_INFO("writing database")
  while(doloop){
    /* inserting items in intervals of 1, 2 and 3 seconds...*/
    CLOG_INFO("pulse")
    if(!(ni%1))example1.push_item(generate_item());
    if(!(ni%2))example2.push_item(generate_item());
    if(!(ni%3))example3.push_item(generate_item());
    
    sleep(1);
    if(ni++ > 30)doloop = false; // ... for a total of 60 seconds.
  }
  CLOG_INFO(" - exiting loop") 
  thr->join();
  CLOG_INFO(" - ending")
  


}