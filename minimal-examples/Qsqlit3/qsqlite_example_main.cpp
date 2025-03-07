
#include "qsqlite_connector_example.hpp"
#include <QCoreApplication>
#include <signal.h>

SqliteDBStopsManager::SqliteDBStopsManager(QObject* parent)
:db_connector_(nullptr)
,manager_timer_(nullptr)
,serviceStopsIO(nullptr)
, serviceStopsCurrent()
, serviceStopsChanged(false)
{
  CLOG_INFO(" manager created ")
}
SqliteDBStopsManager::~SqliteDBStopsManager(){
  CLOG_WARN(" destroying manager")
  if(db_connector_)delete db_connector_;
}
bool
SqliteDBStopsManager::Start(){
  db_connector_  = new coyot3::ddbb::sqlite::QSqlit3Connector(this);
  serviceStopsIO = new ServiceStopDAOQSqliteIO(this);
  manager_timer_ = new QTimer(this);

  connect(manager_timer_,&QTimer::timeout, 
          this, &SqliteDBStopsManager::timer_check_status_callback);

  db_connector_->database_name("database_path.sqlite"); //master db connector
  serviceStopsIO->table_name("table_positions");
  // service stops io will behave as a table manager acting over the database
  // specified at db-connector.
  serviceStopsIO->master_attach(*db_connector_);  
  
  
  manager_timer_->setInterval(1000);

  db_connector_->Init();
  db_connector_->Start();

  std::string errString;
  
  if(initialize_read() == false){
    CLOG_ERROR(" error starting manager. initialization error")
    return false;
  }

  manager_timer_->start();
  CLOG_INFO("db stops manager started.")
  return true;

}

const ServiceStopDAOStack& SqliteDBStopsManager::stop_data() const{
  std::string err;
  serviceStopsIO->select_table_items(err);
  return serviceStopsIO->query_stack;
}
bool
SqliteDBStopsManager::Stop(){

  return false;
}

bool 
SqliteDBStopsManager::initialize_read(){
  std::string err;
  if(!serviceStopsIO->check_create_table(err)){
    CLOG_ERROR(" initialize-read error checking table [" << err << "]")
    return false;
  }
  return true;
}


bool SqliteDBStopsManager::pushStop(const ServiceStopDAO& stop){
  CLOG_INFO("pushing stop data")
  return serviceStopsIO->stack.push_back(stop);
}


void SqliteDBStopsManager::timer_check_status_callback(){
  std::string err;
  int numitems = serviceStopsIO->stack.size();
  if(!serviceStopsIO->insert_table_items(err)){
    CLOG_WARN("error pushing [" << numitems << "] items")
  }else{
    CLOG_INFO("pushed " << numitems << " items")
  }
  delete_random_line();
  update_random_line();
}
void SqliteDBStopsManager::update_random_line(){
  ServiceStopDAO oneItem;
  oneItem.id(std::rand() % stop_data().size());
  oneItem.name("he sido actualizado");
  oneItem.active(true);
  oneItem.description("he sido actualizado");
  oneItem.latitude(1.1),
  oneItem.altitude(3.3);
  oneItem.longitude(2.2);
  std::string err;
  if(!serviceStopsIO->update_table_item(oneItem,err)){
    CLOG_WARN("error updating item : " << err)
  }
}
void SqliteDBStopsManager::delete_random_line(){
  ServiceStopDAO oneItem(stop_data()[std::rand() % stop_data().size()]);
  std::string err;
  if(!serviceStopsIO->delete_table_item(oneItem,err)){
    CLOG_WARN("error deleting random item [" << oneItem.to_string() << "] : error [" << err << "]")
  }
}


////
////

bool DO_STUFF = true;
QCoreApplication* app;
void on_control_c(int s){
  CLOG_WARN(" - catched CONTROL-C")
  DO_STUFF = false;
  app->exit();
}

//
//
int main(int argc, char** argv){

  QCoreApplication *app = new QCoreApplication(argc,argv);
  signal(SIGINT,&on_control_c);
  SqliteDBStopsManager dbmanager(app);

  if(!dbmanager.Start()){
    CLOG_ERROR("manager start error")
    exit(1);
  }

  std::thread* thr = new std::thread([&](){
    int iteration = 0;
    while(DO_STUFF){
      CLOG_INFO("generating random item")
      ServiceStopDAO item;
      item.name("stopName_" + coyot3::tools::generate_string_aphanumeric(10));
      item.description("any description");
      item.latitude(coyot3::tools::generate_real_number(-180,180));
      item.longitude(coyot3::tools::generate_real_number(-180,180));
      item.altitude(coyot3::tools::generate_real_number(0,500.0));
      item.active(static_cast<bool>(coyot3::tools::generate_natural_number(0,10)%2));
      dbmanager.pushStop(item);

      iteration++;

      if(iteration%10 == 0){
        CLOG_INFO(" - current service stop data:")
        dbmanager.stop_data().for_each([&](const ServiceStopDAO& i){
          CLOG_INFO("   item : " << i.id() << ",name:" << i.name() 
            << ",description:" << i.description() << ",active=" << i.active() 
            << ",lat=" << i.latitude() << ",lon=" << i.longitude() << ",alt=" 
            << i.altitude())
          return true;
        });
      }
      usleep(250000);
    }
  });
  
  app->exec();

  thr->join();
  delete thr;


  
}
