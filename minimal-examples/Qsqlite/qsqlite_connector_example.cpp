#include "qsqlite_connector_example.hpp"

  CYT3MACRO_model_class_definitions(
    DatabaseEntityDAO
    , 
    , ( )
    , ( )
      , id                  , int64_t           , 0
      , name                , std::string       , ""
      , description         , std::string       , ""
      , metadata            , std::string       , ""
  )

    CYT3MACRO_model_class_serializable_json_definitions(
      DatabaseEntityDAO
      , 
      , 
      , ( )
      , ( )
        , id                  , "id"                  ,
        , name                , "name"                ,
        , description         , "description"         ,
        , metadata            , "metadata"            ,
    )

// common-dao : end

// service-stop-dao : begin 

  CYT3MACRO_model_class_definitions(
    ServiceStopDAO
    , DatabaseEntityDAO
    , ( )
    , ( )
      , active              , bool                         , false
      , latitude            , double                       , 0.0
      , longitude           , double                       , 0.0
      , altitude            , double                       , 0.0
  )
    CYT3MACRO_model_class_set_stack_definitions(ServiceStopDAO,)

    CYT3MACRO_model_class_serializable_json_definitions(
    ServiceStopDAO
    , DatabaseEntityDAO
    ,
    , ( )
    , ( )
      , active              , "active"                            ,
      , latitude            , "latitude"                          ,
      , longitude           , "longitude"                         ,
      , altitude            , "altitude"                          ,
    )
    CYT3MACRO_model_class_set_stack_serializable_json_definitions(ServiceStopDAO)


  CYT3MACRO_model_class_serializable_qsqlite_definitions(
      ServiceStopDAO
    , ( )
    , id                  , "INTEGER PRIMARY KEY AUTOINCREMENT"   , "id"
    , name                , "TEXT"                                , "name"
    , description         , "TEXT"                                , "description"
    , active              , "INTEGER"                             , "active"
    , latitude            , "NUMERIC"                             , "latitude"
    , longitude           , "NUMERIC"                             , "longitude"
    , altitude            , "NUMERIC"                             , "altitude"
  )

SqliteDBStopsManager::SqliteDBStopsManager(QObject* parent = nullptr)
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
  db_connector_  = new coyot3::ddbb::sqlite::SqliteConnector(this);
  serviceStopsIO = new ServiceStopDAOQSqliteIO(this);
  manager_timer_ = new QTimer(this);

  connect(manager_timer_,&QTimer::timeout, 
          this, &SqliteDBStopsManager::timer_check_status_callback);

  db_connector_->database_name("database_path.sqlite");
  serviceStopsIO->set_table_name("table_positions");
  serviceStopsIO->master_attach(*db_connector_);
  manager_timer_->setInterval(1000);

  db_connector_->Init();
  db_connector_->Start();

  std::string errString;
  
  if(initialize_read() == false){
    CLOG_ERROR(" error starting manager.")
    return false;
  }



}
bool
SqliteDBStopsManager::Stop(){

}

bool 
SqliteDBStopsManager::initialize_read(){
  serviceStopsIO->
}

int main(int argc, char** argv){
  
}