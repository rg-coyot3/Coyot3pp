#include <Coyot3pp/QSqlit3/QSqlit3Connector/QSqlit3Connector.hpp>



namespace coyot3{
namespace ddbb{
namespace sqlite{

CYT3MACRO_model_class_definitions(
  QSqlit3ConnectorConfigObject
  , 
  , ( )
  , ( )
    , db_location         , std::string           , ""
    , connection_name     , std::string           , ""
    , check_state_interval, int64_t               , 60000
)

  CYT3MACRO_model_class_serializable_json_definitions(
    QSqlit3ConnectorConfigObject
    , 
    , 
    , ( )
    , ( )
      , db_location          , "db_location"          ,
      , connection_name      , "connection_name"      ,
      , check_state_interval , "check_state_interval" ,
  )

  QSqlit3Connector::QSqlit3Connector(QObject* parent)
  :QObject(parent)

  ,state(State::ERROR)

  ,owns_db_(true)
  ,external_instance_(nullptr)
  ,database(nullptr)
  ,connection_name_()
  ,dbname()
  
  ,timerStatus(nullptr)

  ,config_()
  {


    CLOG_INFO("SQLITE CONNECTOR : instance created");
    state = State::CREATED;
    CLOG_DEBUG_LEVEL_SET(7);
    
  }
  QSqlit3Connector::~QSqlit3Connector()
  {
    CLOG_WARN("sqlite-connector : destroying instance")
    if(external_instance_ != nullptr){
      master_detach();
    }
    for(std::map<QSqlit3Connector*,const QSqlit3Connector*>::iterator it = slaves_.begin();
        it != slaves_.end();++it){
      it->first->master_detach();
    }
    closeDatabase();
  }

  void QSqlit3Connector::set_debug_level_(int level)
  {
    CLOG_DEBUG_LEVEL_SET(level);
  }

  bool QSqlit3Connector::setConfiguration(const Json::Value& cfg)
  {
    
    QSqlit3ConnectorConfigObjectJsIO buffer;
    if(buffer.from_json(cfg) == false){
      CLOG_WARN("sqlite-connector : set-configuration : error importing configuration")
      return false;
    }
    return setConfiguration(buffer);
  }
  bool 
  QSqlit3Connector::setConfiguration(const QSqlit3ConnectorConfigObject& cfg){
    config_ = cfg;
    dbname                = config_.db_location();
    connection_name_      = config_.connection_name();
    timer_status_interval = config_.check_state_interval();
    return true;
  }
  // bool QSqlit3Connector::check_configuration_structure()
  // {
  //   if( 
  //       !coyot3::tools::json_contains_member(config,Config::db_location)
  //     ||!coyot3::tools::json_contains_member(config,Config::connection_name)
  //     ||!coyot3::tools::json_contains_member(config,Config::check_state_interval)
  //   ){
  //     CLOG_ERROR("SQLITE CONNECTOR : set-configuration : review the configuration structure");
  //     return false;
  //   }
  //   try{
  //     dbname = config[Config::db_location].asString();
  //     timer_status_interval = config[Config::check_state_interval].asInt();
  //     connection_name = config[Config::connection_name].asString();
  //   }catch(const Json::Exception& e)
  //   {
  //     CLOG_ERROR("SQLITE CONNECTOR : init : error acquiring configuration : "
  //       "err(" << e.what());
  //     state = State::ERROR;
  //     return false;
  //   }catch(...){
  //     CLOG_ERROR("SQLITE CONNECTOR : init : error acquiring configuration : "
  //       "unkown");
  //     return false;
  //   }
  //   CLOG_DEBUG(5,"SQLITE CONNECTOR : set-configuration : config structure ok");
  //   return true;
  // }
  bool 
  QSqlit3Connector::Init()
  {
    //apply configuration
    if(external_instance_ != nullptr){
      CLOG_WARN("sqlite-connector : init : there is an attached external "
      "instance. init() is not applicable.")
      return false;
    }
    if(state != State::CREATED)
    {
      CLOG_ERROR("SQLITE CONNECTOR : init : state does not permit "
        "initialization: (" << ModuleStateToString(state) << ")");
      return false;
    }
    
    // if(!check_configuration_structure())
    // {
    //   CLOG_ERROR("SQLITE CONNECTOR : init : check configuration structure error");
    //   state = State::ERROR;
    //   return false;
    // }

    if(timer_status_interval < 3000){
      CLOG_WARN("sqlite-connector : init : timer status interval updated to 3 "
        "seconds")
      timer_status_interval = 3000;
    }
    timerStatus = new QTimer(this);
    timerStatus->setInterval(timer_status_interval);
    if(!connect(timerStatus,&QTimer::timeout,this,&QSqlit3Connector::evaluateState))
    {
      CLOG_ERROR("SQLITE CONNECTOR : init : error connecting qtimer");
      state = State::ERROR;
      return false;
    }
    
    
    state = State::CONFIGURED;
    CLOG_INFO("SQLITE CONNECTOR : init : done");
    return true;
  }
  bool 
  QSqlit3Connector::Start()
  {
    if(external_instance_ != nullptr){
      CLOG_WARN("sqlite-connector : start : there is an attached external "
      "instance. start() is not applicable.")
      return false;
    }
    if(state != State::CONFIGURED)
    {
      CLOG_ERROR("SQLITE CONNECTOR : start : state does not permit starting : ("
        << ModuleStateToString(state) << ")");
      return false;
    }
    state = State::LAUNCHING;
    if(owns_db_ == true)
    {
      if(!openDatabase())
      {
        CLOG_ERROR("SQLITE CONNECTOR : start : error opening database");
        state = State::ERROR;
        return false;
      }
      timerStatus->start();
    }else{
      CLOG_INFO("sqlite-connector : start : this instance does not own the "
      "qdatabase. Ignoring step")
    }    
    state = State::LAUNCHED;
    return true;
  }
  
  bool 
  QSqlit3Connector::Stop()
  {
    if( (state != State::LAUNCHED)
        &&(state != State::LAUNCHING))
    {
      CLOG_WARN("SLITE CONNECTOR : stop : state is not valid to stop (" 
        << ModuleStateToString(state) << ")");
      return false;
    }
    timerStatus->stop();
    if(owns_db_ == true){
      if(!closeDatabase())
      {
        CLOG_WARN("SQLITE CONNECTOR : stop : error closing database");
        state = State::ERROR;
        return false;
      }
    }else{
      CLOG_INFO("sqlite-connector : stop : this instance does not own the "
      "database. ignoring step")
    }
    state = State::STOPPED;
    return true;
  }

  bool 
  QSqlit3Connector::Shutdown()
  {
    if(state != State::STOPPED)
    {
      CLOG_ERROR("SQLITE CONNECTOR : shutdown : state is not valid for shutdown");
      return false;
    }
    delete timerStatus;
    timerStatus = nullptr;
    return false;
  }


  bool 
  QSqlit3Connector::openDatabase()
  {
    CLOG_DEBUG(5,"SQLITE CONNECTOR : open-database : locking mutex. opening "
    "database for [" << connection_name_ << "]");
    std::lock_guard<std::mutex> guard(dbmutex);
    if(!database)
    {
      if(!QSqlDatabase::isDriverAvailable("QSQLITE"))
      {
        CLOG_ERROR("SQLITE CONNECTOR : open-database : error : driver "
        "QSQLITE driver is not available");
        return false;
      }
      database = new(std::nothrow) QSqlDatabase();
      if(!database){
        CLOG_ERROR("SQLITE CONNECTOR : open-database : error creating connector" 
          "instance");
        return false;
      }
      
      (*database) = QSqlDatabase::addDatabase(
        "QSQLITE",QString::fromStdString(connection_name_));

      CLOG_DEBUG(3,"SQLITE CONNECTOR : open-database : setting database name [" 
        << dbname << "]");
      database->setDatabaseName(QString::fromStdString(dbname));

    }
    
    if(database->isOpen())
    {
      CLOG_DEBUG(5,"SQLITE CONNECTOR : open-database : database is already opened");
      state = State::LAUNCHED;
      return true;
    }
    if(!database->open())
    {
      CLOG_ERROR("SQLITE CONNECTOR : open-database : error opening database");
      CLOG_ERROR("SQLITE CONNECTOR : open-database : error : [" 
        << database->lastError().text().toStdString() << "]");
      delete database;
      database = nullptr;
      return false;

    }
    CLOG_INFO("SQLITE CONNECTOR : open-databsae : database open at [" 
      << dbname << "]");
    state = State::LAUNCHED;
    return true;
  }

  bool 
  QSqlit3Connector::closeDatabase()
  {

    CLOG_DEBUG(5,"SQLITE CONNECTOR : close-database : locking mutex");
    std::lock_guard<std::mutex> guard(dbmutex);
    if(!database)
    {
      CLOG_WARN("SQLITE CONNECTOR : close-database : db already closed");
      return true;
    }

    if(database->isOpen())
    {
      CLOG_INFO("SQLITE CONNECTOR : close-database : now closing database");
      database->close();
      
    }

    delete database;
    database = nullptr;

    state = State::CONFIGURED;
    return true;
  }

  bool 
  QSqlit3Connector::makeLinkControlQuery()
  {
    std::lock_guard<std::mutex> guard(dbmutex);

    if(!database || state != State::LAUNCHED)
    {
      CLOG_ERROR("SQLITE CONNECTOR : make-link-control-query : state does not "
        "permit check database state control query");
      return false;
    }
    QSqlQuery q(*database);
    q.prepare("SELECT 1");
    if(!q.exec())
    {
      CLOG_WARN("SQLITE CONNECTOR : make-link-control-query : error making "
        "control query : [" 
        << database->lastError().text().toStdString() << "]");
      lastErrorString = q.lastError().text().toStdString();
      return false;
    }
    return true;
  }
  


  bool 
  QSqlit3Connector::makeSelectQuery(const QString& queryString,QSqlQuery& q){
    std::string err;
    return makeSelectQuery(queryString, q, err);
  }
  


  bool 
  QSqlit3Connector::makeSelectQuery(const QString& queryString,QSqlQuery& q, std::string& err)
  {

    std::lock_guard<std::mutex> guard(dbmutex);
    if(external_instance_!= nullptr){
      CLOG_DEBUG(6,"sqlite-connector : make-select-query : to external entity")
      return external_instance_->makeSelectQuery(queryString, q, err);
    }



    QSqlQuery lc(*database);
    
    if(state != State::LAUNCHED)
    {
      CLOG_WARN("SQLITE-CONNECTOR : make-select-query : database is not open");
    }
    if(!database || state != State::LAUNCHED)
    {
      CLOG_WARN("SQLITE CONNECTOR : make-query-select : warn : making query "
        "with unlaunched connector");
      return false; 
    }
    if(!lc.prepare(queryString))
    {
      CLOG_ERROR("SQLITE CONNECTOR : make-query-select : error preparing "
        "query : [" << queryString.toStdString() << "] ::: (" 
        << database->lastError().text().toStdString() << ")");
      lastErrorString = q.lastError().text().toStdString();
      return false;
    }
    if(!lc.exec())
    {
      CLOG_ERROR("SQLITE CONNECTOR : make-query-select : error executing query"
        " ::: (" << database->lastError().text().toStdString() << ") :: for query " 
        << queryString.toStdString().substr(0,(queryString.size() > 100? 100 : queryString.toStdString().size())) 
        << "]");
      err = lastErrorString = q.lastError().text().toStdString();
      return false;
    }
    q = lc;
    return true;
    
  }




  bool 
  QSqlit3Connector::makeInsertionQuery(const QString& queryString){
    std::string err;
    return makeInsertionQuery(queryString, err);
  }




  bool 
  QSqlit3Connector::makeInsertionQuery(const QString& queryString, std::string& err)
  {
    bool success = false;
    std::lock_guard<std::mutex> guard(dbmutex);
    if(external_instance_ != nullptr){
      return external_instance_->makeInsertionQuery(queryString, err);
    }
    if(state != State::LAUNCHED)
    {
      CLOG_WARN("SQLITE-CONNECTOR : make-insertion-query : state is not "
        "LAUNCHED : (" << ModuleStateToString(state) << ")");
      return false;
    }
    QSqlQuery q(*database);

    if(!q.prepare(queryString))
    {
      lastErrorString = q.lastError().text().toStdString();
      CLOG_WARN("SQLITE-CONNECTOR : make-insertion-query : "
        "error preparing query [" << lastErrorString << "](( " << queryString.toStdString() << " ))");
      
      return false;
    }
    CLOG_DEBUG(6,"SQLITE-CONNECTOR : make-insertion-query : sending query ((" 
      << queryString.toStdString() 
      << "))");
    if(q.exec())
    {
      success = true;
      CLOG_DEBUG(3,"SQLITE-CONNECTOR : make insertion query : succeeded");
    }else{
      lastErrorString = q.lastError().text().toStdString();
      CLOG_WARN("SQLITE-CONNECTOR : make insertion query : error making "
        "insertion query : [" << lastErrorString << "](( "<< queryString.toStdString() << " ))");
    }
    err = lastErrorString;
    return success;
  }

  bool 
  QSqlit3Connector::makeCreateTableQuery(const QString& queryString){
    std::string err;
    return makeCreateTableQuery(queryString,err);
  }

  bool 
  QSqlit3Connector::makeCreateTableQuery(const QString& queryString,std::string& err)
  {
    bool success = false;
    std::lock_guard<std::mutex> guard(dbmutex);
    if(external_instance_ != nullptr){
      return external_instance_->makeCreateTableQuery(queryString, err);
    }
    if(state != State::LAUNCHED)
    {
      CLOG_WARN("SQLITE-CONNECTOR : make-create-table-query : error : state "
        "is not launched : (" << ModuleStateToString(state) << ")");
      
      return false;
    }
    QSqlQuery q(*database);
    q.prepare(queryString);
    if(q.exec())
    {
      success = true;
      CLOG_DEBUG(3,"SQLITE-CONNECTOR : make-create-table-query : "
      "query done OK");
    }else{
      CLOG_WARN("SQLITE-CONNECTOR : make-create-table-query : "
      "error making query error:(" << q.lastError().text().toStdString());
      err = lastErrorString = q.lastError().text().toStdString();
    }
    return success;
  }

  void QSqlit3Connector::evaluateState()
  {   
    if(!database)
    {
      return;
    }
    

    if(database->isOpen())
    {
      CLOG_DEBUG(5,"SQLITE-CONNECTOR : eval-state :  OPEN OK : [" << database->databaseName().toStdString() << "]");
    }else{
      CLOG_WARN("SQLITE-CONNECTOR : eval-state : NOT OPEN : [" << database->databaseName().toStdString() << "]");
    }
    
  }

  bool QSqlit3Connector::database_owned(){return owns_db_;}
    
  std::string   QSqlit3Connector::database_name() const{
    return dbname;
  }
  std::string   QSqlit3Connector::database_name(const std::string& n){
    return (dbname = n);
  }
  std::string   QSqlit3Connector::connection_name() const{
    return connection_name_;
  }
  std::string   QSqlit3Connector::connection_name(const std::string& n){
    return connection_name_ = n;
  }
  int64_t QSqlit3Connector::keepalive_interval() const{
    return timer_status_interval;
  }
  int64_t QSqlit3Connector::keepalive_interval(int64_t i){
    return timer_status_interval = i;
  }

  bool QSqlit3Connector::master_attach(const QSqlit3Connector& master){
    if(external_instance_ != nullptr){
      CLOG_WARN("sqlite-connector : master-attach : error attaching instance. "
      "already attached to other master instance.")
      return false;
    }
    if((state == State::LAUNCHED) 
    || (state == State::LAUNCHING) 
    || (database != nullptr)){
      CLOG_WARN("sqlite-connector : master-attach : internal state is not "
      "ready to attach master instance.")
      return false;
    }
    CLOG_INFO("sqlite-connector : master-attach : attaching external master "
    "instance")
    external_instance_ = const_cast<QSqlit3Connector*>(&master);
    external_instance_->slave_attach_(this);
    return true;
  }
  
  bool
  QSqlit3Connector::master_detach(){
    external_instance_->slave_detach_(this);
    if(external_instance_ == nullptr){
      CLOG_WARN("sqlite-connector : master-detach : no master instance is "
      "attached to this one.")
      return true;
    }
    external_instance_ = nullptr;
    return true;
  }

  bool QSqlit3Connector::slave_attach_(QSqlit3Connector* c){
    std::lock_guard guard(dbmutex);
    std::map<QSqlit3Connector*, const QSqlit3Connector*>::iterator it;
    it = slaves_.find(c);
    if(it != slaves_.end()){
      return true;
    }
    slaves_.insert(std::make_pair(c, this));
    return true;
  }
  bool QSqlit3Connector::slave_detach_(QSqlit3Connector* c){
    std::lock_guard guard(dbmutex);
    std::map<QSqlit3Connector*, const QSqlit3Connector*>::iterator it;
    it = slaves_.find(c);
    if(it == slaves_.end()){
      return true;
    }
    slaves_.erase(it);
    return true;
  }

}
}
}