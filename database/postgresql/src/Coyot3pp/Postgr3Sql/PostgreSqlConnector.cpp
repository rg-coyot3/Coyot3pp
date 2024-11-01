//#include "PostgreSqlConnector.h"
#include <Coyot3pp/Postgr3Sql/PostgreSqlConnector.hpp>


namespace coyot3{
namespace ddbb{
namespace postgresql{

CYT3MACRO_enum_class_definitions(
  ModuleState
  ,
    ,CREATED            
    ,CONFIGURED         
    ,LAUNCHING          
    ,LAUNCHED           
    ,STOPPED            
    ,SHUTDOWN           
    ,DISCONNECTED       
    ,CONNECTED          
    ,ERROR              
)


PostgreSqlConnector::PostgreSqlConnector()
:databaseName()
,username()
,password()
,hostAddr()
,hostPort()

,connect_timeout(_CYT_PSQLCTR_CFG_DEFAULT_CONNECTTIMEOUT_)
,keepalives(true)
,keepalives_iddle(_CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVEIDDLE_)
,keepalives_interval(_CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVEINTERVAL_)
,keepalives_count(_CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVECOUNT_)

,dbHandler(nullptr)
,dbWork(nullptr)

,state(static_cast<State>(-1))

,controlThreadPri(nullptr)
,controlThreadSec(nullptr)
,controlThFlagPri(false)
,controlThFlagSec(false)
,restartConnection(true)

,lastSuccessOperationTs(0)
{
  state = State::CREATED;
  CLOG_INFO("psql-connector : constructor : init OK");
}

PostgreSqlConnector::~PostgreSqlConnector()
{
  CLOG_WARN("psql-connector : constructor : removing instance");
}


bool PostgreSqlConnector::Init(){
  if(state != State::CREATED)
  {
    CLOG_WARN("psql-connector : init : invoked init in other than [CREATED]"
      " state : state = " << state);
  }
  CLOG_DEBUG(5,"psql-connector : init : creating timer");

  controlThreadPri = new(std::nothrow) coyot3::tools::ControlThread(
    std::bind(&PostgreSqlConnector::controlLink,this));
  controlThreadSec = new(std::nothrow) coyot3::tools::ControlThread(
    std::bind(&PostgreSqlConnector::controlTransaction,this));

  if((!controlThreadPri) || (!controlThreadSec))
  {
    CLOG_ERROR("psql-connector : init : ERROR CREATING CONTROL THREADS! nomem?");
    state = State::ERROR;
    if(databaseConnectorError_)databaseConnectorError_();
    return false;
  }
  CLOG_INFO("psql-connector : init : OK");
  state = State::DISCONNECTED;
  return true;
}

bool PostgreSqlConnector::Start(){
  bool res;
  CLOG_INFO("psql-connector : start : opening database : this may take some seconds");
  lastSuccessOperationTs = coyot3::tools::getCurrentTimestamp();
  
  controlThreadPri->start(3000);
  controlThreadSec->start(3000);
  return res;
}

bool PostgreSqlConnector::Stop()
{
  
  {
    //guard
    switch(state)
    {

      case State::CREATED:
      case State::ERROR:
      default:
        CLOG_WARN("psql-connector : stop : connector state is not stoppable."
          " state[" << state << "]");
        return false;
    }
    CLOG_DEBUG(5,"psql-connector : stop : stopping timer");
  }
  controlThreadPri->stop();
  controlThreadSec->stop();
  closeDatabase();
  state=State::DISCONNECTED;
  return true;
}

bool PostgreSqlConnector::Shutdown()
{
  if(state == State::CONNECTED)
  {
    Stop();
  }
  if(controlThreadPri!= nullptr)
  {
    delete controlThreadPri;
    controlThreadPri = nullptr;
  }
  if(controlThreadSec != nullptr)
  {
    delete controlThreadSec;
    controlThreadSec = nullptr;
  }
  
  state = State::SHUTDOWN;
  return true;
}

PostgreSqlConnector::State PostgreSqlConnector::getState(){return state;}



//invoked from controltransaction only.
bool PostgreSqlConnector::openDatabase()
{
  bool done = false;
  std::string cs;
  if(state != State::DISCONNECTED)
  {
    CLOG_ERROR("psql-connector : open database : connector is not in "
      "[CONFIGURED] state : current [" << state << "]");
      return false;
  } 
  if(dbHandler!=nullptr)
  {
    CLOG_WARN("psql-connector : open database : connector seems to be created");
    return false;
  }
  std::lock_guard<std::mutex> guard(connectionmtx);
  cs = getConnectionString();
  try{
    CLOG_INFO("psql-connector : open database : creating connection instance");
    dbHandler = new(std::nothrow) pqxx::connection(cs);
    if(!dbHandler)
    {
      CLOG_ERROR("psql-connector : open database : unable to create : NO MEM?");
      if(databaseConnectorError_)databaseConnectorError_();
      state = State::ERROR;
      return false;
    }
    CLOG_DEBUG(5,"psql-connector : open database : instance creation success");
    lastSuccessOperationTs = coyot3::tools::getCurrentTimestamp();
    state = State::CONNECTED;
    done = true;
  }
  catch(pqxx::sql_error const &e)
  {
    CLOG_WARN("psql-connector : open database : error opening database "
      ": reason sql except : " << e.what() << " :: query : " << e.query());
    if(dbHandler)
    {
      delete      dbHandler;
      dbHandler = nullptr;
    }
    state = State::DISCONNECTED;
  }
  catch(std::exception const &e)
  {
    CLOG_WARN("psql-connector : open database : error opening database "
      ": reason std except : " << e.what());
    if(dbHandler)
    {
      delete      dbHandler;
      dbHandler = nullptr;
    }
  } 
  if(done)
  {
    state = State::CONNECTED;
  }
  return done;
}


#include <sys/socket.h>

bool PostgreSqlConnector::closeDatabase()
{
  switch(state)
  {
    case State::CONNECTED:
    case State::DISCONNECTED:
      break;
    case State::CREATED:
    case State::ERROR:
    default:
      CLOG_WARN("psql-connector : close database : the connector state does not"
        " permit this operation [" << state << "]");
      return false;
  }
  if(!dbHandler)
  {
    CLOG_WARN("psql-connector : close database : invoking to close empty instance");
    return false;
  }
  
  CLOG_DEBUG(5,"psql-connector : close database : closing/stopping connection");
  try{
    //dbHandler->cancel_query(); //interrupt any transaction or ongoing connection
    // dbHandler->close();
  }catch(pqxx::sql_error const &e)
  {
    CLOG_ERROR("psql-connector : close database : SQLEXCEPT : cancel_query / close : " << e.what());
  }catch(std::exception  const &e)
  {
    CLOG_ERROR("psql-connector : close database : STDEXCEPT : cancel_query / close : " << e.what());
  }
  connectionmtx.lock();            //ensure that any potential pqxx::worker instance that points to this handler finishes its work
  state = State::DISCONNECTED;
  
  CLOG_WARN("psql-connector : close database : closing socket");
  shutdown(dbHandler->sock(),SHUT_RDWR);
  close(dbHandler->sock());
  CLOG_WARN("psql-connector : close database : trying to lock transaction mutex");
  int counter = 0;
  while(!transactionmtx.try_lock())
  {

    CLOG_WARN("psql-connector : close database : waiting transaction thread : " << counter << " : " << dbHandler->is_open() << " : " << dbHandler->sock());
    sleep(1);
  }
  CLOG_WARN("psql-connector : close database : deleting db handler");
  delete dbHandler;
  dbHandler = nullptr;
  cleanWork();
 
  CLOG_WARN("psql-connector : close database : deleting db work");
  CLOG_DEBUG(5,"psql-connector : close database : deleting work instance");
  // if(dbWork)
  // {
  //   //witch should not be the case
  //   CLOG_ERROR("psql-connector : to del : deleting dbwork. should not happen")
  //   delete dbWork;
  //   dbWork = nullptr;
  // }
  
  CLOG_DEBUG(5,"psql-connector : close database : deleting instance");
    
  transactionmtx.unlock();
  connectionmtx.unlock();

  CLOG_INFO("psql-connector : close database : closed OK");
  state = State::DISCONNECTED;
  return true;
}



void PostgreSqlConnector::controlLink()
{
  CLOG_INFO("psql-connector : control link : starting");
  int64_t diff;
  lastSuccessOperationTs = coyot3::tools::getCurrentTimestamp();
  while(controlThFlagPri)
  {
    diff = coyot3::tools::getCurrentTimestamp() - lastSuccessOperationTs;
    if(diff > _CYT_PSQLCTR_CFG_DEFAULT_NOTRX_RECONNECT_)
    {
      CLOG_WARN("psql-connector : control link : connection seems inactive "
        "after too long. It may be blocked. Now closing");
        closeDatabase(); // the control transaction will reopen the database
    }
    else
    {
      //nothing to do. all ok
    }
    sleep(3);

  }

  CLOG_WARN("psql-connector : link control : exited loop : ending");
}

void PostgreSqlConnector::controlTransaction()
{
  int64_t diff;
  CLOG_INFO("psql-connector : control transaction : starting");
  //the control transaction first steps are only invoked when "starting"
  // so this is the point to open the first connection
  bool res = openDatabase();
  if(res == true)
  {
    lastSuccessOperationTs = coyot3::tools::getCurrentTimestamp();
  }
  while(controlThFlagSec)
  {
    CLOG_DEBUG(5,"psql-connector : control transaction : pulse");
    diff = coyot3::tools::getCurrentTimestamp() - lastSuccessOperationTs;
    if(!dbHandler)
    {
      CLOG_DEBUG(5,"psql-connector : control transaction : re-opening database");
      openDatabase();
    }
    else
    {
      if(diff > _CYT_PSQLCTR_CFG_DEFAULT_NOTRX_MAKETEXTTRX_)
      {
        CLOG_DEBUG(5,"psql-connector : control transaction : making transaction");
        makeTestTransaction();
      }  
    }
    sleep(3);
  }
  CLOG_WARN("psql-connector : control transaction : ending : flag = " << controlThFlagSec);
  
}


void PostgreSqlConnector::cleanWork()
{
  std::lock_guard<std::mutex> guard(cleanworkmtx);
  if(dbWork)
  {
    dbWork->abort();
    delete dbWork;
    dbWork = nullptr;
  }
  
}
bool PostgreSqlConnector::makeTestTransaction()
{
  pqxx::result r;
  bool op = sendQuery("SELECT 1;",r);
  if( op == false)
  {
    CLOG_WARN("psql-connector : test transaction : failed!");
  }
  else
  {
    CLOG_DEBUG(5,"psql-connector : test transaction : ok");
  }
  return op;
}

bool PostgreSqlConnector::sendQuery(const std::string& q,pqxx::result& r)
{
  if(transactionmtx.try_lock() == false)
  {
    CLOG_WARN("psql-connector : send query : currently active transaction");
    return false;
  }
  if(connectionmtx.try_lock() == false)
  {
    CLOG_WARN("psql-connector : send query : connection management in progress");
    transactionmtx.unlock();
    return false;
  }
  if(!dbHandler)
  {
    CLOG_WARN("psql-connector : send query : no db handler");
    connectionmtx.unlock();
    transactionmtx.unlock();
    return false;
  }
  switch(state)
  {
    case State::CONNECTED:
    case State::DISCONNECTED:
      break;
    case State::CREATED:
    case State::ERROR:
    default:
      //not in a proper state to make a test
      CLOG_DEBUG(5,"psql-connector : make test transaction : not in a proper state : " 
        << state);
      transactionmtx.unlock();
      return false;
      break;
  }

  connectionmtx.unlock();
  try
  {
    cleanWork();
    CLOG_DEBUG(5,"psql-connector : make test transacion : launching");
    dbWork = new(std::nothrow) pqxx::work(*dbHandler);

    r = dbWork->exec(q);
    CLOG_DEBUG(5,"psql-connector : make test transacion : done ok : r size = " << r.size());

    state = State::CONNECTED;
    lastSuccessOperationTs = coyot3::tools::getCurrentTimestamp();
    CLOG_DEBUG(5,"psql-connector : make test transacion : done OK");
  }
  catch(pqxx::sql_error const &e)
  {
    CLOG_WARN("psql-connector : make test transaction : done : SQL EXCP : " << e.what());
    state = State::DISCONNECTED;
  }catch(std::exception const &e)
  {
    CLOG_WARN("psql-connector : make test transaction : done : STD EXCP : " << e.what());
    state = State::DISCONNECTED;
  }

  transactionmtx.unlock();
  return (state == State::CONNECTED);
}






std::string PostgreSqlConnector::getConnectionString()
  {
    std::stringstream s;
    debugCurrentConfig();
    // create connection string - begin
    s << "postgresql://";
    if(username.size() > 0)
    {

      s << username;
      if(password.size() != 0)
      {
        s << ":" << password;
      }
      s << "@";
    }
    s << hostAddr << ":" << hostPort;
    if(databaseName.size())
    {
      s << "/" << databaseName;
    }
    //params
    s << "?connect_timeout=" << connect_timeout;
    if(keepalives)
    {
      s << "&keepalives="          << (keepalives==true?1:0);
      s << "&keepalives_interval=" << keepalives_interval;
      s << "&keepalives_count="    << keepalives_count;
    }
    //CLOG_WARN("psql-connector : TO DELETE! : connection string : " << s.str());
    return s.str();
  }

void PostgreSqlConnector::debugCurrentConfig()
{
  CLOG_DEBUG(5,"psql-connector : current config : databaseName = " << databaseName);
  CLOG_DEBUG(5,"psql-connector : current config : username = " << username);
  CLOG_DEBUG(5,"psql-connector : current config : password = " << password);
  CLOG_DEBUG(5,"psql-connector : current config : hostAddr = " << hostAddr);
  CLOG_DEBUG(5,"psql-connector : current config : hostPort = " << hostPort);
  CLOG_DEBUG(5,"psql-connector : current config : connect_timeout; = " << connect_timeout;);
  CLOG_DEBUG(5,"psql-connector : current config : keepalives = " << keepalives);
  CLOG_DEBUG(5,"psql-connector : current config : keepalives_iddle = " << keepalives_iddle);
  CLOG_DEBUG(5,"psql-connector : current config : keepalives_interval = " << keepalives_interval);
  CLOG_DEBUG(5,"psql-connector : current config : keepalives_count = " << keepalives_count);
}






bool PostgreSqlConnector::setConfig(const Json::Value& config)
{
  std::lock_guard<std::mutex> mtxguard(connectionmtx);

  if( !config.isMember(_CYT_PSQLCTR_CFG_DATABASENAME_)
    ||!config.isMember(_CYT_PSQLCTR_CFG_USERNAME_) 
    ||!config.isMember(_CYT_PSQLCTR_CFG_PASSWORD_) 
    ||!config.isMember(_CYT_PSQLCTR_CFG_HOSTADDR_) 
    ||!config.isMember(_CYT_PSQLCTR_CFG_HOSTPORT_) 
  ){
    std::stringstream ss;
    ss << config;
    CLOG_ERROR("psql-connector : error : the configuratio structure has to have"
      " the following members");
    CLOG_ERROR("psql-connector :  - " _CYT_PSQLCTR_CFG_DATABASENAME_);
    CLOG_ERROR("psql-connector :  - " _CYT_PSQLCTR_CFG_USERNAME_);
    CLOG_ERROR("psql-connector :  - " _CYT_PSQLCTR_CFG_PASSWORD_);
    CLOG_ERROR("psql-connector :  - " _CYT_PSQLCTR_CFG_HOSTADDR_);
    CLOG_ERROR("psql-connector :  - " _CYT_PSQLCTR_CFG_HOSTPORT_);
    CLOG_ERROR("psql-connector :  - PLEASE - REVIEW THE STRUCTURE");
    CLOG_ERROR("psql-connector :  : source : " << ss.str());
    return false;
  }
  CLOG_INFO("psql-connector : setConfig : setting configuration");
  try{
    databaseName = config[_CYT_PSQLCTR_CFG_DATABASENAME_].asString();
    username     = config[_CYT_PSQLCTR_CFG_USERNAME_].asString();
    password     = config[_CYT_PSQLCTR_CFG_PASSWORD_].asString();
    hostAddr     = config[_CYT_PSQLCTR_CFG_HOSTADDR_].asString();
    hostPort     = config[_CYT_PSQLCTR_CFG_HOSTPORT_].asInt64();
  }catch(Json::Exception e){
    CLOG_ERROR("psql-connector : setConfig : error gathering data : " << e.what());
    return false;
  }
  
  if(config.isMember(_CYT_PSQLCTR_CFG_CONNECTTIMEOUT_))
  {
    
    connect_timeout = config[_CYT_PSQLCTR_CFG_CONNECTTIMEOUT_].asInt();
    CLOG_INFO("psql-connector : set config : connection timeout : " << connect_timeout);
  }
  if(config.isMember(_CYT_PSQLCTR_CFG_KEEPALIVE_))
  {
    keepalives = config[_CYT_PSQLCTR_CFG_KEEPALIVE_].asBool();
    CLOG_INFO("psql-connector : set config : keepalives: " << keepalives);
  }
  if(config.isMember(_CYT_PSQLCTR_CFG_KEEPALIVEIDDLE_))
  {
    keepalives_iddle = config[_CYT_PSQLCTR_CFG_KEEPALIVEIDDLE_].asInt();
    CLOG_INFO("psql-connector : set config : keeaplives iddle: " << keepalives_iddle);

  }
  if(config.isMember(_CYT_PSQLCTR_CFG_KEEPALIVEINTERVAL_))
  {
    CLOG_INFO("psql-connector : set config : keeaplives iddle parameter");
    keepalives_interval = config[_CYT_PSQLCTR_CFG_KEEPALIVEINTERVAL_].asInt();
    CLOG_INFO("psql-connector : set config : keeaplives interval: " << keepalives_interval);
  }
  if(config.isMember(_CYT_PSQLCTR_CFG_KEEPALIVECOUNT_))
  {
    CLOG_INFO("psql-connector : set config : keepalives count: " << keepalives_count);
    keepalives_count = config[_CYT_PSQLCTR_CFG_KEEPALIVECOUNT_].asInt();
  }
  return true;
}

void PostgreSqlConnector::callback_set_database_open(EventFunction cb){
  databaseIsOpen_ = cb; 
}
void PostgreSqlConnector::callback_set_database_closed(EventFunction cb){
  databaseHasClosed_ = cb;
}
void PostgreSqlConnector::callback_set_connector_error(EventFunction cb){
  databaseConnectorError_ = cb;
}


}//postgresql
}//end of namespace database
}//end of namespace coyot3
