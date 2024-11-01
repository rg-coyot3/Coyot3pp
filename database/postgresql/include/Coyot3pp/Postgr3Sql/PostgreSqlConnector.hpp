#pragma once



/**
 * json object config example
 
 {
   "database_name"  : "oneDb",
   "username"    : "user"
   "password"    : "pass"
   "host_address": "192.168.101.11"
   "host_port"   : 5678
 }

 * 
 * 
 * 
 * 
 * */
#include <unistd.h>
#include <string>
#include <mutex>
#include <functional>
#include <pqxx/pqxx>


#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include <Coyot3pp/Cor3/Coyot3.hpp>


#define _CYT_PSQLCTR_CFG_DATABASENAME_       "database_name"
#define _CYT_PSQLCTR_CFG_USERNAME_           "username"
#define _CYT_PSQLCTR_CFG_PASSWORD_           "password"
#define _CYT_PSQLCTR_CFG_HOSTADDR_           "host_address"
#define _CYT_PSQLCTR_CFG_HOSTPORT_           "host_port"
#define _CYT_PSQLCTR_CFG_CONNECTTIMEOUT_     "connect_timeout"

#define _CYT_PSQLCTR_CFG_KEEPALIVE_          "keepalives"
#define _CYT_PSQLCTR_CFG_KEEPALIVEIDDLE_     "keepalives_iddle"
#define _CYT_PSQLCTR_CFG_KEEPALIVEINTERVAL_  "keepalives_interval"
#define _CYT_PSQLCTR_CFG_KEEPALIVECOUNT_     "keepalives_count"



#define _CYT_PSQLCTR_CFG_DEFAULT_CONNECTTIMEOUT_       10
#define _CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVE_            true
#define _CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVEIDDLE_       3
#define _CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVEINTERVAL_    3
#define _CYT_PSQLCTR_CFG_DEFAULT_KEEPALIVECOUNT_       3

#define _CYT_PSQLCTR_CFG_DEFAULT_NOTRX_RECONNECT_      15000
#define _CYT_PSQLCTR_CFG_DEFAULT_NOTRX_NOTRXITIMEOUT_  6000
#define _CYT_PSQLCTR_CFG_DEFAULT_NOTRX_MAKETEXTTRX_    3000


/**
 * @brief class to be used as a base of an object to connect to postgresql 
 *    databases.
 * 
 * 
 * */
namespace coyot3{
namespace ddbb{
namespace postgresql{

CYT3MACRO_enum_class_declarations(
  ModuleState
  ,
    ,CREATED             = 1
    ,CONFIGURED          = 2
    ,LAUNCHING           = 3
    ,LAUNCHED            = 4
    ,STOPPED             = 5
    ,SHUTDOWN            = 6
    ,DISCONNECTED        = 7
    ,CONNECTED           = 8
    ,ERROR               = 0
)
  
class PostgreSqlConnector {


  public :
    typedef ec::ModuleState State;
    typedef std::function<void()>      EventFunction;
    
    PostgreSqlConnector();

    virtual ~PostgreSqlConnector();

    /** 
     * @brief : reserves memory for the control threads
     * */
    virtual bool Init();
    /**
     * @brief : starts the connection and the link control
     * */
    virtual bool Start();
    /**
     * @brief : stops the connection and the link control
     * */
    virtual bool Stop();

    /**
     * @brief : frees all resources and prepares the instance 
     *          to be initializated and started
     * 
     * @return true 
     * @return false 
     */
    virtual bool Shutdown();

    
    State getState();
    /**
     * @param q query string
     * @param r result . a pqxx::result object.
     * @return true if the transaction was correctly done, else in other case.
     * */
    bool sendQuery(const std::string& q,pqxx::result& r);

    /** 
     * @brief sets the configuration using a json::value object
     *   example  {
          "database_name"  : "oneDb",
          "username"    : "user"
          "password"    : "pass"
          "host_address": "192.168.101.11"
          "host_port"   : 5678,

          "connect_timeout\"     : 3, << not required
          "keepalives\"          : true, << not required
          "keepalives_iddle\"    : 1, << not required
          "keepalives_interval\" : 1, << not required
          "keepalives_count\"    : 3  << not required
        }
      */
    bool setConfig(const Json::Value& config);

    void callback_set_database_open(EventFunction cb);
    void callback_set_database_closed(EventFunction cb);
    void callback_set_connector_error(EventFunction cb);

    
  private: 
    Json::Value _config_src;


  protected:

    std::string databaseName;
    std::string username;
    std::string password;
    std::string hostAddr;
    int32_t     hostPort;

    int         connect_timeout; //seconds
    bool        keepalives;
    int         keepalives_iddle;
    int         keepalives_interval;
    int         keepalives_count;

    pqxx::connection* dbHandler;
    pqxx::work*       dbWork;
    

    State state;


    std::mutex  connectionmtx;
    std::mutex  transactionmtx;
    std::mutex  cleanworkmtx;
      void cleanWork();

    coyot3::tools::ControlThread* controlThreadPri;
    coyot3::tools::ControlThread* controlThreadSec;
    volatile bool                   controlThFlagPri;
    volatile bool                   controlThFlagSec;
    void                            controlLink();        //validates that there have been transactions
    void                            controlTransaction(); //make a transaction if no transaction has been done in a while
    bool                            restartConnection;

    int64_t     lastSuccessOperationTs;
    

    bool openDatabase();
    bool closeDatabase();


    void debugCurrentConfig();
    bool makeTestTransaction();
    std::string getConnectionString();


    EventFunction databaseIsOpen_;
    EventFunction databaseHasClosed_;
    EventFunction databaseConnectorError_;


};

/* NOTES
connection string : https://www.postgresql.org/docs/current/libpq-connect.html#LIBPQ-CONNSTRING
params: 
  - host
  - hostaddr
  - port
  - dbname (default same as username)
  - user
  - password
  - passfile (defaults to ~/.bgpass or %APPDATA%\postgresql\pgpass.conf )
  - connect_timeout (minimum = 2 seconds (for each host))
  - options
  - application_name (the postgresql server will log this)
  - fallback_application_name
  - keepalives (TCP keepalives. ignored for connections made via unix-domain socket)
  - keepalives_interval (number of seconds between keepalive - system with TCP_KEEPCNT & win, else no effect)
  - keepalives_count (num keepalives lost before consider connection is dead)
  - sslmode
    * disable (try non-ssl)
    * allow (first, try non-ssl, after... it tries ssl)
    * prefer (firs try ssl, after: non-ssl)
    * require (only ssl)
    * verify-ca (only ssl and verify server with a trusted CA [system - inet])
    * verify-full (only ssl, trusted CA and checks CA with host's)
  - requiressl (deprecated in favor of sslmode)
  - sslcompression : 0 | 1 (requires openssl 1.0.0 or later)
  - sslcert : path string (default = ~/.postgresql/postgresql.crt)
  - sslkey : path string (default = ~/.postgresql/postgresql.key)
  - sslrootcert : path string (default = ~/.postgresql/root.crt)
  - sslcrl : path string (default = ~/.postgresql/root.crl)
  - requirepeer : name string : checks the server name, if doesn't match, closes connection
  - krbsrvname : kerberos service name to use when authenticating with GSSAPI. https://www.postgresql.org/docs/10/auth-methods.html#GSSAPI-AUTH
  - gsslib
  - service
  - target_session_attrs

*/
}
}
}