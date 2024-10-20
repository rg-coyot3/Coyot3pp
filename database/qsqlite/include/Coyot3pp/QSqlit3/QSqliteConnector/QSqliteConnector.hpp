#pragma once


#include <Coyot3pp/Cor3/Coyot3.hpp>

#include "cyt_macro_model_class_serializable_qsqlite.hpp"

#include <QObject>
#include <QtSql/QSql>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlDriver>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QtSql/QSqlError>
#include <QString>
#include <QTimer>
#include <QVariant>
#include <mutex>
#include <jsoncpp/json/json.h>


/**
 * 
 * 
 * 
 * 
 * 
 * */





namespace coyot3{
namespace ddbb{
namespace sqlite{

CYT3MACRO_model_class_declarations(
  SqliteConnectorConfigObject
  , 
  , ( )
  , ( )
    , db_location         , std::string           , ""
    , connection_name     , std::string           , ""
    , check_state_interval, int64_t               , 60000
)

  CYT3MACRO_model_class_serializable_json_declarations(
    SqliteConnectorConfigObject
    , 
    , 
    , ( )
    , ( )
      , db_location          , "db_location"          ,
      , connection_name      , "connection_name"      ,
      , check_state_interval , "check_state_interval" ,
  )

class SqliteConnector : public QObject{
  Q_OBJECT
  public:
    struct Config{
      static constexpr const char* db_location = "db_location";
      static constexpr const char* connection_name = "connection_name";
      static constexpr const char* check_state_interval = "check_state_interval";

    };
    typedef coyot3::tools::ModuleState State;
  
    SqliteConnector(QObject* parent = nullptr);

    virtual ~SqliteConnector();

    bool setConfiguration(const Json::Value& cfg);
    bool setConfiguration(const SqliteConnectorConfigObject& obj);

    virtual bool Init();
    virtual bool Start();
    virtual bool Stop();
    virtual bool Shutdown();


    bool          master_attach(const SqliteConnector& master);
    bool          master_detach();

    bool          database_owned();

    std::string   database_name() const;
    std::string   database_name(const std::string& n);
    std::string   connection_name() const;
    std::string   connection_name(const std::string& n);
    int64_t       keepalive_interval() const;
    int64_t       keepalive_interval(int64_t i);
  signals:


  public slots:

  protected:
    State state;

    bool           owns_db_;
      SqliteConnector* external_instance_;
    QSqlDatabase*  database;
      std::string    dbname;
      std::string    connection_name_;
      std::mutex     dbmutex;

        bool openDatabase();
        bool closeDatabase();

        bool makeInsertionQuery(const QString& queryString);
        bool makeInsertionQuery(const QString& queryString, std::string& err);
        bool makeCreateTableQuery(const QString& queryString);
        bool makeCreateTableQuery(const QString& queryString, std::string& err);
        bool makeSelectQuery(const QString& queryString,QSqlQuery& q, std::string& err);
        bool makeSelectQuery(const QString& queryString,QSqlQuery& q);
        bool makeLinkControlQuery();

    //bool check_configuration_structure();

        bool slave_attach_(SqliteConnector* c);
        bool slave_detach_(SqliteConnector* c);
          std::map<SqliteConnector*, const SqliteConnector*> slaves_;

    QTimer*       timerStatus;
      int           timer_status_interval;

    //Json::Value config;
    SqliteConnectorConfigObject config_;

    std::string lastErrorString;


    void set_debug_level_(int level);
  protected slots:

    void evaluateState();



  private:


  private slots:


};



}
}
}
