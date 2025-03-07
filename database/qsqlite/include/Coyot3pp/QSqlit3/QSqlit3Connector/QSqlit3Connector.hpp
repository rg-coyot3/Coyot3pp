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
  QSqlit3ConnectorConfigObject
  , 
  , ( )
  , ( )
    , db_location         , std::string           , ""
    , connection_name     , std::string           , ""
    , check_state_interval, int64_t               , 60000
)

  CYT3MACRO_model_class_serializable_json_declarations(
    QSqlit3ConnectorConfigObject
    , 
    , 
    , ( )
    , ( )
      , db_location          , "db_location"          ,
      , connection_name      , "connection_name"      ,
      , check_state_interval , "check_state_interval" ,
  )

class QSqlit3Connector : public QObject{
  Q_OBJECT
  public:
    struct Config{
      static constexpr const char* db_location = "db_location";
      static constexpr const char* connection_name = "connection_name";
      static constexpr const char* check_state_interval = "check_state_interval";

    };
    typedef coyot3::tools::ModuleState State;
  
    QSqlit3Connector(QObject* parent = nullptr);

    virtual ~QSqlit3Connector();

    bool setConfiguration(const Json::Value& cfg);
    bool setConfiguration(const QSqlit3ConnectorConfigObject& obj);

    virtual bool Init();
    virtual bool Start();
    virtual bool Stop();
    virtual bool Shutdown();


    bool          master_attach(const QSqlit3Connector& master);
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
      QSqlit3Connector* external_instance_;
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
        bool makeQuery(const QString& queryString, std::string& err);
        bool makeQuery(const QString& queryString);
        bool makeLinkControlQuery();

    //bool check_configuration_structure();

        bool slave_attach_(QSqlit3Connector* c);
        bool slave_detach_(QSqlit3Connector* c);
          std::map<QSqlit3Connector*, const QSqlit3Connector*> slaves_;

    QTimer*       timerStatus;
      int           timer_status_interval;

    //Json::Value config;
    QSqlit3ConnectorConfigObject config_;

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
