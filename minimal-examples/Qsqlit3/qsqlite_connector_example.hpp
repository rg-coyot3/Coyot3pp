#include <Coyot3pp/QSqlit3/QSqlit3Connector/QSqlit3Connector.hpp>



  CYT3MACRO_model_class_declarations(
    DatabaseEntityDAO
    , 
    , ( )
    , ( )
      , id                  , int64_t           , 0
      , name                , std::string       , ""
      , description         , std::string       , ""
      , metadata            , std::string       , ""
  )

    CYT3MACRO_model_class_serializable_json_declarations(
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

  CYT3MACRO_model_class_declarations(
    ServiceStopDAO
    , DatabaseEntityDAO
    , ( std::string to_string())
    , ( )
      , active              , bool                         , false
      , latitude            , double                       , 0.0
      , longitude           , double                       , 0.0
      , altitude            , double                       , 0.0
  )
    CYT3MACRO_model_class_set_stack_declarations(ServiceStopDAO,)

    CYT3MACRO_model_class_serializable_json_declarations(
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
    CYT3MACRO_model_class_set_stack_serializable_json_declarations(ServiceStopDAO)


  CYT3MACRO_model_class_serializable_qsqlite_declarations(
      ServiceStopDAO
    , ( )
    , id                  , "id"            , "INTEGER PRIMARY KEY AUTOINCREMENT"   

    , name                , "name"          , "TEXT"                                
    , description         , "description"   , "TEXT"                                
    , active              , "active"        , "INTEGER"                             
    , latitude            , "latitude"      , "NUMERIC"                             
    , longitude           , "longitude"     , "NUMERIC"                             
    , altitude            , "altitude"      , "NUMERIC"                             
  )




  class SqliteDBStopsManager
  : public QObject{
    Q_OBJECT

    public:
      SqliteDBStopsManager(QObject* parent = nullptr);
      virtual ~SqliteDBStopsManager();

      const ServiceStopDAOStack&  stop_data() const;

      bool pushStop(const ServiceStopDAO& stop);

      bool Start();
      bool Stop();

    signals:
      void  data_model_changed();
      void  database_io_errors();

    protected:
      // master db connector
      coyot3::ddbb::sqlite::QSqlit3Connector*  db_connector_;
      QTimer*                                 manager_timer_;
      std::mutex                              mtx_connector_;

      // DAO-IO. Can work as standalone, but will be attached to the master connector.
      ServiceStopDAOQSqliteIO*                serviceStopsIO;
        ServiceStopDAOStack                     serviceStopsCurrent;
        bool                                    importTableServiceStops(bool genSignal = true);
        bool                                    serviceStopsChanged;
  

      bool                                    initialize_read();
      std::size_t                             ts_;
    protected slots:
      void timer_check_status_callback();
      void update_random_line();
      void delete_random_line();



  };
