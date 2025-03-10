class ServiceStopDAOQSqliteIO : public coyot3::ddbb::sqlite::QSqlit3Connector
{
public:
  enum class DataType
  {
    INTEGER,
    NUMERIC,
    TEXT,
    BLOB,
    NULL_T
  };
  struct ColType
  {
    static constexpr const char *id = "INTEGER PRIMARY KEY AUTOINCREMENT";
    static constexpr const char *name = "TEXT";
    static constexpr const char *description = "TEXT";
    static constexpr const char *active = "INTEGER";
    static constexpr const char *latitude = "NUMERIC";
    static constexpr const char *longitude = "NUMERIC";
    static constexpr const char *altitude = "NUMERIC";
  };
  struct ColsTypeE
  {
    DataType id;
    DataType name;
    DataType description;
    DataType active;
    DataType latitude;
    DataType longitude;
    DataType altitude;
  };
  struct ColName
  {
    static constexpr const char *id = "id";
    static constexpr const char *name = "name";
    static constexpr const char *description = "description";
    static constexpr const char *active = "active";
    static constexpr const char *latitude = "latitude";
    static constexpr const char *longitude = "longitude";
    static constexpr const char *altitude = "altitude";
  };
  struct ColsFlags
  {
    bool id = true;
    bool name = true;
    bool description = true;
    bool active = true;
    bool latitude = true;
    bool longitude = true;
    bool altitude = true;
  };
  ServiceStopDAOQSqliteIO(QObject *parent = nullptr);
  virtual ~ServiceStopDAOQSqliteIO();
  ServiceStopDAOStack stack;
  ServiceStopDAOStack query_stack;
  bool select_table_items(std::string &err, const std::string &where = std::string());
  bool insert_table_items(std::string &err);
  bool insertion_field_activation(const std::string &colName, bool activation);
  std::string table_name(const std::string &name);
  std::string table_name() const;
  bool check_create_table(std::string &err);
  bool volatile_stack() const;
  bool volatile_stack(bool v);

protected:
  std::string tablename_;
  ServiceStopDAOStack stack_exch_;
  std::mutex mtx_obj_;
  ColsFlags cols_w_act_;
  ColsFlags cols_w_str_;
  ColsTypeE cols_ttr_;
  bool cols_params_calculated_;
  bool volatile_stack_;
  void conf_insertion_types_();
  bool conf_datatype_matx_();
  std::string query_iq_prefix_;
  void calc_iq_string_prefix_();
  std::string query_sq_prefix_;
  void calc_sq_string_prefix_();
  std::string query_cr_table_;
  void calc_cr_query();

public:
  static std::string query_create_table(const std::string &tableName);
  static std::string query_alter_table(const std::string &tableName);
  static constexpr const char *INTEGER = "INTEGER";
  static constexpr const char *NUMERIC = "NUMERIC";
  static constexpr const char *TEXT = "TEXT";
  static constexpr const char *BLOB = "BLOB";
  static constexpr const char *NULL_T = "NULL";
  static constexpr const char *NOT_NULL = "NOT NULL";
  static constexpr const char *AUTOINCREMENT = "AUTOINCREMENT";
  static bool extract(const QVariant &source, bool &destination);
  static bool extract(const QVariant &source, int &destination);
  static bool extract(const QVariant &source, uint &destination);
  static bool extract(const QVariant &source, int64_t &destination);
  static bool extract(const QVariant &source, uint64_t &destination);
  static bool extract(const QVariant &source, double &destination);
  static bool extract(const QVariant &source, std::string &destination);
};

ServiceStopDAOQSqliteIO::ServiceStopDAOQSqliteIO(QObject *parent) : QSqlit3Connector(parent), stack(), query_stack(), tablename_(), stack_exch_(), cols_w_act_(), cols_w_str_(), cols_params_calculated_(), volatile_stack_(true), query_iq_prefix_(), query_sq_prefix_(), query_cr_table_()
{
  conf_datatype_matx_();
  calc_iq_string_prefix_();
  conf_insertion_types_();
}
ServiceStopDAOQSqliteIO::~ServiceStopDAOQSqliteIO() {}
void ServiceStopDAOQSqliteIO::conf_insertion_types_()
{
  (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find("TEXT") != std::string::npos ? cols_w_str_.id = true : cols_w_str_.id = false);
  (std::string("TEXT").find("TEXT") != std::string::npos ? cols_w_str_.name = true : cols_w_str_.name = false);
  (std::string("TEXT").find("TEXT") != std::string::npos ? cols_w_str_.description = true : cols_w_str_.description = false);
  (std::string("INTEGER").find("TEXT") != std::string::npos ? cols_w_str_.active = true : cols_w_str_.active = false);
  (std::string("NUMERIC").find("TEXT") != std::string::npos ? cols_w_str_.latitude = true : cols_w_str_.latitude = false);
  (std::string("NUMERIC").find("TEXT") != std::string::npos ? cols_w_str_.longitude = true : cols_w_str_.longitude = false);
  (std::string("NUMERIC").find("TEXT") != std::string::npos ? cols_w_str_.altitude = true : cols_w_str_.altitude = false);
  cols_params_calculated_ = true;
}
void ServiceStopDAOQSqliteIO::calc_iq_string_prefix_()
{
  std::stringstream sstr;
  sstr << "INSERT INTO '" << tablename_ << "' (";
  bool firstItem = true;
  if (cols_w_act_.id == true)
  {
    firstItem = false;
    sstr << "id";
  }
  if (cols_w_act_.name == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "name";
  }
  if (cols_w_act_.description == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "description";
  }
  if (cols_w_act_.active == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "active";
  }
  if (cols_w_act_.latitude == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "latitude";
  }
  if (cols_w_act_.longitude == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "longitude";
  }
  if (cols_w_act_.altitude == true)
  {
    if (firstItem == false)
      sstr << ",";
    else
      firstItem = false;
    sstr << " "
            "altitude";
  }
  sstr << ") VALUES ";
  query_iq_prefix_ = sstr.str();
}
void ServiceStopDAOQSqliteIO::calc_sq_string_prefix_()
{
  std::stringstream sstr;
  sstr << "SELECT ";
  sstr << "id";
  sstr << ", "
          "name";
  sstr << ", "
          "description";
  sstr << ", "
          "active";
  sstr << ", "
          "latitude";
  sstr << ", "
          "longitude";
  sstr << ", "
          "altitude";
  sstr << " FROM " << tablename_;
  query_sq_prefix_ = sstr.str();
}
bool ServiceStopDAOQSqliteIO::insertion_field_activation(const std::string &colName, bool activation)
{
  bool found = false;
  if (colName.compare(ColName::id) == 0)
    found = (true | (cols_w_act_.id = activation));
  if (colName.compare(ColName::name) == 0)
    found = (true | (cols_w_act_.name = activation));
  if (colName.compare(ColName::description) == 0)
    found = (true | (cols_w_act_.description = activation));
  if (colName.compare(ColName::active) == 0)
    found = (true | (cols_w_act_.active = activation));
  if (colName.compare(ColName::latitude) == 0)
    found = (true | (cols_w_act_.latitude = activation));
  if (colName.compare(ColName::longitude) == 0)
    found = (true | (cols_w_act_.longitude = activation));
  if (colName.compare(ColName::altitude) == 0)
    found = (true | (cols_w_act_.altitude = activation));
  calc_iq_string_prefix_();
  return found;
}
std::string ServiceStopDAOQSqliteIO::table_name(const std::string &name)
{
  tablename_ = name;
  calc_iq_string_prefix_();
  calc_sq_string_prefix_();
  return tablename_;
}
std::string ServiceStopDAOQSqliteIO::table_name() const { return tablename_; }
bool ServiceStopDAOQSqliteIO::volatile_stack() const { return volatile_stack_; }
bool ServiceStopDAOQSqliteIO::volatile_stack(bool v) { return (volatile_stack_ = v); }
bool ServiceStopDAOQSqliteIO::conf_datatype_matx_()
{
  bool done_ok = true;
  {
    bool lf_ = false;
    if (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.id = DataType::INTEGER));
    if (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.id = DataType::TEXT));
    if (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.id = DataType::BLOB));
    if (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.id = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("INTEGER PRIMARY KEY AUTOINCREMENT").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.id = false;
  }
  {
    bool lf_ = false;
    if (std::string("TEXT").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.name = DataType::INTEGER));
    if (std::string("TEXT").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.name = DataType::TEXT));
    if (std::string("TEXT").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.name = DataType::BLOB));
    if (std::string("TEXT").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.name = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("TEXT").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.name = false;
  }
  {
    bool lf_ = false;
    if (std::string("TEXT").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.description = DataType::INTEGER));
    if (std::string("TEXT").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.description = DataType::TEXT));
    if (std::string("TEXT").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.description = DataType::BLOB));
    if (std::string("TEXT").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.description = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("TEXT").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.description = false;
  }
  {
    bool lf_ = false;
    if (std::string("INTEGER").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.active = DataType::INTEGER));
    if (std::string("INTEGER").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.active = DataType::TEXT));
    if (std::string("INTEGER").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.active = DataType::BLOB));
    if (std::string("INTEGER").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.active = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("INTEGER").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.active = false;
  }
  {
    bool lf_ = false;
    if (std::string("NUMERIC").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.latitude = DataType::INTEGER));
    if (std::string("NUMERIC").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.latitude = DataType::TEXT));
    if (std::string("NUMERIC").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.latitude = DataType::BLOB));
    if (std::string("NUMERIC").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.latitude = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("NUMERIC").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.latitude = false;
  }
  {
    bool lf_ = false;
    if (std::string("NUMERIC").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.longitude = DataType::INTEGER));
    if (std::string("NUMERIC").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.longitude = DataType::TEXT));
    if (std::string("NUMERIC").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.longitude = DataType::BLOB));
    if (std::string("NUMERIC").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.longitude = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("NUMERIC").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.longitude = false;
  }
  {
    bool lf_ = false;
    if (std::string("NUMERIC").find(INTEGER) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.altitude = DataType::INTEGER));
    if (std::string("NUMERIC").find(TEXT) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.altitude = DataType::TEXT));
    if (std::string("NUMERIC").find(BLOB) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.altitude = DataType::BLOB));
    if (std::string("NUMERIC").find(NUMERIC) != std::string::npos)
      lf_ = (true | static_cast<int>(cols_ttr_.altitude = DataType::NUMERIC));
    done_ok &= lf_;
    if (std::string("NUMERIC").find(AUTOINCREMENT) != std::string::npos)
      cols_w_act_.altitude = false;
  }
  return done_ok;
}
bool ServiceStopDAOQSqliteIO::insert_table_items(std::string &err)
{
  std::stringstream sstr;
  sstr << query_iq_prefix_;
  int numItem = 0;
  if (stack.size() == 0)
    return true;
  std::lock_guard guard(mtx_obj_);
  stack.for_each([&](const ServiceStopDAO &item)
                 { if(numItem != 0)sstr << ", "; else sstr << " "; sstr << "( "; bool firstItem=true; if(cols_w_act_.id == true){ if(cols_w_str_.id == true) sstr << "'"; sstr << item.id(); if(cols_w_str_.id == true) sstr << "'"; firstItem=false; } if(cols_w_act_.name == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.name == true) sstr << "'"; sstr << item.name(); if(cols_w_str_.name == true) sstr << "'"; } if(cols_w_act_.description == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.description == true) sstr << "'"; sstr << item.description(); if(cols_w_str_.description == true) sstr << "'"; } if(cols_w_act_.active == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.active == true) sstr << "'"; sstr << item.active(); if(cols_w_str_.active == true) sstr << "'"; } if(cols_w_act_.latitude == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.latitude == true) sstr << "'"; sstr << item.latitude(); if(cols_w_str_.latitude == true) sstr << "'"; } if(cols_w_act_.longitude == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.longitude == true) sstr << "'"; sstr << item.longitude(); if(cols_w_str_.longitude == true) sstr << "'"; } if(cols_w_act_.altitude == true){ if(firstItem==false)sstr << ", ";else firstItem=false; if(cols_w_str_.altitude == true) sstr << "'"; sstr << item.altitude(); if(cols_w_str_.altitude == true) sstr << "'"; } sstr << ")"; numItem++; return true; });
  sstr << ";";
  QString queryString = QString::fromStdString(sstr.str());
  bool result = makeInsertionQuery(queryString);
  if (result == false)
  {
    {
      ::coyot3::logger::SimpleLoggerClass slc(::coyot3::logger::SimpleLoggerClass::LogLineType::WARNING, coyot3::logger::Terminal::module_terminal_index);
      slc << "cyt3macro-qsqlite-io : "
             "ServiceStopDAO"
             " : error : query["
          << sstr.str() << "]";
    }
    return false;
  }
  if (volatile_stack_ == true)
    stack.clear();
  return result;
}
bool ServiceStopDAOQSqliteIO::select_table_items(std::string &err, const std::string &where)
{
  std::stringstream sstr;
  std::lock_guard<std::mutex> guard(mtx_obj_);
  sstr << query_sq_prefix_;
  if (where.size() != 0)
  {
    sstr << " WHERE " << where;
  }
  sstr << ";";
  QSqlQuery q;
  bool result;
  result = makeSelectQuery(QString::fromStdString(sstr.str()), q, err);
  if (result == false)
    return false;
  int index_id = q.record().indexOf("id");
  if (index_id == -1)
  {
    err += "COLUMN '"
           "id"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_name = q.record().indexOf("name");
  if (index_name == -1)
  {
    err += "COLUMN '"
           "name"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_description = q.record().indexOf("description");
  if (index_description == -1)
  {
    err += "COLUMN '"
           "description"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_active = q.record().indexOf("active");
  if (index_active == -1)
  {
    err += "COLUMN '"
           "active"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_latitude = q.record().indexOf("latitude");
  if (index_latitude == -1)
  {
    err += "COLUMN '"
           "latitude"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_longitude = q.record().indexOf("longitude");
  if (index_longitude == -1)
  {
    err += "COLUMN '"
           "longitude"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  int index_altitude = q.record().indexOf("altitude");
  if (index_altitude == -1)
  {
    err += "COLUMN '"
           "altitude"
           "' DOES NOT EXIST IN TABLE;";
    result = false;
  }
  if (result == false)
    return false;
  query_stack.clear();
  while (q.next())
  {
    ServiceStopDAO buffer;
    result &= extract(q.value(index_id), buffer.id());
    result &= extract(q.value(index_name), buffer.name());
    result &= extract(q.value(index_description), buffer.description());
    result &= extract(q.value(index_active), buffer.active());
    result &= extract(q.value(index_latitude), buffer.latitude());
    result &= extract(q.value(index_longitude), buffer.longitude());
    result &= extract(q.value(index_altitude), buffer.altitude());
    query_stack.push_back(buffer);
  }
  return result;
}
bool ServiceStopDAOQSqliteIO::check_create_table(std::string &err)
{
  std::stringstream sstr;
  sstr << "CREATE TABLE IF NOT EXISTS `" << tablename_ << "` (";
  sstr << " `"
          "id"
          "` "
          "INTEGER PRIMARY KEY AUTOINCREMENT";
  sstr << ", `"
          "name"
          "` "
          "TEXT";
  sstr << ", `"
          "description"
          "` "
          "TEXT";
  sstr << ", `"
          "active"
          "` "
          "INTEGER";
  sstr << ", `"
          "latitude"
          "` "
          "NUMERIC";
  sstr << ", `"
          "longitude"
          "` "
          "NUMERIC";
  sstr << ", `"
          "altitude"
          "` "
          "NUMERIC";
  sstr << ");";
  bool result = makeCreateTableQuery(QString::fromStdString(sstr.str()), err);
  return result;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, bool &destination)
{
  destination = source.toBool();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, int &destination)
{
  destination = source.toInt();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, uint &destination)
{
  destination = source.toUInt();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, int64_t &destination)
{
  destination = source.toLongLong();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, uint64_t &destination)
{
  destination = source.toULongLong();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, double &destination)
{
  destination = source.toDouble();
  return true;
}
bool ServiceStopDAOQSqliteIO::extract(const QVariant &source, std::string &destination)
{
  destination = source.toString().toStdString();
  return true;
}
