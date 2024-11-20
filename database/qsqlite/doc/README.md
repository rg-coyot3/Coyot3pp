# QSqlit3


QSqlite connector


## INTRODUCTION and blahblah

Sqlite is a good solution for minimal systems with complex enough datasets, when external robust database systems (such PostgreSQL) are not available. Sqlite is a minimal file-system-based database that permit to the developper to see its project grow, and imagine a future migration to a more complex and robust system and conserve all the data generated along the lifecycle of the project.

My main objective with this connector and the macros was to be able to work directly with the data-models as if they were DAO. While I was making projects, I realized that the *core* of any project would not have any interest in the database itself, BUT to *just simply* know if the access to the database is *up* or *down*... and nothing else. And I also wanted to use the adventages of using the `model-class` preprocessor macros that I defined at the `Cor3`. 

I *realized* (*I see in my mind*) that, when dealing with databases, there are mainly 3 types of applications: feeders and consumers, and finally the complex database dealers. For c++ IoT applications, I am mainly working with feeders where every object produce data that needs to be traced.

 For the projects I made I identified a stage of maturity where I really had nothing but some little real needs:

* read the current stored data at the database. Mainly to read configuration data.
* insert new data at the tables. Mainly logs and traces generated in real time.
* no real need to *UPDATE* data at the database.

This connector is not intended to be used by *projects* or software pieces that need to use enhanced SQL operations, such as the selection of a range of data, or updating operations. If it is needed to operate directly with the content of the database (such as making reports), another software pieces should be used.

This connector and the preprocessor macros give the possibility to push data to a stack, and let the connector to automatically push it to a database. The "push" operation is not done in real time, but every "x" seconds by a parallel thread, so the main *user's* thread is not interrupted.

This connector and the preprocessor macros give also the possibility to read data from the tables, so it is conceiveble to imagine to use `.sqlite` files to seed the configuration (even thou for early states of a project I prefer JSON), that may be "easily" updated by the user of the final application using any kind of GUI.

### what these wrappers are not prepared to do

These wrappers do not contain methods to `ALTER TABLE`s or `UPDATE` data. They are created to make basic `INSERT` operations and `SELECT` items from a table. If *you* need to *alter* one table, and conserv the previous data, it will have to be done manually.

> If I keep using them, I may include some methods to do so... but for the moment, I did not...

## HOW TO USE IT

The goal is to be able to use the `CYT3MACRO_model_class` benefits, and upgrade it to be able to create a relation between a `_set_stack` and a table from a database.

The starting point is one *model class*, lets say `ExampleModelClass` declared with `CYT3MACRO_model_class` macros, containing a set of properties. **It will also be needed to invoke the macros `CYT3MACRO_model_class_set_stack` macros.**


The `CYT3MACRO_model_class-serializable_qsqlite`, macros will describe an object of type `ExampleModelClassQSqliteIO` that will be linked to a table into a database. The database may be directly managed by the `ExampleModelClassQSqliteIO` object, or could be related to a database managed by another `QSqlit3Connector`.

* offers a `stack` property where the user will push the data to be added to the table. The`...QSqliteIO` object will push all the data of the `stack` to the table every time `insert_table_items` is invoked. If the stack is declared as *volatile* with `volatile_stack(true)` (witch is the value by default), the stack will be cleaned.
* offers a `query_stack` property where the user will gather the data done after querying the table. The standard `SELECT *` query will be done by invoking the method `select_table_items`


### DECLARATION AND DEFINITION

As the other `CYT3MACRO`s, the proposed syntax is matrix-like:

* BASE:

  Starting from this point:
  
  ```cpp
    CYT3MACRO_model_class_<declarations & definitions>(
      ExampleModelClass
      , 
      , ( )
      , ( )
        , id                  , int                          , 0
        , property1           , double                       , 0.0
        , property2           , std::string                  , 0.0
        , property3           , bool                         , true
    )
  
    CYT3MACRO_model_class_set_stack_<declarations & definitions>(ExampleModelClass,)
  
  ```

It will be possible to invoke the `serializable_qsqlite_` macros:

```cpp
CYT3MACRO_model_class_serializable_qsqlite_<declarations & definitions>(
  ExampleModelClass
  , ( )
  , id             , "INTEGER PRIMARY KEY AUTOINCREMENT"      , "id"
  , property1      , "NUMERIC"                                , "prop1_title"
  , property2      , "TEXT"                                   , "prop2_title"
  , property3      , "INTEGER"                                , "prop3_title"
)
```

**IMPORTANT:**

It is important to know that the resulting class will recognize the `AUTOINCREMENT` SQL keyword, so for the insertions, the concerned property will be ignored.

#### format:

  ```
  CYT3MACRO_model_class_serializable_qsqlite_<declarations & definitions>(
    name of the base class
    , ( options )
    
    , property of the base class, "sqlite related type"   , "table field name"
    ,     ...                   ,     ...                 , ...
  )
  ```


Once declared and defined, the user will be able to invoke it from a thread:

```cpp
  QCoreApplication app(argc, argv);
  //
  ExampleModelClassQSqliteIO exampleio(&app);

  exampleio.set_table_name("name_of_the_table");

```

* **using 1 unique database with a DAO**

  The database target has to be configured at the same `SqliteIO` instance:
  
  ```cpp
    exampleio.database_name("path_to_the_database.sqlite");
  ```
  
  In this case, it is the `exampleio` instance that needs to be *Init*ialized and *Start*ed.
  
  ```cpp
    exampleio.Init();
    exampleio.Start();
  ```
  From this moment, it will be possible to exchange with the database and the table.


* **using multiple DAOs with a database**

  Theres a previous instantiation of a `QSqlit3Connector`. We attach it using the `master_attach` method.
  
  ```cpp
    QSqlit3Connector masterDatabase(&app);
    masterDatabase.database_name("path_to_the_database.sqlite");
  
    exampleio.master_attach(&masterDatabase);
  ```
  In this case, it is the master that needs to be *Init*ialized and *Start*ed.
  
  
  ```cpp
    masterDatabase.Init();
    masterDatabase.Start();
  ```
  
  From this moment, all the attached `SqliteIO` instances will be able to exchange with their related tables.



* Creating items and insert them to the table.

  ```cpp
  int numItems = 100;
  while((numItems--)>0){
    ExampleModelClass item;
    item.property1(coyot3::tools::generate_real_number(-100.0,100.0));
    item.property2(coyot3::tools::generate_string_alphanumeric(10));
    item.property3(coyot3::tools::generate_natural_number(-50,50));
    exampleio.stack.push(item);
  }
  std::string err;
  if(exampleio.insert_table_items(err)== false){
    CLOG_ERROR(" error inserting items: " << err)
  }
  ```

* Recovering all the items of a table:
  
  ```cpp
  std::string errselect;
  if(!exampleio.select_table_items(errselect) == false){
    CLOG_ERROR(" error obtaining all the data from the table" << errselect)
  }else{
    CLOG_INFO(" : obtained data from table : " << exampleio.table_name())
    exampleio.query_stack.for_each([&](const ExampleModelClass& item){
      CLOG_INFO("      id=" << item.id() << ",p1=" << item.property1() 
        << ",p2=" << item.property2() << ",p3=" << item.property3())
      return true;
    });
  }
  ```
  It is also possible to include a sql query string at the `select_table_items` method:

  ```cpp
  exampleio.select_table_items(errselect,"prop1_title > 0.0")
  ```


You can check the minimal example for the `QSqlit3` component.







