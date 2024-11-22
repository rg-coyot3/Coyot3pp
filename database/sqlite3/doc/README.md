# Sqlit3

## INTRODUCTION

The goal of this module is to create a simple wrapper with the `sqlite3` lib with a simple class to make basic insertions and querys.


## `Sqlit3Connector`

Is a basic *module* that will open a database pointed by the user (`set_database_name(string)`).

The application will be able to `make_query`s to the sqlite database. If the application needs a result, the solution will be given in the form of an `sqlite3_stmt*`. It will be also possible for the application to gather the query error throw a string:

* make query without result (like a `select`, `create table`, `insert into...`):
  * `bool make_query(const std::string& q);`
  * `bool make_query(const std::string& q, std::string& err);`
* make query with a result (like `select`):
  * `bool make_query(const std::string& q, sqlite3_stmt* &r);`
  * `bool make_query(const std::string& q, sqlite3_stmt* &r, std::string& err);`
  

The class offers a `master_attach(Sqlit3Connector* connector)` method that permits to the application to make the queries to the underlying `sqlit3` instance of an external application. This is interesting when using multiple derived classes of the `Sqlit3Connector` that deals with different tables of the same database.

The `Sqlit3Connector` integrates thread-safe locks.

### HOW TO USE:

1. create the instance
2. set the database name
3. Start() : will open the database or return error.
4. make querys: if a query extracts data from the database, then the application will need to create the `sqlite3_stmt` pointer, and invoke `sqlite3_finalize` once the datas are extracted.
5. Stop() : will close the database.
6. destroy the instance. In any case, destroying the instance will `Stop()` the connector if it is not already stopped.

## `CYT3MACROS` : `Sqlit3Connector-IO`

This macros extend the use of the `model-class` preprocessor macros, and give them the hability to serialize and deserialize to a table in an sqlite database, using a `Sqlit3Connector` as a base class.

It is needed that the desired `model-class` has already declared the `set-stack` extension previously to the invokation of the `serializable-sqlit3` macro.



### SYNTAX:

```cpp
  CYT3MACRO_model_class_serializable_sqlit3_definitions(
    <name of the class>
    , ( not required: options)
    //triples 
    , <property name>     , "sqlite type"     , "column name"
    ...
    ...
  )
```

* **name of the class:** name of the `model-class`. 
* **options:** not yet implemented.
* **triples:**
  * **property name:** as declared in the `model-class`
  * **sqlite type:** the sqlite type of the column associated to the *propery-name*.
    * **The class supports and will understand an AUTOINCREMENT keyword**
  * **column name:** of the table.


### `Sqlit3Connector` extension:

The Sqlit3Connector will be extended with the following methods and properties:

* Table name:
  * `std::string set_table_name() const;`
  * `std::string set_table_name(const std::string& tableName) const;`


The `serializable-sqlit3io` macro will extend the `Sqlit3Connector` class and create 