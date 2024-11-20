# COYOT3PP::COR3 FUNTIONS AND CLASSES


## INTRODUCTION

This component includes all the basic tools that require nothing more than standard c++ libraries, as well as any standard libraries included in standard repositories.

## FUNCTIONS

The main idea is to keep them abstract from the platform... and near of my needs and way of thiking...

* timestamp:
  * `int64_t     get_current_timestamp(bool sec_msec = true)`
  * `int64_t     microseconds_from_epoch();`
  * `int64_t     nanoseconds_from_epoch();`
  * `std::string timestamp_string(int64_t timestamp = -1);`
  * `std::string get_current_utc_string();`
  * std::string timestamp_to_utc_string(int64_t utc_timestamp_msecs);
* random:
  * `std::string generate_string_aphanumeric(int sz);`
  * `int64_t     generate_natural_number();`
  * `int64_t     generate_natural_number(int64_t min, int64_t max);`
  * `double      generate_real_number();`
  * `double      generate_real_number(double min, double max);`
* files and dirs:
  * `bool load_json_from_file(const char* location,Json::Value& destination);`
  * `bool save_json_to_file(const char* path,const Json::Value& source);`
  * `bool file_exists(const std::string& path);`
  * `int get_dir_path_depth(const std::string& path);`
  * `std::string get_dir_path_parent(const std::string& path,int level = 1);`
  * `std::string find_file_in_dir(const std::string& fileName,const std::string& initial_path);`
  * `std::string get_current_executable_path();`
* strings
  * `std::string as_string(multiple-overloads src);`
    * *Idem*
  * `int get_dir_path_depth(const std::string& path);`
  * `std::string get_dir_path_parent(const std::string& path,int level = 1);`
* json
  * `bool        json_import_value(const Json::Value& source, multiple-overloads)`
* arguments of the executable:
  * `bool  has_launch_command_argument(const std::string& argument,int argc, char** argv);`
    * Will search for an argument of the executable... 
  * `std::string get_launch_command_argument(const std::string& argument,int argc,char** argv);`
    * Will serve the next argument of the searched one...
    * *in fact, i will rewrite this to benefit of `std::optional` of c++17.*


## CLASSES

### `JsonSerializablePacket`

A class to be used by inherited model classes. Mainly contains the following methods:

* `bool from_json(const Json::Value& source)`
  * To be overloaded at the inherited class. The *user* will need to gather the information of the json and put the data at the model's properties. It will *serve* `true` if the 
  * `from_json_stream` 
  * `from_json_string`
* `virtual Json::Value                                  to_json() const;`
  * To be overloaded at the inherited class. The *user* will need to set up the resulting `Json::Value`
  * `to_json_stream`
  * `to_json_string`
  

### `CytStringSet`

A wrapper of `std::vector<std::string>` created to be able to wrap `CytStringSetJsIO`

### `WorkerThread`

Inspired in a `QTimer`, creates a thread that launches a function/method as a single shoot, or iterates its invokation at some given interval.


### `ControlLatency`

Kind of a chrono that I use to measure the processing time.

### `LapClockLogger`

Same spirit as `ControlLatency`

### `Value`

A wrapper to be able to create a low typed variable, inspired by the `jsoncpp` library.

See the minimal example `value-example.cpp`.