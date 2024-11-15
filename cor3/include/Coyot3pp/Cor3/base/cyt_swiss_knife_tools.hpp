#pragma once

#include <chrono>
#include "cyt_simple_logger.hpp"
#include <string>
//#include <xmlrpcpp/XmlRpcValue.h>
#include <time.h>
#include <assert.h>

#include <ostream>
#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <thread>
#include <functional>
#include <chrono>
#include <limits.h>
#include <mutex>
#include <condition_variable>
#include <unistd.h>
#include <oping.h>
#include <sys/stat.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>




#define EARTH_RADIUS_KM 6371.0

namespace coyot3{
namespace tools{

void InitializeToolkit();






bool        DebugLevelSet(int debug_level);
bool load_content_from_file_text(const std::string& path,std::string& content);

std::string getEnvironmentVariable(const std::string& env);

int64_t     getCurrentTimestamp();
/**
 * @brief : timestamp in seconds or milliseconds from epoch
 * @param sec_msec : false = returns in seconds; true(default) = returns in milliseconds
*/
int64_t     get_current_timestamp(bool sec_msec = true);
int64_t     now();

int64_t     seconds_from_epoch();
int64_t     milliseconds_from_epoch();
int64_t     microseconds_from_epoch();
int64_t     nanoseconds_from_epoch();


std::string timestampToString(int64_t timestamp = -1);
std::string timestamp_string(int64_t timestamp = -1);

std::string getCurrentUtcString();
std::string get_current_utc_string();

std::string timestampToString(int64_t timestamp);

std::string timestamp_to_utc_string(int64_t utc_timestamp_msecs);
std::string timestampToUtcString(int64_t utc_timestamp_msecs);
std::string timestampToUtcString2(int64_t utc_timestamp_msecs);


/**
 * @brief generates a random string of size 'sz'
*/
std::string generate_string_aphanumeric(int sz);

int64_t     generate_natural_number();
int64_t     generate_natural_number(int64_t min, int64_t max);
double      generate_real_number();
double      generate_real_number(double min, double max);



/**
 * @brief stringifies input
 * @param d : data payload pointer
 * @param s : size of the payload in bytes
 * @return string
*/
std::string to_string(const uint8_t* d,size_t s);
std::string toString(const uint8_t* d,size_t s);


std::string JsonStringify(const Json::Value& source);
std::string to_string(const Json::Value& source);


bool        load_json_from_file(const char* location,Json::Value& destination);
bool        save_json_to_file(const char* path,const Json::Value& source);
bool        JsonParse(Json::Value& destination,const uint8_t* d,size_t l);
bool        JsonParse(Json::Value& destination, const std::string& s);
/**
 * @brief checks the existence of a file
 * @param path : string path to file to check
 * @return true if file exists
*/
bool        file_exists(const std::string& path);



bool        initialize_json_parser();
bool        jsonize(Json::Value& dest,const uint8_t* s,size_t l);
bool        jsonize(Json::Value& dest,const std::string& s);
bool        json_contains_member(const Json::Value& source,const std::string& property);
Json::Value json_get_member(const Json::Value& source,const std::string& property);


bool        load_yaml_from_file(const char* location,YAML::Node& destination);
bool        parse_yaml(const std::string& source,YAML::Node& destination);

/**
 * @brief imports a value contained at a json value object
 * @param source : data source
 * @param property : separated by '.', the path to the value
 * @param destination : destination to store the value (multiple types)
 * @return true if the value was found and has been stored at destination.
*/
bool        json_import_value(const Json::Value& source, const std::string& property, std::string& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, double& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, int& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, int64_t& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, uint& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, uint64_t& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, bool& destination);
bool        json_import_value(const Json::Value& source, const std::string& property, Json::Value& destination);

/**
 * @brief useful wrapper when constructing preprocessor macros.
 * @param src : data source to jsonize
 * @return Json::Value
*/
Json::Value as_json(const std::string& src);
Json::Value as_json(const double& src);
Json::Value as_json(const int& src);
Json::Value as_json(const int64_t& src);
Json::Value as_json(const uint& src);
Json::Value as_json(const uint64_t& src);
Json::Value as_json(const bool& src);
Json::Value as_json(const Json::Value& src);


rapidjson::Value as_rjson(const int& val);
rapidjson::Value as_rjson(const int64_t& val);
rapidjson::Value as_rjson(const uint& val);
rapidjson::Value as_rjson(const uint64_t& val);
rapidjson::Value as_rjson(const std::string& val);
rapidjson::Value as_rjson(const char* val);
rapidjson::Value as_rjson(const bool& val);
rapidjson::Value as_rjson(const double& val);
rapidjson::Value as_rjson(const float& val);






/**
 * @brief useful wrapper when constructing preprocessor macros
 * @param src : data source to stringify
 * @return std::string
*/
std::string as_string(const std::string& src);
std::string as_string(const double& src);
std::string as_string(const int& src);
std::string as_string(const int64_t& src);
std::string as_string(const uint& src);
std::string as_string(const uint64_t& src);
std::string as_string(const bool& src);
std::string as_string(const Json::Value& src);

bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, int& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, int64_t& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, uint& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, uint64_t& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, std::string& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, bool& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, float& dest);
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, double& dest);
bool rjson_has_member(const rapidjson::Value& source, const std::string& prop);
const rapidjson::Value& rjson_get_member(const rapidjson::Value& source, const std::string& prop);

/**
 * @brief to capital letters
*/
//std::string toUpper(const std::string& source);
std::string to_upper(const std::string& source);
constexpr auto toUpper = to_upper;
/**
 * @brief to lower letters
*/
//std::string toLower(const std::string& source);

std::string to_lower(const std::string& source);
constexpr auto toLower = to_lower;

/**
 * @brief mathematical gadgets
*/
double      radian_2_degree(double rad);
double      degree_2_radian(double degree);
/**
 * @brief distance in meters of 2 given coordinates (input radians)
 * @return distance in meters 
*/
double      distance_m_earth_input_radian(double lat1r,double lon1r,double lat2r,double lon2r);
/**
 * @brief distance in meters of 2 given coordinates (input latitude,longitude)
 * @return distance in meters 
*/
double      distance_m_earth_input_degrees(double lat1d,double lon1d,double lat2d,double lon2d);

double      hypotenuse(double x1,double y1,double x2,double y2);

/**
 * @brief searches an argument used when the executable is launched.
 * @param searched argument : i.e.: "--help"
 * @return true if the argument exists in the command line
*/
bool  has_launch_command_argument(const std::string& argument,int argc, char** argv);
constexpr auto hasLaunchCommandArgument = has_launch_command_argument;
/**
 * @brief serves an argument used when the executable is launched.
 * @param searched argument : i.e.: "--num-instances 4"
 * @return the value relative to the searched argumnet.
*/
std::string get_launch_command_argument(const std::string& argument,int argc,char** argv);
constexpr auto getLaunchCommandArgument = get_launch_command_argument;

std::string get_real_path(const std::string relativePath);
constexpr auto getRealPath = get_real_path;


size_t      string_split(const std::string& str,const char* delim, std::vector<std::string>& out);
constexpr auto stringSplit = string_split;

std::string string_replace(std::string str,const std::string& from,const std::string& to);
constexpr auto stringReplace = string_replace;

std::string stringify(const Json::Value& js);
std::string stringify(const uint8_t* pptr,size_t psize);


int         get_dir_path_depth(const std::string& path);
constexpr auto getDirPathDepth = get_dir_path_depth;


std::string get_dir_path_parent(const std::string& path,int level = 1);
constexpr auto getDirPathParent = get_dir_path_parent;



std::string find_file_in_dir(const std::string& fileName,const std::string& initial_path);
std::string exec(const char* cmd);

std::string get_current_executable_path();
constexpr auto getCurrentExecutablePath = get_current_executable_path;


  /**
   * I constantly use the same states for all the modules... no excuse not to
   *  create this model
   * */
  enum class ModuleState{
    CREATED             = 1,
    CONFIGURED          = 2,
    LAUNCHING           = 3,
    LAUNCHED            = 4,
    STOPPED             = 5,
    SHUTDOWN            = 6,
    DISCONNECTED        = 7,
    CONNECTED           = 8,
    ERROR               = 0,
    UNKNOWN_WRONG_STATE = -1
  };
  const char* ModuleStateToString(ModuleState s);
  ModuleState ModuleStateFromString(const char* s);

  std::string indentation(int l__);
  bool is_number(const std::string& s);
  
  

}//eons tools
}//eons coyot3

std::ostream& operator<<(std::ostream& o,const coyot3::tools::ModuleState& s);
