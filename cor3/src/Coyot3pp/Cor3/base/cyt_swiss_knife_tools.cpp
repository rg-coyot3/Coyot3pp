#include <Coyot3pp/Cor3/base/cyt_swiss_knife_tools.hpp>

namespace coyot3{
namespace tools{
std::random_device *  __cyt_random_device = nullptr;
std::mt19937*         __cyt_mt_generator  = nullptr;
void __initialize_random_device_generator(){
  if(__cyt_random_device != nullptr)return;
  std::cout << "csk_tools : --initialize-random-device-generator : initializing" 
    << std::endl;
  __cyt_random_device = new std::random_device();
  __cyt_mt_generator  = new std::mt19937((*__cyt_random_device)());
}
void InitializeToolkit()
{
  CLOG_INFO("CYT-TOOLKIT : initialize");
  std::srand(std::time(nullptr));
  __initialize_random_device_generator();
}
bool DebugLevelSet(int debug_level)
{
  CLOG_DEBUG_LEVEL_SET(debug_level);
  return true;
}




// TO-DO
//std::string getCurrentExecutablePath(){
//  //readlink <linux
//  //GetModuleFilename(NULL, buf, bufsize) < win
//}

// TO-DO
//std::string getCurrentExecutablePath(){
//  //readlink <linux
//  //GetModuleFilename(NULL, buf, bufsize) < win
//}




std::string getEnvironmentVariable(const std::string& env)
{
  //linux
  //https://stackoverflow.com/questions/5866134/how-to-read-linux-environment-variables-in-c
  const char * val = std::getenv( env.c_str() );
     if ( val == nullptr ) { // invalid to assign nullptr to std::string
         return "";
     }
     else {
         return val;
     }
  //win
  //DWORD getEnvironmentVariableLPCTSTR lpName,LPTSTR  lpBuffer,DWORD   nSize);

}
bool is_number(const std::string& s)
{
    return !s.empty() 
        && std::find_if(
          s.begin(), 
          s.end(), 
          [](unsigned char c) { return !std::isdigit(c); }) 
            == s.end();
}

int64_t getCurrentTimestamp()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t get_current_timestamp(bool secs_msec){
  if(secs_msec){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
int64_t now(){
  return get_current_timestamp(true);
}

int64_t     seconds_from_epoch(){
  return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
int64_t     milliseconds_from_epoch(){
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
int64_t     microseconds_from_epoch(){
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
int64_t     nanoseconds_from_epoch(){
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string 
getCurrentUtcString()
{
  return timestampToUtcString(getCurrentTimestamp());
}
std::string 
get_current_utc_string(){
  return timestamp_to_utc_string(milliseconds_from_epoch());
}

std::string 
timestamp_string(int64_t timestamp){
  if(timestamp == -1)
    timestamp = milliseconds_from_epoch();

  std::string utc_timestamp_string;
  struct timeval tv;
  tv.tv_sec = timestamp/1000;
  tv.tv_usec = (timestamp%1000)*1000;
  //struct tm* tm_info = localtime(&tv.tv_sec);
  struct tm* tm_info = std::gmtime(&tv.tv_sec);
  char buffer[26];
  int msecs = timestamp%1000;
  strftime(buffer,26,"%Y/%m/%dT%H:%M:%S",tm_info);
  utc_timestamp_string = buffer ;
  utc_timestamp_string+='.';
  utc_timestamp_string+=(msecs<100?(msecs<10?"00":"0"):"");
  utc_timestamp_string+=std::to_string(msecs);
  return utc_timestamp_string;
}
std::string
timestampToString(int64_t timestamp){
  return timestamp_string(timestamp);
}


std::string 
timestamp_to_utc_string(int64_t utc_timestamp_msecs)
{
  std::string utc_timestamp_string;
  struct timeval tv;
  tv.tv_sec = utc_timestamp_msecs/1000;
  tv.tv_usec = (utc_timestamp_msecs%1000)*1000;
  //struct tm* tm_info = localtime(&tv.tv_sec);
  struct tm* tm_info = std::gmtime(&tv.tv_sec);
  char buffer[26];
  int msecs = utc_timestamp_msecs%1000;
  strftime(buffer,26,"%Y-%m-%dT%H:%M:%S",tm_info);
  utc_timestamp_string = buffer ;
  utc_timestamp_string+='.';
  utc_timestamp_string+=(msecs<100?(msecs<10?"00":"0"):"");
  utc_timestamp_string+=std::to_string(msecs);
  return utc_timestamp_string;
}
std::string 
timestampToUtcString(int64_t utc_timestamp_msecs){
  return timestamp_to_utc_string(utc_timestamp_msecs);
}

std::string timestampToUtcString2(int64_t utc_timestamp_msecs)
{
  std::string utc_timestamp_string;
  struct timeval tv;
  tv.tv_sec = utc_timestamp_msecs/1000;
  tv.tv_usec = (utc_timestamp_msecs%1000)*1000;
  //struct tm* tm_info = localtime(&tv.tv_sec);
  struct tm* tm_info = std::gmtime(&tv.tv_sec);
  char buffer[26];
  strftime(buffer,26,"%Y-%m-%d %H:%M:%S",tm_info);
  utc_timestamp_string = buffer ;
  return utc_timestamp_string;
}

bool        file_exists(const std::string& path)
{
  struct stat buffer;   
  return (stat (path.c_str(), &buffer) == 0); 
}

std::string generate_string_aphanumeric(int sz)
{
  const char* source = "abcdefghijklmnopqrtsuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  size_t source_l = strlen(source);
  std::string res = "";
  for(int i=0;i<sz;i++)
  {
    res+=source[random()%source_l];
  }
  return res;
}


#include <math.h>
#include <cmath> 
#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
double degree_2_radian(double deg) {
  return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double radian_2_degree(double rad) {
  return (rad * 180 / M_PI);
}

double distance_m_earth_input_radian(double lat1r,double lon1r,double lat2r,double lon2r)
{
    double u,v;
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    return 2.0 * EARTH_RADIUS_KM * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)) * 1000.0;
}
double distance_m_earth_input_degrees(double lat1d,double lon1d,double lat2d,double lon2d)
{
    double lat1r, lon1r, lat2r, lon2r;
    lat1r = degree_2_radian(lat1d);
    lon1r = degree_2_radian(lon1d);
    lat2r = degree_2_radian(lat2d);
    lon2r = degree_2_radian(lon2d);
    return distance_m_earth_input_radian(lat1r,lon1r,lat2r,lon2r);
}

std::string currentTimedateString()
{
    int64_t utc_timestamp_msecs = getCurrentTimestamp();
    std::string utc_timestamp_string;
    struct timeval tv;
    tv.tv_sec = utc_timestamp_msecs/1000;
    tv.tv_usec = (utc_timestamp_msecs%1000)*1000;
    //struct tm* tm_info = localtime(&tv.tv_sec);
    struct tm* tm_info = std::gmtime(&tv.tv_sec);
    char buffer[26];
    int msecs = utc_timestamp_msecs%1000;
    strftime(buffer,26,"%Y-%m-%dT%H:%M:%S",tm_info);
    utc_timestamp_string = buffer ;
    utc_timestamp_string+='.';
    utc_timestamp_string+=(msecs<100?(msecs<10?"00":"0"):"");
    utc_timestamp_string+=std::to_string(msecs);
    return utc_timestamp_string;
}



bool load_content_from_file_text(const std::string& path,std::string& content)
{
  try{

  
  std::ifstream ifs(path.c_str());
  content = std::string((std::istreambuf_iterator<char>(ifs)),(std::istreambuf_iterator<char>()));
  return ifs.good();

  }catch(std::exception e)
  {
    CLOG_ERROR("error loading file " << e.what());
  }
  return false;
}
bool load_json_from_file(const char* path,Json::Value& destination)
{
    CLOG_INFO("CYTGadget::load json from file [" << path << "]");
    std::ifstream ifs(path);
    Json::Value buffer;
    Json::Reader reader;

    if(!ifs.good())
    {
        CLOG_ERROR("CYTGadget::load json from file [" << path << 
            "] : ERROR : file doesn't exist");
        return false;
    }
    bool result = reader.parse(ifs,buffer);
    if(!result)
    {
        CLOG_ERROR("CYTGadget::load json from file [" << path << 
            "] : ERROR : impossible to load file");
        CLOG_ERROR("CYTGadget::load json from file [" << path << 
            "] : ERROR! : " << reader.getFormattedErrorMessages());
        return false;
    }
    CLOG_INFO("CYTGadget::load json from file [" << path << "] loaded correctly");
    CLOG_DEBUG(7,"CYTGadget::load json from file [" << path << "] loaded content:");
    CLOG_DEBUG(7,buffer);
    CLOG_DEBUG(7,"CYTGadget::load json from file [" << path << "]");

    destination = buffer;
    return true;
}
bool JsonParse(Json::Value& destination, const std::string& s)
{
  Json::Reader reader;
  bool res;
  try{
    res = reader.parse(s,destination);
  }catch(...)
  {
    return false;
  }
  return res;
}
bool JsonParse(Json::Value& destination,const uint8_t* d,size_t s)
{
  std::string buffer;
  for(size_t i=0;i<s;++i)
  {
    buffer+=(char)d[i];
  }
  return JsonParse(destination,buffer);
}


bool save_json_to_file(const char* path,const Json::Value& source)
{
    std::ofstream ofs(path);
    Json::StyledStreamWriter ssw;
    CLOG_INFO("CYTGadget::save json to file [" << path << "] : starting to save");
    ssw.write(ofs,source);
    CLOG_INFO("CYTGadget::save json to file [" << path << "] : saved json to file");
    return true;
}

bool load_yaml_from_file(const char* location,YAML::Node& destination)
{
  bool res = true;
  try{
    destination = YAML::LoadFile(std::string(location));
    CLOG_DEBUG(7,"CYTGadget::load yaml from location : loaded from [" 
      << location << "]");
  }catch(YAML::ParserException& e){
    res = false;
    CLOG_ERROR("CYTGadget::load yaml from location : [" << location << "] resulted "
      "in error : reason ((" << e.what() << ")");
  }catch(...){
    res = false;
    CLOG_ERROR("CYTGadget::load yaml from location : [" << location << "] resulted "
      "in error : reason ((unknown?)");
  }
  return res;
}

bool parse_yaml(const std::string& source,YAML::Node& destination)
{
  bool res = true;
  try{
    destination = YAML::Load(std::string(source));
    CLOG_DEBUG(7,"CYTGadget::parse yaml : parsed ok from [" 
      << source << "]");
  }catch(YAML::ParserException& e){
    res = false;
    CLOG_ERROR("CYTGadget::parse yaml : error parsing [" << source << "] resulted "
      "in error : reason ((" << e.what() << ")");
  }catch(...){
    CLOG_ERROR("CYTGadget::parse yaml : error parsing [" << source << "] resulted "
      "in error : reason ((unknown?)");
    res = false;
  }
  return res;
}



std::string to_upper(const std::string& source)
{
    std::string buffer = source;
    std::transform(buffer.begin(),buffer.end(),buffer.begin(),::toupper);
    return buffer;
}
std::string to_lower(const std::string& source)
{
    std::string buffer = source;
    std::transform(buffer.begin(),buffer.end(),buffer.begin(),::tolower);
    return buffer;
}

std::string 
to_string(const uint8_t* d, size_t s){
  char* a = new char[s+1];
  memcpy(a,d,s);
  a[s] = '\0';
  std::string r(a);
  delete[] a;
  return r;
}
std::string 
toString(const uint8_t* d,size_t s)
{
  return to_string(d,s);
}
std::string to_string(const Json::Value& source){
  std::stringstream s;
  s << source;
  return s.str();
}
std::string JsonStringify(const Json::Value& source){
  return to_string(source);
}

double hypotenuse(double x1,double y1,double x2,double y2)
{
    return sqrt(pow(x2-x1,2.0) + pow(y2-y1,2.0));
}

bool has_launch_command_argument(const std::string& argument,int argc, char** argv)
{
  for(int i = 0;i<argc;++i)
  {
    if(!argument.compare(argv[i]))
    {
      return true;
    }
  }
  return false;
}

std::string get_launch_command_argument(const std::string& argument,int argc,char** argv)
{
  std::string res;
  for(int i=0;i<argc;++i)
  {
    if(!argument.compare(argv[i]))
    {
      if(i+1 < argc)
      {
        res = argv[i+1];
      }
    }
  }
  return res;
}

std::string get_real_path(const std::string relativePath)
{
  std::string res;
  char* a = new char[256];
  if(!realpath(relativePath.c_str(),a))
  {
    return "";
  }
  res = a;
  return res;
}



bool json_contains_member(const Json::Value& source,const std::string& property)
{
  std::vector<std::string> fields;
  Json::Value inner = source;
  string_split(property,'.',fields);


  for(auto field : fields)
  {
    if(inner.isMember(field))
    {
      inner = inner[field];
    }else{
      CLOG_WARN("libcoyot3 : tools : json contains member : field [" 
        << property << "] was not found at given source");
      return false;
    }
  }
  CLOG_DEBUG(3,"libcoyot3 : tools : json contains member : member [" << property
    << "] found at json source structure");
  return true;
}


Json::Value json_get_member(const Json::Value& source,const std::string& property)
{
  std::vector<std::string> fields;
  if(property.find_first_of(".") == std::string::npos){
    if(source.type() != Json::objectValue){
      return Json::Value(Json::nullValue);
    }
    if(source.isMember(property)==false){
      return Json::Value(Json::nullValue);
    }
    return source[property];
  }
  
  Json::Value inner = source;
  
  string_split(property,'.',fields);
  for(auto field : fields)
  {
    if(inner.isMember(field))
    {
      inner = inner[field];
    }else{
      CLOG_WARN("libcoyot3 : tools : json get member : member [" << property 
        << "] does not belong to the object. returning 'null'");
      return Json::Value(Json::nullValue);
    }
  }
  CLOG_DEBUG(3,"libcoyot3 : tools : json get member : member [" << property
    << "] found (" << inner << ")");
  return inner;
}
bool json_import_value(const Json::Value& source, const std::string& property, std::string& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination.clear();
    return false;
  }
  try{
    destination = v.asString();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<std::string> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, double& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = 0;
    return false;
  }
  try{
    destination = v.asDouble();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<double> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, int& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = 0;
    return false;
  }
  try{
    destination = v.asInt();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<int> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, int64_t& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = 0;
    return false;
  }
  try{
    destination = v.asLargestInt();
  }
  catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<int64_t> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, uint& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = 0;
    return false;
  }
  try{
    destination = v.asUInt();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<uint> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, uint64_t& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = 0;
    return false;
  }
  try{
    destination = v.asLargestUInt();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<uint64_t> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}
bool json_import_value(const Json::Value& source, const std::string& property, bool& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = false;
    return false;
  }
  try{
    destination = v.asBool();
  }catch(Json::Exception& e)
  {
    CLOG_WARN("coyot3::tools::json_import_value<bool> : requesting (" << property << ") error " << e.what());
    return false;
  }
  return true;
}

bool json_import_value(const Json::Value& source, const std::string& property, Json::Value& destination)
{
  Json::Value v;
  if(property.size() == 0)v = source;
  else v = json_get_member(source,property);
  if(v.type() == Json::nullValue)
  {
    destination = false;
    return false;
  }
  destination = v;
  return true;
}



Json::CharReader* json_parser;
std::string       json_parser_error;
bool initialize_json_parser()
{
  Json::CharReaderBuilder b;
  json_parser = b.newCharReader();
  if(json_parser == nullptr)
  {
    CLOG_ERROR("libcoyot3 : tools : error creating parser");
    return false;
  }
  return true;
}
bool jsonize(Json::Value& dest,const uint8_t* s,size_t l)
{
  bool res;
  res = json_parser->parse((char*)s,(char*)s+l,&dest,&json_parser_error);
  if(res == false)
  {
    CLOG_ERROR("libcoyot3 : tools : jsonize - stream : error parsing input : [" 
      << json_parser_error << "]");
  }
  return res;
}
bool jsonize(Json::Value& dest,const std::string& s)
{
  return jsonize(dest,(uint8_t*)s.c_str(),s.size());
}



std::string stringify(const Json::Value& js)
{
  std::stringstream ss;
  ss << js;
  return ss.str();
}
std::string stringify(const uint8_t* pptr,size_t psize)
{
  std::string s;
  for(size_t i = 0;i < psize;++i)
  {
    s+=(char)pptr[i];
  }
  return s;
}


std::string get_current_executable_path()
{
  char result[511];
  ssize_t count = readlink("/proc/self/exe",result,511);
  std::string executableFullPath = std::string(result,(count>0?count:0));
  size_t it1,it2;
  it1 = it2 = 0;
  do{
    it2 = executableFullPath.find('/',it1+1);
    if(it2!=std::string::npos)
    {
      it1 = it2;
    }
  }while(it2 != std::string::npos);
  return executableFullPath.substr(0,it1);
}


int         get_dir_path_depth(const std::string& path)
{
  size_t it1,it2;
  int count = 0;
  it1 = it2 = 0;
  do{
    it2 = path.find('/',it1+1);
    if(it2!=std::string::npos)
    {
      it1 = it2;
      ++count;
    }
  }while(it2 != std::string::npos);
  return count;
}

std::string get_dir_path_parent(const std::string& path,int level)
{
  size_t it1,it2;
  it1=it2=0;
  std::string buffer;
  buffer = path;
  for(int n = 0;((n<level) && (getDirPathDepth(buffer) > 0));n++)
  {
    do{
      it2 = buffer.find('/',it1+1);
      if(it2!= std::string::npos)
      {
        it1 = it2;
        
      }
    }while(it2 != std::string::npos);
    buffer = buffer.substr(0,it1);
    it1 = it2 = 0;
  }
  return buffer;
}

std::string exec(const char* cmd) 
{
  char buffer[128];
  std::string result = "";
  FILE* pipe = popen(cmd, "r");
  if (!pipe) throw std::runtime_error("popen() failed!");
  try {
      while (fgets(buffer, sizeof buffer, pipe) != NULL) {
          result += buffer;
      }
  } catch (...) {
      CLOG_ERROR("coyot3 : tools : exec : error launching command [" << cmd << "]");
      pclose(pipe);
      //throw;
      return "";
  }
  pclose(pipe);
  return result;
}


std::string find_file_in_dir(const std::string& fileName,const std::string& initial_path)
{
  std::string r;
  std::ostringstream o;

  o << "find " << initial_path << " -iname \"" << fileName << "\"";
  r = exec(o.str().c_str());
  CLOG_INFO("find nearest, at path " << o.str() << " : debug : r(" << r.size() << ") = " << r);
  o.clear();


  if(r.size()!=0)
  {
    std::ostringstream o;
    CLOG_INFO("debug : current r " << r);
    r = r.substr(0,r.size()-1);
    o.clear();
    o << "cd $(dirname " << r << ") >> /dev/null 2>&1 && pwd || echo \"error\"";
    r = exec(o.str().c_str());
    CLOG_INFO("debug : current r = " << r);
    if(r.find("error")==std::string::npos)r=r.substr(0,r.size()-1);
    else r.clear();
  }
  return r;
}


const char* ModuleStateToString(ModuleState s)
{
  switch(s)
  {
    case ModuleState::CREATED:      return "CREATED";break;
    case ModuleState::CONFIGURED:   return "CONFIGURED";break;
    case ModuleState::LAUNCHING:    return "LAUNCHING";break;
    case ModuleState::LAUNCHED:     return "LAUNCHED";break;
    case ModuleState::STOPPED:      return "STOPPED";break;
    case ModuleState::SHUTDOWN:     return "SHUTDOWN";break;
    case ModuleState::DISCONNECTED: return "DISCONNECTED";break;
    case ModuleState::CONNECTED:    return "CONNECTED";break;
    case ModuleState::ERROR:        return "ERROR";break;
    default:
      return "module-state-decode-error";
  }
}
ModuleState ModuleStateFromString(const std::string& s)
{
  if(!s.compare(ModuleStateToString(ModuleState::CREATED))){return ModuleState::CREATED;}
  if(!s.compare(ModuleStateToString(ModuleState::CONFIGURED))){return ModuleState::CONFIGURED;}
  if(!s.compare(ModuleStateToString(ModuleState::LAUNCHING))){return ModuleState::LAUNCHING;}
  if(!s.compare(ModuleStateToString(ModuleState::LAUNCHED))){return ModuleState::LAUNCHED;}
  if(!s.compare(ModuleStateToString(ModuleState::STOPPED))){return ModuleState::STOPPED;}
  if(!s.compare(ModuleStateToString(ModuleState::SHUTDOWN))){return ModuleState::SHUTDOWN;}
  if(!s.compare(ModuleStateToString(ModuleState::ERROR))){return ModuleState::ERROR;}
  return ModuleState::UNKNOWN_WRONG_STATE;
}

size_t string_split(const std::string& str,const char delim, std::vector<std::string>& out)
{
  //from [https://www.codegrepper.com/code-examples/cpp/std+string+split+c%2B%2B+17]
  std::string scopy = str;
  size_t start;
  size_t end=0;
  out.clear();
  while((start = scopy.find_first_not_of(delim,end)) != std::string::npos)
  {
    end = scopy.find(delim,start);
    out.push_back(scopy.substr(start,end-start));
    // CLOG_INFO("to-delete : string-split : added substring (" << scopy << ")[" 
    //   << start << ":" << end-start << "]=" << scopy.substr(start,end-start));
  }
  //CLOG_INFO("to-delete : string-split : serving [" << out.size() << "] splitted strings");
  return out.size();
}


std::string string_replace(std::string str,const std::string& from,const std::string& to)
{
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
      str.replace(start_pos, from.length(), to);
      start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
  }
  return str;
}

Json::Value as_json(const std::string& src){return src;}
Json::Value as_json(const double& src){return src;}
Json::Value as_json(const int& src){return Json::Value(src);}
Json::Value as_json(const int64_t& src){return static_cast<Json::LargestInt>(src);}
Json::Value as_json(const uint& src){return static_cast<Json::UInt>(src);}
Json::Value as_json(const uint64_t& src){return static_cast<Json::LargestUInt>(src);}
Json::Value as_json(const bool& src){return src;}
Json::Value as_json(const Json::Value& src){return src;}

std::string as_string(const std::string& src){return src;}
std::string as_string(const double& src){return std::to_string(src);}
std::string as_string(const int& src){return std::to_string(src);}
std::string as_string(const int64_t& src){return std::to_string(src);}
std::string as_string(const uint& src){return std::to_string(src);}
std::string as_string(const uint64_t& src){return std::to_string(src);}
std::string as_string(const bool& src){return std::to_string(src);}
std::string as_string(const Json::Value& src){std::stringstream ss; ss << src;return ss.str();}



template<typename T>
bool _rjson_import_value_common(const rapidjson::Value& source, const std::string& prop, T& dest){
  std::string membername,rest;
  std::size_t pos = std::string::npos;
  if(pos != std::string::npos){
    membername = prop.substr(0,pos);
    rest       = prop.substr(pos+1);
  }else{
    membername = prop;
  }
  if(source.IsObject() == false)return false;
  for(rapidjson::Value::ConstMemberIterator it = source.MemberBegin();
      it != source.MemberEnd(); ++it){
    if(membername.compare(it->name.GetString()) == 0){
      return rjson_import_value(it->value,rest,dest);
    }
  }
  return false;
}

bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, int& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsInt() == false)return false;
    dest = source.GetInt();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, int64_t& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsInt64() == false)return false;
    dest = source.GetInt64();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, uint& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsUint() == false)return false;
    dest = source.GetUint();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, uint64_t& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsUint64() == false)return false;
    dest = source.GetUint64();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, std::string& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsString() == false)return false;
    dest = source.GetString();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, bool& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsBool() == false)return false;
    dest = source.GetBool();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, float& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsFloat() == false)return false;
    dest = source.GetFloat();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_import_value(const rapidjson::Value& source, const std::string& prop, double& dest){
  
  if(prop.size() == 0)
  {
    if(source.IsDouble() == false)return false;
    dest = source.GetDouble();
    return true;
  }
  return _rjson_import_value_common(source,prop,dest);
}
bool rjson_has_member(const rapidjson::Value& source, const std::string& prop){
    for(rapidjson::Value::ConstMemberIterator it = source.MemberBegin();
      it != source.MemberEnd(); ++it){
    if(prop.compare(it->name.GetString()) == 0){
      return true;
    }
  }
  return false;

}

const rapidjson::Value& rjson_get_member(const rapidjson::Value& source, const std::string& prop)
{

  for(rapidjson::Value::ConstMemberIterator it = source.MemberBegin();
    it != source.MemberEnd(); ++it){
    if(prop.compare(it->name.GetString()) == 0){
      return it->value;
    }
  }
  return source;

}


std::string indentation(int l__){
  std::string r;
  for(int i=0;i<l__;++i){
    r+="  ";
  }
  return r;
}


int64_t generate_natural_number(){
  return generate_natural_number(std::numeric_limits<int64_t>::max(),std::numeric_limits<int64_t>::min());
}
int64_t generate_natural_number(int64_t max, int64_t min){
  if(__cyt_mt_generator == nullptr)__initialize_random_device_generator();
  std::uniform_int_distribution<int64_t> dis(min,max);
  return dis(*__cyt_mt_generator);
}

double generate_real_number(){
  
  return generate_real_number(std::numeric_limits<double>::min(),std::numeric_limits<double>::max());
}
double generate_real_number(double min,double max){
  if(__cyt_mt_generator == nullptr)__initialize_random_device_generator();
  std::uniform_real_distribution<double> dis(min,max);
  return dis(*__cyt_mt_generator);
}


rapidjson::Value as_rjson(const int& val){rapidjson::Value ret; ret.SetInt(val);         return ret;}
rapidjson::Value as_rjson(const int64_t& val){rapidjson::Value ret; ret.SetInt64(val);         return ret;}
rapidjson::Value as_rjson(const uint& val){rapidjson::Value ret; ret.SetUint(val);         return ret;}
rapidjson::Value as_rjson(const uint64_t& val){rapidjson::Value ret; ret.SetUint64(val);         return ret;}
rapidjson::Value as_rjson(const std::string& val){rapidjson::Value ret; ret.SetString(val.c_str(), val.size());         return ret;}
rapidjson::Value as_rjson(const char* val){rapidjson::Value ret; ret.SetString(val,strlen(val));         return ret;}
rapidjson::Value as_rjson(const bool& val){rapidjson::Value ret; ret.SetBool(val);         return ret;}
rapidjson::Value as_rjson(const double& val){rapidjson::Value ret; ret.SetDouble(val);         return ret;}
rapidjson::Value as_rjson(const float& val){rapidjson::Value ret; ret.SetFloat(val);         return ret;}



}//namespace tools        
}//namespace coyot3

std::ostream& operator<<(std::ostream& o,const coyot3::tools::ModuleState& s)
{
  return (o << coyot3::tools::ModuleStateToString(s));
}