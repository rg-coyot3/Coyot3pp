#include <Coyot3pp/Cor3/base/cyt_complex_tools.hpp>


namespace coyot3{
namespace tools{


std::string  CytStringSetStringify(const CytStringSet& i)
{
  std::string res;
  for(const std::string& item : i){
    res+= item;
  }
  return res;
}
std::string  CytStringSetStringify(const CytStringSet& i,
                                  const std::string& joinString){
  std::string res;
  bool notFirst = false;
  for(const std::string& item : i){
    if(notFirst){
      res+=joinString;
    }
    res+= item;
    notFirst = true;
  }
  return res;
}

CytStringSet CytStringSetCreate(const std::string& input,const char key)
{
  CytStringSet res;
  stringSplit(input,key,res);
  return res;
}

bool CytStringSetJsIO::json_from_file(const std::string& file_path){
  return save_json_to_file(file_path.c_str(),to_json());
}
bool CytStringSetJsIO::json_to_file(const std::string& file_path){
  Json::Value js;
  if(!load_json_from_file(file_path.c_str(),js)== false)return false;
  return from_json(js);
}

CytStringSetJsIO::CytStringSetJsIO():CytStringSet(){}
CytStringSetJsIO::CytStringSetJsIO(const CytStringSet& o):CytStringSet(o){}
CytStringSetJsIO::~CytStringSetJsIO(){}


Json::Value CytStringSetJsIO::to_json() const {
  Json::Value js;
  for(const std::string& i : *this){
    js.append(i);
  }
  return js;
}


bool CytStringSetJsIO::from_json(const Json::Value& source){
  if(source.type() != Json::arrayValue)return false;
  Json::ArrayIndex i, s = source.size();
  bool res = true;
  clear();
  for(i = 0;i < s;++i){
    if(source[i].type() != Json::stringValue){
      CLOG_INFO("to delete : item [" << i << "] is NOT a string!")
      res = false;
      continue;
    }
    push_back(source[i].asString());
  }
  return res;
}
std::string CytStringSetJsIO::get_serialization_model_template(int l__){
  return indentation(l__) + "[array-of-strings]";
}

Json::Value as_json(const coyot3::tools::JsonSerializablePacketBase& src){return src.to_json();}





LapClockLogger::LapClockLogger(const std::string& name,bool silent):n(name),silent(silent),dlevel(4){}
LapClockLogger::~LapClockLogger(){}

int64_t LapClockLogger::Start(const std::string& note)
{
    laps.clear();
    time_start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    laps.push_back(std::make_pair(std::string(note),time_start));
    CLOG_DEBUG(dlevel,"LAP CLOCK LOGGER : " << n << " : Started at [" << time_start << "]");
    return time_start;
}

void LapClockLogger::Silent(bool s)
{
    silent = s;
}
int64_t LapClockLogger::Stop(const std::string& note)
{
    time_end = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    laps.push_back(std::make_pair(std::string(note),time_end));

    CLOG_DEBUG(dlevel,"LAP CLOCK LOGGER : stop : " << n   << " : Started at [" 
      << time_start << "] number of laps [" << laps.size()          
      << "] : end at " << time_end << "] total elapsed ["  
      << time_end - time_start << "]");

    return laps[laps.size() - 1].second - laps[0].second;

}
// -- lap Clock Logger : begin

void LapClockLogger::Report()
{
    std::vector<std::pair<std::string,int64_t>>::iterator it1 = laps.begin();
    std::vector<std::pair<std::string,int64_t>>::iterator it2 = it1+1;
    int lapCount = 0;
    CLOG_INFO("LAP CLOCK LOGGER : report for [" << n << "] : Started at [" 
      << laps[0].second << "]");
    for(;it2 != laps.end();++it1,++it2)
    {
        CLOG_INFO("LAP CLOCK LOGGER : report for [" << n << "] : Lap tick " 
          << ++lapCount <<  " : [" << it1->first << "] -> " 
          << (it2->second - it1->second) << " -> [" << it2->first << "]");
    }
    CLOG_INFO("LAP CLOCK LOGGER : report for [" << n << "] : Ended at [" 
      << laps[laps.size() - 1].second << " total = " 
      << (laps[laps.size() -1].second - laps[0].second));
} 
int64_t LapClockLogger::Lap(const std::string& note)
{
    int64_t val;
    laps.push_back(std::make_pair(note,lap = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    val = laps[laps.size() - 1].second - laps[laps.size() - 2].second;
    if(!silent)
        CLOG_INFO(" LAP CLOCK LOGGER : " << n
                << ": Lap ["
                << val
                << "] microsecs ("
                << note
                << ")");
    return val;
}
// -- lap Clock Logger : end ----------------------




// -- worker thread : begin
WorkerThread::WorkerThread(
  std::function<void()> func,
  const std::string& name)
: _flag(false)
, _th(nullptr)
, _f(func)
, _name(name)
, _interval(-1)
{
  CLOG_INFO("lcyt3 : worker-thread : " << _name << " : constructor");
}
WorkerThread::~WorkerThread()
{
  stop();
}
void WorkerThread::setInterval(int milliseconds)
{
  _interval = milliseconds;
}
bool WorkerThread::start(int interval)
{
  _interval = interval;
  return start();
}
bool WorkerThread::start()
{
  if(_th)
  {
    CLOG_WARN("lcyt3 : worker-thread : " << _name << " : start : thread is already up");
    return false;
  }
  _flag = true;
  if(_interval < 0)
  {
    _th = new(std::nothrow) std::thread([&](){
      CLOG_DEBUG(5,"lcyt3 : worker-thread : " << _name 
        << " : start : th : launched thread");
      _f();
      CLOG_DEBUG(5,"lcyt3 : worker-thread : " 
        << _name << " : start : th : ended thread flux");
    });
  }else{
    _th = new(std::nothrow) std::thread([&](){
      std::unique_lock<std::mutex> lock(this->_w_mtx);
      std::cv_status s;
      while(_flag == true)
      {
        _f();
        s = this->_w_cv.wait_for(lock,std::chrono::milliseconds(this->_interval));
        if(s == std::cv_status::no_timeout)
        {
          CLOG_WARN("lcyt3 : worker-thread : " << _name << " : start : th : wait unlock");
        }
      }
    });
  }
  if(!_th)
  {
    CLOG_ERROR("lcyt3 : worker-thread : " << _name << " : start : error creating thread");
    return false;
  }else{
    CLOG_INFO("lcyt3 : worker-thread : " << _name << " : start : OK");
  }
  return true;
}



bool 
WorkerThread::start_one_shot()
{
  if(_th != nullptr)
  {
    CLOG_WARN("lcyt3 : control-thread : start-one-shot : it seems that the lcyt3 : worker-thread is already launched!");
    return false;
  }

  _flag = true;
  _th = new(std::nothrow) std::thread([&](){
    CLOG_DEBUG(3, "lcyt3 : control-thread : start-one-shot : [" << _name 
      << "] launching threaded method");
    _f();
    CLOG_DEBUG(3, "lcyt3 : control-thread : start-one-shot : [" << _name 
      << "] launching threaded ended");
  });
  if(_th == nullptr){
    CLOG_ERROR("lcyt3 : control-thread : start-one-shot : [" << _name << "] error launching threaded method!")
    return false;
  }

  CLOG_DEBUG(1,"lcyt3 : control-thread : start-one-shot : [" << _name << "] launched");
  return true;
}




void WorkerThread::triggerNow(){
  if(_flag == false)return;
  _w_cv.notify_all();
}




bool 
WorkerThread::stop()
{
  if(_flag == false)
  {
    CLOG_WARN("lcyt3 : worker-thread : " << _name << " : stop : flag is already dropped");
  }
  _flag = false;

  if(!_th)
  {
    CLOG_WARN("lcyt3 : worker-thread : " << _name << " : stop : thread is not up");
    return false;
  }
  if(_interval >= 0)
  {
    CLOG_WARN("lcyt3 : worker-thread : " << _name << " : stop : unlocking thread");
    _w_cv.notify_all();
  }
  _th->join();
  delete _th;
  _th = nullptr;
  return true;
}
bool WorkerThread::isRunning()
{
  if(_th && _flag)
  {
    return true;
  }
  return false;
}
// lcyt3 : worker-thread - end



int ControlLatency::default_max_historic = 100;
int ControlLatency::default_interval = 1000;
int ControlLatency::default_timeout  = 500;

ControlLatency::ControlLatency(const std::string& targetUrl,const std::string& name)
:name_(name)
,turl_(targetUrl)
,po_(nullptr)
,cth_(nullptr)
,average_(0.0)
,latencies_()
,vector_max_length_(default_max_historic)
,interval_(default_interval)
,timeout_(default_timeout)
,new_data_(false)
,last_ping_ts(0)
,last_err_()
,last_err_ts_(0)
,show_notices_(false)
,is_valid_(false)
{
  CLOG_INFO("cytool : control-latency (" << name << ") : instance created [" << targetUrl << "]");
  po_ = ping_construct();
  if(!po_){
    CLOG_ERROR("cytool : control-latency(" << name << ") : error creating ping instance!!! no-memory???");
    exit(1);
  
  }
  int v = AF_INET;
  if((ping_setopt(po_,PING_OPT_AF,(void*)&v) != 0))
  {
    CLOG_ERROR("cytool : control-latency(" << name << ") : error configuring ping instance because (" << ping_get_error(po_) << ")");
  }
  timeout(timeout_);
  if(ping_host_add(po_,(turl_ + std::string("\0")).c_str())){
    CLOG_ERROR("cytool : control-latency(" << name << ") : error adding host (" << turl_ << ") because (" << ping_get_error(po_) << ")");
  }else{
    is_valid_ = true;
  }
}

ControlLatency::~ControlLatency(){
  std::lock_guard<std::mutex> guardp(mtxp_);
  std::lock_guard<std::mutex> guardd(mtxd_);

  stop_();
  ping_destroy(po_);
  po_ = nullptr;

}

double ControlLatency::getAverage(){
  if(!new_data_)return average_;
  {
    std::lock_guard<std::mutex> guard(mtxd_);
    average_=0.0;
    for(int v: latencies_){
      average_+=(double)v;
    }
  }
  average_/=(double)latencies_.size();
  return average_;
}
int ControlLatency::interval(){return interval_;}
int ControlLatency::interval(int i){return (interval_ = i);}

int ControlLatency::timeout(){return timeout_;}
int ControlLatency::timeout(int t){
  timeout_ = t;
  {
    std::lock_guard<std::mutex> guard(mtxp_);
    double to = static_cast<double>(timeout_)/1000.0;
    CLOG_INFO("cytool : control-latency : timeout : set to [" << to << "]");
    //ping_setopt(po_,PING_OPT_TIMEOUT,&to);
  }
  return timeout_;
}
bool   ControlLatency::showNotices(){return show_notices_;}
bool   ControlLatency::showNotices(bool s){return (show_notices_ = s);}

const std::list<int>& ControlLatency::getHistoric(){return latencies_;}

void ControlLatency::ping_controller(){
  last_ping_ts = getCurrentTimestamp();
  CLOG_DEBUG(7,"cytool : control-latency : ping-controller (" << name_ << ") : sending ping to (" << turl_ << ")");
  if(is_valid_ == false){
    CLOG_ERROR("cytool : control-latency : ping-controller : (" << name_ << ") : is not a valid test endpoint for [" << turl_ << "]");
    return;
  }
  int r;
  {
    std::lock_guard<std::mutex> guardp(mtxp_);
    r = ping_send(po_);
  }
  if(r < 0)
  {
    CLOG_DEBUG(6,"cytool : ping-controller (" << name_ << ") : sending ping to (" << turl_ << ") : error receiving");
    last_err_ = ping_get_error(po_);
    last_err_ts_ = last_ping_ts;

  }else if(r == 0){
    CLOG_DEBUG(6,"cytool : ping-controller (" << name_ << ") : sending ping to (" << turl_ << ") : zero?");
  }else{
    CLOG_DEBUG(6,"cytool : ping-controller (" << name_ << ") : sending ping to (" << turl_ << ") : got pong");
  }

  {
    std::lock_guard<std::mutex> guardd(mtxd_);
    last_latency_ = getCurrentTimestamp() - last_ping_ts;
    if(show_notices_){
      if(r>=0){
        CLOG_INFO("cytool : ping-controller : (" << name_ << ")(" << turl_ << ") last latency [" << last_latency_ << "]");
      }else{
        CLOG_WARN("cytool : ping-controller : (" << name_ << ")(" << turl_ << ") : resulted in error : [" << last_err_ << "]");
      }
    }
  }
}

bool ControlLatency::isStarted(){

  if(cth_ == nullptr){
    return false;
  }
  return true;
  ping_destroy(po_);
  CLOG_DEBUG(4,"cytool : control-latency : (" << name_ << ")(" << turl_ << ") - ping instance removed");
  po_ = nullptr;
}
bool ControlLatency::Start()
{
  std::lock_guard<std::mutex> guardd(mtxd_);
  std::lock_guard<std::mutex> guardp(mtxp_);
  if(isStarted()){
    CLOG_WARN("cytool : control-latency : start : (" << name_ << ")(" << turl_ << ") seems to be launched");
    return false;
  }
  return start_();
}
bool ControlLatency::start_()
{
  cth_ = new(std::nothrow) WorkerThread(std::bind(&ControlLatency::ping_controller,this),name_);
  if(cth_ == nullptr){
    CLOG_ERROR("cytool : control-latency : (" << name_ << ")(" << turl_ << "): error creating controller thread! NO-MEM???");
    return false;
  }
  bool r;
  r = cth_->start(interval_);
  
  if(r == false){
    CLOG_ERROR("cytool : control-latency : start : error starting control thread!!!");
    delete cth_;
    cth_ = nullptr;
  }
  CLOG_DEBUG(5,"cytool : control-latency : (" << name_ << ")(" << turl_ << ") launched");
  return r;
}

bool ControlLatency::Stop(){
  std::lock_guard<std::mutex> guardd(mtxd_);
  std::lock_guard<std::mutex> guardp(mtxp_);
  if(cth_ == nullptr){
    CLOG_DEBUG(3,"cyool : control-latency : (" << name_ << ")(" << turl_ << ") seems not to be launched");
    return false;
  }
  return stop_();
}
bool ControlLatency::stop_()
{
  CLOG_DEBUG(5,"cytool : control-latency : (" << name_ << ")(" << turl_ << ") - stopping latency controller");
  cth_->stop();
  CLOG_DEBUG(3,"cytool : control-latency : (" << name_ << ")(" << turl_ << ") - stopped");
  delete cth_;
  CLOG_DEBUG(4,"cytool : control-latency : (" << name_ << ")(" << turl_ << ") - thread wrapper instance removed");
  cth_ = nullptr;
  return true;
}

}
}