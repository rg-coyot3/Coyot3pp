#pragma once

#include "cyt_swiss_knife_tools.hpp"
#include "JsonSerializablePacketBase.hpp"

#include <thread>
#include <functional>
#include <oping.h>

namespace coyot3{
namespace tools{


typedef std::vector<std::string> CytStringSet;

std::string  CytStringSetStringify(const CytStringSet& i);
std::string  CytStringSetStringify(const CytStringSet& i,const std::string& joinString);
CytStringSet CytStringSetCreate(const std::string& input,const char* key);

  struct CytStringSetJsIO 
  : public JsonSerializablePacketBase
  , public CytStringSet{
    CytStringSetJsIO();
    CytStringSetJsIO(const CytStringSet& o);
    virtual ~CytStringSetJsIO();

    virtual bool        from_json(const Json::Value& source) override;
    virtual Json::Value to_json() const override;

    //compatibility with cyt3macros
    static std::string get_serialization_model_template(int l__ = 0);
  };

Json::Value as_json(const coyot3::tools::JsonSerializablePacketBase& src);



/**
 * @brief : creates a worker thread that will invoke the loop over func
 * 
*/
class WorkerThread{
  public:
    WorkerThread(std::function<void()> func,const std::string& name = std::string("no-name"));
    virtual ~WorkerThread();

    void         setInterval(int milliseconds);
    /**
     * @brief triggers the 'func' even if it wait interval has not yet finished
    */
    void         triggerNow();
    bool         start();
    bool         start(int interval);
    bool         start_one_shot();
    /**
     * @brief stops the control thread and makes the invoker to wait until ops are done
     * @return false on error. true otherwise
    */
    bool         stop();
    bool         isRunning();

  protected:
    volatile bool           _flag;
    std::thread*            _th;
    std::function<void()>   _f;
    std::string             _name;
    int                     _interval;
    std::mutex              _w_mtx;
    std::condition_variable _w_cv;

};
typedef WorkerThread ControlThread;





class ControlLatency{
  public:
    static int default_max_historic;
    static int default_interval;
    static int default_timeout;
    ControlLatency(const std::string& targetUrl,const std::string& name = std::string());
    virtual ~ControlLatency();

    bool Start();
    bool Stop();
    bool isStarted();

    double getAverage();
    int    interval();
    int    interval(int i);
    int    timeout();
    int    timeout(int t);
    const std::list<int>& getHistoric();
    bool   showNotices();
    bool   showNotices(bool s);
  protected:
    
    void ping_controller();

    bool start_();
    bool stop_();
    

    std::string    name_;
    std::string    turl_;
    pingobj_t*     po_;
    WorkerThread* cth_;

    double           average_;
    std::list<int> latencies_;
    int              vector_max_length_;

    int             interval_;
    int             timeout_;
    bool            new_data_;
    
    int64_t         last_ping_ts;
    int64_t         last_latency_;
    
    std::mutex      mtxd_;
    std::mutex      mtxp_;

    std::string     last_err_;
    int64_t         last_err_ts_;
    
    bool            show_notices_;

    bool            is_valid_;
};




/**
 * @brief : a simple class to count processing time.
 * Methods : Start , Lap, Stop and Report
 * */
class LapClockLogger {
public:
    LapClockLogger(const std::string& name,bool silent = false);
    ~LapClockLogger();

    int64_t Start(const std::string& note= std::string("start"));
    int64_t Stop(const std::string& note = std::string("end"));
    int64_t Lap(const std::string& note = std::string("*"));
    void    Report();
    void Silent(bool s);
    void DebugLevel(int l);
protected:
    std::string n;
    int64_t time_start;
    int64_t lap;
    int64_t time_end;

    bool silent;
    int dlevel;


    std::vector<std::pair<std::string,int64_t>> laps;
};


/*
template <typename MapT1, typename MapT2>
class Mapper{
  public:
    Mapper();

    Mapper(const Mapper<MapT1,MapT2>& o){
      map_1_to_2 = o.map_1_to_2;
      map_2_to_1 = o.map_2_to_1;
    }
    virtual ~Mapper();

    bool insert(const MapT1& i1, const MapT2& i2){
      std::map<MapT1,MapT2>::iterator it12;
      std::map<MapT2,MapT1>::iterator it21;
      it12 = map_1_to_2.find(i1);
      it21 = map_2_to_1.find(i2);

      if(
          (it12 != map_1_to_2.end())
        ||(it21 != map_2_to_1.end())
      ){
        return false;
      }
      map_1_to_2.insert(std::make_pair(i1,i2));
      map_2_to_1.insert(std::make_pair(i2,i1));
      return true;
    }
    MapT1& operator[](const MapT1& i){MapT2 o;get(i,o);return o;}
    MapT2& operator[](const MapT2& i){MapT1 o;get(i,o);return o;}

    bool get(const MapT1& i, MapT2& o){
      std::map<MapT1,MapT2>::iterator it12 = map_1_to_2.find(i1);
      if(it12 == map_1_to_2.end()){return false;}
      o = map_1_to_2.second;
      return true;
    }
    bool get(const MapT2& i, MapT1& o){
      std::map<MapT2,MapT1>::iterator it21 = map_2_to_1.find(i1);
      if(it12 == map_2_to_1.end()){return false;}
      o = map_2_to_1.second;
      return true;
    }
    bool erase(const MapT1& i){
      std::map<MapT1,MapT2>::iterator it12;
      std::map<MapT2,MapT1>::iterator it21;
      it12 = map_1_to_2.find(i1);
      if(it12 == map_1_to_2.end()){return false;}
      it21 = map_2_to_1.find(it12->second);
      map_1_to_2.erase(it1);
      map_2_to_1.erase(it2);
      return true;
    }
    bool erase(const MapT2& i){
      std::map<MapT1,MapT2>::iterator it12;
      std::map<MapT2,MapT1>::iterator it21;
      it21 = map_2_to_1.find(i1);
      if(it12 == map_2_to_1.end()){return false;}
      it21 = map_1_to_2.find(it12->second);
      map_1_to_2.erase(it1);
      map_2_to_1.erase(it2);
      return true;
    }
  protected:
    std::map<MapT1,MapT2>     map_1_to_2;
    std::map<MapT2,MapT1>     map_2_to_1;
  private:
};
*/



// class ModuleBaseInterface{
//   public:
//     enum class State{
//       CREATED = 0,
//       INITIALIZED = 1,
//       STARTED     = 2,
//       STOPPED     = 3,
//       ERROR       = 4
//     };
//     const char* StateToString(State s);

//     ModuleBaseInterface();

//     State state() const;

//     virtual bool Init()   = 0;
//     virtual bool Start()  = 0;
//     virtual bool Stop()   = 0;
//     virtual bool Clear()  = 0;




//   protected:

//     State state(State s);

//   private:
//     State state_;


// };



}
}