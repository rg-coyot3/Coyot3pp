#include <milla_connect_tools/ros2_node_interface/Ros2DiagnosticsBook.h>


namespace milla{
namespace connapp{
namespace ros2{



DiagnosticsBookRos2::DiagnosticsBookRos2()
:DiagnosticsBookItem()
{

}
DiagnosticsBookRos2::DiagnosticsBookRos2(const std::string& t)
:DiagnosticsBookItem(t)
{
  
}


DiagnosticsBookRos2::~DiagnosticsBookRos2(){
 
}

DiagnosticsBookRos2::DiagnosticsBookRos2(const DiagnosticsBookItem& o)
:DiagnosticsBookItem(o)
{
  
}


bool DiagnosticsBookRos2::update_properties_from_diag(const diagnostic_msgs::msg::DiagnosticStatus& d)
{
  bool ok = true;
  switch(d.level)
  {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      level(DiagnosticLevel::OK);
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      level(DiagnosticLevel::WARN);
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      level(DiagnosticLevel::ERROR);
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      level(DiagnosticLevel::DATA_TIMEOUT);
      break;
    default:
      level(DiagnosticLevel::ERROR_DATA_SOURCE);     
      ok = false;
  }

  KeyValueMap::iterator it;
  for(const diagnostic_msgs::msg::KeyValue& p : d.values)
  { 
    value(p.key,p.value);
  }
  name(d.name);
  message(d.message);
  update_now();
  return ok;
}

bool DiagnosticsBookRos2::update(const diagnostic_msgs::msg::DiagnosticArray& rdiagarr)
{
  int64_t current_ts = (rdiagarr.header.stamp.sec * 1000) 
                      + static_cast<int64_t>(rdiagarr.header.stamp.nanosec / 1000000);
  update_ts_ = current_ts; 
  
  bool ok = true;
  for(const diagnostic_msgs::msg::DiagnosticStatus& s : rdiagarr.status)
  {
    ok &= update(s);
  }
  return ok;
}

bool DiagnosticsBookRos2::update(const diagnostic_msgs::msg::DiagnosticStatus& rdiag)
{

  // CLOG_INFO("diagnostics-book-ros : update : updating for ["<<rdiag.name << "]");
  std::vector<std::string> tags;
  if(milla::connapp::stringSplit(rdiag.name,"/",tags) == 0){
    CLOG_WARN("diagnostic-data-source : update(diag-status) : error obtaining name!");
    return false;
  }
  DdsSiblingsMap::iterator it;
  if(child_get(tags[0],it) == false)
  
  {
    //create new
    DiagnosticsBookItem n(tags[0]);
    child_set(n);
    it = children_.find(tags[0]);
  }
  update_now();
  return update_engine_(rdiag,it,tags);
  
}


bool DiagnosticsBookRos2::update_engine_(const diagnostic_msgs::msg::DiagnosticStatus& r
                                        ,DdsSiblingsMap::iterator& it
                                        ,const std::vector<std::string>& tags
                                        ,int depth)
{
  bool ok = true;
  // CLOG_INFO("diagnostics-book-ros : update-engine(d=" << depth << ") : "
  //   "updating for ["<< r.name << "](t=" << tags[depth] << ")/(" << it->first << ") : ");

  if(depth < (tags.size() - 1))
  {
    //search inside
    DdsSiblingsMap::iterator itc;
    if(it->second.child_get(tags[depth+1],itc)== false)
    {
      //create new and point it with itc
      // CLOG_INFO("diagnostics-book-ros : update-engine(d=" << depth << ") : "
      //   "updating for ["<< r.name << "](t=" << tags[depth] << ")/(" 
      //   << it->first << ") : creating item [" << tags[depth+1] << "]");
      DiagnosticsBookRos2 n(tags[depth+1]);
      if((depth+1) == (tags.size() - 1))
      {

        // CLOG_INFO("diagnostics-book-ros : update-engine(d=" << depth << ") : "
        //   "updating for ["<< r.name << "](t=" << tags[depth] << ")/(" 
        //   << it->first << ") : creating item [" << tags[depth+1] 
        //   << "] - previewing edge [" << r.name << "]");
        n.name(r.name);
        n.hardid(r.hardware_id);
        
      }
      it->second.child_set(n);
      it->second.child_get(n.tag(),itc);
    }
    return update_engine_(r,itc,tags,depth+1);
  }
  DiagnosticsBookRos2 dr1;
  dr1 = it->second;
  dr1.update_properties_from_diag(r);
  it->second = dr1;

  CLOG_DEBUG(7,"diag-data-source : update-engine- : updating for diagnostic "
    "msg : (" << r.name << ":" << tags[depth] << ")")
  return true;
  
  
}

bool DiagnosticsBookRos2::update_as_list(const diagnostic_msgs::msg::DiagnosticArray& rdiagarr)
{
  int64_t current_ts = (rdiagarr.header.stamp.sec* 1000) 
                      + static_cast<int64_t>(rdiagarr.header.stamp.nanosec / 1000000);
  update_ts_ = current_ts; 
  
  bool ok = true;
  for(const diagnostic_msgs::msg::DiagnosticStatus& s : rdiagarr.status)
  {
    ok &= update_as_list(s);
  }
  return ok;
}
bool DiagnosticsBookRos2::update_as_list(const diagnostic_msgs::msg::DiagnosticStatus& rdiag)
{
  DdsSiblingsMap::iterator it;
  DiagnosticsBookRos2 n;
  if(child_get(rdiag.name,it) == false)
  {
    n.tag(rdiag.name);
    n.name(rdiag.name);
    n.hardid(rdiag.hardware_id);
    child_set(n);
    child_get(rdiag.name,it);
  }else{
    n = it->second;
  }
  n.update_properties_from_diag(rdiag);
  it->second = n;
  return true;
}

}
}
}