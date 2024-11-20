#pragma once



#include "Ros2NodeInterfaceBase.h"
#include <milla_connect_tools/tools/cyt_diagnostics_book.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace milla{
namespace connapp{
namespace ros2{

  class DiagnosticsBookRos2
  :public milla::connapp::DiagnosticsBookItem{
    public:

      DiagnosticsBookRos2();
      DiagnosticsBookRos2(const std::string& t);
      DiagnosticsBookRos2(const DiagnosticsBookItem& o);
      virtual ~DiagnosticsBookRos2();

      bool update_as_list(const diagnostic_msgs::msg::DiagnosticArray& rdiagarr);
      bool update_as_list(const diagnostic_msgs::msg::DiagnosticStatus& rdiag);
      bool update(const diagnostic_msgs::msg::DiagnosticArray&  rdiagarr);
      bool update(const diagnostic_msgs::msg::DiagnosticStatus& rdiag);

      bool                  update_properties_from_diag(const diagnostic_msgs::msg::DiagnosticStatus& d);
    protected:


      bool update_engine_(
         const diagnostic_msgs::msg::DiagnosticStatus& rdiag
        ,DdsSiblingsMap::iterator& it
        ,const std::vector<std::string>& tags
        ,int depth = 0);

      

    private:




  };


}
}
}