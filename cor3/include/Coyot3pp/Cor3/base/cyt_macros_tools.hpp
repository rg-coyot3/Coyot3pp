#pragma once
#include "cyt_foreach_macro_helpers.hpp"
#include "cyt_macro_enum_class.hpp"
#include <functional>

//todo

#define CYT3MACRO_OPS_BOOLS_SEQUENCE(CYT_log_prefix, CYT_operations,...) \
  if( ((CYT_operations) == false) ){\
    CLOG_WARN( CYT_log_prefix ": operation : "}