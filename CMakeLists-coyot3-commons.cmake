



function (cyt_show_list cmessage cinput)
  string(REPLACE ";" "\n " cinput_mod "${cinput}")
  message("------ -")
  message("${cmessage} :")
  message("")
  message("${cinput_mod}")
  message("")
  message("-")
  message("")
endfunction()