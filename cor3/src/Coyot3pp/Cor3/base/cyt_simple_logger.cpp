#include <Coyot3pp/Cor3/base/cyt_simple_logger.hpp>

namespace coyot3{
namespace logger{

int SimpleLoggerClass::GlobalLogDebugLevel = CY_SIMPLELOGGER_DEFAULT_DEBUG_LEVEL;


std::string _datetime()
{
  
  int64_t utc_timestamp_msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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

const char* SimpleLoggerClass::LogLineTypeToString(SimpleLoggerClass::LogLineType t)
{
  switch(t)
  {
    

    case LogLineType::INFO:return "INFO";break;
    case LogLineType::WARNING:return "WARNING";break;
    case LogLineType::ERROR:return "ERROR";break;
    case LogLineType::DEBUG:return "DEBUG";break;
    default:
      return "type-unknown";
  }
}
SimpleLoggerClass::SimpleLoggerClass(SimpleLoggerClass::LogLineType tp,int tid)
:slcstream()
,t(tp)
,_tid(tid)
{
    slcstream << _datetime() << " : " << LogLineTypeToString(tp) << ": ";
}
SimpleLoggerClass::SimpleLoggerClass(SimpleLoggerClass::LogLineType tp,const std::string& header)
:slcstream()
,t(tp)
{
    slcstream << header;
}




#if CY_LOGGER_WRAPPER_MODEL == CY_LOGGER_WRAPPER_FOR_STDOUT

SimpleLoggerClass::~SimpleLoggerClass()
{
  switch(t)
  {
    case LogLineType::INFO:
        std::cout << __SIMPLELOGGER_COLOR_GREEN_ << slcstream.str() << __SIMPLELOGGER_STDOUTCOLORS_RESET_ << std::endl;
      break;
    case LogLineType::WARNING:
        std::cout << __SIMPLELOGGER_COLOR_YELLOW_ << slcstream.str() << __SIMPLELOGGER_STDOUTCOLORS_RESET_<< std::endl;
      break;
    case LogLineType::ERROR:
        std::cout << __SIMPLELOGGER_COLOR_RED_ << slcstream.str() << __SIMPLELOGGER_STDOUTCOLORS_RESET_<< std::endl;
      break;
    case LogLineType::DEBUG:
        std::cout << __SIMPLELOGGER_COLOR_BLUE_ << slcstream.str() << __SIMPLELOGGER_STDOUTCOLORS_RESET_<< std::endl;
      break;
    default:
          std::cout << "ERROR SIMPLE LOGGER " << std::endl;
  }
}
#endif




}//end of namespace logger
}//end of namespace coyot3




// #if CY_LOGGER_WRAPPER_MODEL == CY_LOGGER_WRAPPER_FOR_NCURSESWRAP

// #include <Coyot3pp/Cor3/cyt_console_kit.hpp>


// namespace coyot3{
// namespace logger{

// SimpleLoggerClass::~SimpleLoggerClass()
// {
//   switch(t)
//   {
//     case LogLineType::INFO:
//           Terminal::appendInfo(slcstream,_tid);
//         break;
//     case LogLineType::WARNING:
//           Terminal::appendWarn(slcstream,_tid);
//         break;
//     case LogLineType::ERROR:
//           Terminal::appendError(slcstream,_tid);
//         break;
//     case LogLineType::DEBUG:
//           Terminal::appendDebug(slcstream,_tid);
//         break;
//     default:
//       std::cout << "ERROR SIMPLE LOGGER " << std::endl;
//   }
// }




// }
// namespace tools{


//   //estáticos - fin 
  
  
  

//   namespace Terminal{
//     int module_terminal_index = 0;
    
//     bool InitializeModule(int module_index)
//     {
//       module_terminal_index = module_index;
//     }
//     class ConsoleNcursesWrapper{
//       public:


//         static void getDimensions(int &x,int &y);

//         static bool setCursorPosition(unsigned int c,unsigned int r);


//         ConsoleNcursesWrapper(int,int);
//         virtual ~ConsoleNcursesWrapper();


//         bool createCells(int nrows,int ncols);
//         bool Init();
//         bool Start();
//         bool Stop();

//         void refresh();
//         bool appendText(std::ostringstream& s,int cpair,int windex = 0);
//       protected:

//         PANEL**  panels;
//         WINDOW** windows;

//         int64_t  last_refresh_ts;

//         bool     initialized;

//         int _rows;
//         int _columns;
//         int _num_cells;
//         int _min_refresh_time;
//     };
//     /* STATICS  - BEGIN*/


//     /* STATICS - END*/

//     ConsoleNcursesWrapper::ConsoleNcursesWrapper(int rows,int columns)
//     :panels(nullptr)
//     ,windows(nullptr)
//     ,last_refresh_ts(0)
//     ,initialized(false)

//     ,_rows(rows)
//     ,_columns(columns)
//     ,_num_cells(0)
//     ,_min_refresh_time(50)
//     {

//     }
//     ConsoleNcursesWrapper::~ConsoleNcursesWrapper()
//     {
//       Stop();
//     }
//     void ConsoleNcursesWrapper::refresh()
//     {
//       if(!initialized)
//         return;
//       int64_t n = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//       if((n - last_refresh_ts) < _min_refresh_time)
//       {
//         return;
//       }
//       last_refresh_ts = n;
//       update_panels();
//       doupdate();
//     }
//     bool ConsoleNcursesWrapper::Init(){
//       initscr();
//       cbreak();
//       noecho();
//       start_color();
//       init_pair(CCONSOLE_COLOR_INFO,COLOR_GREEN,COLOR_BLACK);
//       init_pair(CCONSOLE_COLOR_WARN,COLOR_YELLOW,COLOR_BLACK);
//       init_pair(CCONSOLE_COLOR_ERROR,COLOR_RED,COLOR_BLACK);
//       init_pair(CCONSOLE_COLOR_DEBUG,COLOR_CYAN,COLOR_BLACK);
//       createCells(_rows,_columns);
//     }
//     bool ConsoleNcursesWrapper::Start(){
      
//     }
//     bool ConsoleNcursesWrapper::appendText(std::ostringstream& s,int cpair,int windex)
//     {
//       if(windex>_num_cells)
//       {
//         windex = 0;
//       }

//       wprintw(windows[windex],"%s\n",s.str().c_str());
//       refresh();
//     }
//     bool ConsoleNcursesWrapper::Stop(){
//       endwin();
//     }
//     bool ConsoleNcursesWrapper::createCells(int nrows,int ncols)
//     {
//       int dx,dy,cdx,cdy,cpx,cpy;
//       if( (nrows< 1) || (ncols < 1))
//       {
//         return false;
//       }
//       int ninstances = nrows*ncols;
//       _num_cells = ninstances-1;
//       panels = new(std::nothrow) PANEL*[ninstances];
//       windows= new(std::nothrow) WINDOW*[ninstances];
//       getmaxyx(stdscr,dy,dx);
//       cdx = dx / ncols;
//       cdy = (dy-1) / ncols;

//       cpx = cpy = 0;
//       for(int i = 0;i<ninstances;++i)
//       {
//         windows[i] = newwin(cdy,cdx,cpy,cpx);
//         scrollok(windows[i],true);
//         panels[i]  = new_panel(windows[i]);
//         if(i!=0)
//         {
//           set_panel_userptr(panels[i-1],panels[i]);
//         }

        
//         if((cpx+cdx)>(dx-1))
//         {
//           cpy+=cdy;
//           cpx = 0;
//         }else{
//           cpx+=cdx;
//         }
//         //si el número de ventanas es impar, la última(abajo)
//         // tendrá toda la dimensión de X.
//         if(((ninstances%2) == 0)&& (i == ninstances-2))
//         {
//           cdx = dx;
//         }
//       }
//       set_panel_userptr(panels[ninstances-1],panels[0]);
//       initialized = true;
//       refresh();
//       return true;

      
//     }


//     ConsoleNcursesWrapper* _console = nullptr;


//     bool Initialize(int cols,int rows){
//       if(_console != nullptr)
//       {
//         return false;
//       }
//       _console = new(std::nothrow) ConsoleNcursesWrapper(cols,rows);
//       if(!_console)
//       {
//         std::cout << "ERROR : CONSOLE NCURSES ERROR : IMPOSSIBLE TO CREATE "
//           ": NO MEM??" << std::endl;
//         return false;
//       }
//       _console->Init();
//       _console->Start();

//       return true;
//     }

//     bool appendText(std::ostringstream& s,int cpair,int windex)
//     {
//       if(!_console)
//       {
//         if(!Initialize(1,1))
//         {
//           std::cout << "CYT CONSOLE : append text : ERROR INITIALIZING!!";
//           return false;
//         }
//       }
//       _console->appendText(s,cpair,windex);
//     }

//     bool appendInfo(std::ostringstream& s,int windex){
//       attron(COLOR_PAIR(CCONSOLE_COLOR_INFO));
//       appendText(s,CCONSOLE_COLOR_INFO,windex);
//       attroff(COLOR_PAIR(CCONSOLE_COLOR_INFO));
//     }
//     bool appendWarn(std::ostringstream& s,int windex){
//       attron(COLOR_PAIR(CCONSOLE_COLOR_WARN));
//       appendText(s,CCONSOLE_COLOR_WARN,windex);
//       attroff(COLOR_PAIR(CCONSOLE_COLOR_WARN));
//     }
//     bool appendError(std::ostringstream& s,int windex){
//       attron(COLOR_PAIR(CCONSOLE_COLOR_ERROR));
//       appendText(s,CCONSOLE_COLOR_ERROR,windex);
//       attroff(COLOR_PAIR(CCONSOLE_COLOR_ERROR));
//     }
//     bool appendDebug(std::ostringstream& s,int windex){
//       attron(COLOR_PAIR(CCONSOLE_COLOR_DEBUG));
//       appendText(s,CCONSOLE_COLOR_DEBUG,windex);
//       attroff(COLOR_PAIR(CCONSOLE_COLOR_DEBUG));
//     }



//  }//END TERMINALS


// }//end of namespace tools

// }//end of namespace coyot3


// #endif