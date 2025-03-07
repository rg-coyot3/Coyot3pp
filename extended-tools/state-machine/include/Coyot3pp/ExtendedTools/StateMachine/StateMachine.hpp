#pragma once


#include <boost/intrusive_ptr.hpp>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/asynchronous_state_machine.hpp>
#include <boost/statechart/fifo_scheduler.hpp>
#include <boost/statechart/event_processor.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/exception_translator.hpp>


#include <Coyot3pp/Cor3/Coyot3.hpp>

namespace sc = boost::statechart;
namespace mpl= boost::mpl;


namespace coyot3::extended::state_machine{



#define CYT3MACRO_boost_statechart_event_simple_declarations(eventName, extraArgumentType, eventDescription) \
struct eventName : public sc::event<eventName>{ \
  static constexpr const char* description = "eventDescription";\
  eventName(\
    IFN(extraArgumentType)(const extraArgumentType& parameter COMMA() )\
    const std::string& r = std::string("")); \
  const char* type(); \
  virtual ~eventName(); \
  std::string reason; \
  IFN(extraArgumentType)(extraArgumentType param;)\
};

#define CYT3MACRO_boost_statechart_event_simple_definitions(eventName, extraArgumentType, eventDescription) \
eventName::eventName(\
  IFN(extraArgumentType)(const extraArgumentType& parameter COMMA())\
  const std::string& r) \
:reason(r)\
{ \
  IFN(extraArgumentType)(param = parameter;)\
  eventName::custom_static_type_ptr(#eventName); \
} \
eventName::~eventName(){ \
  CLOG_DEBUG(6,#eventName" : event destructor"); \
} \
const char* eventName::type(){ \
  return custom_dynamic_type_ptr<char>(); \
}

#define CYT3MACRO_boost_statechart_event_list_declarations(...)\
  FOR_EACH_TRIPLES(CYT3MACRO_boost_statechart_event_simple_declarations, __VA_ARGS__)

#define CYT3MACRO_boost_statechart_event_list_definitions(...)\
  FOR_EACH_TRIPLES(CYT3MACRO_boost_statechart_event_simple_definitions, __VA_ARGS__)

namespace coyot3::state_machine{


  CYT3MACRO_model_class_declarations(
    StateMachineLogLine
    , 
    , ( 
        StateMachineLogLine(const std::string& d)
      , static constexpr  const int ERROR   = 0
      , static constexpr  const int WARN    = 1
      , static constexpr  const int INFO    = 2
      , static constexpr  const int DEBUG   = 3
    )
    , ( )
      , timestamp         , int64_t           , 
      , level             , int               , 3
      , description       , std::string       , 
  )

    CYT3MACRO_model_class_set_stack_declarations(StateMachineLogLine, 2000)
}

  #define CYT3MACRO_boost_statechart_header(stateMachineName, initialState)\
  \
    struct                         stateMachineName##StateMachine;\
    struct                         initialState;\
    typedef sc::fifo_scheduler<>   stateMachineName##FifoScheduler;\
    typedef std::allocator<void>   stateMachineName##Allocator;

  /**
   * @brief : REMEMBER: THIS NEEDS TO BE DECLARED AT THE GLOBAL_NAMESPACE
   * 
   */
  #define CYT3MACRO_boost_statechart_implementation_declarations(stateMachineName, initialState)\
  namespace boost{\
  namespace statechart{\
    template<>\
    inline void asynchronous_state_machine<\
       stateMachineName##StateMachine\
      ,initialState\
      ,stateMachineName##FifoScheduler\
      ,stateMachineName##Allocator\
    >::initiate_impl(){}\
  }}


    #define cytemacro_boost_statechart_machine_declarations_additionalmp_(stateMachinePropertyOrMethod)\
      stateMachinePropertyOrMethod ; 

  #define CYT3MACRO_boost_statechart_machine_declarations(stateMachineName, initialState, ownerClass, additionalMethodsAndProperties)\
  struct stateMachineName##StateMachine\
  : public sc::asynchronous_state_machine<\
              stateMachineName##StateMachine\
      COMMA() initialState\
      COMMA() stateMachineName##FifoScheduler\
      COMMA() stateMachineName##Allocator>\
  {\
    public:\
      stateMachineName##StateMachine(my_context ctx, ownerClass* owner);\
      virtual ~stateMachineName##StateMachine();\
      \
      const std::string& current_state();\
      void  log(const std::string& desc, int level = coyot3::extended::state_machine::StateMachineLogLine::DEBUG);\
    \
    protected:\
      std::string stateMachineState;\
        void          calculate_state_string();\
    \
      coyot3::extended::state_machine::StateMachineLogLineStack      logs_;\
    private:\
      virtual void initiate_impl();\
    \
    public:\
      FOR_EACH(cytemacro_boost_statechart_machine_declarations_additionalmp_,PASS_PARAMETERS(additionalMethodsAndProperties))\
  };



  // struct StOutOfCircuit;
  // struct StMaNotAvailable;
  
  // struct StConnected 
  // : public sc::state<
  //                 StConnected
  //                 ,VehicleObjectCloudStateMachine
  //                 ,mpl::list<StOutOfCircuit, StMaNotAvailable>
  //                 //,sc::has_deep_history // when the vehicle recovers connection, do we want it to get to the last remembered state?
  // >{
  //     typedef mpl::list< 
  //     sc::custom_reaction<EvDisconnected>
      
  //     ,sc::custom_reaction<EvForcedDisconnection>
  //   > reactions;

  //   /**
  //    * @brief : entering or re-entering the state CONNECTED will check the 
  //    *  millaVehicle.status.arrival, and the distance to the last known destination.
  //    * */
  //   StConnected(my_context ctx);
  //   virtual ~StConnected();
    
    
  //   sc::result react(const EvDisconnected& e);

  //   sc::result react(const EvForcedDisconnection& e);
  // };



      #define cytemacro_boost_statechart_state_transitions_extraitem_decl(eventNameE, futureStateE)\
          (sc::custom_reaction < eventNameE >)

    #define cytemacro_boost_statechart_state_transitions_decl(...)\
      CHAIN_COMMA(\
        FOR_EACH_PAIR(cytemacro_boost_statechart_state_transitions_extraitem_decl, __VA_ARGS__)\
      )


    #define cytemacro_boost_statechart_state_reactions_decl(eventType, futureState)\
      sc::result react(const eventType& e);\

  #define CYT3MACRO_boost_statechart_state_declarations(\
                    stateName, \
                    parentStateOrFsm, \
                    defaultChildState, \
                    extraParams,\
                    extraPropsAndMethods,\
                    ...)\
  struct stateName\
  : public sc::state<\
    stateName\
    COMMA() parentStateOrFsm\
    IFN(defaultChildState)(COMMA() defaultChildState)\
    >{\
      typedef mpl::list<\
        cytemacro_boost_statechart_state_transitions_decl(__VA_ARGS__)\
      > reactions;\
      \
      \
      stateName(my_context ctx);\
      virtual ~stateName();\
      \
      FOR_EACH_PAIR(cytemacro_boost_statechart_state_reactions_decl, __VA_ARGS__)\
    };







          #define cytemacro_boost_statechart_state_commons_def_destr_extrap02(destructorExtra,...)\
            IFN(destructorExtra)(destructorExtra();)
        
        #define cytemacro_boost_statechart_state_commons_def_destr_extrap01(a1,...)\
          IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap02(__VA_ARGS__))

      #define cytemacro_boost_statechart_state_commons_def_destr_extrap00(a1,...)\
        IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap01(__VA_ARGS__))

    #define cytemacro_boost_statechart_state_commons_def_destr_extrap(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap00(__VA_ARGS__))


        #define cytemacro_boost_statechart_state_commons_def_constr_extrap01(constructionExtra,...)\
          IFN(constructionExtra)(constructionExtra();)

      #define cytemacro_boost_statechart_state_commons_def_constr_extrap00(a1,...)\
        IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap01(__VA_ARGS__))

    #define cytemacro_boost_statechart_state_commons_def_constr_extrap(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap00(__VA_ARGS__))


    #define cytemacro_boost_statechart_state_commons_def_constr_destr(stateName, parentStateOrFsm, defaultChildState, extraParams, extraPropsAndMethods, ...)\
      stateName::stateName(my_context ctx)\
      :my_base(ctx){\
        stateName::custom_static_type_ptr(#stateName);\
        IFN(extraParams)(cytemacro_boost_statechart_state_commons_def_constr_extrap(PASS_PARAMETERS(extraParams)))\
      }\
      stateName::~stateName(){\
        IFN(extraParams)(cytemacro_boost_statechart_state_commons_def_destr_extrap(PASS_PARAMETERS(extraParams)))\
      }

    
    
    #define cytemacro_boost_statechart_state_commons_def_reactions_auto(stateName, eventName, futureState)\
      IFN(futureState)(sc::result stateName::react(const eventName& e){return transit<futureState>();})


    #define CYT3MACRO_boost_statechart_state_definitions(\
        stateName, \
        parentStateOrFsm, \
        defaultChildState, \
        extraParams,\
        extraPropsAndMethods,\
        ...)\
      \
      cytemacro_boost_statechart_state_commons_def_constr_destr(stateName, parentStateOrFsm, defaultChildState, extraParams, extraPropsAndMethods, __VA_ARGS__)\
      \
      FOR_EACH_PAIR_WITH_CONSTANT(cytemacro_boost_statechart_state_commons_def_reactions_auto, stateName, __VA_ARGS__)\


    
      

////



  
  #define CYT3MACRO_boost_statechart_owner_cyt3models_declarations(stateMachineName)\
    stateMachineName##FifoScheduler                    stateMachineName##Instance;\
    stateMachineName##FifoScheduler::processor_handle  stateMachineName##ProcessorHandle;\
    bool                                               stateMachineName##AcceptEvents_;\
    bool                                               stateMachineName##Create();\
    coyot3::extended::state_machine::StateMachineLogLineStack      stateMachineName##_logs;\
    void    stateMachineName@@Trace(const std::string& desc, int level = coyot3::extended::state_machine::StateMachineLogLine::DEBUG);\
    \
    template<class T>\
    void stateMachineName_push_event(T* event)\
    {\
      std::stringstream ss;\
      ss << "pushing event to state machine : " << std::string(event->type()) \
        << ":" << event->reason;\
      stateMachineNameTrace(ss.str());\
      stateMachineName##Instance.queue_event(state_machine_proc_handle,IntrusivePtr(event));\
      stateMachineName##Instance();\
    }


