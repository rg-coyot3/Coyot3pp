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

//namespace mpl = boost::mpl;
//namespace sc  = boost::statechart;

template< class T>
boost::intrusive_ptr< T > IntrusivePtr(T* pointer){
  return boost::intrusive_ptr<T>(pointer);
}


/**
 * @brief to be invoked at .hpp . 
 * @param eventName : required : name of the event
 * @param extraArgumentType : not required : data type : if defined, will include the an 
 *            extra parameter of this type. The constructor of the event will 
 *            need it. The parameter will be accessible with the property name 'param'
 * @param eventDescription : not required : string describing the reason of the
 *            creation of the event. Accessible as const char* throw the parameter
 *            'description'.
 */

#define CYT3MACRO_boost_statechart_event_simple_declarations(eventName, extraArgumentType, eventDescription) \
struct eventName : public boost::statechart::event<eventName>{ \
  static constexpr const char* description =  #eventDescription ;\
  eventName(\
    IFN(extraArgumentType)(const extraArgumentType& parameter COMMA() )\
    const std::string& r = std::string("")); \
  const char* type(); \
  virtual ~eventName(); \
  std::string reason; \
  IFN(extraArgumentType)(extraArgumentType param;)\
};


/**
 * @brief idem as declarations
 * 
 */
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
  CLOG_DEBUG(6,#eventName" : event destructor") \
} \
const char* eventName::type(){ \
  return custom_dynamic_type_ptr<char>(); \
}



/**
 * @brief to ease the declaration of an events list. The variadic arguments are
 *  interpreted as triplets. Each triplet is:
 *    1st : eventName
 *    2nd : extraArgumentType
 *    3rd : eventDescription
 */
#define CYT3MACRO_boost_statechart_event_list_declarations(...)\
  FOR_EACH_TRIPLES(CYT3MACRO_boost_statechart_event_simple_declarations, __VA_ARGS__)


/**
 * @brief same as declarations.
 * 
 */
#define CYT3MACRO_boost_statechart_event_list_definitions(...)\
  FOR_EACH_TRIPLES(CYT3MACRO_boost_statechart_event_simple_definitions, __VA_ARGS__)



// /**
//  * @brief 
//  *  , stateName
//  *  , parentState
//  *  , ortogonalOrder
//  *  , innerInitialState : optional
//  *  , additionalMethods & vars
//  *    ... transitions
//  */
// #define CYT3MACRO_boost_statechart_state_declarations(stateName,parentState, ortogonalOrder, innerState, additionalMethodsVars, ...)\
//   struct stateName\
//   : sc::state<\
//     stateName ,\
//     parentState IFN(ortogonalOrder)(::orthogonal< ortogonalOrder >)>\
//     IFN(innerInitialState)(COMMA() innerInitialState)\
//   >{\
//   \
//     stateName(my_context ctx);\
//     virtual ~stateName();\
//   };







      #define cyt3macro_boost_statechart_state_transitions_extraitem_decl(eventNameE, futureStateE)\
          (boost::statechart::custom_reaction < eventNameE >)

    #define cyt3macro_boost_statechart_state_transitions_decl(...)\
      CHAIN_COMMA(\
        FOR_EACH_PAIR(cyt3macro_boost_statechart_state_transitions_extraitem_decl, __VA_ARGS__)\
      )


    #define cyt3macro_boost_statechart_state_reactions_decl(eventType, futureState)\
      boost::statechart::result react(const eventType& e);\


          #define cytemacro_boost_statechart_state_extraparams_proc_destr_decl_(destructorExtra, ...)\
            IFN(destructorExtra)(void destructorExtra();)\
            

        #define cytemacro_boost_statechart_state_extraparams_proc_constr_decl_(constructorExtra,...)\
          IFN(constructorExtra)(void constructorExtra();)\
          IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_extraparams_proc_destr_decl_(__VA_ARGS__))

      #define cytemacro_boost_statechart_state_extraparams_proc_ignoref_decl_(paramHistory,...)\
        IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_extraparams_proc_constr_decl_(__VA_ARGS__))

    #define cytemacro_boost_statechart_state_extraparams_proc_decl_(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_extraparams_proc_ignoref_decl_(__VA_ARGS__))

      #define cytemacro_boost_statechart_state_extraparams_header_proc_decl_(extraParam, ...)\
        IFN(extraParam)(COMMA() extraParam)

    #define cytemacro_boost_statechart_state_extraparams_header_decl_(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_extraparams_header_proc_decl_(__VA_ARGS__))

    
      #define cytemacro_boost_statechart_state_extraparamsmethods_proc_item_decl_(mp)\
        mp;
    
    #define cytemacro_boost_statechart_state_extraparamsmethods_proc_decl_(...)\
      IFN(__VA_ARGS__)(FOR_EACH(cytemacro_boost_statechart_state_extraparamsmethods_proc_item_decl_,__VA_ARGS__))




  /**
   * @brief declarations for a boost state machine state instance.
   * @param stateName         : name of the state-struct
   * @param parentStateOrFsm  : parent state-struct or state-machine-struct
   * @param defaultChildState : optional : if contains inner states, it points to the internal one.
   * @param extraParams : [ 
   *                        param-history,     : not-required : if not empty, it activates param-history.
   *                        constructor extra, : not-required : if not empty, 
   *                                             it will invoke to void '<constructor-extra>()' 
   *                                             function at the constitution of the state
   *                        destructor extra,  : not-required : if not empty, it will 
   *                                             invoke to the void <destructor-extra>() function
   *                        event-confirm-transition-name : not-required : 
   *                                           
   *                      ]
   * @param extraPropsAndMethods : extra methods to be declared at the state-struct
   *                      the user must define them after.
   * @param variadic_arguments : the reactions of the state-struct, defined as 
   *                      a set of pairs where each pair:
   *          1st argument : event to react.
   *          2nd argument : if defined  : final state-struct
   *                         not defined : will be defined as a custom reaction.
   * 
   */
  #define CYT3MACRO_boost_statechart_state_declarations(\
                    stateName, \
                    parentStateOrFsm, \
                    defaultChildState, \
                    extraParams,\
                    extraPropsAndMethods,\
                    ...)\
  struct stateName\
  : public boost::statechart::state<\
    stateName\
    COMMA() parentStateOrFsm\
    IFN(defaultChildState)(COMMA() defaultChildState)\
    cytemacro_boost_statechart_state_extraparams_header_decl_(PASS_PARAMETERS(extraParams))\
    >{\
      typedef boost::mpl::list<\
        cyt3macro_boost_statechart_state_transitions_decl(__VA_ARGS__)\
      > reactions;\
      \
      \
      stateName(my_context ctx);\
      virtual ~stateName();\
      \
      cytemacro_boost_statechart_state_extraparams_proc_decl_(PASS_PARAMETERS(extraParams))\
      \
      cytemacro_boost_statechart_state_extraparamsmethods_proc_decl_(PASS_PARAMETERS(extraPropsAndMethods))\
      \
      FOR_EACH_PAIR(cyt3macro_boost_statechart_state_reactions_decl, __VA_ARGS__)\
    };



// -----------------------------------------------------------------------------


          #define cytemacro_boost_statechart_state_commons_def_destr_extrap02(destructorExtra,...)\
            IFN(destructorExtra)(destructorExtra();)
        
        #define cytemacro_boost_statechart_state_commons_def_destr_extrap01(a1,...)\
          IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap02(__VA_ARGS__))

      #define cytemacro_boost_statechart_state_commons_def_destr_extrap00(a1,...)\
        IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap01(__VA_ARGS__))

    #define cytemacro_boost_statechart_state_commons_def_destr_extrap(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_destr_extrap00(__VA_ARGS__))



            #define cytemacro_boost_statechart_state_commons_def_constr_extrap03(TransitionConfirmationEventReason,...)\
            post_event(IntrusivePtr(new coyot3::tools::statemachine::EvStateMachineTransitionConfirmation(TransitionConfirmationEventReason)));

          #define cytemacro_boost_statechart_state_commons_def_constr_extrap02(destext, ...)\
          IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap03(__VA_ARGS__))

        #define cytemacro_boost_statechart_state_commons_def_constr_extrap01(constructionExtra,...)\
          IFN(constructionExtra)(constructionExtra();)\
          IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap02(__VA_ARGS__))

      #define cytemacro_boost_statechart_state_commons_def_constr_extrap00(a1,...)\
        IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap01(__VA_ARGS__))

    #define cytemacro_boost_statechart_state_commons_def_constr_extrap(...)\
      IFN(__VA_ARGS__)(cytemacro_boost_statechart_state_commons_def_constr_extrap00(__VA_ARGS__))


    #define cytemacro_boost_statechart_state_commons_def_constr_destr(stateName, parentStateOrFsm, defaultChildState, extraParams, extraPropsAndMethods, ...)\
      stateName::stateName(my_context ctx)\
      :my_base(ctx){\
        stateName::custom_static_type_ptr( #stateName );\
        outermost_context().st_confirm_transition(stateName::custom_static_type_ptr<char>());\
        IFN(extraParams)(cytemacro_boost_statechart_state_commons_def_constr_extrap(PASS_PARAMETERS(extraParams)))\
      }\
      stateName::~stateName(){\
        IFN(extraParams)(cytemacro_boost_statechart_state_commons_def_destr_extrap(PASS_PARAMETERS(extraParams)))\
      }

    
    
    #define cytemacro_boost_statechart_state_commons_def_reactions_auto(stateName, eventName, futureState)\
      IFN(futureState)(boost::statechart::result stateName::react(const eventName& e){\
          outermost_context().transition_reason_add(stateName::custom_static_type_ptr<char>());\
        return transit<futureState>();})


    /**
     * @brief same as for declarations. 
     * 
     */
    #define CYT3MACRO_boost_statechart_state_definitions(\
        stateName, \
        parentStateOrFsm, \
        defaultChildState, \
        extraParams,\
        extraPropsAndMethods,\
        ...)\
      \
      cytemacro_boost_statechart_state_commons_def_constr_destr(stateName, \
                                                                parentStateOrFsm, \
                                                                "-", \
                                                                extraParams, \
                                                                extraPropsAndMethods, \
                                                                __VA_ARGS__)\
      \
      FOR_EACH_PAIR_WITH_CONSTANT(cytemacro_boost_statechart_state_commons_def_reactions_auto, stateName, __VA_ARGS__)\

// -----------------------------------------------------------------------------


  /**
   * @brief to be included at the owner class.
   * @param stateMachineOwnerClass as in the declaration of the state machine
   * @param stateMachinePropertyName as wanted to be named for the containing class
   * @param stateMachineNamespace namespace where the state machine class can be found.
   * 
   */
  #define CYT3MACRO_boost_statechart_owner_class_declarations(\
    stateMachineOwnerClass,\
    stateMachinePropertyName,\
    stateMachineNamespace,\
    ...\
  )\
  \
  stateMachineNamespace::stateMachineOwnerClass##StateMachineFifoScheduler stateMachinePropertyName;\
  stateMachineNamespace::stateMachineOwnerClass##StateMachineFifoScheduler::processor_handle stateMachinePropertyName##_proc_handle;\
  \
  bool  push_event_ignore_push_         = false;\
  bool  push_event_debug_traces_show_   = false;\
  bool  permit_events() const;\
  bool  permit_events(bool p);\
  bool  push_traces() const;\
  bool  push_traces(bool p);\
  \
  bool  create_state_machine();\
  \
  template<class T>\
  void push_state_machine_event_(T* event){\
    std::stringstream ss;\
    if(push_event_ignore_push_ == true){CLOG_WARN(#stateMachineOwnerClass \
      "StateMachine : push-state-machine-event : ignoring event push [" \
      << std::string(event->type()) << "]")\
      delete event;\
      return;\
    }\
    \
    if(push_event_debug_traces_show_ == true){\
      ss << "pushing event to state machine : " << std::string(event->type()) \
      << ":" << event->reason;\
      CLOG_INFO(#stateMachineOwnerClass "StateMachine : push-state-machine-event : " \
        << ss.str());\
    }\
    stateMachinePropertyName.queue_event(stateMachinePropertyName##_proc_handle,IntrusivePtr(event));\
    stateMachinePropertyName();\
  }

  /**
   * @brief as in declarations.
   *  bool <owner-class>StateMachine::create_state_machine();
   */
  #define CYT3MACRO_boost_statechart_owner_class_definitions(\
    stateMachineOwnerClass,\
    stateMachinePropertyName,\
    stateMachineNamespace,\
    ...\
  )\
  bool stateMachineOwnerClass::create_state_machine(){\
    stateMachinePropertyName##_proc_handle = \
    stateMachinePropertyName.create_processor<stateMachineNamespace::stateMachineOwnerClass##StateMachine>(this);\
    stateMachinePropertyName.initiate_processor(stateMachinePropertyName##_proc_handle);\
    stateMachinePropertyName();\
    return true;\
  }\
  bool  stateMachineOwnerClass::permit_events() const{\
    return !push_event_ignore_push_;\
  }\
  bool  stateMachineOwnerClass::permit_events(bool p){\
    return !(push_event_ignore_push_ = !p);\
  }\
  bool  stateMachineOwnerClass::push_traces() const{\
    return push_event_debug_traces_show_;\
  }\
  bool  stateMachineOwnerClass::push_traces(bool p){\
    return push_event_debug_traces_show_ = p;\
  }\



//
//

  /**
   * @brief state machine pre-declarations for the state-machine class name, 
   *        fifo scheduler, allocator, and initial state. To be invoked prior to the other
   *        state machine macro declarations.
   * @param stateMachineOwnerClass : name of the owner class. The state machine will be declared with the
   *            postfix 'StateMachine'
   * @param initialState : initial state-struct name.
   * 
   */
  #define CYT3MACRO_boost_statechart_header(stateMachineOwnerClass, initialState)\
  \
    struct                                        stateMachineOwnerClass##StateMachine;\
    struct                                        initialState;\
    typedef boost::statechart::fifo_scheduler<>   stateMachineOwnerClass##StateMachineFifoScheduler;\
    typedef std::allocator<void>                  stateMachineOwnerClass##StateMachineAllocator;

  /**
   * @brief : REMEMBER: THIS NEEDS TO BE DECLARED AT THE GLOBAL_NAMESPACE. 
   *          defines the initiate_impl() structure.
   *          to-do : test if not implemented...
   * @param stateMachineOwnerClass : state machine owner name. 
   * @param initialState : initial state-struct name
   */
  #define CYT3MACRO_boost_statechart_implementation_declarations(stateMachineOwnerClass, initialState)\
  namespace boost{\
  namespace statechart{\
    template<>\
    inline void asynchronous_state_machine<\
       stateMachineOwnerClass##StateMachine\
      ,initialState\
      ,stateMachineOwnerClass##StateMachineFifoScheduler\
      ,stateMachineOwnerClass##StateMachineAllocator\
    >::initiate_impl(){}\
  }}

//
//------------------------------------------------------------------------------
//

    #define cytemacro_boost_statechart_machine_declarations_additionalmp_(stateMachinePropertyOrMethod)\
      stateMachinePropertyOrMethod ; 



  /**
   * @brief declarations of the boost-statechart state machine.
   * @param stateMachineOwnerClass : the postfix 'StateMachine' will be added
   * @param initialState : initial state-struct class type.
   * @param attachedClass : not required, if defined, will be used as the reference class to be attached to the state machine
   * @param additionalPublicMethodsAndProperties : list of public methods that may be accessed by the states
   * @param additionalProtectedMethodsAndProperties : list of protected methods that the user will define
   * 
   */
  #define CYT3MACRO_boost_statechart_machine_declarations(\
      stateMachineOwnerClass\
    , initialState\
    , attachedClass\
    , additionalPublicMethodsAndProperties\
    , additionalProtectedMethodsAndProperties)\
  \
  CYT3MACRO_boost_statechart_event_simple_declarations(EvConfirmTransition##stateMachineOwnerClass, , "transition-confirmation")\
  \
  struct stateMachineOwnerClass##StateMachine\
  : public boost::statechart::asynchronous_state_machine<\
              stateMachineOwnerClass##StateMachine\
      COMMA() initialState\
      COMMA() stateMachineOwnerClass##StateMachineFifoScheduler\
      COMMA() stateMachineOwnerClass##StateMachineAllocator>\
  {\
    public:\
      \
      typedef coyot3::tools::statemachine::StateMachineLogLine StateMachineLogLine;\
      typedef coyot3::tools::statemachine::StateMachineLogLineStack StateMachineLogLineStack;\
      \
      stateMachineOwnerClass##StateMachine(my_context ctx, \
        IFE(attachedClass)(stateMachineOwnerClass* owner)\
        IFN(attachedClass)(attachedClass* owner)\
      );\
      virtual ~stateMachineOwnerClass##StateMachine();\
      \
      const std::string& current_state();\
      void  log(const std::string& desc, int level = StateMachineLogLine::DEBUG);\
    \
      void transition_reason_set(const std::string& r);\
      void transition_reason_add(const std::string& r);\
      void st_confirm_transition(const std::string& nstate = std::string());\
    \
      void unconsumed_event(const boost::statechart::event_base& e);\
      \
      stateMachineOwnerClass* owner();\
      const stateMachineOwnerClass* owner() const;\
      \
    protected:\
      \
      void user_unconsumed_event(const boost::statechart::event_base& e);\
      void user_constructor_post_operations();\
      void user_destructor_pre_operations();\
      \
      std::string current_state_string_;\
        void calculate_state_string_();\
        void log_current_transition_();\
          std::string transition_reason_;\
          int64_t     transition_last_ts_;\
      \
      stateMachineOwnerClass*       owner_;\
      \
      StateMachineLogLineStack      logs_;\
    private:\
      virtual void initiate_impl();\
    \
    public:\
    \
      FOR_EACH(cytemacro_boost_statechart_machine_declarations_additionalmp_,PASS_PARAMETERS(additionalPublicMethodsAndProperties))\
    \
    protected:\
    \
      FOR_EACH(cytemacro_boost_statechart_machine_declarations_additionalmp_,PASS_PARAMETERS(additionalProtectedMethodsAndProperties))\
  };

  /**
   * @brief the user needs to define : 
   *  void <stateMachineOwnerClass>StateMachine::user_unconsumed_event(const boost::statechart::event_base& e)
   *  void <stateMachineOwnerClass>StateMachine::user_constructor_post_operations(stateMachineOwnerClass* ownerInstance);\
   *  void <stateMachineOwnerClass>StateMachine::user_destructor_pre_operations();\
   */
  #define CYT3MACRO_boost_statechart_machine_definitions(\
    stateMachineOwnerClass\
    , initialState\
    , attachedClass\
    , additionalPublicMethodsAndProperties\
    , additionalProtectedMethodsAndProperties)\
    \
    CYT3MACRO_boost_statechart_event_simple_definitions(EvConfirmTransition##stateMachineOwnerClass, , "transition-confirmation")\
    \
    stateMachineOwnerClass##StateMachine::stateMachineOwnerClass##StateMachine(my_context ctx, \
      IFE(attachedClass)(stateMachineOwnerClass* )\
      IFN(attachedClass)(attachedClass* )\
      ownerInstance)\
    : my_base(ctx)\
    , current_state_string_()\
    , transition_reason_()\
    , transition_last_ts_(0)\
    , owner_(ownerInstance)\
    , logs_()\
    {\
      CLOG_INFO(#stateMachineOwnerClass " state-machine : contructor")\
      current_state_string_ = "not-initialized";\
      user_constructor_post_operations();\
    }\
    stateMachineOwnerClass##StateMachine::~stateMachineOwnerClass##StateMachine(){\
      user_destructor_pre_operations();\
      CLOG_WARN(#stateMachineOwnerClass " state-machine : destructor")\
    }\
    void stateMachineOwnerClass##StateMachine::initiate_impl(){\
      CLOG_INFO(#stateMachineOwnerClass " state-machine : initiate-impl-")\
      boost::statechart::state_machine<stateMachineOwnerClass##StateMachine, initialState, stateMachineOwnerClass##StateMachineAllocator>::initiate();\
    }\
    void stateMachineOwnerClass##StateMachine::calculate_state_string_(){\
      \
      char reg = '0';\
      std::stringstream sstr;\
      for(stateMachineOwnerClass##StateMachine::state_iterator it = state_begin()\
        ;it != state_end(); ++it){\
        sstr << " Region(" << reg++ << "):";\
        const stateMachineOwnerClass##StateMachine::state_base_type* pState= &*it;\
        while(pState!=0){\
          sstr << std::string(pState->custom_dynamic_type_ptr<char>());\
          pState = pState->outer_state_ptr();\
          if(pState)current_state_string_.insert(0,"->");\
        }\
      }\
      current_state_string_ = sstr.str();\
    }\
    const std::string& stateMachineOwnerClass##StateMachine::current_state(){\
      return current_state_string_;\
    }\
    void stateMachineOwnerClass##StateMachine::unconsumed_event(const boost::statechart::event_base& e){\
      if(e.dynamic_type() == coyot3::tools::statemachine::EvStateMachineTransitionConfirmation::static_type()){\
        log_current_transition_();\
      }\
      user_unconsumed_event(e);\
    }\
    void stateMachineOwnerClass##StateMachine::st_confirm_transition(const std::string& nstate){\
      \
    }\
    void stateMachineOwnerClass##StateMachine::log_current_transition_(){\
      std::string& oldState = current_state_string_;\
      calculate_state_string_();\
      logs_.push_back(StateMachineLogLine(transition_reason_,StateMachineLogLine::TRANSITION));\
      transition_reason_.clear();\
    }\
    \
    void stateMachineOwnerClass##StateMachine::transition_reason_set(const std::string& r){\
      transition_reason_ = r;\
    }\
    void stateMachineOwnerClass##StateMachine::transition_reason_add(const std::string& r){\
      if(transition_reason_.size())transition_reason_+=";";\
      transition_reason_+=r;\
    }\
    \
    stateMachineOwnerClass* stateMachineOwnerClass##StateMachine::owner(){return owner_;}\
    const stateMachineOwnerClass* stateMachineOwnerClass##StateMachine::owner() const{return owner_;}\




  
