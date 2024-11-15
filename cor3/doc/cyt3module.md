# COYOT3PP::COR3 MODULE


## INTRODUCTION

As I create little *ecosystems*, I have some layers of abstraction. I need to be able to completely abstract lower levels from higher levels, and I do it using this wrapper of a *Module*. That way I can create kind of *agents* that I instantiate in higher levels and treat them as a *set*, invoking basic methods to tell them to *Initialize*, *Start*, *Pause*, *Stop* and *End*.

The class is also composed by another classes giving it additional properties:

* `LoggerCapability`
  * Giving the possibility to the module to generate its own modules.
  * This class is taught to be extended and be able to be connected to other modules, for example: database loggers... or socket-like connectors to be able to broadcast the logs.
* `RpcCapability`
  * Giving to the module the capability to expose its methods.



## HOW TO USE:


The `coyot3::mod::ModuleBase` class is tought to be extended. It contains a set of states:

* `CREATED` : State at module instantiation.
* `INITIALIZING` - `INITIALIZED` : Transition expected when invoking `Init()`
* `STARTING` - `STARTED`  : Transition expected when invoking `Start()`
* `PAUSING` - `PAUSED`  : Transition expected when invoking `Stop()`
* `STOPPING` - `STOPPED` : Transition expected when invoking `Stop()`
* `ENDING` - `END_OF_LIFE` : Transition expected when invoking `End(bool force)` 
* `MAINTENANCE` : general purpose state
* `MODULE_ERROR` : State when module detects some internal error.


These states evolve when invoking the following functions:

* `Init()` : to be used to prepare the instance for a continuous work.
* `Start()` : preprares and *launches* the instance to make its works.
* `Pause()` : pauses the continuous works of the module.
* `Stop()` : to stop the continuous works of the module ant put it into a *standby* `stopped` state. 
* `End(bool force = true)` : to include functions to totally free all the resources allocated by the module.

The initialization, start, pause, stop and end tasks are configured in parallel with the following methods:

* `bool add_task_init(ModuleTaskFunction f,bool prepend = false);`
* `bool add_task_start(ModuleTaskFunction f,bool prepend = false);`
* `bool add_task_pause(ModuleTaskFunction f,bool prepend = false);`
* `bool add_task_stop(ModuleTaskFunction f,bool prepend = false);`
* `bool add_task_end(ModuleTaskFunction f,bool prepend = false);`

The `ModuleTaskFunction` is a `bool()` function/method.

The `ModuleTaskFunction` MUST return `true` if the internal tasks were successful or `false` otherwise. If the returned value is `true`, then it is considered that the state of the module can evolve. If the returned value is `false`, the class state will transit to `MODULE_ERROR`.

The implementation of the inherited `ModuleBase` shoud add all the tasks related to the transition to each state.

When a transition is invoked, but it has no tasks associated, then, the module will remain at its present state.

It can be traced how many transitions is the module configured to have by invoking `std::string module_configuration()` will return the following string: `ispxe`. When the letter is capital, it means that the module needs invokation of specific function to make evolve its state: `I` needs `Init();`; `S` needs `Start();`; `P` needs `Pause();`; `X` needs `Stop();`; `E` needs `End()`.

See the minimal example at `cor3/module-example.cpp` (it will be compiled as `example_module`).



## `LoggerCapability`

Adds the following functions:

* `instance_name(const std::string& in)` : sets the name of the instance
* `class_name(const std::string& cn)` : set the name of the class.
* `mod_log_verbosity() const`, `mod_log_verbosity(int l)`, `mod_log_verbosity(LogLevel l)`
  * read and set the log level. 
  * The log levels are
    * `0` : no log at all
    * `1` : error level
    * `2` : error and warning
    * `3` : error, warning and info
    * `4` : all others and debug.
    * `>4` : log level depth.
* *both instance and class name will be used as prefix for the log lines*.
* `log_info(const std::string& line)` `log_info(const std::ostringstream& o)`
* `log_warn(const std::string& line)` `log_warn(const std::ostringstream& o)`
* `log_err(const std::string& line)` `log_err(const std::ostringstream& o)`
* `log_debug(int level, const std::string& line)` `log_debug(int level, const std::ostringstream& o)`


## `RpcCapability`

Dev ongoing.