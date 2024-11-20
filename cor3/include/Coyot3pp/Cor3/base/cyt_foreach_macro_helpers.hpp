#pragma once

#include "va_opt.hpp"

#define PARENS ()

#define ESC(...) __VA_ARGS__

#define _Args(...) __VA_ARGS__
#define STRIP_PARENS(X) X
#define PASS_PARAMETERS(X) STRIP_PARENS( _Args X )

#define PRE_COMMA COMMA()
#define COMMA() ,
#define EMPTY()
#define DEFER(id) id EMPTY()
#define CHAIN_COMMA_1(x) DEFER(COMMA)() x CHAIN_COMMA_2
#define CHAIN_COMMA_2(x) DEFER(COMMA)() x CHAIN_COMMA_1

// Rescan macro tokens 256 times
#define STRINGIFY(V_TOSTR) V_TOSTR
#define EXPAND(arg) EXPAND1(EXPAND1(EXPAND1(EXPAND1(arg))))
#define EXPAND1(arg) EXPAND2(EXPAND2(EXPAND2(EXPAND2(arg))))
#define EXPAND2(arg) EXPAND3(EXPAND3(EXPAND3(EXPAND3(arg))))
#define EXPAND3(arg) EXPAND4(EXPAND4(EXPAND4(EXPAND4(arg))))
#define EXPAND4(arg) arg

#define ADD_COMMA(...) , __VA_ARGS__

// concatenation creating commas
#define CAT(x, y) CAT_(x, y)
#define CAT_(x,y) x ## y // error from CHAIN_COMMA: passed 4 arguments

#define CHAIN_COMMA(chain) CAT(CHAIN_COMMA_0 chain, _END) // error
#define CHAIN_COMMA_0(x) x CHAIN_COMMA_1
#define CHAIN_COMMA_1(x) DEFER(COMMA)() x CHAIN_COMMA_2
#define CHAIN_COMMA_2(x) DEFER(COMMA)() x CHAIN_COMMA_1
#define CHAIN_COMMA_0_END
#define CHAIN_COMMA_1_END
#define CHAIN_COMMA_2_END
// -----------------------------

/// foreach argument : begin

  #define FOR_EACH_AGAIN() FOR_EACH_HELPER

    #define FOR_EACH_WITH_CONSTANT(macro, V_CONSTANT, ...) \
      __VA_OPT__(EXPAND(FOR_EACH_PAIR_WITH_CONSTANT_HELPER(macro, V_CONSTANT, __VA_ARGS__)))

#define FOR_EACH_HELPER(macro, a1, ...) \
        macro(a1) \
        __VA_OPT__(FOR_EACH_AGAIN PARENS (macro, __VA_ARGS__))

#define FOR_EACH(macro, ...) \
  __VA_OPT__(EXPAND(FOR_EACH_HELPER(macro, __VA_ARGS__)))

/// foreach argument : end


// foreach use pair arguments : begin



#define FOR_EACH_PAIR(macro, ...) \
  __VA_OPT__(EXPAND(FOR_EACH_PAIR_HELPER(macro, __VA_ARGS__)))

  #define FOR_EACH_PAIR_AGAIN() FOR_EACH_PAIR_HELPER
  
  #define FOR_EACH_PAIR_HELPER(macro, a1, a2, ...)\
    macro(a1, a2) \
    __VA_OPT__(FOR_EACH_PAIR_AGAIN PARENS (macro, __VA_ARGS__))




#define FOR_EACH_PAIR_DEFER(macro, parens_arg) \
  FOR_EACH_PAIR(macro, DEFER parens_arg)

    #define FOR_EACH_PAIR_AGAIN_WITH_CONSTANT() FOR_EACH_PAIR_WITH_CONSTANT_HELPER

  #define FOR_EACH_PAIR_WITH_CONSTANT_HELPER(macro, a1, a2, ...)                 \
    macro(a1,a2)                                                     \
    __VA_OPT__(FOR_EACH_PAIR_AGAIN_WITH_CONSTANT PARENS (macro, a1, __VA_ARGS__))

#define FOR_EACH_PAIR_WITH_CONSTANT(macro, ...) \
  __VA_OPT__(EXPAND(FOR_EACH_PAIR_WITH_CONSTANT_HELPER(macro, __VA_ARGS__)))
// foreach use pair arguments : end


// foreach with triples : begin
#define FOR_EACH_TRIPLES(macro, ...) \
  __VA_OPT__(EXPAND(FOR_EACH_TRIPLES_HELPER(macro, __VA_ARGS__)))

#define FOR_EACH_TRIPLES_AGAIN() FOR_EACH_TRIPLES_HELPER

#define FOR_EACH_TRIPLES_HELPER(macro, a1, a2, a3, ...) \
  macro(a1, a2, a3) \
  __VA_OPT__(FOR_EACH_TRIPLES_AGAIN PARENS (macro, __VA_ARGS__))
// foreach with triples : end



// foreach triples with 1 static : begin
#define FOR_EACH_TRIPLES_WITH_01_STATIC(macro,V_STATIC,...) \
  __VA_OPT__(EXPAND(FOR_EACH_TRIPLES_WITH_01_STATIC_HELPER(macro,V_STATIC,__VA_ARGS__)))

#define FOR_EACH_TRIPLES_WITH_01_STATIC_HELPER_RE() FOR_EACH_TRIPLES_WITH_01_STATIC_HELPER

#define FOR_EACH_TRIPLES_WITH_01_STATIC_HELPER(macro,V_STATIC,a1,a2,a3,...) \
  macro(V_STATIC,a1,a2,a3) \
  __VA_OPT__(FOR_EACH_TRIPLES_WITH_01_STATIC_HELPER_RE PARENS (macro, V_STATIC , __VA_ARGS__ ))
// foreach triples with 1 static : end
