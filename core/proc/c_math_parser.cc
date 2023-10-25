/*
 * mathparser.c
 *
 *  Created on: Mar 14, 2012
 *      Author: amyznikov
 *
 *  Updated :
 *    October 24, 2023
 */

#include <malloc.h>
#include <math.h>
#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include "c_math_parser.h"

#define MAX_NODE_ARGS         9
#define MAX_PRIORITY_LEVELS   10
#define MAX_OPERATORS         100
#define MAX_FUNCTIONS         1000
#define MAX_CONSTANTS         1000

#define MAX_ERR_MSG           256
#define MAX_NAME              256
#define MAX_OPERATOR_NAME     32

/*
 * Node hierarchy
 *    abstract_node
 *      |--> value_node
 *      |--> bound_parameter_node
 *      |--> arg_node
 *      |--> functional_node
 *            |--> fn*_node
 *            |--> ff_node
 */

//

struct c_abstract_node
{
  double (*eval)( struct c_abstract_node * _this, const double args[] );
  void (*destructor)( struct c_abstract_node * _this );
};

struct c_value_node
{
  c_abstract_node base;
  double value;
};

struct c_arg_node
{
  c_abstract_node base;
  int argindex;
};

struct c_bound_parameter_node
{
  c_abstract_node base;
  double * value;
};

struct c_functional_node
{
  c_abstract_node base;
  c_abstract_node * node_args[MAX_NODE_ARGS];
  int num_node_args;
};

struct c_ff_node
{
  c_functional_node base;
  cmpxff_t fn;
  void * param;
};


#define DECLARE_FN_NODE(NUM_ARGS) \
typedef struct fn##NUM_ARGS##_node { \
  c_functional_node base; \
  cmpxf##NUM_ARGS##_t fn; \
} fn##NUM_ARGS##_node; \
\
static double fn##NUM_ARGS##_node_eval( c_abstract_node * _this, const double args[] ); \
static c_abstract_node * fn##NUM_ARGS##_node_create( cmpxf##NUM_ARGS##_t fn, c_abstract_node * args[]) \
{ \
  fn##NUM_ARGS##_node * p = (fn##NUM_ARGS##_node * )functional_node_create(args, \
    NUM_ARGS, \
    sizeof(fn##NUM_ARGS##_node), \
    fn##NUM_ARGS##_node_eval, \
    functional_node_destructor \
    ); \
  if ( p ) \
  { \
    p->fn = fn; \
  } \
  return (c_abstract_node * )p; \
}


struct unary_operation
{
  cmpxf1_t fn;
  char name[MAX_OPERATOR_NAME];
};

struct binary_operation
{
  cmpxf2_t fn;
  char name[MAX_OPERATOR_NAME];
};

struct binary_operation_table
{
  int count;
  binary_operation desc[MAX_OPERATORS];
};

struct function_desc {
  double (*fn)(); /*!< dummy function pointer */
  int numargs;
  int is_volatile;
  void * param;   /*!< for functors only */
  char name[MAX_NAME];
};

struct arg_desc
{
  int index;
  char name[MAX_NAME];
};

struct const_desc
{
  double value;
  char name[MAX_NAME];
};

struct binding_desc
{
  double * value;
  char name[MAX_NAME];
}c;

struct c_math_parser
{
  c_abstract_node * root;

  binary_operation_table binops[MAX_PRIORITY_LEVELS];
  int num_priority_numlevels;

  unary_operation unops[MAX_OPERATORS];
  int unops_count;

  function_desc functions[MAX_FUNCTIONS];
  int functions_count;

  arg_desc args[MAX_NODE_ARGS];
  int args_count;

  const_desc constants[MAX_CONSTANTS];
  int const_count;

  binding_desc bindings[MAX_CONSTANTS];
  int bindings_count;

  int OBRACE, CBRACE, ARGLIST_DELIMITER;

  const char * pointer_to_syntax_error;
  char error_msg[MAX_ERR_MSG];
};


/*
 * Some forward declarations
 */
static void set_error_msg(c_math_parser * ctx, const char * format, ...)
    __attribute__ ((__format__ (__printf__, 2, 3)));

static int parse_expression(c_math_parser * ctx, size_t priority_level, const char ** curpos,
    c_abstract_node ** ppnode);

static int parse_terminal_token(c_math_parser * ctx, const char ** curpos,
    c_abstract_node ** ppnode);

/******************************************************************************************************************
 * Abstract Node
 */

static c_abstract_node * create_node( size_t size, double(*eval)( c_abstract_node *, const double args[] ),
  void(*destructor)( c_abstract_node * ) )
{
  c_abstract_node * p = (c_abstract_node * )calloc(1, size);
  if ( p ) {
    p->eval = eval;
    p->destructor = destructor;
  }
  return p;
}

static void destroy_node( c_abstract_node * p )
{
  if ( p ) {
    if ( p->destructor ) {
      p->destructor(p);
    }
    free(p);
  }
}

static double cmathexpression_eval( void * param, const double args[], int numargs )
{
  c_math_parser * p = (c_math_parser*) param;
  return p->root->eval(p->root, args);
}

/******************************************************************************************************************
 * Value Node
 */

static double value_node_eval( c_abstract_node * _this, const double args[] )
{
  return ( (c_value_node*) _this )->value;
}

static c_abstract_node * value_node_create( double value )
{
  c_value_node * p =
      (c_value_node *) create_node(sizeof(c_value_node),
          value_node_eval,
          0);

  if ( p ) {
    p->value = value;
  }

  return (c_abstract_node *) p;
}


/******************************************************************************************************************
 * Arg Node
 */

static double arg_node_eval( c_abstract_node * _this, const double args[] )
{
  return args[( (c_arg_node*) _this )->argindex];
}

static c_abstract_node * arg_node_create( int argindex )
{
  c_arg_node * p =
      (c_arg_node *) create_node(sizeof(c_arg_node),
          arg_node_eval,
          0);

  if ( p ) {
    p->argindex = argindex;
  }

  return (c_abstract_node *) p;
}



/******************************************************************************************************************
 * Bound Parameter Node
 */

static double bound_parameter_node_eval( c_abstract_node * _this, const double args[] )
{
  return *( (c_bound_parameter_node*) _this )->value;
}

static c_abstract_node* bound_parameter_node_create(double * value)
{
  c_bound_parameter_node * p =
      (c_bound_parameter_node*) create_node(sizeof(c_bound_parameter_node),
          bound_parameter_node_eval,
          0);

  if( p ) {
    p->value = value;
  }
  return (c_abstract_node*) p;
}


/******************************************************************************************************************
 * Functional Node
 */

static void functional_node_destructor( c_abstract_node * _this )
{
  int i = 0;

  while ( i < ( (c_functional_node*) _this )->num_node_args ) {
    destroy_node(( (c_functional_node*) _this )->node_args[i++]);
  }
}

static c_abstract_node* functional_node_create(c_abstract_node * args[], int num_node_args, size_t size,
    double (*eval)(c_abstract_node*, const double args[]), void (*destructor)(c_abstract_node*))
{
  c_functional_node *p =
      (c_functional_node*) create_node(size,
          eval,
          destructor);

  if( p && args ) {
    memcpy(p->node_args, args, num_node_args * sizeof(args[0]));
    p->num_node_args = num_node_args;
  }

  return (c_abstract_node*) p;
}


/******************************************************************************************************************
 * FN* Nodes
 */

DECLARE_FN_NODE(0)
static double fn0_node_eval( c_abstract_node * _this, const double args[] )
{
  return ( (fn0_node*) _this )->fn();
}

DECLARE_FN_NODE(1)
static double fn1_node_eval( c_abstract_node * _this, const double args[] )
{
  fn1_node * p = (fn1_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  return p->fn(arg0->eval(arg0, args));
}

DECLARE_FN_NODE(2)
static double fn2_node_eval( c_abstract_node * _this, const double args[] )
{
  fn2_node * p = (fn2_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args));
}

DECLARE_FN_NODE(3)
static double fn3_node_eval( c_abstract_node * _this, const double args[] )
{
  fn3_node * p = (fn3_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args));
}

DECLARE_FN_NODE(4)
static double fn4_node_eval( c_abstract_node * _this, const double args[] )
{
  fn4_node * p = (fn4_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args));
}

DECLARE_FN_NODE(5)
static double fn5_node_eval( c_abstract_node * _this, const double args[] )
{
  fn5_node * p = (fn5_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  c_abstract_node * arg4 = p->base.node_args[4];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args),
    arg4->eval(arg4, args));
}

DECLARE_FN_NODE(6)
static double fn6_node_eval( c_abstract_node * _this, const double args[] )
{
  fn6_node * p = (fn6_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  c_abstract_node * arg4 = p->base.node_args[4];
  c_abstract_node * arg5 = p->base.node_args[5];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args),
    arg4->eval(arg4, args), arg5->eval(arg5, args));
}

DECLARE_FN_NODE(7)
static double fn7_node_eval( c_abstract_node * _this, const double args[] )
{
  fn7_node * p = (fn7_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  c_abstract_node * arg4 = p->base.node_args[4];
  c_abstract_node * arg5 = p->base.node_args[5];
  c_abstract_node * arg6 = p->base.node_args[6];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args),
    arg4->eval(arg4, args), arg5->eval(arg5, args), arg6->eval(arg6, args));
}

DECLARE_FN_NODE(8)
static double fn8_node_eval( c_abstract_node * _this, const double args[] )
{
  fn8_node * p = (fn8_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  c_abstract_node * arg4 = p->base.node_args[4];
  c_abstract_node * arg5 = p->base.node_args[5];
  c_abstract_node * arg6 = p->base.node_args[6];
  c_abstract_node * arg7 = p->base.node_args[7];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args),
    arg4->eval(arg4, args), arg5->eval(arg5, args), arg6->eval(arg6, args), arg7->eval(arg7, args));
}

DECLARE_FN_NODE(9)
static double fn9_node_eval( c_abstract_node * _this, const double args[] )
{
  fn9_node * p = (fn9_node*) _this;
  c_abstract_node * arg0 = p->base.node_args[0];
  c_abstract_node * arg1 = p->base.node_args[1];
  c_abstract_node * arg2 = p->base.node_args[2];
  c_abstract_node * arg3 = p->base.node_args[3];
  c_abstract_node * arg4 = p->base.node_args[4];
  c_abstract_node * arg5 = p->base.node_args[5];
  c_abstract_node * arg6 = p->base.node_args[6];
  c_abstract_node * arg7 = p->base.node_args[7];
  c_abstract_node * arg8 = p->base.node_args[7];
  return p->fn(arg0->eval(arg0, args), arg1->eval(arg1, args), arg2->eval(arg2, args), arg3->eval(arg3, args),
    arg4->eval(arg4, args), arg5->eval(arg5, args), arg6->eval(arg6, args), arg7->eval(arg7, args),
    arg8->eval(arg8, args));
}


/******************************************************************************************************************
 * FF Node
 */

static double ff_node_eval( c_abstract_node * _this, const double args[] )
{
  c_ff_node * p = (c_ff_node *) _this;
  const int numargs = p->base.num_node_args;
  double ff_args[numargs];
  int i;

  for ( i = 0; i < numargs; ++i )
  {
    c_abstract_node * a = p->base.node_args[i];
    ff_args[i] = a->eval(a, args);
  }

  return p->fn(p->param, ff_args, numargs);
}

c_abstract_node * ff_node_create( cmpxff_t fn, int numargs, void * param, c_abstract_node * args[] )
{
  c_ff_node * p = (c_ff_node *) functional_node_create(args, numargs, sizeof(c_ff_node), ff_node_eval,
    functional_node_destructor);

  if ( p )
  {
    p->fn = fn;
    p->param = param;
  }

  return (c_abstract_node *) p;
}


/******************************************************************************************************************
 * Math Expression Parser
 */

static int unary_operation_compare_by_name_len( const void * p1, const void * p2 )
{
  int l1 = strlen(( (unary_operation*) p1 )->name);
  int l2 = strlen(( (unary_operation*) p2 )->name);
  return l1 < l2 ? 1 : l1 > l2 ? -1 : 0;
}

static int binary_operation_compare_by_name_len( const void * p1, const void * p2 )
{
  int l1 = strlen(( (binary_operation*) p1 )->name);
  int l2 = strlen(( (binary_operation*) p2 )->name);
  return l1 < l2 ? 1 : l1 > l2 ? -1 : 0;
}

static void set_error_msg( c_math_parser * ctx, const char * format, ... )
{
  if ( !*ctx->error_msg )
  {
    va_list arglist;
    va_start(arglist,format);
    vsnprintf(ctx->error_msg, MAX_ERR_MSG - 1, format, arglist);
    va_end(arglist);
  }
}

static const char * skip_white_spaces( const char ** curpos )
{
  while ( isspace(**curpos) )
    ++*curpos;
  return *curpos;
}

static unary_operation * lookup_unary_operator( c_math_parser * ctx, const char *curpos )
{
  int i;
  for ( i = 0; i < ctx->unops_count; ++i )
  {
    if ( strncmp(ctx->unops[i].name, curpos, strlen(ctx->unops[i].name)) == 0 )
    {
      return &ctx->unops[i];
    }
  }
  return nullptr;
}

static binary_operation * lookup_binary_operator( c_math_parser * ctx, int priority_level, const char * curpos )
{
  if ( priority_level >= 0 && priority_level < ctx->num_priority_numlevels )
  {
    int i;
    for ( i = 0; i < ctx->binops[priority_level].count; ++i )
    {
      binary_operation * desc = &ctx->binops[priority_level].desc[i];
      if ( strncmp(desc->name, curpos, strlen(desc->name)) == 0 )
      {
        return &ctx->binops[priority_level].desc[i];
      }
    }
  }
  return nullptr;
}

static arg_desc * lookup_argument( c_math_parser * ctx, const char * name )
{
  int i;
  for ( i = 0; i < ctx->args_count; ++i )
  {
    if ( strcmp(ctx->args[i].name, name) == 0 )
    {
      return &ctx->args[i];
    }
  }
  return nullptr;
}

static binding_desc * lookup_bindings(c_math_parser * ctx, const char * name)
{
  int i;
  for ( i = 0; i < ctx->bindings_count; ++i )
  {
    if ( strcmp(ctx->bindings[i].name, name) == 0 )
    {
      return &ctx->bindings[i];
    }
  }
  return nullptr;
}

static const_desc * lookup_constant( c_math_parser * ctx, const char * name )
{
  int i;
  for ( i = 0; i < ctx->const_count; ++i )
  {
    if ( strcmp(ctx->constants[i].name, name) == 0 )
    {
      return &ctx->constants[i];
    }
  }
  return nullptr;
}

static function_desc * lookup_function( c_math_parser * ctx, const char * name )
{
  const int functions_count = ctx->functions_count;
  int i;

  for ( i = 0; i < functions_count; ++i )
  {
    if ( strcmp(ctx->functions[i].name, name) == 0 )
    {
      return &ctx->functions[i];
    }
  }

  return nullptr;
}


static int parse_number( double * value, const char * curpos, char ** endptr )
{
  *value = strtod(curpos, endptr);
  return *endptr > curpos;
}

static int can_be_part_of_identifier( char ch )
{
  return isalnum(ch) || ch == '_' || ch == '$';
}

static int parse_terminal_token(c_math_parser * ctx, const char ** curpos, c_abstract_node ** ppnode )
{
  /*
   * terminal_token:
   *  [unary-operation] <(expression)|numerical-constant|argument-name|bound-parameter-name|named-constant|function-name>
   */

  const unary_operation * unop = 0;
  int arg_is_const ;
  char name[MAX_NAME];

  arg_desc * af = 0;
  binding_desc * pf = 0;
  const_desc * cf = 0;
  function_desc * ff = 0;

  const char * tmp1;
  char * tmp2;

  *ppnode = 0;

  /* Check if we have an unary operation */
  if ( ( unop = lookup_unary_operator(ctx, skip_white_spaces(curpos)) ) != 0 )
  {
    *curpos += strlen(unop->name);

    if ( parse_terminal_token(ctx, curpos, ppnode) )
    {
      arg_is_const = ( *ppnode )->eval == value_node_eval;

      *ppnode = fn1_node_create(unop->fn, ppnode);

      if ( arg_is_const )
      {
        double value = ( *ppnode )->eval(( *ppnode ), nullptr);
        destroy_node(*ppnode);
//        ( *ppnode )->destructor(*ppnode), free(*ppnode);
        *ppnode = value_node_create(value);
      }
    }
    return *ppnode != 0;
  }

  /* Check if we have an subexpression in braces */
  if ( **curpos == ctx->OBRACE )
  {
    ++*curpos;

    if ( !parse_expression(ctx, 0, curpos, ppnode) )
    {
      return 0;
    }

    if ( *skip_white_spaces(curpos) != ctx->CBRACE )
    {
      set_error_msg(ctx, "expected '%c' ", ctx->CBRACE);
      //( *ppnode )->destructor(*ppnode), free(*ppnode), *ppnode = 0;
      destroy_node(*ppnode), *ppnode = 0;
      return 0;
    }
    ++*curpos;
    return 1;
  }

  /* Check if we have an numerical value */
  if ( 1 )
  {
    double value;
    char * endptr;
    if ( parse_number(&value, *curpos, &endptr) )
    {
      *curpos = endptr;
      *ppnode = value_node_create(value);
      skip_white_spaces(curpos);
      return 1;
    }
  }

  /*
   * Check if we have an named object
   * TODO: check array bounds for name[]
   */
  tmp2 = name;
  tmp1 = skip_white_spaces(curpos);
  while ( can_be_part_of_identifier(*tmp1) )
  {
    *tmp2++ = *tmp1++;
  }
  *tmp2 = 0;

  if ( ( af = lookup_argument(ctx, name) ) )
  {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = arg_node_create(af->index);
  }
  else if ( ( pf = lookup_bindings(ctx, name) ) )
  {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = bound_parameter_node_create(pf->value);
  }
  else if ( ( cf = lookup_constant(ctx, name) ) )
  {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = value_node_create(cf->value);
  }
  else if( (ff = lookup_function(ctx, name)) )
  {
    c_abstract_node * args[MAX_NODE_ARGS] = {0};
    int numargs = 0;
    c_abstract_node * arg = 0;
    int success = 1;
    int all_args_are_contants = 1;

    *curpos = tmp1;
    if ( *skip_white_spaces(curpos) != ctx->OBRACE )
    {
      set_error_msg(ctx, "missing '%c' in function call '%s'", ctx->OBRACE, ff->name);
      return 0;
    }

    //
    // parse argument list
    //
    ++*curpos;

    while( numargs < ff->numargs )
    {
      if ( *skip_white_spaces(curpos) == ctx->CBRACE )
      {
        success = 0, set_error_msg(ctx, "%s expects %d arguments", name, ff->numargs);
        break;
      }

      if ( !( success = parse_expression(ctx, 0, curpos, &arg) ) )
      {
        break;
      }

      args[numargs++] = arg;

      if ( arg->eval != value_node_eval )
      {
        all_args_are_contants = 0;
      }

      if ( *skip_white_spaces(curpos) == ctx->ARGLIST_DELIMITER && numargs < ff->numargs )
      {
        ++*curpos;
      }
    }

    if ( numargs != ff->numargs )
    {
      success = 0;
      set_error_msg(ctx, "%s expects %d arguments", name, ff->numargs);
    }
    else if ( **curpos != ctx->CBRACE )
    {
      success = 0;
      set_error_msg(ctx, "expected '%c'", ctx->CBRACE);
    }

    if ( !success )
    {
      while ( numargs-- )
      {
        destroy_node(args[numargs]);
      }
      return 0;
    }

    if ( ff->param )
    {
      *ppnode = ff_node_create((cmpxff_t) ff->fn, ff->numargs, ff->param, args);
    }
    else
    {
      switch ( ff->numargs )
      {
      case 0: *ppnode = fn0_node_create((cmpxf0_t)ff->fn,args); break;
      case 1: *ppnode = fn1_node_create((cmpxf1_t)ff->fn,args); break;
      case 2: *ppnode = fn2_node_create((cmpxf2_t)ff->fn,args); break;
      case 3: *ppnode = fn3_node_create((cmpxf3_t)ff->fn,args); break;
      case 4: *ppnode = fn4_node_create((cmpxf4_t)ff->fn,args); break;
      case 5: *ppnode = fn5_node_create((cmpxf5_t)ff->fn,args); break;
      case 6: *ppnode = fn6_node_create((cmpxf6_t)ff->fn,args); break;
      case 7: *ppnode = fn7_node_create((cmpxf7_t)ff->fn,args); break;
      case 8: *ppnode = fn8_node_create((cmpxf8_t)ff->fn,args); break;
      case 9: *ppnode = fn9_node_create((cmpxf9_t)ff->fn,args); break;
  #if MAX_NODE_ARGS != 9
  # error Update This code please to reflect the changes in MAX_NODE_ARGS parameter
  #endif
      default:
        set_error_msg(ctx, "BUG IN MATH PARSER SOURCE AT %s:%d",__FILE__, __LINE__);
        while ( numargs-- )
          destroy_node(args[numargs]);
        return 0;
      }
    }

    if ( all_args_are_contants && !ff->is_volatile )
    {
      double value = ( *ppnode )->eval(*ppnode, nullptr);
      destroy_node(*ppnode);
      *ppnode = value_node_create(value);
    }

    ++*curpos;
  }
  else if ( *name )
  {
    set_error_msg(ctx, "unknown identifier '%s'", name);
  }
  else if ( **curpos )
  {
    set_error_msg(ctx, "unexpected symbol '%c' in expression", **curpos);
  }

  return *ppnode != 0;
}


static int parse_expression( c_math_parser * ctx, size_t priority_level, const char ** curpos, c_abstract_node ** ppnode )
{
  /*
   * expression:
   *  arg1 binary-operation arg2 ...
   *
   * Binary operation priorities must be arranged from lowest priority at the begin of table
   * to the highest priority at the end of table
   */

  const binary_operation * binop = 0;
  c_abstract_node * args[2] = { 0, 0 };
  int args_are_constants;

  *ppnode = 0;

  if ( priority_level >= ctx->num_priority_numlevels )
  {
    return parse_terminal_token(ctx, curpos, ppnode);
  }

  if ( !parse_expression(ctx, priority_level + 1, curpos, &args[0]) )
  {
    return 0;
  }

  while ( ( binop = lookup_binary_operator(ctx, priority_level, skip_white_spaces(curpos)) ) )
  {
    *curpos += strlen(binop->name);

    if ( !parse_expression(ctx, priority_level + 1, curpos, &args[1]) )
    {
      destroy_node(args[0]);
      return 0;
    }

    args_are_constants = ( args[0]->eval == value_node_eval && args[1]->eval == value_node_eval );

    args[0] = fn2_node_create(binop->fn, args);

    if ( args_are_constants )
    {
      double value = args[0]->eval(args[0], nullptr);
      destroy_node(args[0]);
      args[0] = value_node_create(value);
    }
  }

  *ppnode = args[0];
  return 1;
}

/********************************************************************************************************************
 * Public functions
 */

/*!
 * Create empty math parser
 * @return pointer to math parser context
 */
c_math_parser * c_math_parser_create(void)
{
  c_math_parser * ctx = (c_math_parser *) calloc(1, sizeof(c_math_parser));
  if ( ctx )
  {
    ctx->OBRACE = '(';
    ctx->CBRACE = ')';
    ctx->ARGLIST_DELIMITER = ',';
  }
  return ctx;
}

/*!
 * Destroy parsed tree
 * @param ctx
 */
void c_math_parser_cleanup( c_math_parser * ctx )
{
  if ( ctx->root ) {
    destroy_node(ctx->root), ctx->root = 0;
  }
}

/*!
 * destroy math parser
 * @param ctx
 */
void c_math_parser_destroy( c_math_parser * ctx )
{
  if ( ctx ) {
    c_math_parser_cleanup(ctx);
    free(ctx);
  }
}

/*!
 * Evaluate parsed tree
 * @param ctx
 * @param args
 * @return
 */
double c_math_parser_eval( c_math_parser * ctx, const double args[] )
{
  return ctx->root->eval(ctx->root, args);
}

/*!
 * set number of binary operations priority levels
 * @param ctx
 * @param levels
 * @return
 */
int c_math_parser_set_priority_levels( c_math_parser * ctx, int levels )
{
  ctx->num_priority_numlevels = levels;
  return 0;
}

const char * c_math_parser_parse( c_math_parser * ctx, const char * string )
{
  *ctx->error_msg = 0;
  ctx->pointer_to_syntax_error = nullptr;

  if ( !parse_expression(ctx, 0, &string, &ctx->root) )
  {
    ctx->pointer_to_syntax_error = string;
  }
  else if ( *string != 0 )
  {
    c_math_parser_cleanup(ctx);
    set_error_msg(ctx, "Unexpected text after end of expression: '%s'", string);
    ctx->pointer_to_syntax_error = string;
  }
  return ctx->pointer_to_syntax_error;
}

const char * c_math_parser_get_error_message( c_math_parser * ctx )
{
  return ctx->error_msg;
}

int c_math_parser_add_constant( c_math_parser * ctx, const double value, const char * name, const char * description )
{
  int count;
  const_desc * desc;

  if ( ( count = ctx->const_count ) >= MAX_CONSTANTS )
  {
    return -1;
  }

  if ( ( desc = lookup_constant(ctx, name) ) != nullptr )
  {
    return -1;
  }

  desc = ctx->constants + count;
  desc->value = value;
  strncpy(desc->name, name, MAX_NAME)[MAX_NAME - 1] = 0;
  ++ctx->const_count;
  return 0;
}

int c_math_parser_add_argument( c_math_parser * ctx, int arg_index, const char * name, const char * description )
{
  int count;
  arg_desc * desc;

  if ( ( count = ctx->args_count ) >= MAX_CONSTANTS ) {
    return -1;
  }

  if( (desc = lookup_argument(ctx, name)) ) {
    return -1;
  }

  desc = ctx->args + count;
  desc->index = arg_index;
  strncpy(desc->name, name, MAX_NAME)[MAX_NAME - 1] = 0;
  ++ctx->args_count;

  return 0;
}

int c_math_parser_bind( c_math_parser * ctx, double * value, const char * name, const char * description )
{
  int count;
  binding_desc * desc;

  if ( ( count = ctx->bindings_count ) >= MAX_CONSTANTS )
  {
    return -1;
  }

  if ( ( desc = lookup_bindings(ctx, name) ) != nullptr )
  {
    return -1;
  }

  desc = ctx->bindings + count;
  desc->value = value;
  strncpy(desc->name, name, MAX_NAME)[MAX_NAME - 1] = 0;
  ++ctx->bindings_count;
  return 0;
}

int c_math_parser_add_unary_operation( c_math_parser * ctx, cmpxf1_t fn, const char * name, const char * description )
{
  int count;
  unary_operation * desc;

  if( (count = ctx->unops_count) >= MAX_OPERATORS ) {
    return -1;
  }

  if( (desc = lookup_unary_operator(ctx, name)) != nullptr && strcmp(desc->name, name) == 0 ) {
    return -1;
  }

  desc = ctx->unops + count;
  desc->fn = fn;
  strncpy(desc->name, name, MAX_OPERATOR_NAME)[MAX_OPERATOR_NAME - 1] = 0;
  qsort(ctx->unops,++ctx->unops_count,sizeof(unary_operation),unary_operation_compare_by_name_len);

  return 0;
}

int c_math_parser_add_binary_operation( c_math_parser * ctx, int priority, cmpxf2_t fn, const char * name, const char * description )
{
  int count;
  binary_operation * desc;

  if ( priority < 0 || priority >= ctx->num_priority_numlevels ) {
    return -1;
  }

  if ( ( count = ctx->binops[priority].count ) >= MAX_OPERATORS ) {
    return -1;
  }

  if ( ( desc = lookup_binary_operator(ctx, priority, name) ) && strcmp(desc->name, name) == 0 ) {
    return -1;
  }

  desc = ctx->binops[priority].desc + count;
  desc->fn = fn;
  strncpy(desc->name, name, MAX_OPERATOR_NAME)[MAX_OPERATOR_NAME - 1] = 0;
  qsort(ctx->binops[priority].desc, ++ctx->binops[priority].count, sizeof(binary_operation),
    binary_operation_compare_by_name_len);

  return 0;
}

int c_math_parser_add_functor(c_math_parser * ctx, cmpxff_t fn, int numargs, void * param, const char * name,
    const char * description)
{
  int count;
  struct function_desc *desc;

  if( (count = ctx->functions_count) >= MAX_FUNCTIONS ) {
    return -1;
  }

  if( (desc = lookup_function(ctx, name)) != nullptr ) {
    return -1;
  }

  desc = ctx->functions + count;
  desc->fn = (double (*)()) fn;
  desc->numargs = numargs;
  desc->param = param;
  desc->is_volatile = 0;
  strncpy(desc->name, name, MAX_NAME)[MAX_NAME - 1] = 0;

  ++ctx->functions_count;

  return 0;
}

int c_math_parser_add_parsed_expression( c_math_parser * ctx, c_math_parser * fctx, const char * name,
  const char * description )
{
  int count;
  struct function_desc * desc;

  if( (count = ctx->functions_count) >= MAX_FUNCTIONS ) {
    return -1;
  }

  if( (desc = lookup_function(ctx, name)) != nullptr ) {
    return -1;
  }

  desc = ctx->functions + count;
  desc->numargs = fctx->args_count;
  desc->fn = (double(*)()) cmathexpression_eval;
  desc->param = fctx;
  desc->is_volatile = 0;
  strncpy(desc->name, name, MAX_NAME)[MAX_NAME - 1] = 0;

  ++ctx->functions_count;

  return 0;
}

#define DEFILE_ADD_FUNCTION(NUMARGS) \
  int c_math_parser_add_function##NUMARGS( c_math_parser * ctx, cmpxf##NUMARGS##_t fn, const char * name, const char * description ) \
  {\
    int count;\
    struct function_desc * desc;\
\
    if ( ( count = ctx->functions_count ) >= MAX_FUNCTIONS ) {\
      return -1;\
    }\
\
    if ( ( desc = lookup_function(ctx, name) ) != nullptr ) {\
      return -1;\
    }\
\
    desc = ctx->functions + count;\
    desc->fn = (double(*)())fn;\
    desc->numargs = NUMARGS; \
    desc->is_volatile = 0; \
    strncpy(desc->name,name,MAX_NAME)[MAX_NAME-1] = 0; \
\
    ++ctx->functions_count; \
\
    return 0; \
  }

DEFILE_ADD_FUNCTION(0)
DEFILE_ADD_FUNCTION(1)
DEFILE_ADD_FUNCTION(2)
DEFILE_ADD_FUNCTION(3)
DEFILE_ADD_FUNCTION(4)
DEFILE_ADD_FUNCTION(5)
DEFILE_ADD_FUNCTION(6)
DEFILE_ADD_FUNCTION(7)
DEFILE_ADD_FUNCTION(8)
DEFILE_ADD_FUNCTION(9)


int c_math_parser_remove_constant( c_math_parser * ctx, const char * name )
{
  return -1;
}

int c_math_parser_remove_all_constants(c_math_parser * ctx )
{
  return -1;
}


int c_math_parser_remove_all_arguments( c_math_parser * ctx )
{
  return -1;
}


int c_math_parser_unbind( c_math_parser * ctx, const char * name )
{
  return -1;
}

int c_math_parser_unbind_all( c_math_parser * ctx )
{
  return -1;
}

////////////////////////////////////////////////////////////////////////////

static double operator_add( double x, double y )
{
  return x + y;
}
static double operator_sub( double x, double y )
{
  return x - y;
}
static double operator_mul( double x, double y )
{
  return x * y;
}
static double operator_div( double x, double y )
{
  return x / y;
}
static double operator_unary_minus( double x )
{
  return -x;
}
static double operator_unary_plus( double x )
{
  return x;
}
static double operator_logical_not( double x )
{
  return !x;
}
static double operator_logical_or( double x, double y )
{
  return x || y;
}
static double operator_logical_and( double x, double y )
{
  return x && y;
}
static double operator_eq( double x, double y )
{
  return x == y;
}
static double operator_not_eq( double x, double y )
{
  return x != y;
}
static double operator_lt( double x, double y )
{
  return x < y;
}
static double operator_le( double x, double y )
{
  return x <= y;
}
static double operator_gt( double x, double y )
{
  return x > y;
}
static double operator_ge( double x, double y )
{
  return x >= y;
}
static double sinc(double x)
{
  return x ? sin(x) / x : 1;
}
static double gexp(double x)
{
  return exp(-x * x / 2);
}
static double frand( double xmin, double xmax )
{
  return xmin + rand() * ( xmax - xmin ) / RAND_MAX;
}

/** wrap l from range 0..2*pi into range -pi..pi */
static inline double lwrap( double l )
{
  return l > M_PI ? l - 2 * M_PI : l;
}

/**
 * Get hammer-aitoff coordinates for given l,b;
 * l in range [-pi,pi], b in range [-pi/2,pi/2]
 */
static inline void haxy( double l, double b, double * x, double * y )
{
  const double z = sqrt(1 + cos(b) * cos(l / 2));
  *x = cos(b) * sin(l / 2) / z;
  *y = sin(b) / z;
}

static double aitofx( double l, double b )
{
  double z;
  l = lwrap(l), z = sqrt(1 + cos(b) * cos(l / 2));
  return ( cos(b) * sin(l / 2) / z );
}

static double aitofy( double l, double b )
{
  double z;
  l = lwrap(l), z = sqrt(1 + cos(b) * cos(l / 2));
  return ( sin(b) / z );
}



/*!
 * Gaussian noise with mean m and variance s,
 * uses the Box-Muller transformation
 */
static double noise( double s, double m )
{
  const double r1 = ( (double) rand() ) / RAND_MAX;
  const double r2 = ( (double) rand() ) / RAND_MAX;
  const double val = sqrt(-2.0 * log(r1)) * cos(2.0 * M_PI * r2);
  return s * val + m;
}

static double ymd( double year, double month, double d )
{
  int i, y = (int) year, m = (int) month; /* TODO: dangerous cast */
  double dy;

  if ( y < 1 || m < 1 || 12 < m || d < 1 || 365 < d )
  {
    return 0.0;
  }

  i = m - 1;
  --d;
  while ( 0 < i )
  {
    d += "DADCDCDDCDCD"[ --i] - '%'; /* days in month    */
  }
  if ( ( y % 4 == 0 && y % 100 != 0 ) || y % 400 == 0 ) /* leap year        */
  {
    if ( 2 < m )
    {
      ++d;
    }
    dy = 366.0;
  }
  else
  {
    dy = 365.0;
  }

  return ( y + d / dy );
}

c_math_parser * c_math_parser_create_stdc(void)
{
  c_math_parser * ctx = c_math_parser_create();
  if ( ctx ) {
    c_math_parser_add_unary_operation(ctx, operator_unary_minus, "-", "unary minus");
    c_math_parser_add_unary_operation(ctx, operator_unary_plus, "+", "unary plus");
    c_math_parser_add_unary_operation(ctx, operator_logical_not, "!", "logical NOT");

    c_math_parser_set_priority_levels(ctx, 7);
    c_math_parser_add_binary_operation(ctx, 0, operator_logical_or,"||", "");
    c_math_parser_add_binary_operation(ctx, 1, operator_logical_and,"&&", "");
    c_math_parser_add_binary_operation(ctx, 2, operator_eq,"==", "");
    c_math_parser_add_binary_operation(ctx, 2, operator_not_eq,"!=", "");
    c_math_parser_add_binary_operation(ctx, 3, operator_lt,"<" , "");
    c_math_parser_add_binary_operation(ctx, 3, operator_le,"<=", "");
    c_math_parser_add_binary_operation(ctx, 3, operator_gt,">" , "");
    c_math_parser_add_binary_operation(ctx, 3, operator_ge,">=", "");
    c_math_parser_add_binary_operation(ctx, 4, operator_add, "+", "binary plus");
    c_math_parser_add_binary_operation(ctx, 4, operator_sub, "-", "binary minus");
    c_math_parser_add_binary_operation(ctx, 5, operator_mul, "*", "multiply");
    c_math_parser_add_binary_operation(ctx, 5, operator_div, "/",  "divide");
    c_math_parser_add_binary_operation(ctx, 6, pow, "**", "Fortran-style power");
    c_math_parser_add_binary_operation(ctx, 6, pow, "^",  "power");

    c_math_parser_add_constant(ctx, M_PI, "PI", 0);
    c_math_parser_add_constant(ctx, M_PI, "pi", 0);
    c_math_parser_add_constant(ctx, M_E, "E", 0);
    c_math_parser_add_constant(ctx, M_E, "e", 0);
    c_math_parser_add_constant(ctx, DBL_MIN, "absmin", "smallest number of given type.");
    c_math_parser_add_constant(ctx, DBL_MAX, "absmax", "greatest number of given type.");
    c_math_parser_add_constant(ctx, DBL_EPSILON, "eps", "smallest such 1 + eps != 1");

    c_math_parser_add_function1(ctx, fabs, "abs", "abs(x) Absolute value of x");
    c_math_parser_add_function1(ctx, fabs, "fabs", "fabs(x) synonym of abs(x)");
    c_math_parser_add_function1(ctx, acos, "acos", "acos(x) Arc cosine of x");
    c_math_parser_add_function1(ctx, asin, "asin", "asin(x) Arc sine of x");
    c_math_parser_add_function1(ctx, atan, "atan", "atan(x) Arc tangent of x");
    c_math_parser_add_function2(ctx, atan2, "atan2", "atan2(x,y) Arc tangent of y/x.");
    c_math_parser_add_function1(ctx, cos, "cos", "cos(x) Cosine of x.");
    c_math_parser_add_function1(ctx, sin, "sin", "sin(x) Sine of x.");
    c_math_parser_add_function1(ctx, tan, "tan", "tan(x) Tangent of x.");
    c_math_parser_add_function1(ctx, cosh, "cosh", "cosh(x) Hyperbolic cosine of x.");
    c_math_parser_add_function1(ctx, sinh, "sinh", "sinh(x) Hyperbolic sine of x.");
    c_math_parser_add_function1(ctx, tanh, "tanh", "tanh(x) Hyperbolic tangent of x.");
    c_math_parser_add_function1(ctx, exp, "exp", "exp(x) Exponential function of x.");
    c_math_parser_add_function1(ctx, log, "ln", "ln(x) Natural logarithm of x.");
    c_math_parser_add_function1(ctx, log, "log", "log(x) Natural logarithm of x.");
    c_math_parser_add_function1(ctx, log10, "lg", "lg(x) Base-ten logarithm of x.");
    c_math_parser_add_function1(ctx, log10, "log10", "log10(x) Base-ten logarithm of x.");
    c_math_parser_add_function2(ctx, pow, "pow", "pow(x,y) Return x to the y power.");
    c_math_parser_add_function1(ctx, sqrt, "sqrt", "sqrt(x) The square root of x.");
    c_math_parser_add_function2(ctx, hypot, "hypot", "hypot(x,y) Return `sqrt(x*x + y*y)'.");
    c_math_parser_add_function1(ctx, ceil, "ceil", "ceil(x) Smallest integral value not less than x.");
    c_math_parser_add_function1(ctx, floor, "floor", "floor(x) Largest integer not greater than x.");
    c_math_parser_add_function2(ctx, fmod, "fmod", "fmod(x,y) Floating-point modulo remainder of x/y.");
    c_math_parser_add_function1(ctx, j0, "j0", "The j0(x) bessel function");
    c_math_parser_add_function1(ctx, j1, "j1", "The j1(x) bessel function");
    c_math_parser_add_function1(ctx, y0, "y0", "The y0(x) bessel function");
    c_math_parser_add_function1(ctx, y1, "y1", "The y1(x) bessel function");

    c_math_parser_add_function1(ctx, sinc, "sinc", "sinc(x)/x");
    c_math_parser_add_function1(ctx, gexp, "gexp", "exp(-x^2/2)");
    c_math_parser_add_function2(ctx, frand, "rand", "rand(min,max) - uniform random number in the range [min..max]");
    c_math_parser_add_function2(ctx, noise, "noise","noise(s,m]) - gaussian noise with mean m and variance s.");

    c_math_parser_add_function1(ctx, acosh, "acosh", "acosh(x) inverse hyperbolic cosine of x");
    c_math_parser_add_function1(ctx, asinh, "asinh", "asinh(x) inverse hyperbolic sine of x");
    c_math_parser_add_function1(ctx, atanh, "atanh", "atanh(x) return the inverse hyperbolic tangent of x");
    c_math_parser_add_function1(ctx, expm1, "expm1", "expm1(x) return exp(x) - 1");
    c_math_parser_add_function1(ctx, cbrt, "cbrt", "cbrt(x) returns the (real) cube root of x");
    c_math_parser_add_function1(ctx, erf, "erf", "erf(x) returns the error function of x");
    c_math_parser_add_function1(ctx, erfc, "erfc", "erfc(x) returns the complementary error function of x,"
      "that is, 1.0 - erf(x)");
    c_math_parser_add_function1(ctx, lgamma, "lgamma", "lgamma(x) returns logarithm of the gamma function");
    c_math_parser_add_function1(ctx, tgamma, "tgamma", "tgamma(x) returns the ('true') gamma function");
    c_math_parser_add_function1(ctx, exp2, "exp2", "exp2(x) returns the value of 2 raised to the power of x");
    c_math_parser_add_function1(ctx, log2, "log2", "log2(x) returns the base 2 logarithm of x");
    c_math_parser_add_function1(ctx, round, "round", "round(x) round x to the nearest integer");
    c_math_parser_add_function1(ctx, trunc, "trunc",
      "trunc(x) round x to the nearest integer not larger in absolute value");
    c_math_parser_add_function1(ctx, log1p, "log1p", "log1p(x) returns the log (1 + x)");
    c_math_parser_add_function1(ctx,logb, "logb", "logb(x) extract the exponent from the internal "
      "floating-point representation of x and return it as a floating-point value");
    c_math_parser_add_function2(ctx, fdim, "fdim", "fdim(x,y) return the positive difference,"
      "max(x-y,0), between x and y");
    c_math_parser_add_function2(ctx, fmax, "fmax", "fmax(x,y) return greater value of x and y");
    c_math_parser_add_function2(ctx, fmin, "fmin", "fmin(x,y) return lesser value of x and y");
    c_math_parser_add_function3(ctx, fma, "fma", "fma(x,y,z) computes x * y + z");
    c_math_parser_add_function2(ctx, copysign, "copysign", "copysign(x,y) ");
    c_math_parser_add_function2(ctx, nextafter, "nextafter",
      "nextafter(x,y) return the next representable floating-point value following x in the direction of y.");
    c_math_parser_add_function2(ctx, remainder, "remainder", "remainder(x,y) computes the remainder of dividing x by y.");
    c_math_parser_add_function2(ctx, drem, "drem", "drem(x,y) Obsolete synonym of remainder(x,y).");
    c_math_parser_add_function1(ctx, nearbyint, "nearbyint",
      "nearbyint(x) round  their argument to an integer value in floating-point format,"
      "using the current rounding direction");
    c_math_parser_add_function1(ctx, rint, "rint", "rint(x) like nearbyint(x) but can raise the inexact exception");
    c_math_parser_add_function1(ctx, significand, "significand",
      "significand(x) returns the mantissa of x scaled to the range [1,2)");

    c_math_parser_add_function2(ctx, aitofx, "aitofx", "aitofx(l,b) return hammer-aitoff X coordinate for given l,b");
    c_math_parser_add_function2(ctx, aitofy, "aitofy", "aitofy(l,b) return hammer-aitoff Y coordinate for given l,b");
    c_math_parser_add_function1(ctx, lwrap, "lwrap", "lwrap(l) wraps l from range 0..2*pi into range -pi..pi");

    c_math_parser_add_function3(ctx, ymd, "ymd", "ymd(year,month,day) computes full year as double");
  }

  return ctx;
}

