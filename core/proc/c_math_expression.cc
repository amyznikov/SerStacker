/*
 * c_math_expression.cc
 *
 *  Created on: Jan 2, 2024
 *      Author: amyznikov
 *
 *

 * Node hierarchy
 *    abstract_node
 *      |--> value_node
 *      |--> bound_parameter_node
 *      |--> arg_node
 *      |--> functional_node
 *            |--> fn*_node
 *            |--> ff_node
 *
 *
 */

#include "c_math_expression.h"
#include <algorithm>
#include <string.h>
#include <climits>
#include <cfloat>
#include <cmath>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace {

typedef c_math_expression::abstract_node
    c_abstract_node;

typedef c_math_expression::func00 func00;
typedef c_math_expression::func01 func01;
typedef c_math_expression::func02 func02;
typedef c_math_expression::func03 func03;
typedef c_math_expression::func04 func04;
typedef c_math_expression::func05 func05;
typedef c_math_expression::func06 func06;
typedef c_math_expression::func07 func07;
typedef c_math_expression::func08 func08;
typedef c_math_expression::func09 func09;
typedef c_math_expression::func10 func10;
typedef c_math_expression::funcfn funcfn;




static double max_func(double a, double b)
{
  return a >= b ? a : b;
}

static double min_func(double a, double b)
{
  return a <= b ? a : b;
}

static double operator_add(double x, double y)
{
  return x + y;
}
static double operator_sub(double x, double y)
{
  return x - y;
}
static double operator_mul(double x, double y)
{
  return x * y;
}
static double operator_div(double x, double y)
{
  return x / y;
}
static double operator_unary_minus(double x)
{
  return -x;
}
static double operator_unary_plus(double x)
{
  return x;
}
static double operator_logical_not(double x)
{
  return !x;
}
static double operator_bitwise_not(double x)
{
  return ~((unsigned int)(x));
}
static double operator_logical_or(double x, double y)
{
  return x || y;
}
static double operator_logical_and(double x, double y)
{
  return x && y;
}
static double operator_bitwise_or(double x, double y)
{
  return ((unsigned int)(x)) | ((unsigned int)(y)) ;
}
static double operator_bitwise_xor(double x, double y)
{
  return ((unsigned int)(x)) ^ ((unsigned int)(y)) ;
}
static double operator_bitwise_and(double x, double y)
{
  return ((unsigned int)(x)) & ((unsigned int)(y)) ;
}
static double operator_bitwise_lshift(double x, double y)
{
  return ((unsigned int)(x)) << ((unsigned int)(y)) ;
}
static double operator_bitwise_rshift(double x, double y)
{
  return ((unsigned int)(x)) >> ((unsigned int)(y)) ;
}
static double operator_remainder(double x, double y)
{
  return ((unsigned int)(x)) % ((unsigned int)(y)) ;
}
static double operator_eq(double x, double y)
{
  return x == y;
}
static double operator_not_eq(double x, double y)
{
  return x != y;
}
static double operator_lt(double x, double y)
{
  return x < y;
}
static double operator_le(double x, double y)
{
  return x <= y;
}
static double operator_gt(double x, double y)
{
  return x > y;
}
static double operator_ge(double x, double y)
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
static double frand(double xmin, double xmax)
{
  return xmin + rand() * (xmax - xmin) / RAND_MAX;
}

/** wrap l from range 0..2*pi into range -pi..pi */
static inline double lwrap(double l)
{
  return l > M_PI ? l - 2 * M_PI : l;
}

static double if_func(double cond, double expr_if_true, double expr_if_false)
{
  return cond ? expr_if_true : expr_if_false;
}

///**
// * Get hammer-aitoff coordinates for given l,b;
// * l in range [-pi,pi], b in range [-pi/2,pi/2]
// */
//static inline void haxy(double l, double b, double * x, double * y)
//{
//  const double z = sqrt(1 + cos(b) * cos(l / 2));
//  *x = cos(b) * sin(l / 2) / z;
//  *y = sin(b) / z;
//}

static double aitofx(double l, double b)
{
  double z;
  l = lwrap(l), z = sqrt(1 + cos(b) * cos(l / 2));
  return (cos(b) * sin(l / 2) / z);
}

static double aitofy(double l, double b)
{
  double z;
  l = lwrap(l), z = sqrt(1 + cos(b) * cos(l / 2));
  return (sin(b) / z);
}


static double ymd( double year, double month, double d )
{
  int i, y = (int) year, m = (int) month; /* TODO: dangerous cast */
  double dy;

  if ( y < 1 || m < 1 || 12 < m || d < 1 || 365 < d ) {
    return 0.0;
  }

  i = m - 1;
  --d;
  while ( 0 < i ) {
    d += "DADCDCDDCDCD"[ --i] - '%'; /* days in month    */
  }

  if ( ( y % 4 == 0 && y % 100 != 0 ) || y % 400 == 0 ) { /* leap year        */
    if ( 2 < m ) {
      ++d;
    }
    dy = 366.0;
  }
  else {
    dy = 365.0;
  }

  return (y + d / dy);
}

/*!
 * Gaussian noise with mean m and variance s,
 * uses the Box-Muller transformation
 */
static double noise(double m, double s)
{
  const double r1 = ( (double) rand() ) / RAND_MAX;
  const double r2 = ( (double) rand() ) / RAND_MAX;
  const double val = sqrt(-2.0 * log(r1)) * cos(2.0 * M_PI * r2);
  return s * val + m;
}

/**
 * @brief Trey Wilson closed-form solution for shortest distance between two angles in given range (0..2pi).
 *      Angles do NOT need to be normalized.
 * @see <https://gist.github.com/shaunlebron/8832585>
**/
static double shortdist(double a1, double a2, double range /*= 2 * M_PI*/)
{
  const double da = fmod(a2 - a1, range);
  return fmod(2 * da, range) - da;
}


static double bgr2gray(double b, double g, double r)
{
  return 0.299 * r + 0.587 * g + 0.114 * b;
}

class c_value_node :
    public c_abstract_node
{
  double value;

public:
  c_value_node(double _value) :
    value (_value)
  {
  }

  double eval(const double args[]) const override
  {
    (void)(args);
    return value;
  }

  bool is_const_expression() const override
  {
    return true;
  }

};

class c_bound_parameter_node :
    public c_abstract_node
{
  double * value;

public:
  c_bound_parameter_node(double * _value) :
    value(_value)
  {
  }

  double eval(const double args[]) const override
  {
    (void)(args);
    return *value;
  }

  bool is_const_expression() const override
  {
    return false;
  }

};

class c_arg_node :
    public c_abstract_node
{
  int argindex;

public:

  c_arg_node(int _argindex) :
    argindex(_argindex)
  {
  }

  double eval(const double args[]) const override
  {
    return args[argindex];
  }

  bool is_const_expression() const override
  {
    return false;
  }

};


class c_functional_node :
    public c_abstract_node
{
public:

  c_functional_node()
  {
  }

  c_functional_node(const func00 & f) :
      f00(f)
  {
  }

  c_functional_node(const func01 & f, c_abstract_node * arg) :
      f01(f)
  {
    args_.emplace_back(arg);
  }

  c_functional_node(const func01 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f01(f)
  {
  }

  c_functional_node(const func02 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f02(f)
  {
  }


  c_functional_node(const func03 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f03(f)
  {
  }

  ////////
  c_functional_node(const func04 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f04(f)
  {
  }

  c_functional_node(const func05 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f05(f)
  {
  }

  c_functional_node(const func06 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f06(f)
  {
  }

  c_functional_node(const func07 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f07(f)
  {
  }

  c_functional_node(const func08 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f08(f)
  {
  }

  c_functional_node(const func09 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f09(f)
  {
  }

  c_functional_node(const func10 & f, const std::vector<c_abstract_node*> & args) :
      args_(args),
      f10(f)
  {
  }

  /////////

  ~c_functional_node() override
  {
    for ( int i = 0, n = (int)args_.size(); i < n; ++i ) {
      delete args_[i];
    }
    args_.clear();
  }

  double eval(const double args[]) const override
  {
    switch (args_.size()) {
      case 0: {
        return f00();
      }
      case 1: {
        return f01(args_[0]->eval(args));
      }
      case 2: {
        return f02(args_[0]->eval(args),
            args_[1]->eval(args));
      }
      case 3: {
        return f03(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args));
      }
      case 4: {
        return f04(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args));
      }
      case 5: {
        return f05(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args));
      }
      case 6: {
        return f06(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args),
            args_[5]->eval(args));
      }
      case 7: {
        return f07(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args),
            args_[5]->eval(args),
            args_[6]->eval(args));
      }
      case 8: {
        return f08(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args),
            args_[5]->eval(args),
            args_[6]->eval(args),
            args_[7]->eval(args));
      }
      case 9: {
        return f09(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args),
            args_[5]->eval(args),
            args_[6]->eval(args),
            args_[7]->eval(args),
            args_[8]->eval(args));
      }
      case 10: {
        return f10(args_[0]->eval(args),
            args_[1]->eval(args),
            args_[2]->eval(args),
            args_[3]->eval(args),
            args_[4]->eval(args),
            args_[5]->eval(args),
            args_[6]->eval(args),
            args_[7]->eval(args),
            args_[8]->eval(args),
            args_[9]->eval(args));
      }

    }

    return 0;
  }

  bool is_const_expression() const override
  {
    if( f02 == &noise ) {
      return false;
    }

    for( const auto arg : args_ ) {
      if( !arg->is_const_expression() ) {
        return false;
      }
    }

    return true;
  }

protected:
  c_functional_node(const std::vector<c_abstract_node*> & args) :
      args_(args)
  {
  }

protected:
  std::vector<c_abstract_node*> args_;

  union {
    func00 f00;
    func01 f01;
    func02 f02;
    func03 f03;
    func04 f04;
    func05 f05;
    func06 f06;
    func07 f07;
    func08 f08;
    func09 f09;
    func10 f10;
  };

};

class c_ff_node:
    public c_functional_node
{
  funcfn fn;
  void * param;

public:
  c_ff_node(funcfn _fn, void * _param, const std::vector<c_abstract_node*> & args) :
      c_functional_node(args),
      fn(_fn),
      param(_param)
  {
  }

  double eval(const double args[]) const override
  {
    const int numargs =
        args_.size();

#if _WIN32
    std::vector<double> ff_args_(numargs);
    double * const ff_args = ff_args_.data();
#else
    double ff_args[numargs];
#endif

    for( int i = 0; i < numargs; ++i ) {
      ff_args[i] = args_[i]->eval(args);
    }

    return fn(param, ff_args, numargs);
  }

  bool is_const_expression() const override
  {
    return false;
  }

};

static const char* skip_white_spaces(const char ** curpos)
{
  while (isspace(**curpos)) {
    ++*curpos;
  }
  return *curpos;
}

static bool parse_number(double * value, const char * curpos, char ** endptr)
{
  *value = strtod(curpos, endptr);
  return *endptr > curpos;
}

static bool can_be_part_of_identifier(char ch)
{
  return isalnum(ch) || ch == '_' || ch == '$';
}

} // namespace

c_math_expression::c_math_expression()
{
  add_unary_operation(operator_unary_minus, "-", "unary minus");
  add_unary_operation(operator_unary_plus, "+", "unary plus");
  add_unary_operation(operator_logical_not, "!", "logical NOT");
  add_unary_operation(operator_bitwise_not, "~", "bitwise NOT");

  add_binary_operation(0, operator_logical_or, "||", "logical OR");
  add_binary_operation(1, operator_logical_and, "&&", "logical AND");
  add_binary_operation(2, operator_bitwise_or, "|", "integer bitwise OR");
  add_binary_operation(3, operator_bitwise_xor, "^", "integer bitwise XOR");
  add_binary_operation(4, operator_bitwise_and, "&", "integer bitwise AND");
  add_binary_operation(5, operator_eq,"==", "Equality");
  add_binary_operation(5, operator_not_eq,"!=", "Not Equality");
  add_binary_operation(6, operator_lt,"<" , "Less than");
  add_binary_operation(6, operator_le,"<=", "Less or equal");
  add_binary_operation(6, operator_gt,">" , "Greater than");
  add_binary_operation(6, operator_ge,">=", "Greater or equal");
  add_binary_operation(7, operator_bitwise_lshift,"<<", "integer bitwise left shift");
  add_binary_operation(7, operator_bitwise_rshift,">>", "integer bitwise right shift");
  add_binary_operation(8, operator_add, "+", "binary plus");
  add_binary_operation(8, operator_sub, "-", "binary minus");
  add_binary_operation(9, operator_mul, "*", "multiply");
  add_binary_operation(9, operator_div, "/",  "divide");
  add_binary_operation(9, operator_remainder, "%",  "integer remainder");
  add_binary_operation(10, pow, "**", "Fortran-style power");

  add_constant(M_PI, "PI", nullptr);
  add_constant(M_PI, "pi", nullptr);
  add_constant(M_E, "E", nullptr);
  add_constant(M_E, "e", nullptr);
  add_constant(DBL_MIN, "absmin", "smallest number of given type.");
  add_constant(DBL_MAX, "absmax", "greatest number of given type.");
  add_constant(DBL_EPSILON, "eps", "smallest such 1 + eps != 1");

  add_function(fabs, "abs", "abs(x) Absolute value of x");
  add_function(fabs, "fabs", "fabs(x) synonym of abs(x)");
  add_function(acos, "acos", "acos(x) Arc cosine of x");
  add_function(asin, "asin", "asin(x) Arc sine of x");
  add_function(atan, "atan", "atan(x) Arc tangent of x");
  add_function(atan2, "atan2", "atan2(x,y) Arc tangent of y/x.");
  add_function(cos, "cos", "cos(x) Cosine of x.");
  add_function(sin, "sin", "sin(x) Sine of x.");
  add_function(tan, "tan", "tan(x) Tangent of x.");
  add_function(cosh, "cosh", "cosh(x) Hyperbolic cosine of x.");
  add_function(sinh, "sinh", "sinh(x) Hyperbolic sine of x.");
  add_function(tanh, "tanh", "tanh(x) Hyperbolic tangent of x.");
  add_function(exp, "exp", "exp(x) Exponential function of x.");
  add_function(log, "ln", "ln(x) Natural logarithm of x.");
  add_function(log, "log", "log(x) Natural logarithm of x.");
  add_function(log10, "lg", "lg(x) Base-ten logarithm of x.");
  add_function(log10, "log10", "log10(x) Base-ten logarithm of x.");
  add_function(pow, "pow", "pow(x,y) Return x to the y power.");
  add_function(sqrt, "sqrt", "sqrt(x) The square root of x.");
  add_function(hypot, "hypot", "hypot(x,y) Return `sqrt(x*x + y*y)'.");
  add_function(ceil, "ceil", "ceil(x) Smallest integral value not less than x.");
  add_function(floor, "floor", "floor(x) Largest integer not greater than x.");
  add_function(fmod, "fmod", "fmod(x,y) Floating-point modulo remainder of x/y.");
  add_function(j0, "j0", "The j0(x) bessel function");
  add_function(j1, "j1", "The j1(x) bessel function");
  add_function(y0, "y0", "The y0(x) bessel function");
  add_function(y1, "y1", "The y1(x) bessel function");

  add_function(sinc, "sinc", "sinc(x)/x");
  add_function(gexp, "gexp", "exp(-x^2/2)");
  add_function(frand, "rand", "rand(min,max) - uniform random number in the range [min..max]");
  add_function(noise, "noise","noise(m, s) - random noise with mean m and variance s using rand() and Box-Muller transformation.");

  add_function(acosh, "acosh", "acosh(x) inverse hyperbolic cosine of x");
  add_function(asinh, "asinh", "asinh(x) inverse hyperbolic sine of x");
  add_function(atanh, "atanh", "atanh(x) return the inverse hyperbolic tangent of x");
  add_function(expm1, "expm1", "expm1(x) return exp(x) - 1");
  add_function(cbrt, "cbrt", "cbrt(x) returns the (real) cube root of x");
  add_function(erf, "erf", "erf(x) returns the error function of x");
  add_function(erfc, "erfc", "erfc(x) returns the complementary error function of x,"
    "that is, 1.0 - erf(x)");
  add_function(lgamma, "lgamma", "lgamma(x) returns logarithm of the gamma function");
  add_function(tgamma, "tgamma", "tgamma(x) returns the ('true') gamma function");
  add_function(exp2, "exp2", "exp2(x) returns the value of 2 raised to the power of x");
  add_function(log2, "log2", "log2(x) returns the base 2 logarithm of x");
  add_function(round, "round", "round(x) round x to the nearest integer");
  add_function(trunc, "trunc", "trunc(x) round x to the nearest integer not larger in absolute value");
  add_function(log1p, "log1p", "log1p(x) returns the log (1 + x)");
  add_function(logb, "logb", "logb(x) extract the exponent from the internal "
    "floating-point representation of x and return it as a floating-point value");
  add_function(fdim, "fdim", "fdim(x,y) return the positive difference, max(x-y,0), between x and y");
  add_function(fmax, "fmax", "fmax(x,y) return greater value of x and y");
  add_function(fmin, "fmin", "fmin(x,y) return lesser value of x and y");
  add_function(fma, "fma", "fma(x,y,z) computes x * y + z");
  add_function(copysign, "copysign", "copysign(x,y) ");
  add_function(nextafter, "nextafter",
    "nextafter(x,y) return the next representable floating-point value following x in the direction of y.");
  add_function(remainder, "remainder", "remainder(x,y) computes the remainder of dividing x by y.");
  // add_function(drem, "drem", "drem(x,y) Obsolete synonym of remainder(x,y).");

#ifndef _WIN32
  add_function(significand, "significand",
    "significand(x) returns the mantissa of x scaled to the range [1,2)");
#endif

  add_function(nearbyint, "nearbyint",
    "nearbyint(x) round  their argument to an integer value in floating-point format,"
    "using the current rounding direction");
  add_function(rint, "rint", "rint(x) like nearbyint(x) but can raise the inexact exception");

  add_function(aitofx, "aitofx", "aitofx(l,b) return hammer-aitoff X coordinate for given l,b");
  add_function(aitofy, "aitofy", "aitofy(l,b) return hammer-aitoff Y coordinate for given l,b");
  add_function(lwrap, "lwrap", "lwrap(l) wraps l from range 0..2*pi into range -pi..pi");

  add_function(ymd, "ymd", "ymd(year,month,day) computes full year as double");

  add_function(if_func, "if", "if(condition, expr_if_true, expr_if_false)");
  add_function(min_func, "min", "min(v1, v2)");
  add_function(max_func, "max", "max(v1, v2)");
  add_function(shortdist, "shortdist", "shortdist(a, b, range=2*pi) is "
      "Trey Wilson closed-form solution for shortest distance between two angles a and b in given range [0..2pi]");

  add_function(bgr2gray, "bgr2gray", "bgr2gray(b, g, r) : "
      "Convert BGR color to gray value : 0.299 * r + 0.587 * g + 0.114 * b");

  std::sort(_functions.begin(), _functions.end(),
      [](const auto & prev, const auto & next) {
        return prev.name < next.name;
      });
}


c_math_expression::~c_math_expression()
{
  clear_args();
}

void c_math_expression::clear_args()
{
  _args.clear();
}

const std::string & c_math_expression::error_message() const
{
  return _errmsg;
}

const char * c_math_expression::pointer_to_syntax_error() const
{
  return _pointer_to_syntax_error;
}

void c_math_expression::clear_errmsg()
{
  _errmsg.clear();
  _pointer_to_syntax_error = nullptr;
}

void c_math_expression::set_errmsg(const char * format, ...)
{
  va_list arglist;

  va_start(arglist, format);
  set_errmsgv(format, arglist);
  va_end(arglist);
}

void c_math_expression::set_errmsgv(const char * format, va_list arglist)
{
  _errmsg.resize(4098);
  vsnprintf(_errmsg.data(), 4097, format, arglist);
  _errmsg[4097] = 0;
}

void c_math_expression::cleanup()
{
  if ( _root ) {
    delete _root;
    _root = 0;
  }
}

double c_math_expression::eval(const double args[]) const
{
  return _root ? _root->eval(args) : 0;
}

const c_math_expression::unary_operation * c_math_expression::lookup_unary_operator(const char *curpos) const
{
  const unary_operation * best_match =
      nullptr;

  size_t maxlen = 0;

  for ( size_t i = 0, n = _unops.size(); i < n; ++i ) {

    const unary_operation * op =
        &_unops[i];

    const size_t oplen =
        strlen(op->name.c_str());

    if ( oplen > maxlen && strncmp(op->name.c_str(), curpos, oplen) == 0 ) {
      maxlen = oplen;
      best_match = op;
    }
  }

  return best_match;
}

const c_math_expression::binary_operation* c_math_expression::lookup_binary_operator(int priority_level, const char * curpos) const
{
  const binary_operation * best_match =
      nullptr;

  if( priority_level >= 0 && priority_level < (int) _binops.size() ) {

    size_t maxlen = 0;
    int best_priority = -1;

    for ( int p = 0; p <= priority_level; ++p ) {
      for ( size_t i = 0, n = _binops[p].size(); i < n; ++i ) {

        const binary_operation * op =
            &_binops[p][i];

        const size_t oplen =
            strlen(op->name.c_str());

        if( oplen > maxlen && strncmp(op->name.c_str(), curpos, oplen) == 0 ) {
          maxlen = oplen;
          best_match = op;
          best_priority = p;
        }
      }
    }

    if ( best_priority != priority_level ) {
      best_match = nullptr;
    }

  }

  return best_match;
}

const c_math_expression::arg_desc* c_math_expression::lookup_argument(const char * name) const
{
  for( size_t i = 0, n = _args.size(); i < n; ++i) {
    const arg_desc * arg = &_args[i];
    if( strcmp(arg->name.c_str(), name) == 0 ) {
      return arg;
    }
  }
  return nullptr;
}

const c_math_expression::binding_desc* c_math_expression::lookup_bindings(const char * name) const
{
  for( size_t i = 0, n = _bindings.size(); i < n; ++i) {
    const binding_desc * binding = &_bindings[i];
    if( strcmp(binding->name.c_str(), name) == 0 ) {
      return binding;
    }
  }
  return nullptr;
}

const c_math_expression::const_desc* c_math_expression::lookup_constant(const char * name) const
{
  for( size_t i = 0, n = _constants.size(); i < n; ++i) {
    const const_desc * c = &_constants[i];
    if( strcmp(c->name.c_str(), name) == 0 ) {
      return c;
    }
  }
  return nullptr;
}

const c_math_expression::function_desc* c_math_expression::lookup_function(const char * name) const
{
  for( size_t i = 0, n = _functions.size(); i < n; ++i) {
    const function_desc * f = &_functions[i];
    if( strcmp(f->name.c_str(), name) == 0 ) {
      return f;
    }
  }
  return nullptr;
}

bool c_math_expression::parse_terminal_token(const char ** curpos, abstract_node ** ppnode)
{
  /*
   * terminal_token:
   *  [unary-operation] <(expression)|numerical-constant|argument-name|bound-parameter-name|named-constant|function-name>
   */

  const unary_operation * unop = 0;

  const arg_desc * af = 0;
  const binding_desc * pf = 0;
  const const_desc * cf = 0;
  const function_desc * ff = 0;

  std::string name;
  const char * tmp1;
  //char * tmp2;

  *ppnode = 0;

  /* Check if we have an unary operation */
  if( (unop = lookup_unary_operator(skip_white_spaces(curpos))) != 0 ) {

    *curpos += unop->name.size();

    if ( parse_terminal_token(curpos, ppnode) ) {

      *ppnode = new c_functional_node(unop->fn,  *ppnode);

      if ( (*ppnode)->is_const_expression() ) {

        const double value =
            (*ppnode)->eval(nullptr);

        delete(*ppnode);

        *ppnode =
            new c_value_node(value);
      }

    }

    return *ppnode != 0;
  }

  /* Check if we have an subexpression in braces */
  if ( **curpos == OBRACE ) {

    ++*curpos;

    if ( !parse_expression(0, curpos, ppnode) ) {
      return 0;
    }

    if ( *skip_white_spaces(curpos) != CBRACE ) {
      set_errmsg("expected '%c' ", CBRACE);
      delete *ppnode;
      *ppnode = nullptr;
      return false;
    }

    ++*curpos;

    return true;
  }

  /* Check if we have an numerical value */
  if ( 1 ) {
    double value;
    char * endptr;
    if ( parse_number(&value, *curpos, &endptr) ) {
      *curpos = endptr;
      *ppnode = new c_value_node(value);
      skip_white_spaces(curpos);
      return 1;
    }
  }

  /*
   * Check if we have an named object
   * TODO: check array bounds for name[]
   */
  //tmp2 = name;
  tmp1 = skip_white_spaces(curpos);
  while ( can_be_part_of_identifier(*tmp1) ) {
    name += *tmp1++;
  }

  if ( ( af = lookup_argument(name.c_str()) ) ) {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = new c_arg_node(af->index);
  }
  else if ( ( pf = lookup_bindings(name.c_str()) ) ) {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = new c_bound_parameter_node(pf->value);
  }
  else if ( ( cf = lookup_constant(name.c_str()) ) ) {
    *curpos = tmp1;
    skip_white_spaces(curpos);
    *ppnode = new c_value_node(cf->value);
  }
  else if( (ff = lookup_function(name.c_str())) ) {
    std::vector<c_abstract_node*> args;
    c_abstract_node * arg = 0;
    bool success = true;

    *curpos = tmp1;
    if ( *skip_white_spaces(curpos) != OBRACE ) {
      set_errmsg("missing '%c' in function call '%s'", OBRACE, ff->name.c_str());
      return 0;
    }

    //
    // parse argument list
    //
    ++*curpos;

    while( args.size() < ff->numargs ) {

      if ( *skip_white_spaces(curpos) == CBRACE ) {
        set_errmsg("%s expects %d arguments", name.c_str(), ff->numargs);
        success = false;
        break;
      }

      if( !(success = parse_expression(0, curpos, &arg)) ) {
        break;
      }

      args.emplace_back(arg);

      if ( *skip_white_spaces(curpos) == ARGLIST_DELIMITER && args.size() < ff->numargs ) {
        ++*curpos;
      }
    }

    if ( args.size() != ff->numargs ) {
      set_errmsg("%s expects %d arguments", name.c_str(), ff->numargs);
      success = false;
    }
    else if ( **curpos != CBRACE ) {
      set_errmsg("expected '%c'", CBRACE);
      success = false;
    }

    if( !success ) {
      for( int ii = 0, nn = args.size(); ii < nn; ++ii ) {
        delete args[ii];
      }
      return false;
    }

    if( ff->param ) {
      *ppnode =
          new c_ff_node(ff->ffn,
              ff->param,
              args);
    }
    else {

      switch (ff->numargs) {

        case 0:
          *ppnode =
              new c_functional_node(ff->f00);
          break;

        case 1:
          *ppnode =
              new c_functional_node(ff->f01,
                  args);
          break;

      case 2:
        *ppnode =
            new c_functional_node(ff->f02,
                args);
        break;

      case 3:
        *ppnode =
            new c_functional_node(ff->f03,
                args);
        break;

      case 4:
        *ppnode =
            new c_functional_node(ff->f04,
                args);
        break;

      case 5:
        *ppnode =
            new c_functional_node(ff->f05,
                args);
        break;

      case 6:
        *ppnode =
            new c_functional_node(ff->f06,
                args);
        break;

      case 7:
        *ppnode =
            new c_functional_node(ff->f07,
                args);
        break;

      case 8:
        *ppnode =
            new c_functional_node(ff->f08,
                args);
        break;

      case 9:
        *ppnode =
            new c_functional_node(ff->f09,
                args);
        break;

      case 10:
        *ppnode =
            new c_functional_node(ff->f10,
                args);
        break;

      default:
        set_errmsg("BUG IN MATH PARSER SOURCE AT %s:%d", __FILE__, __LINE__);
        for( int ii = 0, nn = args.size(); ii < nn; ++ii ) {
          delete args[ii];
        }
        return false;
      }
    }

    if( (*ppnode)->is_const_expression() ) {

      const double value =
          (*ppnode)->eval(nullptr);

      delete (*ppnode);

      *ppnode =
          new c_value_node(value);
    }

    ++*curpos;
  }
  else if ( !name.empty() ) {
    set_errmsg("unknown identifier '%s'", name.c_str());
  }
  else if ( **curpos ) {
    set_errmsg("unexpected symbol '%c' in expression", **curpos);
  }

  return *ppnode != 0;

}


bool c_math_expression::parse_expression(size_t priority_level, const char ** curpos, abstract_node ** ppnode)
{
  /*
   * expression:
   *  arg1 binary-operation arg2 ...
   *
   * Binary operation priorities must be arranged from lowest priority at the begin of table
   * to the highest priority at the end of table
   */

  const binary_operation * binop = nullptr;
  std::vector<c_abstract_node*> args(2, nullptr);

  *ppnode = nullptr;

  if( priority_level >= _binops.size() ) {
    return parse_terminal_token(curpos, ppnode);
  }

  if( !parse_expression(priority_level + 1, curpos, &args[0]) ) {
    return false;
  }

  while ((binop = lookup_binary_operator(priority_level, skip_white_spaces(curpos)))) {

    *curpos +=
        binop->name.size();

    if( !parse_expression(priority_level + 1, curpos, &args[1]) ) {
      delete args[0];
      return false;
    }

    args[0] =
        new c_functional_node(binop->fn,
            args);

    if( args[0]->is_const_expression() ) {

      const double value =
          args[0]->eval(nullptr);

      delete args[0];

      args[0] =
          new c_value_node(value);
    }
  }

  *ppnode = args[0];

  return true;
}

bool c_math_expression::parse(const char * string)
{
  cleanup();
  clear_errmsg();

  if( !parse_expression(0, &string, &_root) ) {
    _pointer_to_syntax_error = string;
    return false;
  }

  if ( *string != 0 ) {
    cleanup();
    set_errmsg("Unexpected text after end of expression: '%s'", string);
    _pointer_to_syntax_error = string;
    return false;
  }

  return true;
}

bool c_math_expression::parse(const std::string & s)
{
  return parse(s.c_str());
}


bool c_math_expression::add_argument(int arg_index, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_args.begin(), _args.end(),
          [name, arg_index](const auto & f) {
            return f.name == name || f.index == arg_index;
          });

  if ( ii != _args.end() ) {
    set_errmsg("Argument '%s' already exists with index %d", name, ii->index);
    return false;
  }

  _args.emplace_back();

  arg_desc & f =
      _args.back();

  f.name = name;
  f.index = arg_index;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_constant(double value, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_constants.begin(), _constants.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _constants.end() ) {
    set_errmsg("Constant '%s' already exists with value = %g", name, ii->value);
    return false;
  }

  _constants.emplace_back();

  const_desc & f =
      _constants.back();

  f.name = name;
  f.value = value;
  if ( desc ) {
    f.desc = desc;
  }
  else {
    f.desc = ssprintf("%.15f", value);
  }

  return true;
}

bool c_math_expression::add_bind(double * value, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_bindings.begin(), _bindings.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _bindings.end() ) {
    set_errmsg("Binding '%s' already exists", name);
    return false;
  }

  _bindings.emplace_back();

  binding_desc & f =
      _bindings.back();

  f.name = name;
  f.value = value;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func00 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f00 = fn;
  f.numargs = 0;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func01 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f01 = fn;
  f.numargs = 1;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func02 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f02 = fn;
  f.numargs = 2;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func03 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f03 = fn;
  f.numargs = 3;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func04 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f04 = fn;
  f.numargs = 4;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func05 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f05 = fn;
  f.numargs = 5;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func06 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f06 = fn;
  f.numargs = 6;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func07 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f07 = fn;
  f.numargs = 7;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func08 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f08 = fn;
  f.numargs = 8;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func09 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f09 = fn;
  f.numargs = 9;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(func10 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.f10 = fn;
  f.numargs = 10;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_function(funcfn fn, void * param, int numargs, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_functions.begin(), _functions.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _functions.end() ) {
    set_errmsg("Function named '%s' already exists", name);
    return false;
  }

  _functions.emplace_back();

  function_desc & f =
      _functions.back();

  f.name = name;
  f.ffn = fn;
  f.numargs = numargs;
  f.param = param;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}


bool c_math_expression::add_unary_operation(func01 fn, const char * name, const char * desc)
{
  const auto ii =
      std::find_if(_unops.begin(), _unops.end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if ( ii != _unops.end() ) {
    set_errmsg("Unary operation '%s' already exists", name);
    return false;
  }

  _unops.emplace_back();

  unary_operation & f =
      _unops.back();

  f.name = name;
  f.fn = fn;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}

bool c_math_expression::add_binary_operation(int priority, func02 fn, const char * name, const char * desc)
{
  if( priority < 0 ) {
    set_errmsg("Invalid binary operation priority=%d for '%s'", priority, name);
    return false;
  }

  if( priority >= _binops.size() ) {
    _binops.resize(priority + 1);
  }

  const auto ii =
      std::find_if(_binops[priority].begin(), _binops[priority].end(),
          [name](const auto & f) {
            return f.name == name;
          });

  if( ii != _binops[priority].end() ) {
    set_errmsg("Binary operation '%s' already exists with priority=%d", name, priority);
    return false;
  }

  _binops[priority].emplace_back();

  binary_operation & f =
      _binops[priority].back();

  f.fn = fn;
  f.name = name;
  if ( desc ) {
    f.desc = desc;
  }

  return true;
}
