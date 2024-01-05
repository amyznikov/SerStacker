/*
 * c_math_expression.h
 *
 *  Created on: Jan 2, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_math_expression_h__
#define __c_math_expression_h__

#include <string>
#include <vector>
#include <memory>
#include <cstdio>
#include <cstdarg>

class c_math_expression
{
public:
  typedef c_math_expression this_class;
  typedef double (*func00)();
  typedef double (*func01)(double);
  typedef double (*func02)(double, double);
  typedef double (*func03)(double, double, double);
  typedef double (*func04)(double, double, double, double);
  typedef double (*func05)(double, double, double, double, double);
  typedef double (*func06)(double, double, double, double, double, double);
  typedef double (*func07)(double, double, double, double, double, double, double);
  typedef double (*func08)(double, double, double, double, double, double, double, double);
  typedef double (*func09)(double, double, double, double, double, double, double, double, double);
  typedef double (*func10)(double, double, double, double, double, double, double, double, double, double);
  typedef double (*funcfn)(void * param, const double args[], int numargs);

  struct abstract_node
  {
    virtual ~abstract_node() = default;
    virtual double eval(const double args[]) const = 0;
  };

  struct unary_operation
  {
    std::string name;
    std::string desc;
    func01 fn = nullptr;
  };

  struct binary_operation
  {
    std::string name;
    std::string desc;
    func02 fn = nullptr;
  };

  struct function_desc
  {
    std::string name;
    std::string desc;
    void * param = nullptr;   /*!< for functors only */
    int numargs = 0;
    int is_volatile = false;

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
      funcfn ffn;
    };

  };

  struct arg_desc
  {
    std::string name;
    std::string desc;
    int index = 0;
  };

  struct const_desc
  {
    std::string name;
    std::string desc;
    double value = 0;
  };

  struct binding_desc
  {
    std::string name;
    std::string desc;
    double * value = nullptr;
  };

  c_math_expression();
  ~c_math_expression();

  void clear_args();
  bool parse(const char * s);
  bool parse(const std::string & s);
  double eval(const double args[]) const;
  const std::string & error_message() const;
  const char * pointer_to_syntax_error() const;

  bool add_argument(int arg_index, const char * name, const char * desc);
  bool add_constant(double value, const char * name, const char * desc);
  bool add_bind(double * value, const char * name, const char * desc);

  bool add_function(func00, const char * name, const char * desc);
  bool add_function(func01, const char * name, const char * desc);
  bool add_function(func02, const char * name, const char * desc);
  bool add_function(func03, const char * name, const char * desc);
  bool add_function(func04, const char * name, const char * desc);
  bool add_function(func05, const char * name, const char * desc);
  bool add_function(func06, const char * name, const char * desc);
  bool add_function(func07, const char * name, const char * desc);
  bool add_function(func08, const char * name, const char * desc);
  bool add_function(func09, const char * name, const char * desc);
  bool add_function(func10, const char * name, const char * desc);
  bool add_function(funcfn, void * param, int numargs, const char * name, const char * desc);
  bool add_unary_operation(func01 fn, const char * name, const char * description);
  bool add_binary_operation(int priority, func02 fn, const char * name, const char * description);

  const std::vector<const_desc> & constants() const
  {
    return constants_;
  }

  const std::vector<function_desc> & functions() const
  {
    return functions_;
  }

  const std::vector<unary_operation> & unary_operations() const
  {
    return unops_;
  }

  const std::vector<std::vector<binary_operation>>& binary_operations() const
  {
    return binops_;
  }

protected:
  void clear_errmsg();
  void set_errmsgv(const char * format, va_list arglist);
  void set_errmsg(const char * format, ...)
#if !_MSC_VER
  __attribute__ ((__format__ (printf, 2, 3)));
#else
;
#endif

protected:
  void cleanup();
  bool parse_expression(size_t priority_level, const char ** curpos, abstract_node ** ppnode);
  bool parse_terminal_token(const char ** curpos, abstract_node ** ppnode);
  const unary_operation * lookup_unary_operator(const char *curpos) const;
  const binary_operation * lookup_binary_operator(int priority_level, const char * curpos) const;
  const arg_desc * lookup_argument(const char * name) const;
  const binding_desc * lookup_bindings(const char * name) const;
  const const_desc * lookup_constant(const char * name) const;
  const function_desc * lookup_function(const char * name) const;

protected:
  abstract_node * root_ = nullptr;
  const char * pointer_to_syntax_error_ = nullptr;
  std::string errmsg_;

  // functions table
  std::vector<function_desc> functions_;

  // unary operation table
  std::vector<unary_operation> unops_;

  // binary operation table
  std::vector<std::vector<binary_operation>> binops_;

  // arguments table
  std::vector<arg_desc> args_;

  // bindings table
  std::vector<binding_desc> bindings_;

  // constants table
  std::vector<const_desc> constants_;


  int OBRACE = '(';
  int CBRACE = ')';
  int ARGLIST_DELIMITER = ',';

};

#endif /* __c_math_expression_h__ */
