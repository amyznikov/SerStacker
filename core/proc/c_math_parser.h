/*
 * c_math_parser.h
 *
 *  Created on: Mar 14, 2012
 *      Author: amyznikov
 *
 *  Updated :
 *    October 24, 2023
 */

#ifndef __c_math_parser_h__
#define __c_math_parser_h__

#include <memory>

// TODO: make c_math_parser thread-safe ?
struct c_math_parser;

//typedef std::shared_ptr<c_math_parser>
//  c_math_parser_ptr;

c_math_parser* c_math_parser_create(void);
c_math_parser* c_math_parser_create_stdc(void);
void c_math_parser_destroy(c_math_parser*);
void c_math_parser_cleanup(c_math_parser*);
const char* c_math_parser_parse(c_math_parser * ctx, const char * string);
double c_math_parser_eval(c_math_parser * ctx, const double args[]);
const char* c_math_parser_get_error_message(c_math_parser * ctx);


typedef double (*cmpxf0_t)();
typedef double (*cmpxf1_t)(double);
typedef double (*cmpxf2_t)(double, double);
typedef double (*cmpxf3_t)(double, double, double);
typedef double (*cmpxf4_t)(double, double, double, double);
typedef double (*cmpxf5_t)(double, double, double, double, double);
typedef double (*cmpxf6_t)(double, double, double, double, double, double);
typedef double (*cmpxf7_t)(double, double, double, double, double, double, double);
typedef double (*cmpxf8_t)(double, double, double, double, double, double, double, double);
typedef double (*cmpxf9_t)(double, double, double, double, double, double, double, double, double);
typedef double (*cmpxff_t)(void * param, const double args[], int numargs);



int c_math_parser_add_constant(c_math_parser * ctx, const double value, const char * name, const char * desc);
int c_math_parser_remove_constant(c_math_parser * ctx, const char * name);
int c_math_parser_remove_all_constants(c_math_parser * ctx);

int c_math_parser_add_argument(c_math_parser * ctx, int arg_index, const char * name, const char * desc);
int c_math_parser_remove_all_arguments(c_math_parser * ctx);

int c_math_parser_bind(c_math_parser * ctx, double * value, const char * name, const char * desc);
int c_math_parser_unbind(c_math_parser * ctx, const char * name);
int c_math_parser_unbind_all(c_math_parser * ctx);

int c_math_parser_add_function0(c_math_parser * ctx, cmpxf0_t f, const char * name, const char * desc);
int c_math_parser_add_function1(c_math_parser * ctx, cmpxf1_t f, const char * name, const char * desc);
int c_math_parser_add_function2(c_math_parser * ctx, cmpxf2_t f, const char * name, const char * desc);
int c_math_parser_add_function3(c_math_parser * ctx, cmpxf3_t f, const char * name, const char * desc);
int c_math_parser_add_function4(c_math_parser * ctx, cmpxf4_t f, const char * name, const char * desc);
int c_math_parser_add_function5(c_math_parser * ctx, cmpxf5_t f, const char * name, const char * desc);
int c_math_parser_add_function6(c_math_parser * ctx, cmpxf6_t f, const char * name, const char * desc);
int c_math_parser_add_function7(c_math_parser * ctx, cmpxf7_t f, const char * name, const char * desc);
int c_math_parser_add_function8(c_math_parser * ctx, cmpxf8_t f, const char * name, const char * desc);
int c_math_parser_add_function9(c_math_parser * ctx, cmpxf9_t f, const char * name, const char * desc);
int c_math_parser_add_functor(c_math_parser * ctx, cmpxff_t f, int numargs, void * param, const char * name, const char * desc);
int c_math_parser_add_parsed_expression(c_math_parser * ctx, c_math_parser * f, const char * name, const char * desc);
int c_math_parser_add_unary_operation(c_math_parser * ctx, cmpxf1_t f, const char * name, const char * desc);
int c_math_parser_add_binary_operation(c_math_parser * ctx, int priority, cmpxf2_t f, const char * name,const char * desc);

int c_math_parser_set_priority_levels(c_math_parser * ctx, int levels);

int c_math_parser_set_obrace(c_math_parser * ctx, int ch);
int c_math_parser_set_cbrace(c_math_parser * ctx, int ch);
int c_math_parser_get_obrace(c_math_parser * ctx);
int c_math_parser_get_cbrace(c_math_parser * ctx);
int c_math_parser_set_arglist_delimiter(c_math_parser * ctx, int ch);
int c_math_parser_get_arglist_delimiter(c_math_parser * ctx);

#endif /* __c_math_parser_h__ */
