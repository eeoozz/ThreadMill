/////////////////////////////////////////////////////////////////////////////
//
// digital_filters.h
//
// Created on: 2018.01.02
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DIGITAL_FILTERS
#define DIGITAL_FILTERS

//#include "std_c.h"

////////////////////////////////////////////////////////////////////////////
// Digital filter identifiers (needed due to use of static arrays)
// TODO: implement a better system using static storage of filter identifiers
////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_DIGITAL_FILTERS		1
#define MAX_ORDER_FILT				6

#define DIGITAL_FILTER_0		0
#define DIGITAL_FILTER_1		1
#define DIGITAL_FILTER_2		2
#define DIGITAL_FILTER_3		3
#define DIGITAL_FILTER_4		4
#define DIGITAL_FILTER_5		5
#define DIGITAL_FILTER_6		6
#define DIGITAL_FILTER_7		7
#define DIGITAL_FILTER_8		8
#define DIGITAL_FILTER_9		9

////////////////////////////////////////////////////////////////////////////
// Filter functions:
////////////////////////////////////////////////////////////////////////////

// General digital filter functions:
void	init_digital_filters();
void	init_filter(int id_i);
void	free_digital_filters();
double	digital_filter_tf(double, const int, const double*, const double*, const int);
double	digital_filter_tf_init(double, const int, const double*, const double*, const int, int*);
void	digital_filter_help(double, double*, double*, const int, const double*, const double*);

////////////////////////////////////////////////////////////////////////////
// Filter parameters:
////////////////////////////////////////////////////////////////////////////

// Low-pass filters:
void param_ff_ctrl(double* B, double* A);

#endif
