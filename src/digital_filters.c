/////////////////////////////////////////////////////////////////////////////
//
// digital_filters.c
//
// Created on: 2018.01.02
// Author: Gabriel Aguirre Ollinger
//
/////////////////////////////////////////////////////////////////////////////

# include "digital_filters.h"

static double*	x[MAX_NUM_DIGITAL_FILTERS]; // input samples arrays
static double*	y[MAX_NUM_DIGITAL_FILTERS]; // output samples arrays

// Digital filters initialization:
void
init_digital_filters() {
	// int	s_i;
	int	id_i;

	// Initialize data arrays for all filters:
	for (id_i = 0; id_i < MAX_NUM_DIGITAL_FILTERS; id_i++) {
		x[id_i] = (double*)malloc((MAX_ORDER_FILT + 1)* sizeof(double));
		y[id_i] = (double*)malloc((MAX_ORDER_FILT + 1)* sizeof(double));
	}

	for (id_i = 0; id_i < MAX_NUM_DIGITAL_FILTERS; id_i++)
		init_filter(id_i);
}

void
init_filter(int id_i) {
	int	s_i;
	for (s_i = 0; s_i <= MAX_ORDER_FILT; s_i++) {
		x[id_i][s_i] = 0;
		y[id_i][s_i] = 0;
	}
}

void
free_digital_filters() {
	// int	s_i;
	int	id_i;

	// Initialize data arrays for all filters:
	for (id_i = 0; id_i < MAX_NUM_DIGITAL_FILTERS; id_i++) {
		free(x[id_i]);
		free(y[id_i]);
	}
}

double
digital_filter_tf(double x_in, const int n_order,
	const double* B, const double* A, const int fil_id) {

	////////////////////////////////////////////////////////////////////////////
	// Recursive filter:
	////////////////////////////////////////////////////////////////////////////

	digital_filter_help(x_in, x[fil_id], y[fil_id], n_order, B, A);

	return y[fil_id][0];
}

double
digital_filter_tf_init(double x_in, const int n_order,
	const double* B, const double* A, const int fil_id, int* initial) {

	if (*initial) {
		init_filter(fil_id);
		*initial = 0;
	}

	return digital_filter_tf(x_in, n_order, B, A, fil_id);
}

void
digital_filter_help(double x_in, double* x, double* y, const int n_order, const double* B, const double* A) {

	int i;

	////////////////////////////////////////////////////////////////////////////
	// Recursive filter:
	////////////////////////////////////////////////////////////////////////////

	x[0] = x_in; // current input signal value
	y[0] = 0; // initialize filter output

	for (i = 0; i <= n_order; i++) {
		y[0] += B[i]*x[i];
		if (i >= 1)
			y[0] -= A[i]*y[i];
	}

	////////////////////////////////////////////////////////////////////////////
	// Update data arrays:
	////////////////////////////////////////////////////////////////////////////

	for (i = n_order; i >= 1; i--) {
		x[i] = x[i - 1];
		y[i] = y[i - 1];
	}
}

////////////////////////////////////////////////////////////////////////////
// LO-PASS filter parameters:
////////////////////////////////////////////////////////////////////////////

//feed forward control

void
param_ff_ctrl(double* B, double* A) {
   B[0] = 1.1712603903505594e+01;
   B[1] = -2.1610718269080696e+01;
   B[2] = -1.530926754557027e+00;
   B[3] = 2.1618066591626327e+01;
   B[4] = -1.0174328826402933e+01;

   A[0] = 1.000000000000000e+00;
   A[1] = -2.424652148014002e+00;
   A[2] = 2.158880722275586e+00;
   A[3] = -8.40791461848115e-01;
   A[4] = 1.21259541582703e-01;
}
