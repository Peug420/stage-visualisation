//define a IIR SOS CMSIS-DSP coefficient array

#include <stdint.h>

#ifndef STAGES
#define STAGES 10
#endif
/*********************************************************/
/*                     IIR SOS Filter Coefficients       */
float32_t ba_coeff[50] = { //b0,b1,b2,a1,a2,... by stage
    +7.814165e-21, +1.562833e-20, +7.814165e-21,
    +1.978814e+00, -9.796728e-01,
    +1.000000e+00, +2.000000e+00, +1.000000e+00,
    +1.978731e+00, -9.797987e-01,
    +1.000000e+00, +2.000000e+00, +1.000000e+00,
    +1.981234e+00, -9.819134e-01,
    +1.000000e+00, +2.000000e+00, +1.000000e+00,
    +1.981529e+00, -9.828021e-01,
    +1.000000e+00, +2.000000e+00, +1.000000e+00,
    +1.984976e+00, -9.855229e-01,
    +1.000000e+00, -2.000000e+00, +1.000000e+00,
    +1.987012e+00, -9.884470e-01,
    +1.000000e+00, -2.000000e+00, +1.000000e+00,
    +1.989173e+00, -9.896336e-01,
    +1.000000e+00, -2.000000e+00, +1.000000e+00,
    +1.993402e+00, -9.938120e-01,
    +1.000000e+00, -2.000000e+00, +1.000000e+00,
    +1.994396e+00, -9.959234e-01,
    +1.000000e+00, -2.000000e+00, +1.000000e+00,
    +1.997556e+00, -9.979443e-01
};
/*********************************************************/
