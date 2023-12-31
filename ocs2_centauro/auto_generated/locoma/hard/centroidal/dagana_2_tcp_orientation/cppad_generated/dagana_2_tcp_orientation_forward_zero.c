#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void dagana_2_tcp_orientation_forward_zero(double const *const * in,
                                           double*const * out,
                                           struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[41];

   v[0] = cos(x[10]);
   v[1] = cos(x[11]);
   v[2] = v[0] * v[1];
   v[3] = sin(x[43]);
   v[4] = 0 - v[3];
   v[5] = 0.984807740023223 * v[4];
   v[6] = sin(x[10]);
   v[7] = 0 - v[6];
   v[8] = sin(x[36]);
   v[9] = 0 - v[8];
   v[10] = sin(x[11]);
   v[11] = v[0] * v[10];
   v[12] = cos(x[36]);
   v[13] = v[7] * v[9] + v[11] * v[12];
   v[14] = cos(x[43]);
   v[15] = 0.500000021132493 * v[14] + 0.150383794856828 * v[4];
   v[11] = v[7] * v[12] + v[11] * v[8];
   v[4] = 0.866025391583588 * v[14] + -0.0868241293351745 * v[4];
   v[7] = v[2] * v[5] + v[13] * v[15] + v[11] * v[4];
   v[16] = cos(x[45]);
   v[17] = -0.173648251331108 * v[2] + 0.85286850868816 * v[13] + -0.492403890823054 * v[11];
   v[18] = cos(x[44]);
   v[19] = sin(x[44]);
   v[20] = 0.984807792117654 * v[18] + -0.173647955888778 * v[19];
   v[21] = 0.984807740023223 * v[14];
   v[22] = 0.500000021132493 * v[3] + 0.150383794856828 * v[14];
   v[14] = 0.866025391583588 * v[3] + -0.0868241293351745 * v[14];
   v[11] = v[2] * v[21] + v[13] * v[22] + v[11] * v[14];
   v[13] = 0.173647955888778 * v[18] + 0.984807792117654 * v[19];
   v[2] = v[17] * v[20] + v[11] * v[13];
   v[3] = sin(x[45]);
   v[23] = v[7] * v[16] + v[2] * v[3];
   v[24] = cos(x[46]);
   v[19] = 0 - v[19];
   v[25] = 0.984807792117654 * v[19] + -0.173647955888778 * v[18];
   v[19] = 0.173647955888778 * v[19] + 0.984807792117654 * v[18];
   v[11] = v[17] * v[25] + v[11] * v[19];
   v[17] = sin(x[46]);
   v[18] = 0 - v[17];
   v[26] = v[23] * v[24] + v[11] * v[18];
   v[27] = cos(x[47]);
   v[28] = 0 - v[3];
   v[2] = v[7] * v[28] + v[2] * v[16];
   v[7] = sin(x[47]);
   v[29] = v[26] * v[27] + v[2] * v[7];
   v[30] = cos(x[48]);
   v[11] = v[23] * v[17] + v[11] * v[24];
   v[23] = sin(x[48]);
   v[31] = 0 - v[23];
   v[32] = v[29] * v[30] + v[11] * v[31];
   v[11] = v[29] * v[23] + v[11] * v[30];
   v[29] = -2.06823107110214e-13 * v[32] + -1 * v[11];
   v[33] = cos(x[9]);
   v[34] = v[33] * v[6];
   v[35] = sin(x[9]);
   v[36] = v[34] * v[1] + v[35] * v[10];
   v[37] = v[33] * v[0];
   v[34] = v[34] * v[10] - v[35] * v[1];
   v[38] = v[37] * v[9] + v[34] * v[12];
   v[34] = v[37] * v[12] + v[34] * v[8];
   v[37] = v[36] * v[5] + v[38] * v[15] + v[34] * v[4];
   v[39] = -0.173648251331108 * v[36] + 0.85286850868816 * v[38] + -0.492403890823054 * v[34];
   v[34] = v[36] * v[21] + v[38] * v[22] + v[34] * v[14];
   v[38] = v[39] * v[20] + v[34] * v[13];
   v[36] = v[37] * v[16] + v[38] * v[3];
   v[34] = v[39] * v[25] + v[34] * v[19];
   v[39] = v[36] * v[24] + v[34] * v[18];
   v[38] = v[37] * v[28] + v[38] * v[16];
   v[37] = v[39] * v[27] + v[38] * v[7];
   v[34] = v[36] * v[17] + v[34] * v[24];
   v[36] = v[37] * v[30] + v[34] * v[31];
   v[34] = v[37] * v[23] + v[34] * v[30];
   v[37] = -1 * v[36] + 2.06823107110214e-13 * v[34];
   v[6] = v[35] * v[6];
   v[40] = v[6] * v[1] - v[33] * v[10];
   v[35] = v[35] * v[0];
   v[6] = v[6] * v[10] + v[33] * v[1];
   v[9] = v[35] * v[9] + v[6] * v[12];
   v[6] = v[35] * v[12] + v[6] * v[8];
   v[4] = v[40] * v[5] + v[9] * v[15] + v[6] * v[4];
   v[15] = -0.173648251331108 * v[40] + 0.85286850868816 * v[9] + -0.492403890823054 * v[6];
   v[6] = v[40] * v[21] + v[9] * v[22] + v[6] * v[14];
   v[13] = v[15] * v[20] + v[6] * v[13];
   v[3] = v[4] * v[16] + v[13] * v[3];
   v[6] = v[15] * v[25] + v[6] * v[19];
   v[18] = v[3] * v[24] + v[6] * v[18];
   v[15] = 0 - v[7];
   v[13] = v[4] * v[28] + v[13] * v[16];
   v[4] = v[18] * v[15] + v[13] * v[27];
   v[2] = v[26] * v[15] + v[2] * v[27];
   v[13] = v[18] * v[27] + v[13] * v[7];
   v[6] = v[3] * v[17] + v[6] * v[24];
   v[31] = v[13] * v[30] + v[6] * v[31];
   v[6] = v[13] * v[23] + v[6] * v[30];
   v[13] = -2.06823107110214e-13 * v[31] + -1 * v[6];
   v[23] = v[2] - v[13];
   v[34] = -2.06823107110214e-13 * v[36] + -1 * v[34];
   v[11] = -1 * v[32] + 2.06823107110214e-13 * v[11];
   v[32] = v[34] - v[11];
   if( v[37] > v[4] ) {
      v[36] = v[23];
   } else {
      v[36] = v[32];
   }
   v[30] = 0 - v[4];
   v[6] = -1 * v[31] + 2.06823107110214e-13 * v[6];
   v[15] = v[39] * v[15] + v[38] * v[27];
   v[38] = v[6] - v[15];
   if( v[37] > v[4] ) {
      v[39] = 1 + v[37] - v[4] - v[29];
   } else {
      v[39] = 1 + v[4] - v[37] - v[29];
   }
   if( v[37] < 0 - v[4] ) {
      v[27] = 1 + v[29] - v[37] - v[4];
   } else {
      v[27] = 1 + v[37] + v[4] + v[29];
   }
   if( v[29] < 0 ) {
      v[27] = v[39];
   } else {
      v[27] = v[27];
   }
   if( v[37] < v[30] ) {
      v[39] = v[38];
   } else {
      v[39] = v[27];
   }
   if( v[29] < 0 ) {
      v[39] = v[36];
   } else {
      v[39] = v[39];
   }
   v[36] = 0.5 / sqrt(v[27]);
   v[39] = v[39] * v[36];
   v[15] = v[6] + v[15];
   if( v[37] > v[4] ) {
      v[6] = v[15];
   } else {
      v[6] = v[27];
   }
   v[13] = v[2] + v[13];
   if( v[37] < 0 - v[4] ) {
      v[32] = v[13];
   } else {
      v[32] = v[32];
   }
   if( v[29] < 0 ) {
      v[32] = v[6];
   } else {
      v[32] = v[32];
   }
   v[32] = v[32] * v[36];
   if( v[37] > v[4] ) {
      v[15] = v[27];
   } else {
      v[15] = v[15];
   }
   v[11] = v[34] + v[11];
   if( v[37] < v[30] ) {
      v[23] = v[11];
   } else {
      v[23] = v[23];
   }
   if( v[29] < 0 ) {
      v[23] = v[15];
   } else {
      v[23] = v[23];
   }
   v[23] = v[23] * v[36];
   if( v[37] > v[4] ) {
      v[11] = v[11];
   } else {
      v[11] = v[13];
   }
   if( v[37] < v[30] ) {
      v[27] = v[27];
   } else {
      v[27] = v[38];
   }
   if( v[29] < 0 ) {
      v[27] = v[11];
   } else {
      v[27] = v[27];
   }
   v[27] = v[27] * v[36];
   y[0] = v[39] * x[49] + v[32] * x[51] - x[52] * v[23] - v[27] * x[50];
   y[1] = v[39] * x[50] + v[27] * x[49] - x[52] * v[32] - v[23] * x[51];
   y[2] = v[39] * x[51] + v[23] * x[50] - x[52] * v[27] - v[32] * x[49];
}

