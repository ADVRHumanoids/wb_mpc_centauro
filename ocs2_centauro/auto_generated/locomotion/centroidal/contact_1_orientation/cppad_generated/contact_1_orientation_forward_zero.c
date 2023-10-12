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

void contact_1_orientation_forward_zero(double const *const * in,
                                        double*const * out,
                                        struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[50];

   v[0] = cos(x[10]);
   v[1] = cos(x[11]);
   v[2] = cos(x[12]);
   v[3] = -1 * -1 * (1 - v[2]) + v[2];
   v[4] = v[0] * v[1] * v[3];
   v[5] = cos(x[13]);
   v[6] = sin(x[10]);
   v[7] = 0 - v[6];
   v[8] = -1 * sin(x[12]);
   v[9] = 0 - v[8];
   v[10] = sin(x[11]);
   v[11] = v[0] * v[10];
   v[12] = v[7] * v[9] + v[11] * v[2];
   v[13] = 4.89663865010925e-12 * v[5];
   v[11] = v[7] * v[2] + v[11] * v[8];
   v[7] = -1 * sin(x[13]);
   v[14] = 0 - v[7];
   v[15] = v[4] * v[5] + v[12] * v[13] + v[11] * v[14];
   v[16] = cos(x[14]);
   v[17] = -1 * -1 * (1 - v[16]) + v[16];
   v[18] = 2.06823107110214e-13 * v[17];
   v[19] = -1 * -1 * (1 - v[5]) + v[5];
   v[20] = -1 * v[19];
   v[19] = 4.89663865010925e-12 * v[19];
   v[21] = v[12] * v[20] + v[4] * v[19];
   v[17] = -1 * v[17];
   v[22] = v[15] * v[18] + v[21] * v[17];
   v[23] = cos(x[15]);
   v[24] = -2.06823107110214e-13 * v[23];
   v[25] = -2.06823107110214e-13 * v[16];
   v[26] = -1 * v[16];
   v[27] = 4.89663865010925e-12 * v[7];
   v[11] = v[4] * v[7] + v[12] * v[27] + v[11] * v[5];
   v[12] = -1 * sin(x[14]);
   v[4] = 0 - v[12];
   v[28] = v[21] * v[25] + v[15] * v[26] + v[11] * v[4];
   v[29] = -1 * v[23];
   v[30] = -2.06823107110214e-13 * v[12];
   v[12] = -1 * v[12];
   v[11] = v[21] * v[30] + v[15] * v[12] + v[11] * v[16];
   v[21] = -1 * sin(x[15]);
   v[15] = 0 - v[21];
   v[31] = v[22] * v[24] + v[28] * v[29] + v[11] * v[15];
   v[32] = cos(x[16]);
   v[33] = -1 * -1 * (1 - v[32]) + v[32];
   v[34] = -1 * -1 * (1 - v[23]) + v[23];
   v[35] = 2.06823107110214e-13 * v[34];
   v[34] = -1 * v[34];
   v[36] = v[28] * v[35] + v[22] * v[34];
   v[37] = 4.89663865010925e-12 * v[33];
   v[38] = v[31] * v[33] + v[36] * v[37];
   v[39] = cos(x[9]);
   v[40] = v[39] * v[0];
   v[41] = v[39] * v[6];
   v[42] = sin(x[9]);
   v[43] = v[41] * v[10] - v[42] * v[1];
   v[44] = v[40] * v[9] + v[43] * v[2];
   v[41] = (v[41] * v[1] + v[42] * v[10]) * v[3];
   v[45] = v[44] * v[20] + v[41] * v[19];
   v[43] = v[40] * v[2] + v[43] * v[8];
   v[40] = v[41] * v[5] + v[44] * v[13] + v[43] * v[14];
   v[43] = v[41] * v[7] + v[44] * v[27] + v[43] * v[5];
   v[41] = v[45] * v[25] + v[40] * v[26] + v[43] * v[4];
   v[44] = v[40] * v[18] + v[45] * v[17];
   v[46] = v[41] * v[35] + v[44] * v[34];
   v[47] = -1 * sin(x[16]);
   v[48] = -1 * v[47];
   v[43] = v[45] * v[30] + v[40] * v[12] + v[43] * v[16];
   v[40] = v[44] * v[24] + v[41] * v[29] + v[43] * v[15];
   v[45] = 4.89663865010925e-12 * v[47];
   v[49] = -2.06823107110214e-13 * v[21];
   v[21] = -1 * v[21];
   v[43] = v[44] * v[49] + v[41] * v[21] + v[43] * v[23];
   v[44] = v[46] * v[48] + v[40] * v[45] + v[43] * v[32];
   v[0] = v[42] * v[0];
   v[42] = v[42] * v[6];
   v[6] = v[42] * v[10] + v[39] * v[1];
   v[9] = v[0] * v[9] + v[6] * v[2];
   v[42] = (v[42] * v[1] - v[39] * v[10]) * v[3];
   v[19] = v[9] * v[20] + v[42] * v[19];
   v[6] = v[0] * v[2] + v[6] * v[8];
   v[14] = v[42] * v[5] + v[9] * v[13] + v[6] * v[14];
   v[6] = v[42] * v[7] + v[9] * v[27] + v[6] * v[5];
   v[4] = v[19] * v[25] + v[14] * v[26] + v[6] * v[4];
   v[17] = v[14] * v[18] + v[19] * v[17];
   v[34] = v[4] * v[35] + v[17] * v[34];
   v[35] = -1 * v[32];
   v[6] = v[19] * v[30] + v[14] * v[12] + v[6] * v[16];
   v[15] = v[17] * v[24] + v[4] * v[29] + v[6] * v[15];
   v[29] = 4.89663865010925e-12 * v[32];
   v[6] = v[17] * v[49] + v[4] * v[21] + v[6] * v[23];
   v[47] = 0 - v[47];
   v[17] = v[34] * v[35] + v[15] * v[29] + v[6] * v[47];
   v[21] = v[22] * v[49] + v[28] * v[21] + v[11] * v[23];
   v[49] = v[36] * v[35] + v[31] * v[29] + v[21] * v[47];
   v[11] = v[15] * v[33] + v[34] * v[37];
   v[28] = v[49] - v[11];
   v[37] = v[40] * v[33] + v[46] * v[37];
   v[21] = v[36] * v[48] + v[31] * v[45] + v[21] * v[32];
   v[36] = v[37] - v[21];
   if( v[44] > v[17] ) {
      v[31] = v[28];
   } else {
      v[31] = v[36];
   }
   v[33] = 0 - v[17];
   v[6] = v[34] * v[48] + v[15] * v[45] + v[6] * v[32];
   v[47] = v[46] * v[35] + v[40] * v[29] + v[43] * v[47];
   v[29] = v[6] - v[47];
   if( v[44] > v[17] ) {
      v[35] = 1 + v[44] - v[17] - v[38];
   } else {
      v[35] = 1 + v[17] - v[44] - v[38];
   }
   if( v[44] < 0 - v[17] ) {
      v[43] = 1 + v[38] - v[44] - v[17];
   } else {
      v[43] = 1 + v[44] + v[17] + v[38];
   }
   if( v[38] < 0 ) {
      v[43] = v[35];
   } else {
      v[43] = v[43];
   }
   if( v[44] < v[33] ) {
      v[35] = v[29];
   } else {
      v[35] = v[43];
   }
   if( v[38] < 0 ) {
      v[35] = v[31];
   } else {
      v[35] = v[35];
   }
   v[31] = 0.5 / sqrt(v[43]);
   v[35] = v[35] * v[31];
   v[47] = v[6] + v[47];
   if( v[44] > v[17] ) {
      v[6] = v[47];
   } else {
      v[6] = v[43];
   }
   v[11] = v[49] + v[11];
   if( v[44] < 0 - v[17] ) {
      v[36] = v[11];
   } else {
      v[36] = v[36];
   }
   if( v[38] < 0 ) {
      v[36] = v[6];
   } else {
      v[36] = v[36];
   }
   v[36] = v[36] * v[31];
   if( v[44] > v[17] ) {
      v[47] = v[43];
   } else {
      v[47] = v[47];
   }
   v[21] = v[37] + v[21];
   if( v[44] < v[33] ) {
      v[28] = v[21];
   } else {
      v[28] = v[28];
   }
   if( v[38] < 0 ) {
      v[28] = v[47];
   } else {
      v[28] = v[28];
   }
   v[28] = v[28] * v[31];
   if( v[44] > v[17] ) {
      v[21] = v[21];
   } else {
      v[21] = v[11];
   }
   if( v[44] < v[33] ) {
      v[43] = v[43];
   } else {
      v[43] = v[29];
   }
   if( v[38] < 0 ) {
      v[43] = v[21];
   } else {
      v[43] = v[43];
   }
   v[43] = v[43] * v[31];
   y[0] = v[35] * x[49] + v[36] * x[51] - x[52] * v[28] - v[43] * x[50];
   y[1] = v[35] * x[50] + v[43] * x[49] - x[52] * v[36] - v[28] * x[51];
   y[2] = v[35] * x[51] + v[28] * x[50] - x[52] * v[43] - v[36] * x[49];
}
