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

void contact_4_orientation_forward_zero(double const *const * in,
                                        double*const * out,
                                        struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[44];

   v[0] = cos(x[10]);
   v[1] = cos(x[11]);
   v[2] = cos(x[30]);
   v[3] = -1 * -1 * (1 - v[2]) + v[2];
   v[4] = v[0] * v[1] * v[3];
   v[5] = cos(x[31]);
   v[6] = sin(x[10]);
   v[7] = 0 - v[6];
   v[8] = -1 * sin(x[30]);
   v[9] = 0 - v[8];
   v[10] = sin(x[11]);
   v[11] = v[0] * v[10];
   v[12] = v[7] * v[9] + v[11] * v[2];
   v[13] = 4.89663865010925e-12 * v[5];
   v[11] = v[7] * v[2] + v[11] * v[8];
   v[7] = sin(x[31]);
   v[14] = 0 - v[7];
   v[15] = v[4] * v[5] + v[12] * v[13] + v[11] * v[14];
   v[16] = -1 * v[12] + 4.89663865010925e-12 * v[4];
   v[17] = 2.06823107110214e-13 * v[15] + -1 * v[16];
   v[18] = cos(x[33]);
   v[19] = -2.06823107110214e-13 * v[18];
   v[20] = cos(x[32]);
   v[21] = -2.06823107110214e-13 * v[20];
   v[22] = -1 * v[20];
   v[23] = 4.89663865010925e-12 * v[7];
   v[11] = v[4] * v[7] + v[12] * v[23] + v[11] * v[5];
   v[12] = sin(x[32]);
   v[4] = 0 - v[12];
   v[24] = v[16] * v[21] + v[15] * v[22] + v[11] * v[4];
   v[25] = -1 * v[18];
   v[26] = -2.06823107110214e-13 * v[12];
   v[12] = -1 * v[12];
   v[11] = v[16] * v[26] + v[15] * v[12] + v[11] * v[20];
   v[16] = sin(x[33]);
   v[15] = 0 - v[16];
   v[27] = v[17] * v[19] + v[24] * v[25] + v[11] * v[15];
   v[28] = cos(x[34]);
   v[29] = -1 * -1 * (1 - v[28]) + v[28];
   v[30] = 2.06823107110214e-13 * v[24] + -1 * v[17];
   v[31] = 4.89663865010925e-12 * v[29];
   v[32] = v[27] * v[29] + v[30] * v[31];
   v[33] = cos(x[9]);
   v[34] = v[33] * v[0];
   v[35] = v[33] * v[6];
   v[36] = sin(x[9]);
   v[37] = v[35] * v[10] - v[36] * v[1];
   v[38] = v[34] * v[9] + v[37] * v[2];
   v[35] = (v[35] * v[1] + v[36] * v[10]) * v[3];
   v[39] = -1 * v[38] + 4.89663865010925e-12 * v[35];
   v[37] = v[34] * v[2] + v[37] * v[8];
   v[34] = v[35] * v[5] + v[38] * v[13] + v[37] * v[14];
   v[37] = v[35] * v[7] + v[38] * v[23] + v[37] * v[5];
   v[35] = v[39] * v[21] + v[34] * v[22] + v[37] * v[4];
   v[38] = 2.06823107110214e-13 * v[34] + -1 * v[39];
   v[40] = 2.06823107110214e-13 * v[35] + -1 * v[38];
   v[41] = -1 * sin(x[34]);
   v[42] = -1 * v[41];
   v[37] = v[39] * v[26] + v[34] * v[12] + v[37] * v[20];
   v[34] = v[38] * v[19] + v[35] * v[25] + v[37] * v[15];
   v[39] = 4.89663865010925e-12 * v[41];
   v[43] = -2.06823107110214e-13 * v[16];
   v[16] = -1 * v[16];
   v[37] = v[38] * v[43] + v[35] * v[16] + v[37] * v[18];
   v[38] = v[40] * v[42] + v[34] * v[39] + v[37] * v[28];
   v[0] = v[36] * v[0];
   v[36] = v[36] * v[6];
   v[6] = v[36] * v[10] + v[33] * v[1];
   v[9] = v[0] * v[9] + v[6] * v[2];
   v[36] = (v[36] * v[1] - v[33] * v[10]) * v[3];
   v[33] = -1 * v[9] + 4.89663865010925e-12 * v[36];
   v[6] = v[0] * v[2] + v[6] * v[8];
   v[14] = v[36] * v[5] + v[9] * v[13] + v[6] * v[14];
   v[6] = v[36] * v[7] + v[9] * v[23] + v[6] * v[5];
   v[4] = v[33] * v[21] + v[14] * v[22] + v[6] * v[4];
   v[22] = 2.06823107110214e-13 * v[14] + -1 * v[33];
   v[21] = 2.06823107110214e-13 * v[4] + -1 * v[22];
   v[36] = -1 * v[28];
   v[6] = v[33] * v[26] + v[14] * v[12] + v[6] * v[20];
   v[15] = v[22] * v[19] + v[4] * v[25] + v[6] * v[15];
   v[25] = 4.89663865010925e-12 * v[28];
   v[6] = v[22] * v[43] + v[4] * v[16] + v[6] * v[18];
   v[41] = 0 - v[41];
   v[22] = v[21] * v[36] + v[15] * v[25] + v[6] * v[41];
   v[16] = v[17] * v[43] + v[24] * v[16] + v[11] * v[18];
   v[43] = v[30] * v[36] + v[27] * v[25] + v[16] * v[41];
   v[11] = v[15] * v[29] + v[21] * v[31];
   v[24] = v[43] - v[11];
   v[31] = v[34] * v[29] + v[40] * v[31];
   v[16] = v[30] * v[42] + v[27] * v[39] + v[16] * v[28];
   v[30] = v[31] - v[16];
   if( v[38] > v[22] ) {
      v[27] = v[24];
   } else {
      v[27] = v[30];
   }
   v[29] = 0 - v[22];
   v[6] = v[21] * v[42] + v[15] * v[39] + v[6] * v[28];
   v[41] = v[40] * v[36] + v[34] * v[25] + v[37] * v[41];
   v[25] = v[6] - v[41];
   if( v[38] > v[22] ) {
      v[36] = 1 + v[38] - v[22] - v[32];
   } else {
      v[36] = 1 + v[22] - v[38] - v[32];
   }
   if( v[38] < 0 - v[22] ) {
      v[37] = 1 + v[32] - v[38] - v[22];
   } else {
      v[37] = 1 + v[38] + v[22] + v[32];
   }
   if( v[32] < 0 ) {
      v[37] = v[36];
   } else {
      v[37] = v[37];
   }
   if( v[38] < v[29] ) {
      v[36] = v[25];
   } else {
      v[36] = v[37];
   }
   if( v[32] < 0 ) {
      v[36] = v[27];
   } else {
      v[36] = v[36];
   }
   v[27] = 0.5 / sqrt(v[37]);
   v[36] = v[36] * v[27];
   v[41] = v[6] + v[41];
   if( v[38] > v[22] ) {
      v[6] = v[41];
   } else {
      v[6] = v[37];
   }
   v[11] = v[43] + v[11];
   if( v[38] < 0 - v[22] ) {
      v[30] = v[11];
   } else {
      v[30] = v[30];
   }
   if( v[32] < 0 ) {
      v[30] = v[6];
   } else {
      v[30] = v[30];
   }
   v[30] = v[30] * v[27];
   if( v[38] > v[22] ) {
      v[41] = v[37];
   } else {
      v[41] = v[41];
   }
   v[16] = v[31] + v[16];
   if( v[38] < v[29] ) {
      v[24] = v[16];
   } else {
      v[24] = v[24];
   }
   if( v[32] < 0 ) {
      v[24] = v[41];
   } else {
      v[24] = v[24];
   }
   v[24] = v[24] * v[27];
   if( v[38] > v[22] ) {
      v[16] = v[16];
   } else {
      v[16] = v[11];
   }
   if( v[38] < v[29] ) {
      v[37] = v[37];
   } else {
      v[37] = v[25];
   }
   if( v[32] < 0 ) {
      v[37] = v[16];
   } else {
      v[37] = v[37];
   }
   v[37] = v[37] * v[27];
   y[0] = v[36] * x[49] + v[30] * x[51] - x[52] * v[24] - v[37] * x[50];
   y[1] = v[36] * x[50] + v[37] * x[49] - x[52] * v[30] - v[24] * x[51];
   y[2] = v[36] * x[51] + v[24] * x[50] - x[52] * v[37] - v[30] * x[49];
}

