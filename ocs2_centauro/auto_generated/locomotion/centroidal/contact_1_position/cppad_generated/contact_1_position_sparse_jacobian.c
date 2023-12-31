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

void contact_1_position_sparse_jacobian(double const *const * in,
                                        double*const * out,
                                        struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[63];

   v[0] = cos(x[16]);
   v[0] = -1 * -1 * (1 - v[0]) + v[0];
   v[1] = 4.89663865010925e-12 * v[0];
   v[2] = -0.00085 + -0.26835 * v[1];
   v[3] = cos(x[15]);
   v[4] = -1 * -1 * (1 - v[3]) + v[3];
   v[5] = -1 * v[4];
   v[6] = -0.139 + -0.26835 * v[0];
   v[7] = -2.06823107110214e-13 * v[3];
   v[8] = 0.10015 + v[2] * v[5] + v[6] * v[7];
   v[9] = cos(x[14]);
   v[10] = -1 * -1 * (1 - v[9]) + v[9];
   v[11] = -1 * v[10];
   v[4] = 2.06823107110214e-13 * v[4];
   v[3] = -1 * v[3];
   v[12] = 0.2 + v[2] * v[4] + v[6] * v[3];
   v[13] = -2.06823107110214e-13 * v[9];
   v[14] = 0 - -1 * sin(x[15]);
   v[15] = v[6] * v[14];
   v[16] = -1 * sin(x[14]);
   v[17] = -2.06823107110214e-13 * v[16];
   v[18] = 0.1182 + v[8] * v[11] + v[12] * v[13] + v[15] * v[17];
   v[19] = cos(x[13]);
   v[20] = -1 * -1 * (1 - v[19]) + v[19];
   v[21] = 4.89663865010925e-12 * v[20];
   v[10] = 2.06823107110214e-13 * v[10];
   v[22] = -1 * v[9];
   v[23] = -1 * v[16];
   v[24] = -0.3 + v[8] * v[10] + v[12] * v[22] + v[15] * v[23];
   v[16] = 0 - v[16];
   v[25] = v[12] * v[16] + v[15] * v[9];
   v[26] = -1 * sin(x[13]);
   v[27] = -0.0625 + v[18] * v[21] + v[24] * v[19] + v[25] * v[26];
   v[28] = cos(x[12]);
   v[29] = -1 * -1 * (1 - v[28]) + v[28];
   v[30] = -0.0566 + v[27] * v[29];
   v[31] = cos(x[11]);
   v[20] = -1 * v[20];
   v[32] = 4.89663865010925e-12 * v[19];
   v[33] = 4.89663865010925e-12 * v[26];
   v[34] = 0.1146 + v[18] * v[20] + v[24] * v[32] + v[25] * v[33];
   v[35] = 0 - v[26];
   v[36] = v[24] * v[35] + v[25] * v[19];
   v[37] = -1 * sin(x[12]);
   v[38] = 0.125 + v[34] * v[28] + v[36] * v[37];
   v[39] = sin(x[11]);
   v[40] = v[30] * v[31] + v[38] * v[39];
   v[41] = sin(x[10]);
   v[42] = 0 - v[37];
   v[43] = 0.25 + v[34] * v[42] + v[36] * v[28];
   v[44] = cos(x[10]);
   v[45] = sin(x[9]);
   v[46] = 0 - v[38];
   v[47] = cos(x[9]);
   jac[1] = 0 - (v[40] * v[41] + v[43] * v[44]) * v[45] + (v[30] * v[39] + v[46] * v[31]) * v[47];
   v[48] = cos(x[9]);
   v[49] = sin(x[10]);
   v[50] = cos(x[10]);
   jac[2] = 0 - v[43] * v[48] * v[49] + v[40] * v[48] * v[50];
   v[43] = v[48] * v[41];
   v[40] = sin(x[9]);
   v[51] = sin(x[11]);
   v[52] = cos(x[11]);
   jac[3] = 0 - (v[30] * v[43] + v[46] * v[40]) * v[51] + (v[30] * v[40] + v[38] * v[43]) * v[52];
   v[46] = v[43] * v[39] - v[40] * v[31];
   v[38] = v[48] * v[44];
   v[43] = v[43] * v[31] + v[40] * v[39];
   v[27] = v[27] * v[43];
   v[30] = sin(x[12]);
   v[53] = cos(x[12]);
   jac[4] = 0 - (v[34] * v[46] + v[36] * v[38] + v[27] - v[27] * -1 * -1) * v[30] + (v[36] * v[46] - v[34] * v[38]) * -1 * v[53];
   v[43] = v[43] * v[29];
   v[27] = v[38] * v[28] + v[46] * v[37];
   v[38] = v[38] * v[42] + v[46] * v[28];
   v[18] = v[18] * v[43] * 4.89663865010925e-12 + v[18] * v[38] * -1;
   v[46] = sin(x[13]);
   v[36] = cos(x[13]);
   jac[5] = 0 - (v[24] * v[43] + v[25] * v[27] + v[24] * v[38] * 4.89663865010925e-12 + v[18] - v[18] * -1 * -1) * v[46] + (v[25] * v[43] + v[25] * v[38] * 4.89663865010925e-12 - v[24] * v[27]) * -1 * v[36];
   v[18] = v[43] * v[26] + v[38] * v[33] + v[27] * v[19];
   v[25] = v[38] * v[20] + v[43] * v[21];
   v[38] = v[43] * v[19] + v[38] * v[32] + v[27] * v[35];
   v[8] = v[8] * v[25] * -1 + v[8] * v[38] * 2.06823107110214e-13;
   v[27] = sin(x[14]);
   v[43] = cos(x[14]);
   jac[6] = 0 - (v[15] * v[18] + v[12] * v[25] * -2.06823107110214e-13 + v[12] * v[38] * -1 + v[8] - v[8] * -1 * -1) * v[27] + (v[15] * v[25] * -2.06823107110214e-13 + v[15] * v[38] * -1 - v[12] * v[18]) * -1 * v[43];
   v[8] = v[38] * v[10] + v[25] * v[11];
   v[15] = v[25] * v[13] + v[38] * v[22] + v[18] * v[16];
   v[2] = v[2] * v[8] * -1 + v[2] * v[15] * 2.06823107110214e-13;
   v[12] = sin(x[15]);
   v[38] = v[25] * v[17] + v[38] * v[23] + v[18] * v[9];
   v[25] = cos(x[15]);
   jac[7] = 0 - (v[6] * v[8] * -2.06823107110214e-13 + v[6] * v[15] * -1 + v[2] - v[2] * -1 * -1) * v[12] + (0 - v[6] * v[38]) * -1 * v[25];
   v[38] = -0.26835 * (v[8] * v[7] + v[15] * v[3] + v[38] * v[14]) + -0.26835 * (v[15] * v[4] + v[8] * v[5]) * 4.89663865010925e-12;
   v[15] = sin(x[16]);
   jac[8] = 0 - (v[38] - v[38] * -1 * -1) * v[15];
   v[38] = -0.00085 + -0.26835 * v[1];
   v[8] = -0.139 + -0.26835 * v[0];
   v[2] = 0.10015 + v[38] * v[5] + v[8] * v[7];
   v[6] = 0.2 + v[38] * v[4] + v[8] * v[3];
   v[18] = v[8] * v[14];
   v[24] = 0.1182 + v[2] * v[11] + v[6] * v[13] + v[18] * v[17];
   v[34] = -0.3 + v[2] * v[10] + v[6] * v[22] + v[18] * v[23];
   v[54] = v[6] * v[16] + v[18] * v[9];
   v[55] = -0.0625 + v[24] * v[21] + v[34] * v[19] + v[54] * v[26];
   v[56] = -0.0566 + v[55] * v[29];
   v[57] = 0 - v[56];
   v[58] = 0.1146 + v[24] * v[20] + v[34] * v[32] + v[54] * v[33];
   v[59] = v[34] * v[35] + v[54] * v[19];
   v[60] = 0.125 + v[58] * v[28] + v[59] * v[37];
   v[61] = v[56] * v[31] + v[60] * v[39];
   v[62] = 0.25 + v[58] * v[42] + v[59] * v[28];
   jac[10] = 0 - (v[57] * v[39] + v[60] * v[31]) * v[45] + (v[61] * v[41] + v[62] * v[44]) * v[47];
   jac[11] = 0 - v[62] * v[40] * v[49] + v[61] * v[40] * v[50];
   v[62] = v[40] * v[41];
   jac[12] = 0 - (v[56] * v[62] + v[60] * v[48]) * v[51] + (v[57] * v[48] + v[60] * v[62]) * v[52];
   v[60] = v[62] * v[39] + v[48] * v[31];
   v[40] = v[40] * v[44];
   v[62] = v[62] * v[31] - v[48] * v[39];
   v[55] = v[55] * v[62];
   jac[13] = 0 - (v[58] * v[60] + v[59] * v[40] + v[55] - v[55] * -1 * -1) * v[30] + (v[59] * v[60] - v[58] * v[40]) * -1 * v[53];
   v[62] = v[62] * v[29];
   v[55] = v[40] * v[28] + v[60] * v[37];
   v[40] = v[40] * v[42] + v[60] * v[28];
   v[24] = v[24] * v[62] * 4.89663865010925e-12 + v[24] * v[40] * -1;
   jac[14] = 0 - (v[34] * v[62] + v[54] * v[55] + v[34] * v[40] * 4.89663865010925e-12 + v[24] - v[24] * -1 * -1) * v[46] + (v[54] * v[62] + v[54] * v[40] * 4.89663865010925e-12 - v[34] * v[55]) * -1 * v[36];
   v[24] = v[62] * v[26] + v[40] * v[33] + v[55] * v[19];
   v[54] = v[40] * v[20] + v[62] * v[21];
   v[40] = v[62] * v[19] + v[40] * v[32] + v[55] * v[35];
   v[2] = v[2] * v[54] * -1 + v[2] * v[40] * 2.06823107110214e-13;
   jac[15] = 0 - (v[18] * v[24] + v[6] * v[54] * -2.06823107110214e-13 + v[6] * v[40] * -1 + v[2] - v[2] * -1 * -1) * v[27] + (v[18] * v[54] * -2.06823107110214e-13 + v[18] * v[40] * -1 - v[6] * v[24]) * -1 * v[43];
   v[2] = v[40] * v[10] + v[54] * v[11];
   v[18] = v[54] * v[13] + v[40] * v[22] + v[24] * v[16];
   v[38] = v[38] * v[2] * -1 + v[38] * v[18] * 2.06823107110214e-13;
   v[40] = v[54] * v[17] + v[40] * v[23] + v[24] * v[9];
   jac[16] = 0 - (v[8] * v[2] * -2.06823107110214e-13 + v[8] * v[18] * -1 + v[38] - v[38] * -1 * -1) * v[12] + (0 - v[8] * v[40]) * -1 * v[25];
   v[40] = -0.26835 * (v[2] * v[7] + v[18] * v[3] + v[40] * v[14]) + -0.26835 * (v[18] * v[4] + v[2] * v[5]) * 4.89663865010925e-12;
   jac[17] = 0 - (v[40] - v[40] * -1 * -1) * v[15];
   v[1] = -0.00085 + -0.26835 * v[1];
   v[0] = -0.139 + -0.26835 * v[0];
   v[40] = 0.10015 + v[1] * v[5] + v[0] * v[7];
   v[18] = 0.2 + v[1] * v[4] + v[0] * v[3];
   v[2] = v[0] * v[14];
   v[38] = 0.1182 + v[40] * v[11] + v[18] * v[13] + v[2] * v[17];
   v[8] = -0.3 + v[40] * v[10] + v[18] * v[22] + v[2] * v[23];
   v[54] = v[18] * v[16] + v[2] * v[9];
   v[24] = -0.0625 + v[38] * v[21] + v[8] * v[19] + v[54] * v[26];
   v[6] = -0.0566 + v[24] * v[29];
   v[55] = 0.1146 + v[38] * v[20] + v[8] * v[32] + v[54] * v[33];
   v[62] = v[8] * v[35] + v[54] * v[19];
   v[34] = 0.125 + v[55] * v[28] + v[62] * v[37];
   jac[19] = 0 - (v[6] * v[31] + v[34] * v[39]) * v[49] + (0 - (0.25 + v[55] * v[42] + v[62] * v[28])) * v[50];
   jac[20] = 0 - v[6] * v[44] * v[51] + v[34] * v[44] * v[52];
   v[39] = v[44] * v[39];
   v[41] = 0 - v[41];
   v[44] = v[44] * v[31];
   v[24] = v[24] * v[44];
   jac[21] = 0 - (v[55] * v[39] + v[62] * v[41] + v[24] - v[24] * -1 * -1) * v[30] + (v[62] * v[39] - v[55] * v[41]) * -1 * v[53];
   v[44] = v[44] * v[29];
   v[37] = v[41] * v[28] + v[39] * v[37];
   v[41] = v[41] * v[42] + v[39] * v[28];
   v[38] = v[38] * v[44] * 4.89663865010925e-12 + v[38] * v[41] * -1;
   jac[22] = 0 - (v[8] * v[44] + v[54] * v[37] + v[8] * v[41] * 4.89663865010925e-12 + v[38] - v[38] * -1 * -1) * v[46] + (v[54] * v[44] + v[54] * v[41] * 4.89663865010925e-12 - v[8] * v[37]) * -1 * v[36];
   v[33] = v[44] * v[26] + v[41] * v[33] + v[37] * v[19];
   v[20] = v[41] * v[20] + v[44] * v[21];
   v[41] = v[44] * v[19] + v[41] * v[32] + v[37] * v[35];
   v[40] = v[40] * v[20] * -1 + v[40] * v[41] * 2.06823107110214e-13;
   jac[23] = 0 - (v[2] * v[33] + v[18] * v[20] * -2.06823107110214e-13 + v[18] * v[41] * -1 + v[40] - v[40] * -1 * -1) * v[27] + (v[2] * v[20] * -2.06823107110214e-13 + v[2] * v[41] * -1 - v[18] * v[33]) * -1 * v[43];
   v[10] = v[41] * v[10] + v[20] * v[11];
   v[16] = v[20] * v[13] + v[41] * v[22] + v[33] * v[16];
   v[1] = v[1] * v[10] * -1 + v[1] * v[16] * 2.06823107110214e-13;
   v[41] = v[20] * v[17] + v[41] * v[23] + v[33] * v[9];
   jac[24] = 0 - (v[0] * v[10] * -2.06823107110214e-13 + v[0] * v[16] * -1 + v[1] - v[1] * -1 * -1) * v[12] + (0 - v[0] * v[41]) * -1 * v[25];
   v[41] = -0.26835 * (v[10] * v[7] + v[16] * v[3] + v[41] * v[14]) + -0.26835 * (v[16] * v[4] + v[10] * v[5]) * 4.89663865010925e-12;
   jac[25] = 0 - (v[41] - v[41] * -1 * -1) * v[15];
   // dependent variables without operations
   jac[0] = 1;
   jac[9] = 1;
   jac[18] = 1;
}

