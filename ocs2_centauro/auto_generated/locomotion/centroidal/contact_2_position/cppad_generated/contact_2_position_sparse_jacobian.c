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

void contact_2_position_sparse_jacobian(double const *const * in,
                                        double*const * out,
                                        struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[54];

   v[0] = cos(x[22]);
   v[0] = -1 * -1 * (1 - v[0]) + v[0];
   v[1] = 4.89663865010925e-12 * v[0];
   v[2] = 0.00085 + -0.26835 * v[1];
   v[3] = -0.139 + -0.26835 * v[0];
   v[4] = cos(x[21]);
   v[5] = -2.06823107110214e-13 * v[4];
   v[6] = -0.10015 + v[2] * -1 + v[3] * v[5];
   v[4] = -1 * v[4];
   v[2] = 0.2 + v[2] * 2.06823107110214e-13 + v[3] * v[4];
   v[7] = cos(x[20]);
   v[8] = -2.06823107110214e-13 * v[7];
   v[9] = 0 - sin(x[21]);
   v[10] = v[3] * v[9];
   v[11] = sin(x[20]);
   v[12] = -2.06823107110214e-13 * v[11];
   v[13] = -0.1182 + v[6] * -1 + v[2] * v[8] + v[10] * v[12];
   v[14] = -1 * v[7];
   v[15] = -1 * v[11];
   v[6] = -0.3 + v[6] * 2.06823107110214e-13 + v[2] * v[14] + v[10] * v[15];
   v[16] = cos(x[19]);
   v[11] = 0 - v[11];
   v[17] = v[2] * v[11] + v[10] * v[7];
   v[18] = sin(x[19]);
   v[19] = -0.0625 + v[13] * 4.89663865010925e-12 + v[6] * v[16] + v[17] * v[18];
   v[20] = cos(x[18]);
   v[21] = -1 * -1 * (1 - v[20]) + v[20];
   v[22] = -0.0566 + v[19] * v[21];
   v[23] = cos(x[11]);
   v[24] = 4.89663865010925e-12 * v[16];
   v[25] = 4.89663865010925e-12 * v[18];
   v[13] = -0.1146 + v[13] * -1 + v[6] * v[24] + v[17] * v[25];
   v[26] = 0 - v[18];
   v[27] = v[6] * v[26] + v[17] * v[16];
   v[28] = -1 * sin(x[18]);
   v[29] = -0.125 + v[13] * v[20] + v[27] * v[28];
   v[30] = sin(x[11]);
   v[31] = v[22] * v[23] + v[29] * v[30];
   v[32] = sin(x[10]);
   v[33] = 0 - v[28];
   v[34] = 0.25 + v[13] * v[33] + v[27] * v[20];
   v[35] = cos(x[10]);
   v[36] = sin(x[9]);
   v[37] = 0 - v[29];
   v[38] = cos(x[9]);
   jac[1] = 0 - (v[31] * v[32] + v[34] * v[35]) * v[36] + (v[22] * v[30] + v[37] * v[23]) * v[38];
   v[39] = cos(x[9]);
   v[40] = sin(x[10]);
   v[41] = cos(x[10]);
   jac[2] = 0 - v[34] * v[39] * v[40] + v[31] * v[39] * v[41];
   v[34] = v[39] * v[32];
   v[31] = sin(x[9]);
   v[42] = sin(x[11]);
   v[43] = cos(x[11]);
   jac[3] = 0 - (v[22] * v[34] + v[37] * v[31]) * v[42] + (v[22] * v[31] + v[29] * v[34]) * v[43];
   v[37] = v[34] * v[30] - v[31] * v[23];
   v[29] = v[39] * v[35];
   v[34] = v[34] * v[23] + v[31] * v[30];
   v[19] = v[19] * v[34];
   v[22] = sin(x[18]);
   v[44] = cos(x[18]);
   jac[4] = 0 - (v[13] * v[37] + v[27] * v[29] + v[19] - v[19] * -1 * -1) * v[22] + (v[27] * v[37] - v[13] * v[29]) * -1 * v[44];
   v[34] = v[34] * v[21];
   v[19] = v[29] * v[20] + v[37] * v[28];
   v[29] = v[29] * v[33] + v[37] * v[20];
   v[37] = sin(x[19]);
   v[27] = cos(x[19]);
   jac[5] = 0 - (v[6] * v[34] + v[17] * v[19] + v[6] * v[29] * 4.89663865010925e-12) * v[37] + (v[17] * v[34] + v[17] * v[29] * 4.89663865010925e-12 - v[6] * v[19]) * v[27];
   v[17] = v[34] * v[18] + v[29] * v[25] + v[19] * v[16];
   v[6] = -1 * v[29] + 4.89663865010925e-12 * v[34];
   v[29] = v[34] * v[16] + v[29] * v[24] + v[19] * v[26];
   v[19] = sin(x[20]);
   v[34] = cos(x[20]);
   jac[6] = 0 - (v[10] * v[17] + v[2] * v[6] * -2.06823107110214e-13 + v[2] * v[29] * -1) * v[19] + (v[10] * v[6] * -2.06823107110214e-13 + v[10] * v[29] * -1 - v[2] * v[17]) * v[34];
   v[10] = 2.06823107110214e-13 * v[29] + -1 * v[6];
   v[2] = v[6] * v[8] + v[29] * v[14] + v[17] * v[11];
   v[13] = sin(x[21]);
   v[29] = v[6] * v[12] + v[29] * v[15] + v[17] * v[7];
   v[6] = cos(x[21]);
   jac[7] = 0 - (v[3] * v[10] * -2.06823107110214e-13 + v[3] * v[2] * -1) * v[13] + (0 - v[3] * v[29]) * v[6];
   v[29] = -0.26835 * (v[10] * v[5] + v[2] * v[4] + v[29] * v[9]) + -0.26835 * (2.06823107110214e-13 * v[2] + -1 * v[10]) * 4.89663865010925e-12;
   v[2] = sin(x[22]);
   jac[8] = 0 - (v[29] - v[29] * -1 * -1) * v[2];
   v[29] = 0.00085 + -0.26835 * v[1];
   v[10] = -0.139 + -0.26835 * v[0];
   v[3] = -0.10015 + v[29] * -1 + v[10] * v[5];
   v[29] = 0.2 + v[29] * 2.06823107110214e-13 + v[10] * v[4];
   v[17] = v[10] * v[9];
   v[45] = -0.1182 + v[3] * -1 + v[29] * v[8] + v[17] * v[12];
   v[3] = -0.3 + v[3] * 2.06823107110214e-13 + v[29] * v[14] + v[17] * v[15];
   v[46] = v[29] * v[11] + v[17] * v[7];
   v[47] = -0.0625 + v[45] * 4.89663865010925e-12 + v[3] * v[16] + v[46] * v[18];
   v[48] = -0.0566 + v[47] * v[21];
   v[49] = 0 - v[48];
   v[45] = -0.1146 + v[45] * -1 + v[3] * v[24] + v[46] * v[25];
   v[50] = v[3] * v[26] + v[46] * v[16];
   v[51] = -0.125 + v[45] * v[20] + v[50] * v[28];
   v[52] = v[48] * v[23] + v[51] * v[30];
   v[53] = 0.25 + v[45] * v[33] + v[50] * v[20];
   jac[10] = 0 - (v[49] * v[30] + v[51] * v[23]) * v[36] + (v[52] * v[32] + v[53] * v[35]) * v[38];
   jac[11] = 0 - v[53] * v[31] * v[40] + v[52] * v[31] * v[41];
   v[53] = v[31] * v[32];
   jac[12] = 0 - (v[48] * v[53] + v[51] * v[39]) * v[42] + (v[49] * v[39] + v[51] * v[53]) * v[43];
   v[51] = v[53] * v[30] + v[39] * v[23];
   v[31] = v[31] * v[35];
   v[53] = v[53] * v[23] - v[39] * v[30];
   v[47] = v[47] * v[53];
   jac[13] = 0 - (v[45] * v[51] + v[50] * v[31] + v[47] - v[47] * -1 * -1) * v[22] + (v[50] * v[51] - v[45] * v[31]) * -1 * v[44];
   v[53] = v[53] * v[21];
   v[47] = v[31] * v[20] + v[51] * v[28];
   v[31] = v[31] * v[33] + v[51] * v[20];
   jac[14] = 0 - (v[3] * v[53] + v[46] * v[47] + v[3] * v[31] * 4.89663865010925e-12) * v[37] + (v[46] * v[53] + v[46] * v[31] * 4.89663865010925e-12 - v[3] * v[47]) * v[27];
   v[46] = v[53] * v[18] + v[31] * v[25] + v[47] * v[16];
   v[3] = -1 * v[31] + 4.89663865010925e-12 * v[53];
   v[31] = v[53] * v[16] + v[31] * v[24] + v[47] * v[26];
   jac[15] = 0 - (v[17] * v[46] + v[29] * v[3] * -2.06823107110214e-13 + v[29] * v[31] * -1) * v[19] + (v[17] * v[3] * -2.06823107110214e-13 + v[17] * v[31] * -1 - v[29] * v[46]) * v[34];
   v[17] = 2.06823107110214e-13 * v[31] + -1 * v[3];
   v[29] = v[3] * v[8] + v[31] * v[14] + v[46] * v[11];
   v[31] = v[3] * v[12] + v[31] * v[15] + v[46] * v[7];
   jac[16] = 0 - (v[10] * v[17] * -2.06823107110214e-13 + v[10] * v[29] * -1) * v[13] + (0 - v[10] * v[31]) * v[6];
   v[31] = -0.26835 * (v[17] * v[5] + v[29] * v[4] + v[31] * v[9]) + -0.26835 * (2.06823107110214e-13 * v[29] + -1 * v[17]) * 4.89663865010925e-12;
   jac[17] = 0 - (v[31] - v[31] * -1 * -1) * v[2];
   v[1] = 0.00085 + -0.26835 * v[1];
   v[0] = -0.139 + -0.26835 * v[0];
   v[31] = -0.10015 + v[1] * -1 + v[0] * v[5];
   v[1] = 0.2 + v[1] * 2.06823107110214e-13 + v[0] * v[4];
   v[29] = v[0] * v[9];
   v[17] = -0.1182 + v[31] * -1 + v[1] * v[8] + v[29] * v[12];
   v[31] = -0.3 + v[31] * 2.06823107110214e-13 + v[1] * v[14] + v[29] * v[15];
   v[10] = v[1] * v[11] + v[29] * v[7];
   v[3] = -0.0625 + v[17] * 4.89663865010925e-12 + v[31] * v[16] + v[10] * v[18];
   v[46] = -0.0566 + v[3] * v[21];
   v[17] = -0.1146 + v[17] * -1 + v[31] * v[24] + v[10] * v[25];
   v[47] = v[31] * v[26] + v[10] * v[16];
   v[53] = -0.125 + v[17] * v[20] + v[47] * v[28];
   jac[19] = 0 - (v[46] * v[23] + v[53] * v[30]) * v[40] + (0 - (0.25 + v[17] * v[33] + v[47] * v[20])) * v[41];
   jac[20] = 0 - v[46] * v[35] * v[42] + v[53] * v[35] * v[43];
   v[30] = v[35] * v[30];
   v[32] = 0 - v[32];
   v[35] = v[35] * v[23];
   v[3] = v[3] * v[35];
   jac[21] = 0 - (v[17] * v[30] + v[47] * v[32] + v[3] - v[3] * -1 * -1) * v[22] + (v[47] * v[30] - v[17] * v[32]) * -1 * v[44];
   v[35] = v[35] * v[21];
   v[28] = v[32] * v[20] + v[30] * v[28];
   v[32] = v[32] * v[33] + v[30] * v[20];
   jac[22] = 0 - (v[31] * v[35] + v[10] * v[28] + v[31] * v[32] * 4.89663865010925e-12) * v[37] + (v[10] * v[35] + v[10] * v[32] * 4.89663865010925e-12 - v[31] * v[28]) * v[27];
   v[25] = v[35] * v[18] + v[32] * v[25] + v[28] * v[16];
   v[18] = -1 * v[32] + 4.89663865010925e-12 * v[35];
   v[32] = v[35] * v[16] + v[32] * v[24] + v[28] * v[26];
   jac[23] = 0 - (v[29] * v[25] + v[1] * v[18] * -2.06823107110214e-13 + v[1] * v[32] * -1) * v[19] + (v[29] * v[18] * -2.06823107110214e-13 + v[29] * v[32] * -1 - v[1] * v[25]) * v[34];
   v[29] = 2.06823107110214e-13 * v[32] + -1 * v[18];
   v[11] = v[18] * v[8] + v[32] * v[14] + v[25] * v[11];
   v[32] = v[18] * v[12] + v[32] * v[15] + v[25] * v[7];
   jac[24] = 0 - (v[0] * v[29] * -2.06823107110214e-13 + v[0] * v[11] * -1) * v[13] + (0 - v[0] * v[32]) * v[6];
   v[32] = -0.26835 * (v[29] * v[5] + v[11] * v[4] + v[32] * v[9]) + -0.26835 * (2.06823107110214e-13 * v[11] + -1 * v[29]) * 4.89663865010925e-12;
   jac[25] = 0 - (v[32] - v[32] * -1 * -1) * v[2];
   // dependent variables without operations
   jac[0] = 1;
   jac[9] = 1;
   jac[18] = 1;
}

