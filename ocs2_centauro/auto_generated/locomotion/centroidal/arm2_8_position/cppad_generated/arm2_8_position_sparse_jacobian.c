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

void arm2_8_position_sparse_jacobian(double const *const * in,
                                     double*const * out,
                                     struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[67];

   v[0] = cos(x[48]);
   v[1] = sin(x[48]);
   v[2] = 0 - v[1];
   v[3] = -0.156 + -0.13025 * v[0] + -1.03928611322883e-14 * v[2] - 0.095;
   v[4] = cos(x[46]);
   v[5] = sin(x[47]);
   v[6] = 0 - v[5];
   v[7] = -0.13025 * v[1] + -1.03928611322883e-14 * v[0];
   v[8] = cos(x[47]);
   v[9] = -0.015 + -0.027 * v[6] + v[7] * v[8];
   v[10] = sin(x[46]);
   v[11] = 0 - v[10];
   v[12] = -0.074 + v[3] * v[4] + v[9] * v[11] - 0.21815;
   v[13] = sin(x[44]);
   v[14] = 0 - v[13];
   v[15] = cos(x[44]);
   v[16] = 0.173647955888778 * v[14] + 0.984807792117654 * v[15];
   v[17] = 0.05515 + -0.027 * v[8] + v[7] * v[5] - 0.05515;
   v[18] = cos(x[45]);
   v[19] = 0.045 + v[3] * v[10] + v[9] * v[4];
   v[20] = sin(x[45]);
   v[21] = v[17] * v[18] + v[19] * v[20];
   v[22] = 0.173647955888778 * v[15] + 0.984807792117654 * v[13];
   v[23] = v[12] * v[16] + v[21] * v[22];
   v[24] = cos(x[43]);
   v[25] = 0.984807740023223 * v[24];
   v[14] = 0.984807792117654 * v[14] + -0.173647955888778 * v[15];
   v[15] = 0.984807792117654 * v[15] + -0.173647955888778 * v[13];
   v[13] = -0.062 + v[12] * v[14] + v[21] * v[15];
   v[26] = 0 - v[20];
   v[27] = 0.09015 + v[17] * v[26] + v[19] * v[18] - 0.09015;
   v[28] = sin(x[43]);
   v[29] = 0 - v[28];
   v[30] = 0.984807740023223 * v[29];
   v[31] = 0.116626 + v[23] * v[25] + v[13] * -0.173648251331108 + v[27] * v[30] + 0.256;
   v[32] = cos(x[11]);
   v[33] = 0.500000021132493 * v[28] + 0.150383794856828 * v[24];
   v[34] = 0.500000021132493 * v[24] + 0.150383794856828 * v[29];
   v[35] = -0.169137 + v[23] * v[33] + v[13] * 0.85286850868816 + v[27] * v[34];
   v[36] = cos(x[36]);
   v[28] = 0.866025391583588 * v[28] + -0.0868241293351745 * v[24];
   v[29] = 0.866025391583588 * v[24] + -0.0868241293351745 * v[29];
   v[13] = 0.0457475 + v[23] * v[28] + v[13] * -0.492403890823054 + v[27] * v[29];
   v[24] = sin(x[36]);
   v[37] = v[35] * v[36] + v[13] * v[24];
   v[38] = sin(x[11]);
   v[39] = v[31] * v[32] + v[37] * v[38];
   v[40] = sin(x[10]);
   v[41] = 0 - v[24];
   v[42] = 0.2 + v[35] * v[41] + v[13] * v[36];
   v[43] = cos(x[10]);
   v[44] = sin(x[9]);
   v[45] = 0 - v[37];
   v[46] = cos(x[9]);
   jac[1] = 0 - (v[39] * v[40] + v[42] * v[43]) * v[44] + (v[31] * v[38] + v[45] * v[32]) * v[46];
   v[47] = cos(x[9]);
   v[48] = sin(x[10]);
   v[49] = cos(x[10]);
   jac[2] = 0 - v[42] * v[47] * v[48] + v[39] * v[47] * v[49];
   v[42] = v[47] * v[40];
   v[39] = sin(x[9]);
   v[50] = sin(x[11]);
   v[51] = cos(x[11]);
   jac[3] = 0 - (v[31] * v[42] + v[45] * v[39]) * v[50] + (v[31] * v[39] + v[37] * v[42]) * v[51];
   v[45] = v[42] * v[38] - v[39] * v[32];
   v[37] = v[47] * v[43];
   v[31] = sin(x[36]);
   v[52] = cos(x[36]);
   jac[4] = 0 - (v[35] * v[45] + v[13] * v[37]) * v[31] + (v[13] * v[45] - v[35] * v[37]) * v[52];
   v[42] = v[42] * v[32] + v[39] * v[38];
   v[13] = v[37] * v[41] + v[45] * v[36];
   v[35] = v[23] * v[13];
   v[37] = v[37] * v[36] + v[45] * v[24];
   v[45] = v[23] * v[37];
   v[53] = v[27] * v[13];
   v[54] = v[27] * v[37];
   v[55] = sin(x[43]);
   v[56] = cos(x[43]);
   jac[5] = 0 - (v[23] * v[42] * 0.984807740023223 + v[35] * 0.150383794856828 + v[45] * -0.0868241293351745 + v[53] * 0.500000021132493 + v[54] * 0.866025391583588) * v[55] + (v[35] * 0.500000021132493 + v[45] * 0.866025391583588 - (v[27] * v[42] * 0.984807740023223 + v[53] * 0.150383794856828 + v[54] * -0.0868241293351745)) * v[56];
   v[54] = v[42] * v[25] + v[13] * v[33] + v[37] * v[28];
   v[53] = v[12] * v[54];
   v[45] = -0.173648251331108 * v[42] + 0.85286850868816 * v[13] + -0.492403890823054 * v[37];
   v[12] = v[12] * v[45];
   v[35] = v[21] * v[54];
   v[21] = v[21] * v[45];
   v[27] = sin(x[44]);
   v[23] = cos(x[44]);
   jac[6] = 0 - (v[53] * 0.984807792117654 + v[12] * -0.173647955888778 + v[35] * 0.173647955888778 + v[21] * 0.984807792117654) * v[27] + (v[35] * 0.984807792117654 + v[21] * -0.173647955888778 - (v[53] * 0.173647955888778 + v[12] * 0.984807792117654)) * v[23];
   v[21] = v[45] * v[15] + v[54] * v[22];
   v[37] = v[42] * v[30] + v[13] * v[34] + v[37] * v[29];
   v[13] = sin(x[45]);
   v[42] = cos(x[45]);
   jac[7] = 0 - (v[17] * v[21] + v[19] * v[37]) * v[13] + (v[19] * v[21] - v[17] * v[37]) * v[42];
   v[45] = v[45] * v[14] + v[54] * v[16];
   v[54] = v[37] * v[18] + v[21] * v[20];
   v[19] = sin(x[46]);
   v[17] = cos(x[46]);
   jac[8] = 0 - (v[3] * v[45] + v[9] * v[54]) * v[19] + (v[3] * v[54] - v[9] * v[45]) * v[17];
   v[37] = v[37] * v[26] + v[21] * v[18];
   v[21] = v[54] * v[4] + v[45] * v[11];
   v[9] = sin(x[47]);
   v[3] = cos(x[47]);
   jac[9] = 0 - (-0.027 * v[37] + v[7] * v[21]) * v[9] + (v[7] * v[37] - -0.027 * v[21]) * v[3];
   v[54] = v[54] * v[10] + v[45] * v[4];
   v[21] = v[21] * v[8] + v[37] * v[5];
   v[37] = sin(x[48]);
   v[45] = cos(x[48]);
   jac[10] = 0 - (-0.13025 * v[54] + -1.03928611322883e-14 * v[21]) * v[37] + (-0.13025 * v[21] - -1.03928611322883e-14 * v[54]) * v[45];
   v[21] = -0.156 + -0.13025 * v[0] + -1.03928611322883e-14 * v[2] - 0.095;
   v[54] = -0.13025 * v[1] + -1.03928611322883e-14 * v[0];
   v[7] = -0.015 + -0.027 * v[6] + v[54] * v[8];
   v[35] = -0.074 + v[21] * v[4] + v[7] * v[11] - 0.21815;
   v[12] = 0.05515 + -0.027 * v[8] + v[54] * v[5] - 0.05515;
   v[53] = 0.045 + v[21] * v[10] + v[7] * v[4];
   v[57] = v[12] * v[18] + v[53] * v[20];
   v[58] = v[35] * v[16] + v[57] * v[22];
   v[59] = -0.062 + v[35] * v[14] + v[57] * v[15];
   v[60] = 0.09015 + v[12] * v[26] + v[53] * v[18] - 0.09015;
   v[61] = 0.116626 + v[58] * v[25] + v[59] * -0.173648251331108 + v[60] * v[30] + 0.256;
   v[62] = 0 - v[61];
   v[63] = -0.169137 + v[58] * v[33] + v[59] * 0.85286850868816 + v[60] * v[34];
   v[59] = 0.0457475 + v[58] * v[28] + v[59] * -0.492403890823054 + v[60] * v[29];
   v[64] = v[63] * v[36] + v[59] * v[24];
   v[65] = v[61] * v[32] + v[64] * v[38];
   v[66] = 0.2 + v[63] * v[41] + v[59] * v[36];
   jac[12] = 0 - (v[62] * v[38] + v[64] * v[32]) * v[44] + (v[65] * v[40] + v[66] * v[43]) * v[46];
   jac[13] = 0 - v[66] * v[39] * v[48] + v[65] * v[39] * v[49];
   v[66] = v[39] * v[40];
   jac[14] = 0 - (v[61] * v[66] + v[64] * v[47]) * v[50] + (v[62] * v[47] + v[64] * v[66]) * v[51];
   v[64] = v[66] * v[38] + v[47] * v[32];
   v[39] = v[39] * v[43];
   jac[15] = 0 - (v[63] * v[64] + v[59] * v[39]) * v[31] + (v[59] * v[64] - v[63] * v[39]) * v[52];
   v[66] = v[66] * v[32] - v[47] * v[38];
   v[47] = v[39] * v[41] + v[64] * v[36];
   v[59] = v[58] * v[47];
   v[39] = v[39] * v[36] + v[64] * v[24];
   v[64] = v[58] * v[39];
   v[63] = v[60] * v[47];
   v[62] = v[60] * v[39];
   jac[16] = 0 - (v[58] * v[66] * 0.984807740023223 + v[59] * 0.150383794856828 + v[64] * -0.0868241293351745 + v[63] * 0.500000021132493 + v[62] * 0.866025391583588) * v[55] + (v[59] * 0.500000021132493 + v[64] * 0.866025391583588 - (v[60] * v[66] * 0.984807740023223 + v[63] * 0.150383794856828 + v[62] * -0.0868241293351745)) * v[56];
   v[62] = v[66] * v[25] + v[47] * v[33] + v[39] * v[28];
   v[63] = v[35] * v[62];
   v[64] = -0.173648251331108 * v[66] + 0.85286850868816 * v[47] + -0.492403890823054 * v[39];
   v[35] = v[35] * v[64];
   v[59] = v[57] * v[62];
   v[57] = v[57] * v[64];
   jac[17] = 0 - (v[63] * 0.984807792117654 + v[35] * -0.173647955888778 + v[59] * 0.173647955888778 + v[57] * 0.984807792117654) * v[27] + (v[59] * 0.984807792117654 + v[57] * -0.173647955888778 - (v[63] * 0.173647955888778 + v[35] * 0.984807792117654)) * v[23];
   v[57] = v[64] * v[15] + v[62] * v[22];
   v[39] = v[66] * v[30] + v[47] * v[34] + v[39] * v[29];
   jac[18] = 0 - (v[12] * v[57] + v[53] * v[39]) * v[13] + (v[53] * v[57] - v[12] * v[39]) * v[42];
   v[64] = v[64] * v[14] + v[62] * v[16];
   v[62] = v[39] * v[18] + v[57] * v[20];
   jac[19] = 0 - (v[21] * v[64] + v[7] * v[62]) * v[19] + (v[21] * v[62] - v[7] * v[64]) * v[17];
   v[39] = v[39] * v[26] + v[57] * v[18];
   v[57] = v[62] * v[4] + v[64] * v[11];
   jac[20] = 0 - (-0.027 * v[39] + v[54] * v[57]) * v[9] + (v[54] * v[39] - -0.027 * v[57]) * v[3];
   v[62] = v[62] * v[10] + v[64] * v[4];
   v[57] = v[57] * v[8] + v[39] * v[5];
   jac[21] = 0 - (-0.13025 * v[62] + -1.03928611322883e-14 * v[57]) * v[37] + (-0.13025 * v[57] - -1.03928611322883e-14 * v[62]) * v[45];
   v[2] = -0.156 + -0.13025 * v[0] + -1.03928611322883e-14 * v[2] - 0.095;
   v[1] = -0.13025 * v[1] + -1.03928611322883e-14 * v[0];
   v[6] = -0.015 + -0.027 * v[6] + v[1] * v[8];
   v[0] = -0.074 + v[2] * v[4] + v[6] * v[11] - 0.21815;
   v[57] = 0.05515 + -0.027 * v[8] + v[1] * v[5] - 0.05515;
   v[62] = 0.045 + v[2] * v[10] + v[6] * v[4];
   v[39] = v[57] * v[18] + v[62] * v[20];
   v[64] = v[0] * v[16] + v[39] * v[22];
   v[54] = -0.062 + v[0] * v[14] + v[39] * v[15];
   v[7] = 0.09015 + v[57] * v[26] + v[62] * v[18] - 0.09015;
   v[21] = 0.116626 + v[64] * v[25] + v[54] * -0.173648251331108 + v[7] * v[30] + 0.256;
   v[53] = -0.169137 + v[64] * v[33] + v[54] * 0.85286850868816 + v[7] * v[34];
   v[54] = 0.0457475 + v[64] * v[28] + v[54] * -0.492403890823054 + v[7] * v[29];
   v[12] = v[53] * v[36] + v[54] * v[24];
   jac[23] = 0 - (v[21] * v[32] + v[12] * v[38]) * v[48] + (0 - (0.2 + v[53] * v[41] + v[54] * v[36])) * v[49];
   jac[24] = 0 - v[21] * v[43] * v[50] + v[12] * v[43] * v[51];
   v[38] = v[43] * v[38];
   v[40] = 0 - v[40];
   jac[25] = 0 - (v[53] * v[38] + v[54] * v[40]) * v[31] + (v[54] * v[38] - v[53] * v[40]) * v[52];
   v[43] = v[43] * v[32];
   v[41] = v[40] * v[41] + v[38] * v[36];
   v[32] = v[64] * v[41];
   v[40] = v[40] * v[36] + v[38] * v[24];
   v[38] = v[64] * v[40];
   v[24] = v[7] * v[41];
   v[36] = v[7] * v[40];
   jac[26] = 0 - (v[64] * v[43] * 0.984807740023223 + v[32] * 0.150383794856828 + v[38] * -0.0868241293351745 + v[24] * 0.500000021132493 + v[36] * 0.866025391583588) * v[55] + (v[32] * 0.500000021132493 + v[38] * 0.866025391583588 - (v[7] * v[43] * 0.984807740023223 + v[24] * 0.150383794856828 + v[36] * -0.0868241293351745)) * v[56];
   v[28] = v[43] * v[25] + v[41] * v[33] + v[40] * v[28];
   v[33] = v[0] * v[28];
   v[25] = -0.173648251331108 * v[43] + 0.85286850868816 * v[41] + -0.492403890823054 * v[40];
   v[0] = v[0] * v[25];
   v[36] = v[39] * v[28];
   v[39] = v[39] * v[25];
   jac[27] = 0 - (v[33] * 0.984807792117654 + v[0] * -0.173647955888778 + v[36] * 0.173647955888778 + v[39] * 0.984807792117654) * v[27] + (v[36] * 0.984807792117654 + v[39] * -0.173647955888778 - (v[33] * 0.173647955888778 + v[0] * 0.984807792117654)) * v[23];
   v[15] = v[25] * v[15] + v[28] * v[22];
   v[40] = v[43] * v[30] + v[41] * v[34] + v[40] * v[29];
   jac[28] = 0 - (v[57] * v[15] + v[62] * v[40]) * v[13] + (v[62] * v[15] - v[57] * v[40]) * v[42];
   v[25] = v[25] * v[14] + v[28] * v[16];
   v[20] = v[40] * v[18] + v[15] * v[20];
   jac[29] = 0 - (v[2] * v[25] + v[6] * v[20]) * v[19] + (v[2] * v[20] - v[6] * v[25]) * v[17];
   v[40] = v[40] * v[26] + v[15] * v[18];
   v[11] = v[20] * v[4] + v[25] * v[11];
   jac[30] = 0 - (-0.027 * v[40] + v[1] * v[11]) * v[9] + (v[1] * v[40] - -0.027 * v[11]) * v[3];
   v[20] = v[20] * v[10] + v[25] * v[4];
   v[11] = v[11] * v[8] + v[40] * v[5];
   jac[31] = 0 - (-0.13025 * v[20] + -1.03928611322883e-14 * v[11]) * v[37] + (-0.13025 * v[11] - -1.03928611322883e-14 * v[20]) * v[45];
   // dependent variables without operations
   jac[0] = 1;
   jac[11] = 1;
   jac[22] = 1;
}

