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

void arm1_8_position_sparse_jacobian(double const *const * in,
                                     double*const * out,
                                     struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[66];

   v[0] = cos(x[42]);
   v[1] = -0.156 + -0.105 * v[0] - 0.095;
   v[2] = cos(x[40]);
   v[3] = sin(x[41]);
   v[4] = 0 - v[3];
   v[5] = sin(x[42]);
   v[6] = -0.105 * v[5];
   v[7] = cos(x[41]);
   v[8] = -0.015 + -0.001 * v[4] + v[6] * v[7];
   v[9] = sin(x[40]);
   v[10] = 0 - v[9];
   v[11] = -0.074 + v[1] * v[2] + v[8] * v[10] - 0.21815;
   v[12] = sin(x[38]);
   v[13] = 0 - v[12];
   v[14] = cos(x[38]);
   v[15] = -0.173647955888778 * v[13] + 0.984807792117654 * v[14];
   v[16] = -0.05515 + -0.001 * v[7] + v[6] * v[3] + 0.05515;
   v[17] = cos(x[39]);
   v[18] = 0.045 + v[1] * v[9] + v[8] * v[2];
   v[19] = sin(x[39]);
   v[20] = v[16] * v[17] + v[18] * v[19];
   v[21] = -0.173647955888778 * v[14] + 0.984807792117654 * v[12];
   v[22] = v[11] * v[15] + v[20] * v[21];
   v[23] = cos(x[37]);
   v[24] = 0.984807740023223 * v[23];
   v[13] = 0.984807792117654 * v[13] + 0.173647955888778 * v[14];
   v[14] = 0.984807792117654 * v[14] + 0.173647955888778 * v[12];
   v[12] = 0.062 + v[11] * v[13] + v[20] * v[14];
   v[25] = 0 - v[19];
   v[26] = 0.09015 + v[16] * v[25] + v[18] * v[17] - 0.09015;
   v[27] = sin(x[37]);
   v[28] = 0 - v[27];
   v[29] = 0.984807740023223 * v[28];
   v[30] = 0.116626 + v[22] * v[24] + v[12] * 0.173648251331108 + v[26] * v[29] + 0.256;
   v[31] = cos(x[11]);
   v[32] = -0.500000021132493 * v[27] + -0.150383794856828 * v[23];
   v[33] = -0.500000021132493 * v[23] + -0.150383794856828 * v[28];
   v[34] = 0.169137 + v[22] * v[32] + v[12] * 0.85286850868816 + v[26] * v[33];
   v[35] = cos(x[36]);
   v[27] = 0.866025391583588 * v[27] + -0.0868241293351745 * v[23];
   v[28] = 0.866025391583588 * v[23] + -0.0868241293351745 * v[28];
   v[12] = 0.0457475 + v[22] * v[27] + v[12] * 0.492403890823054 + v[26] * v[28];
   v[23] = sin(x[36]);
   v[36] = v[34] * v[35] + v[12] * v[23];
   v[37] = sin(x[11]);
   v[38] = v[30] * v[31] + v[36] * v[37];
   v[39] = sin(x[10]);
   v[40] = 0 - v[23];
   v[41] = 0.2 + v[34] * v[40] + v[12] * v[35];
   v[42] = cos(x[10]);
   v[43] = sin(x[9]);
   v[44] = 0 - v[36];
   v[45] = cos(x[9]);
   jac[1] = 0 - (v[38] * v[39] + v[41] * v[42]) * v[43] + (v[30] * v[37] + v[44] * v[31]) * v[45];
   v[46] = cos(x[9]);
   v[47] = sin(x[10]);
   v[48] = cos(x[10]);
   jac[2] = 0 - v[41] * v[46] * v[47] + v[38] * v[46] * v[48];
   v[41] = v[46] * v[39];
   v[38] = sin(x[9]);
   v[49] = sin(x[11]);
   v[50] = cos(x[11]);
   jac[3] = 0 - (v[30] * v[41] + v[44] * v[38]) * v[49] + (v[30] * v[38] + v[36] * v[41]) * v[50];
   v[44] = v[41] * v[37] - v[38] * v[31];
   v[36] = v[46] * v[42];
   v[30] = sin(x[36]);
   v[51] = cos(x[36]);
   jac[4] = 0 - (v[34] * v[44] + v[12] * v[36]) * v[30] + (v[12] * v[44] - v[34] * v[36]) * v[51];
   v[41] = v[41] * v[31] + v[38] * v[37];
   v[12] = v[36] * v[40] + v[44] * v[35];
   v[34] = v[22] * v[12];
   v[36] = v[36] * v[35] + v[44] * v[23];
   v[44] = v[22] * v[36];
   v[52] = v[26] * v[12];
   v[53] = v[26] * v[36];
   v[54] = sin(x[37]);
   v[55] = cos(x[37]);
   jac[5] = 0 - (v[22] * v[41] * 0.984807740023223 + v[34] * -0.150383794856828 + v[44] * -0.0868241293351745 + v[52] * -0.500000021132493 + v[53] * 0.866025391583588) * v[54] + (v[34] * -0.500000021132493 + v[44] * 0.866025391583588 - (v[26] * v[41] * 0.984807740023223 + v[52] * -0.150383794856828 + v[53] * -0.0868241293351745)) * v[55];
   v[53] = v[41] * v[24] + v[12] * v[32] + v[36] * v[27];
   v[52] = v[11] * v[53];
   v[44] = 0.173648251331108 * v[41] + 0.85286850868816 * v[12] + 0.492403890823054 * v[36];
   v[11] = v[11] * v[44];
   v[34] = v[20] * v[53];
   v[20] = v[20] * v[44];
   v[26] = sin(x[38]);
   v[22] = cos(x[38]);
   jac[6] = 0 - (v[52] * 0.984807792117654 + v[11] * 0.173647955888778 + v[34] * -0.173647955888778 + v[20] * 0.984807792117654) * v[26] + (v[34] * 0.984807792117654 + v[20] * 0.173647955888778 - (v[52] * -0.173647955888778 + v[11] * 0.984807792117654)) * v[22];
   v[20] = v[44] * v[14] + v[53] * v[21];
   v[36] = v[41] * v[29] + v[12] * v[33] + v[36] * v[28];
   v[12] = sin(x[39]);
   v[41] = cos(x[39]);
   jac[7] = 0 - (v[16] * v[20] + v[18] * v[36]) * v[12] + (v[18] * v[20] - v[16] * v[36]) * v[41];
   v[44] = v[44] * v[13] + v[53] * v[15];
   v[53] = v[36] * v[17] + v[20] * v[19];
   v[18] = sin(x[40]);
   v[16] = cos(x[40]);
   jac[8] = 0 - (v[1] * v[44] + v[8] * v[53]) * v[18] + (v[1] * v[53] - v[8] * v[44]) * v[16];
   v[36] = v[36] * v[25] + v[20] * v[17];
   v[20] = v[53] * v[2] + v[44] * v[10];
   v[8] = sin(x[41]);
   v[1] = cos(x[41]);
   jac[9] = 0 - (-0.001 * v[36] + v[6] * v[20]) * v[8] + (v[6] * v[36] - -0.001 * v[20]) * v[1];
   v[6] = sin(x[42]);
   v[34] = cos(x[42]);
   jac[10] = 0 - -0.105 * (v[53] * v[9] + v[44] * v[2]) * v[6] + -0.105 * (v[20] * v[7] + v[36] * v[3]) * v[34];
   v[20] = -0.156 + -0.105 * v[0] - 0.095;
   v[36] = -0.105 * v[5];
   v[53] = -0.015 + -0.001 * v[4] + v[36] * v[7];
   v[44] = -0.074 + v[20] * v[2] + v[53] * v[10] - 0.21815;
   v[11] = -0.05515 + -0.001 * v[7] + v[36] * v[3] + 0.05515;
   v[52] = 0.045 + v[20] * v[9] + v[53] * v[2];
   v[56] = v[11] * v[17] + v[52] * v[19];
   v[57] = v[44] * v[15] + v[56] * v[21];
   v[58] = 0.062 + v[44] * v[13] + v[56] * v[14];
   v[59] = 0.09015 + v[11] * v[25] + v[52] * v[17] - 0.09015;
   v[60] = 0.116626 + v[57] * v[24] + v[58] * 0.173648251331108 + v[59] * v[29] + 0.256;
   v[61] = 0 - v[60];
   v[62] = 0.169137 + v[57] * v[32] + v[58] * 0.85286850868816 + v[59] * v[33];
   v[58] = 0.0457475 + v[57] * v[27] + v[58] * 0.492403890823054 + v[59] * v[28];
   v[63] = v[62] * v[35] + v[58] * v[23];
   v[64] = v[60] * v[31] + v[63] * v[37];
   v[65] = 0.2 + v[62] * v[40] + v[58] * v[35];
   jac[12] = 0 - (v[61] * v[37] + v[63] * v[31]) * v[43] + (v[64] * v[39] + v[65] * v[42]) * v[45];
   jac[13] = 0 - v[65] * v[38] * v[47] + v[64] * v[38] * v[48];
   v[65] = v[38] * v[39];
   jac[14] = 0 - (v[60] * v[65] + v[63] * v[46]) * v[49] + (v[61] * v[46] + v[63] * v[65]) * v[50];
   v[63] = v[65] * v[37] + v[46] * v[31];
   v[38] = v[38] * v[42];
   jac[15] = 0 - (v[62] * v[63] + v[58] * v[38]) * v[30] + (v[58] * v[63] - v[62] * v[38]) * v[51];
   v[65] = v[65] * v[31] - v[46] * v[37];
   v[46] = v[38] * v[40] + v[63] * v[35];
   v[58] = v[57] * v[46];
   v[38] = v[38] * v[35] + v[63] * v[23];
   v[63] = v[57] * v[38];
   v[62] = v[59] * v[46];
   v[61] = v[59] * v[38];
   jac[16] = 0 - (v[57] * v[65] * 0.984807740023223 + v[58] * -0.150383794856828 + v[63] * -0.0868241293351745 + v[62] * -0.500000021132493 + v[61] * 0.866025391583588) * v[54] + (v[58] * -0.500000021132493 + v[63] * 0.866025391583588 - (v[59] * v[65] * 0.984807740023223 + v[62] * -0.150383794856828 + v[61] * -0.0868241293351745)) * v[55];
   v[61] = v[65] * v[24] + v[46] * v[32] + v[38] * v[27];
   v[62] = v[44] * v[61];
   v[63] = 0.173648251331108 * v[65] + 0.85286850868816 * v[46] + 0.492403890823054 * v[38];
   v[44] = v[44] * v[63];
   v[58] = v[56] * v[61];
   v[56] = v[56] * v[63];
   jac[17] = 0 - (v[62] * 0.984807792117654 + v[44] * 0.173647955888778 + v[58] * -0.173647955888778 + v[56] * 0.984807792117654) * v[26] + (v[58] * 0.984807792117654 + v[56] * 0.173647955888778 - (v[62] * -0.173647955888778 + v[44] * 0.984807792117654)) * v[22];
   v[56] = v[63] * v[14] + v[61] * v[21];
   v[38] = v[65] * v[29] + v[46] * v[33] + v[38] * v[28];
   jac[18] = 0 - (v[11] * v[56] + v[52] * v[38]) * v[12] + (v[52] * v[56] - v[11] * v[38]) * v[41];
   v[63] = v[63] * v[13] + v[61] * v[15];
   v[61] = v[38] * v[17] + v[56] * v[19];
   jac[19] = 0 - (v[20] * v[63] + v[53] * v[61]) * v[18] + (v[20] * v[61] - v[53] * v[63]) * v[16];
   v[38] = v[38] * v[25] + v[56] * v[17];
   v[56] = v[61] * v[2] + v[63] * v[10];
   jac[20] = 0 - (-0.001 * v[38] + v[36] * v[56]) * v[8] + (v[36] * v[38] - -0.001 * v[56]) * v[1];
   jac[21] = 0 - -0.105 * (v[61] * v[9] + v[63] * v[2]) * v[6] + -0.105 * (v[56] * v[7] + v[38] * v[3]) * v[34];
   v[0] = -0.156 + -0.105 * v[0] - 0.095;
   v[5] = -0.105 * v[5];
   v[4] = -0.015 + -0.001 * v[4] + v[5] * v[7];
   v[56] = -0.074 + v[0] * v[2] + v[4] * v[10] - 0.21815;
   v[38] = -0.05515 + -0.001 * v[7] + v[5] * v[3] + 0.05515;
   v[61] = 0.045 + v[0] * v[9] + v[4] * v[2];
   v[63] = v[38] * v[17] + v[61] * v[19];
   v[36] = v[56] * v[15] + v[63] * v[21];
   v[53] = 0.062 + v[56] * v[13] + v[63] * v[14];
   v[20] = 0.09015 + v[38] * v[25] + v[61] * v[17] - 0.09015;
   v[52] = 0.116626 + v[36] * v[24] + v[53] * 0.173648251331108 + v[20] * v[29] + 0.256;
   v[11] = 0.169137 + v[36] * v[32] + v[53] * 0.85286850868816 + v[20] * v[33];
   v[53] = 0.0457475 + v[36] * v[27] + v[53] * 0.492403890823054 + v[20] * v[28];
   v[46] = v[11] * v[35] + v[53] * v[23];
   jac[23] = 0 - (v[52] * v[31] + v[46] * v[37]) * v[47] + (0 - (0.2 + v[11] * v[40] + v[53] * v[35])) * v[48];
   jac[24] = 0 - v[52] * v[42] * v[49] + v[46] * v[42] * v[50];
   v[37] = v[42] * v[37];
   v[39] = 0 - v[39];
   jac[25] = 0 - (v[11] * v[37] + v[53] * v[39]) * v[30] + (v[53] * v[37] - v[11] * v[39]) * v[51];
   v[42] = v[42] * v[31];
   v[40] = v[39] * v[40] + v[37] * v[35];
   v[31] = v[36] * v[40];
   v[39] = v[39] * v[35] + v[37] * v[23];
   v[37] = v[36] * v[39];
   v[23] = v[20] * v[40];
   v[35] = v[20] * v[39];
   jac[26] = 0 - (v[36] * v[42] * 0.984807740023223 + v[31] * -0.150383794856828 + v[37] * -0.0868241293351745 + v[23] * -0.500000021132493 + v[35] * 0.866025391583588) * v[54] + (v[31] * -0.500000021132493 + v[37] * 0.866025391583588 - (v[20] * v[42] * 0.984807740023223 + v[23] * -0.150383794856828 + v[35] * -0.0868241293351745)) * v[55];
   v[27] = v[42] * v[24] + v[40] * v[32] + v[39] * v[27];
   v[32] = v[56] * v[27];
   v[24] = 0.173648251331108 * v[42] + 0.85286850868816 * v[40] + 0.492403890823054 * v[39];
   v[56] = v[56] * v[24];
   v[35] = v[63] * v[27];
   v[63] = v[63] * v[24];
   jac[27] = 0 - (v[32] * 0.984807792117654 + v[56] * 0.173647955888778 + v[35] * -0.173647955888778 + v[63] * 0.984807792117654) * v[26] + (v[35] * 0.984807792117654 + v[63] * 0.173647955888778 - (v[32] * -0.173647955888778 + v[56] * 0.984807792117654)) * v[22];
   v[14] = v[24] * v[14] + v[27] * v[21];
   v[39] = v[42] * v[29] + v[40] * v[33] + v[39] * v[28];
   jac[28] = 0 - (v[38] * v[14] + v[61] * v[39]) * v[12] + (v[61] * v[14] - v[38] * v[39]) * v[41];
   v[24] = v[24] * v[13] + v[27] * v[15];
   v[19] = v[39] * v[17] + v[14] * v[19];
   jac[29] = 0 - (v[0] * v[24] + v[4] * v[19]) * v[18] + (v[0] * v[19] - v[4] * v[24]) * v[16];
   v[39] = v[39] * v[25] + v[14] * v[17];
   v[10] = v[19] * v[2] + v[24] * v[10];
   jac[30] = 0 - (-0.001 * v[39] + v[5] * v[10]) * v[8] + (v[5] * v[39] - -0.001 * v[10]) * v[1];
   jac[31] = 0 - -0.105 * (v[19] * v[9] + v[24] * v[2]) * v[6] + -0.105 * (v[10] * v[7] + v[39] * v[3]) * v[34];
   // dependent variables without operations
   jac[0] = 1;
   jac[11] = 1;
   jac[22] = 1;
}

