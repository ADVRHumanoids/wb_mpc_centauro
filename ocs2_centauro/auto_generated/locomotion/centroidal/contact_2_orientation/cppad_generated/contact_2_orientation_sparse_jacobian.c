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

void contact_2_orientation_sparse_jacobian(double const *const * in,
                                           double*const * out,
                                           struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[158];

   v[0] = cos(x[9]);
   v[1] = cos(x[10]);
   v[2] = v[0] * v[1];
   v[3] = -1 * sin(x[18]);
   v[4] = 0 - v[3];
   v[5] = sin(x[10]);
   v[6] = v[0] * v[5];
   v[7] = sin(x[11]);
   v[8] = sin(x[9]);
   v[9] = cos(x[11]);
   v[10] = v[6] * v[7] - v[8] * v[9];
   v[11] = cos(x[18]);
   v[12] = v[2] * v[4] + v[10] * v[11];
   v[13] = v[6] * v[9] + v[8] * v[7];
   v[14] = -1 * -1 * (1 - v[11]) + v[11];
   v[15] = v[13] * v[14];
   v[16] = -1 * v[12] + 4.89663865010925e-12 * v[15];
   v[17] = cos(x[20]);
   v[18] = -2.06823107110214e-13 * v[17];
   v[19] = cos(x[19]);
   v[20] = 4.89663865010925e-12 * v[19];
   v[21] = v[2] * v[11] + v[10] * v[3];
   v[22] = sin(x[19]);
   v[23] = 0 - v[22];
   v[24] = v[15] * v[19] + v[12] * v[20] + v[21] * v[23];
   v[25] = -1 * v[17];
   v[26] = 4.89663865010925e-12 * v[22];
   v[27] = v[15] * v[22] + v[12] * v[26] + v[21] * v[19];
   v[28] = sin(x[20]);
   v[29] = 0 - v[28];
   v[30] = v[16] * v[18] + v[24] * v[25] + v[27] * v[29];
   v[31] = 2.06823107110214e-13 * v[24] + -1 * v[16];
   v[32] = 2.06823107110214e-13 * v[30] + -1 * v[31];
   v[33] = -1 * sin(x[22]);
   v[34] = -1 * v[33];
   v[35] = cos(x[21]);
   v[36] = -2.06823107110214e-13 * v[35];
   v[37] = -1 * v[35];
   v[38] = -2.06823107110214e-13 * v[28];
   v[28] = -1 * v[28];
   v[39] = v[16] * v[38] + v[24] * v[28] + v[27] * v[17];
   v[40] = sin(x[21]);
   v[41] = 0 - v[40];
   v[42] = v[31] * v[36] + v[30] * v[37] + v[39] * v[41];
   v[43] = 4.89663865010925e-12 * v[33];
   v[44] = -2.06823107110214e-13 * v[40];
   v[40] = -1 * v[40];
   v[45] = v[31] * v[44] + v[30] * v[40] + v[39] * v[35];
   v[46] = cos(x[22]);
   v[47] = v[32] * v[34] + v[42] * v[43] + v[45] * v[46];
   v[48] = v[8] * v[1];
   v[49] = v[8] * v[5];
   v[50] = v[49] * v[7] + v[0] * v[9];
   v[51] = v[48] * v[4] + v[50] * v[11];
   v[52] = v[49] * v[9] - v[0] * v[7];
   v[53] = v[52] * v[14];
   v[54] = -1 * v[51] + 4.89663865010925e-12 * v[53];
   v[55] = v[48] * v[11] + v[50] * v[3];
   v[56] = v[53] * v[19] + v[51] * v[20] + v[55] * v[23];
   v[57] = v[53] * v[22] + v[51] * v[26] + v[55] * v[19];
   v[58] = v[54] * v[18] + v[56] * v[25] + v[57] * v[29];
   v[59] = 2.06823107110214e-13 * v[56] + -1 * v[54];
   v[60] = 2.06823107110214e-13 * v[58] + -1 * v[59];
   v[61] = -1 * v[46];
   v[62] = v[54] * v[38] + v[56] * v[28] + v[57] * v[17];
   v[63] = v[59] * v[36] + v[58] * v[37] + v[62] * v[41];
   v[64] = 4.89663865010925e-12 * v[46];
   v[65] = v[59] * v[44] + v[58] * v[40] + v[62] * v[35];
   v[33] = 0 - v[33];
   v[66] = v[60] * v[61] + v[63] * v[64] + v[65] * v[33];
   v[67] = v[1] * v[9];
   v[68] = v[67] * v[14];
   v[69] = 0 - v[5];
   v[70] = v[1] * v[7];
   v[71] = v[69] * v[4] + v[70] * v[11];
   v[72] = v[69] * v[11] + v[70] * v[3];
   v[73] = v[68] * v[19] + v[71] * v[20] + v[72] * v[23];
   v[74] = -1 * v[71] + 4.89663865010925e-12 * v[68];
   v[75] = 2.06823107110214e-13 * v[73] + -1 * v[74];
   v[76] = v[68] * v[22] + v[71] * v[26] + v[72] * v[19];
   v[77] = v[74] * v[18] + v[73] * v[25] + v[76] * v[29];
   v[78] = v[74] * v[38] + v[73] * v[28] + v[76] * v[17];
   v[79] = v[75] * v[36] + v[77] * v[37] + v[78] * v[41];
   v[80] = -1 * -1 * (1 - v[46]) + v[46];
   v[81] = 2.06823107110214e-13 * v[77] + -1 * v[75];
   v[82] = 4.89663865010925e-12 * v[80];
   v[83] = v[79] * v[80] + v[81] * v[82];
   v[84] = -1 * x[50];
   if( v[47] > v[66] ) {
      v[85] = 1 + v[47] - v[66] - v[83];
   } else {
      v[85] = 1 + v[66] - v[47] - v[83];
   }
   v[86] = 0 - v[66];
   if( v[47] < v[86] ) {
      v[87] = 1 + v[83] - v[47] - v[66];
   } else {
      v[87] = 1 + v[47] + v[66] + v[83];
   }
   if( v[83] < 0 ) {
      v[87] = v[85];
   } else {
      v[87] = v[87];
   }
   v[85] = sqrt(v[87]);
   v[88] = 0.5 / v[85];
   v[89] = v[84] * v[88];
   if( v[83] < 0 ) {
      v[90] = v[89];
   } else {
      v[90] = 0;
   }
   if( v[47] > v[66] ) {
      v[91] = 0;
   } else {
      v[91] = v[90];
   }
   v[92] = 0 - v[66];
   v[93] = x[51] * v[88];
   if( v[83] < 0 ) {
      v[94] = 0;
   } else {
      v[94] = v[93];
   }
   if( v[47] < v[92] ) {
      v[95] = v[94];
   } else {
      v[95] = 0;
   }
   v[95] = v[91] + v[95];
   v[91] = x[49] * v[88];
   if( v[83] < 0 ) {
      v[96] = v[91];
   } else {
      v[96] = 0;
   }
   if( v[47] > v[66] ) {
      v[97] = v[96];
   } else {
      v[97] = 0;
   }
   v[98] = 0 - v[66];
   v[99] = -1 * x[52];
   v[100] = v[99] * v[88];
   if( v[83] < 0 ) {
      v[101] = 0;
   } else {
      v[101] = v[100];
   }
   if( v[47] < v[98] ) {
      v[102] = 0;
   } else {
      v[102] = v[101];
   }
   v[102] = v[97] + v[102];
   v[97] = v[95] - v[102];
   v[103] = v[75] * v[44] + v[77] * v[40] + v[78] * v[35];
   v[104] = v[81] * v[61] + v[79] * v[64] + v[103] * v[33];
   v[105] = v[63] * v[80] + v[60] * v[82];
   v[106] = v[104] - v[105];
   v[107] = v[42] * v[80] + v[32] * v[82];
   v[108] = v[81] * v[34] + v[79] * v[43] + v[103] * v[46];
   v[109] = v[107] - v[108];
   if( v[47] > v[66] ) {
      v[110] = v[106];
   } else {
      v[110] = v[109];
   }
   v[111] = v[60] * v[34] + v[63] * v[43] + v[65] * v[46];
   v[112] = v[32] * v[61] + v[42] * v[64] + v[45] * v[33];
   v[113] = v[111] - v[112];
   if( v[47] < v[98] ) {
      v[114] = v[113];
   } else {
      v[114] = v[87];
   }
   if( v[83] < 0 ) {
      v[114] = v[110];
   } else {
      v[114] = v[114];
   }
   v[108] = v[107] + v[108];
   v[105] = v[104] + v[105];
   if( v[47] > v[66] ) {
      v[104] = v[108];
   } else {
      v[104] = v[105];
   }
   if( v[47] < v[98] ) {
      v[113] = v[87];
   } else {
      v[113] = v[113];
   }
   if( v[83] < 0 ) {
      v[113] = v[104];
   } else {
      v[113] = v[113];
   }
   v[112] = v[111] + v[112];
   if( v[47] > v[66] ) {
      v[111] = v[112];
   } else {
      v[111] = v[87];
   }
   if( v[47] < v[92] ) {
      v[105] = v[105];
   } else {
      v[105] = v[109];
   }
   if( v[83] < 0 ) {
      v[105] = v[111];
   } else {
      v[105] = v[105];
   }
   if( v[47] > v[66] ) {
      v[112] = v[87];
   } else {
      v[112] = v[112];
   }
   if( v[47] < v[98] ) {
      v[108] = v[108];
   } else {
      v[108] = v[106];
   }
   if( v[83] < 0 ) {
      v[108] = v[112];
   } else {
      v[108] = v[108];
   }
   if( v[83] < 0 ) {
      v[91] = 0;
   } else {
      v[91] = v[91];
   }
   if( v[47] < v[98] ) {
      v[112] = 0;
   } else {
      v[112] = v[91];
   }
   if( v[83] < 0 ) {
      v[89] = 0;
   } else {
      v[89] = v[89];
   }
   if( v[47] < v[98] ) {
      v[106] = v[89];
   } else {
      v[106] = 0;
   }
   if( v[83] < 0 ) {
      v[93] = v[93];
   } else {
      v[93] = 0;
   }
   if( v[47] > v[66] ) {
      v[87] = 0;
   } else {
      v[87] = v[93];
   }
   if( v[83] < 0 ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[47] > v[66] ) {
      v[111] = v[100];
   } else {
      v[111] = 0;
   }
   v[111] = ((0 - (x[49] * v[114] + v[84] * v[113] + x[51] * v[105] + v[99] * v[108]) * 1 / v[85] * v[88]) * 1 / v[85]) / 2. + v[112] + v[106] + v[87] + v[111];
   if( v[83] < 0 ) {
      v[87] = 0;
   } else {
      v[87] = v[111];
   }
   if( v[47] < v[86] ) {
      v[106] = v[87];
   } else {
      v[106] = 0;
   }
   if( v[47] < v[86] ) {
      v[87] = 0;
   } else {
      v[87] = v[87];
   }
   if( v[83] < 0 ) {
      v[111] = v[111];
   } else {
      v[111] = 0;
   }
   if( v[47] > v[66] ) {
      v[112] = v[111];
   } else {
      v[112] = 0;
   }
   if( v[47] > v[66] ) {
      v[111] = 0;
   } else {
      v[111] = v[111];
   }
   v[99] = 0 - v[106] + v[87] - v[112] + v[111];
   if( v[47] < v[98] ) {
      v[91] = v[91];
   } else {
      v[91] = 0;
   }
   if( v[47] < v[98] ) {
      v[89] = 0;
   } else {
      v[89] = v[89];
   }
   v[89] = v[91] + v[89];
   if( v[47] > v[66] ) {
      v[93] = v[93];
   } else {
      v[93] = 0;
   }
   if( v[47] > v[66] ) {
      v[100] = 0;
   } else {
      v[100] = v[100];
   }
   v[100] = v[93] + v[100];
   v[93] = v[89] + v[100];
   v[91] = v[97] * v[82] + v[99] * v[61] + v[93] * v[34];
   v[84] = v[97] * v[80] + v[99] * v[64] + v[93] * v[43];
   v[109] = v[99] * v[33] + v[93] * v[46];
   v[104] = v[91] * -1 + v[84] * v[36] + v[109] * v[44];
   v[91] = v[91] * 2.06823107110214e-13 + v[84] * v[37] + v[109] * v[40];
   v[107] = v[84] * v[41] + v[109] * v[35];
   v[110] = v[104] * -1 + v[91] * v[18] + v[107] * v[38];
   v[104] = v[104] * 2.06823107110214e-13 + v[91] * v[25] + v[107] * v[28];
   v[115] = v[91] * v[29] + v[107] * v[17];
   v[116] = v[110] * 4.89663865010925e-12 + v[104] * v[19] + v[115] * v[22];
   v[117] = v[116] * v[14];
   v[118] = 0 - v[117];
   v[110] = v[110] * -1 + v[104] * v[20] + v[115] * v[26];
   v[119] = v[104] * v[23] + v[115] * v[19];
   v[120] = v[110] * v[11] + v[119] * v[3];
   if( v[47] > v[66] ) {
      v[96] = 0;
   } else {
      v[96] = v[96];
   }
   if( v[47] < v[92] ) {
      v[94] = 0;
   } else {
      v[94] = v[94];
   }
   v[94] = v[96] + v[94];
   if( v[47] > v[66] ) {
      v[90] = v[90];
   } else {
      v[90] = 0;
   }
   if( v[47] < v[98] ) {
      v[101] = v[101];
   } else {
      v[101] = 0;
   }
   v[101] = v[90] + v[101];
   v[90] = v[94] + v[101];
   v[100] = 0 - v[89] + v[100];
   v[89] = 0 - v[106] + v[87] + v[112] - v[111];
   v[96] = v[90] * v[82] + v[100] * v[61] + v[89] * v[34];
   v[121] = v[90] * v[80] + v[100] * v[64] + v[89] * v[43];
   v[122] = v[100] * v[33] + v[89] * v[46];
   v[123] = v[96] * -1 + v[121] * v[36] + v[122] * v[44];
   v[96] = v[96] * 2.06823107110214e-13 + v[121] * v[37] + v[122] * v[40];
   v[124] = v[121] * v[41] + v[122] * v[35];
   v[125] = v[123] * -1 + v[96] * v[18] + v[124] * v[38];
   v[123] = v[123] * 2.06823107110214e-13 + v[96] * v[25] + v[124] * v[28];
   v[126] = v[96] * v[29] + v[124] * v[17];
   v[127] = v[125] * 4.89663865010925e-12 + v[123] * v[19] + v[126] * v[22];
   v[128] = v[127] * v[14];
   v[125] = v[125] * -1 + v[123] * v[20] + v[126] * v[26];
   v[129] = v[123] * v[23] + v[126] * v[19];
   v[130] = v[125] * v[11] + v[129] * v[3];
   v[131] = v[128] * v[9] + v[130] * v[7];
   v[132] = v[125] * v[4] + v[129] * v[11];
   v[133] = sin(x[9]);
   v[134] = v[117] * v[9] + v[120] * v[7];
   v[135] = v[110] * v[4] + v[119] * v[11];
   v[136] = 0 - v[130];
   v[137] = cos(x[9]);
   jac[0] = 0 - (v[118] * v[7] + v[120] * v[9] + v[131] * v[5] + v[132] * v[1]) * v[133] + (v[134] * v[5] + v[135] * v[1] + v[128] * v[7] + v[136] * v[9]) * v[137];
   v[111] = v[106] + v[87] - v[112] - v[111];
   v[102] = v[95] + v[102];
   v[101] = 0 - v[94] + v[101];
   v[94] = v[111] * v[82] + v[102] * v[61] + v[101] * v[34];
   v[95] = v[111] * v[80] + v[102] * v[64] + v[101] * v[43];
   v[112] = v[102] * v[33] + v[101] * v[46];
   v[87] = v[94] * -1 + v[95] * v[36] + v[112] * v[44];
   v[94] = v[94] * 2.06823107110214e-13 + v[95] * v[37] + v[112] * v[40];
   v[106] = v[95] * v[41] + v[112] * v[35];
   v[138] = v[87] * -1 + v[94] * v[18] + v[106] * v[38];
   v[87] = v[87] * 2.06823107110214e-13 + v[94] * v[25] + v[106] * v[28];
   v[139] = v[94] * v[29] + v[106] * v[17];
   v[140] = v[138] * 4.89663865010925e-12 + v[87] * v[19] + v[139] * v[22];
   v[141] = v[140] * v[14];
   v[138] = v[138] * -1 + v[87] * v[20] + v[139] * v[26];
   v[142] = v[87] * v[23] + v[139] * v[19];
   v[143] = v[138] * v[11] + v[142] * v[3];
   v[144] = sin(x[10]);
   v[145] = cos(x[10]);
   jac[1] = 0 - (v[141] * v[9] + v[143] * v[7] + v[135] * v[8] + v[132] * v[0]) * v[144] + (0 - (v[138] * v[4] + v[142] * v[11]) + v[134] * v[8] + v[131] * v[0]) * v[145];
   v[135] = sin(x[11]);
   v[134] = cos(x[11]);
   jac[2] = 0 - (v[141] * v[1] + v[117] * v[49] + v[120] * v[0] + v[128] * v[6] + v[136] * v[8]) * v[135] + (v[143] * v[1] + v[118] * v[0] + v[120] * v[49] + v[128] * v[8] + v[130] * v[6]) * v[134];
   v[140] = v[140] * v[67] + v[116] * v[52] + v[127] * v[13];
   v[127] = sin(x[18]);
   v[116] = cos(x[18]);
   jac[3] = 0 - (v[138] * v[70] + v[110] * v[50] + v[125] * v[10] + v[142] * v[69] + v[119] * v[48] + v[129] * v[2] + v[140] - v[140] * -1 * -1) * v[127] + (v[142] * v[70] + v[119] * v[50] + v[129] * v[10] - (v[138] * v[69] + v[110] * v[48] + v[125] * v[2])) * -1 * v[116];
   v[140] = sin(x[19]);
   v[142] = cos(x[19]);
   jac[4] = 0 - (v[87] * v[68] + v[104] * v[53] + v[123] * v[15] + v[139] * v[72] + v[115] * v[55] + v[126] * v[21] + (v[87] * v[71] + v[104] * v[51] + v[123] * v[12]) * 4.89663865010925e-12) * v[140] + (v[139] * v[68] + v[115] * v[53] + v[126] * v[15] + (v[139] * v[71] + v[115] * v[51] + v[126] * v[12]) * 4.89663865010925e-12 - (v[87] * v[72] + v[104] * v[55] + v[123] * v[21])) * v[142];
   v[139] = sin(x[20]);
   v[87] = cos(x[20]);
   jac[5] = 0 - (v[106] * v[76] + v[107] * v[57] + v[124] * v[27] + (v[94] * v[74] + v[91] * v[54] + v[96] * v[16]) * -2.06823107110214e-13 + (v[94] * v[73] + v[91] * v[56] + v[96] * v[24]) * -1) * v[139] + ((v[106] * v[74] + v[107] * v[54] + v[124] * v[16]) * -2.06823107110214e-13 + (v[106] * v[73] + v[107] * v[56] + v[124] * v[24]) * -1 - (v[94] * v[76] + v[91] * v[57] + v[96] * v[27])) * v[87];
   v[106] = sin(x[21]);
   v[94] = cos(x[21]);
   jac[6] = 0 - (v[112] * v[78] + v[109] * v[62] + v[122] * v[39] + (v[95] * v[75] + v[84] * v[59] + v[121] * v[31]) * -2.06823107110214e-13 + (v[95] * v[77] + v[84] * v[58] + v[121] * v[30]) * -1) * v[106] + ((v[112] * v[75] + v[109] * v[59] + v[122] * v[31]) * -2.06823107110214e-13 + (v[112] * v[77] + v[109] * v[58] + v[122] * v[30]) * -1 - (v[95] * v[78] + v[84] * v[62] + v[121] * v[39])) * v[94];
   v[111] = v[111] * v[79] + v[97] * v[63] + v[90] * v[42] + (v[111] * v[81] + v[97] * v[60] + v[90] * v[32]) * 4.89663865010925e-12;
   v[90] = sin(x[22]);
   v[97] = cos(x[22]);
   jac[7] = 0 - (v[101] * v[103] + v[93] * v[65] + v[89] * v[45] + (v[102] * v[81] + v[99] * v[60] + v[100] * v[32]) * -1 + (v[102] * v[79] + v[99] * v[63] + v[100] * v[42]) * 4.89663865010925e-12 + v[111] - v[111] * -1 * -1) * v[90] + ((v[101] * v[81] + v[93] * v[60] + v[89] * v[32]) * -1 + (v[101] * v[79] + v[93] * v[63] + v[89] * v[42]) * 4.89663865010925e-12 - (v[102] * v[103] + v[99] * v[65] + v[100] * v[45])) * -1 * v[97];
   v[111] = x[49] * v[88];
   if( v[83] < 0 ) {
      v[101] = v[111];
   } else {
      v[101] = 0;
   }
   if( v[47] > v[66] ) {
      v[102] = 0;
   } else {
      v[102] = v[101];
   }
   v[89] = -1 * x[52];
   v[100] = v[89] * v[88];
   if( v[83] < 0 ) {
      v[93] = 0;
   } else {
      v[93] = v[100];
   }
   if( v[47] < v[92] ) {
      v[99] = v[93];
   } else {
      v[99] = 0;
   }
   v[99] = v[102] + v[99];
   v[102] = x[50] * v[88];
   if( v[83] < 0 ) {
      v[112] = v[102];
   } else {
      v[112] = 0;
   }
   if( v[47] > v[66] ) {
      v[95] = v[112];
   } else {
      v[95] = 0;
   }
   v[122] = -1 * x[51];
   v[121] = v[122] * v[88];
   if( v[83] < 0 ) {
      v[109] = 0;
   } else {
      v[109] = v[121];
   }
   if( v[47] < v[98] ) {
      v[84] = 0;
   } else {
      v[84] = v[109];
   }
   v[84] = v[95] + v[84];
   v[95] = v[99] - v[84];
   if( v[83] < 0 ) {
      v[102] = 0;
   } else {
      v[102] = v[102];
   }
   if( v[47] < v[98] ) {
      v[124] = 0;
   } else {
      v[124] = v[102];
   }
   if( v[83] < 0 ) {
      v[111] = 0;
   } else {
      v[111] = v[111];
   }
   if( v[47] < v[98] ) {
      v[96] = v[111];
   } else {
      v[96] = 0;
   }
   if( v[83] < 0 ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[47] > v[66] ) {
      v[107] = 0;
   } else {
      v[107] = v[100];
   }
   if( v[83] < 0 ) {
      v[121] = v[121];
   } else {
      v[121] = 0;
   }
   if( v[47] > v[66] ) {
      v[91] = v[121];
   } else {
      v[91] = 0;
   }
   v[91] = ((0 - (x[50] * v[114] + x[49] * v[113] + v[89] * v[105] + v[122] * v[108]) * 1 / v[85] * v[88]) * 1 / v[85]) / 2. + v[124] + v[96] + v[107] + v[91];
   if( v[83] < 0 ) {
      v[107] = 0;
   } else {
      v[107] = v[91];
   }
   if( v[47] < v[86] ) {
      v[96] = v[107];
   } else {
      v[96] = 0;
   }
   if( v[47] < v[86] ) {
      v[107] = 0;
   } else {
      v[107] = v[107];
   }
   if( v[83] < 0 ) {
      v[91] = v[91];
   } else {
      v[91] = 0;
   }
   if( v[47] > v[66] ) {
      v[124] = v[91];
   } else {
      v[124] = 0;
   }
   if( v[47] > v[66] ) {
      v[91] = 0;
   } else {
      v[91] = v[91];
   }
   v[122] = 0 - v[96] + v[107] - v[124] + v[91];
   if( v[47] < v[98] ) {
      v[102] = v[102];
   } else {
      v[102] = 0;
   }
   if( v[47] < v[98] ) {
      v[111] = 0;
   } else {
      v[111] = v[111];
   }
   v[111] = v[102] + v[111];
   if( v[47] > v[66] ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[47] > v[66] ) {
      v[121] = 0;
   } else {
      v[121] = v[121];
   }
   v[121] = v[100] + v[121];
   v[100] = v[111] + v[121];
   v[102] = v[95] * v[82] + v[122] * v[61] + v[100] * v[34];
   v[89] = v[95] * v[80] + v[122] * v[64] + v[100] * v[43];
   v[126] = v[122] * v[33] + v[100] * v[46];
   v[123] = v[102] * -1 + v[89] * v[36] + v[126] * v[44];
   v[102] = v[102] * 2.06823107110214e-13 + v[89] * v[37] + v[126] * v[40];
   v[115] = v[89] * v[41] + v[126] * v[35];
   v[104] = v[123] * -1 + v[102] * v[18] + v[115] * v[38];
   v[123] = v[123] * 2.06823107110214e-13 + v[102] * v[25] + v[115] * v[28];
   v[138] = v[102] * v[29] + v[115] * v[17];
   v[129] = v[104] * 4.89663865010925e-12 + v[123] * v[19] + v[138] * v[22];
   v[125] = v[129] * v[14];
   v[119] = 0 - v[125];
   v[104] = v[104] * -1 + v[123] * v[20] + v[138] * v[26];
   v[110] = v[123] * v[23] + v[138] * v[19];
   v[143] = v[104] * v[11] + v[110] * v[3];
   if( v[47] > v[66] ) {
      v[112] = 0;
   } else {
      v[112] = v[112];
   }
   if( v[47] < v[92] ) {
      v[93] = 0;
   } else {
      v[93] = v[93];
   }
   v[93] = v[112] + v[93];
   if( v[47] > v[66] ) {
      v[101] = v[101];
   } else {
      v[101] = 0;
   }
   if( v[47] < v[98] ) {
      v[109] = v[109];
   } else {
      v[109] = 0;
   }
   v[109] = v[101] + v[109];
   v[101] = v[93] + v[109];
   v[121] = 0 - v[111] + v[121];
   v[111] = 0 - v[96] + v[107] + v[124] - v[91];
   v[112] = v[101] * v[82] + v[121] * v[61] + v[111] * v[34];
   v[141] = v[101] * v[80] + v[121] * v[64] + v[111] * v[43];
   v[136] = v[121] * v[33] + v[111] * v[46];
   v[130] = v[112] * -1 + v[141] * v[36] + v[136] * v[44];
   v[112] = v[112] * 2.06823107110214e-13 + v[141] * v[37] + v[136] * v[40];
   v[128] = v[141] * v[41] + v[136] * v[35];
   v[120] = v[130] * -1 + v[112] * v[18] + v[128] * v[38];
   v[130] = v[130] * 2.06823107110214e-13 + v[112] * v[25] + v[128] * v[28];
   v[118] = v[112] * v[29] + v[128] * v[17];
   v[117] = v[120] * 4.89663865010925e-12 + v[130] * v[19] + v[118] * v[22];
   v[132] = v[117] * v[14];
   v[120] = v[120] * -1 + v[130] * v[20] + v[118] * v[26];
   v[131] = v[130] * v[23] + v[118] * v[19];
   v[146] = v[120] * v[11] + v[131] * v[3];
   v[147] = v[132] * v[9] + v[146] * v[7];
   v[148] = v[120] * v[4] + v[131] * v[11];
   v[149] = v[125] * v[9] + v[143] * v[7];
   v[150] = v[104] * v[4] + v[110] * v[11];
   v[151] = 0 - v[146];
   jac[8] = 0 - (v[119] * v[7] + v[143] * v[9] + v[147] * v[5] + v[148] * v[1]) * v[133] + (v[149] * v[5] + v[150] * v[1] + v[132] * v[7] + v[151] * v[9]) * v[137];
   v[91] = v[96] + v[107] - v[124] - v[91];
   v[84] = v[99] + v[84];
   v[109] = 0 - v[93] + v[109];
   v[93] = v[91] * v[82] + v[84] * v[61] + v[109] * v[34];
   v[99] = v[91] * v[80] + v[84] * v[64] + v[109] * v[43];
   v[124] = v[84] * v[33] + v[109] * v[46];
   v[107] = v[93] * -1 + v[99] * v[36] + v[124] * v[44];
   v[93] = v[93] * 2.06823107110214e-13 + v[99] * v[37] + v[124] * v[40];
   v[96] = v[99] * v[41] + v[124] * v[35];
   v[152] = v[107] * -1 + v[93] * v[18] + v[96] * v[38];
   v[107] = v[107] * 2.06823107110214e-13 + v[93] * v[25] + v[96] * v[28];
   v[153] = v[93] * v[29] + v[96] * v[17];
   v[154] = v[152] * 4.89663865010925e-12 + v[107] * v[19] + v[153] * v[22];
   v[155] = v[154] * v[14];
   v[152] = v[152] * -1 + v[107] * v[20] + v[153] * v[26];
   v[156] = v[107] * v[23] + v[153] * v[19];
   v[157] = v[152] * v[11] + v[156] * v[3];
   jac[9] = 0 - (v[155] * v[9] + v[157] * v[7] + v[150] * v[8] + v[148] * v[0]) * v[144] + (0 - (v[152] * v[4] + v[156] * v[11]) + v[149] * v[8] + v[147] * v[0]) * v[145];
   jac[10] = 0 - (v[155] * v[1] + v[125] * v[49] + v[143] * v[0] + v[132] * v[6] + v[151] * v[8]) * v[135] + (v[157] * v[1] + v[119] * v[0] + v[143] * v[49] + v[132] * v[8] + v[146] * v[6]) * v[134];
   v[154] = v[154] * v[67] + v[129] * v[52] + v[117] * v[13];
   jac[11] = 0 - (v[152] * v[70] + v[104] * v[50] + v[120] * v[10] + v[156] * v[69] + v[110] * v[48] + v[131] * v[2] + v[154] - v[154] * -1 * -1) * v[127] + (v[156] * v[70] + v[110] * v[50] + v[131] * v[10] - (v[152] * v[69] + v[104] * v[48] + v[120] * v[2])) * -1 * v[116];
   jac[12] = 0 - (v[107] * v[68] + v[123] * v[53] + v[130] * v[15] + v[153] * v[72] + v[138] * v[55] + v[118] * v[21] + (v[107] * v[71] + v[123] * v[51] + v[130] * v[12]) * 4.89663865010925e-12) * v[140] + (v[153] * v[68] + v[138] * v[53] + v[118] * v[15] + (v[153] * v[71] + v[138] * v[51] + v[118] * v[12]) * 4.89663865010925e-12 - (v[107] * v[72] + v[123] * v[55] + v[130] * v[21])) * v[142];
   jac[13] = 0 - (v[96] * v[76] + v[115] * v[57] + v[128] * v[27] + (v[93] * v[74] + v[102] * v[54] + v[112] * v[16]) * -2.06823107110214e-13 + (v[93] * v[73] + v[102] * v[56] + v[112] * v[24]) * -1) * v[139] + ((v[96] * v[74] + v[115] * v[54] + v[128] * v[16]) * -2.06823107110214e-13 + (v[96] * v[73] + v[115] * v[56] + v[128] * v[24]) * -1 - (v[93] * v[76] + v[102] * v[57] + v[112] * v[27])) * v[87];
   jac[14] = 0 - (v[124] * v[78] + v[126] * v[62] + v[136] * v[39] + (v[99] * v[75] + v[89] * v[59] + v[141] * v[31]) * -2.06823107110214e-13 + (v[99] * v[77] + v[89] * v[58] + v[141] * v[30]) * -1) * v[106] + ((v[124] * v[75] + v[126] * v[59] + v[136] * v[31]) * -2.06823107110214e-13 + (v[124] * v[77] + v[126] * v[58] + v[136] * v[30]) * -1 - (v[99] * v[78] + v[89] * v[62] + v[141] * v[39])) * v[94];
   v[91] = v[91] * v[79] + v[95] * v[63] + v[101] * v[42] + (v[91] * v[81] + v[95] * v[60] + v[101] * v[32]) * 4.89663865010925e-12;
   jac[15] = 0 - (v[109] * v[103] + v[100] * v[65] + v[111] * v[45] + (v[84] * v[81] + v[122] * v[60] + v[121] * v[32]) * -1 + (v[84] * v[79] + v[122] * v[63] + v[121] * v[42]) * 4.89663865010925e-12 + v[91] - v[91] * -1 * -1) * v[90] + ((v[109] * v[81] + v[100] * v[60] + v[111] * v[32]) * -1 + (v[109] * v[79] + v[100] * v[63] + v[111] * v[42]) * 4.89663865010925e-12 - (v[84] * v[103] + v[122] * v[65] + v[121] * v[45])) * -1 * v[97];
   v[91] = -1 * x[52];
   v[109] = v[91] * v[88];
   if( v[83] < 0 ) {
      v[84] = v[109];
   } else {
      v[84] = 0;
   }
   if( v[47] > v[66] ) {
      v[111] = 0;
   } else {
      v[111] = v[84];
   }
   v[121] = -1 * x[49];
   v[100] = v[121] * v[88];
   if( v[83] < 0 ) {
      v[122] = 0;
   } else {
      v[122] = v[100];
   }
   if( v[47] < v[92] ) {
      v[101] = v[122];
   } else {
      v[101] = 0;
   }
   v[101] = v[111] + v[101];
   v[111] = x[51] * v[88];
   if( v[83] < 0 ) {
      v[95] = v[111];
   } else {
      v[95] = 0;
   }
   if( v[47] > v[66] ) {
      v[124] = v[95];
   } else {
      v[124] = 0;
   }
   v[99] = x[50] * v[88];
   if( v[83] < 0 ) {
      v[136] = 0;
   } else {
      v[136] = v[99];
   }
   if( v[47] < v[98] ) {
      v[141] = 0;
   } else {
      v[141] = v[136];
   }
   v[141] = v[124] + v[141];
   v[124] = v[101] - v[141];
   if( v[83] < 0 ) {
      v[111] = 0;
   } else {
      v[111] = v[111];
   }
   if( v[47] < v[98] ) {
      v[126] = 0;
   } else {
      v[126] = v[111];
   }
   if( v[83] < 0 ) {
      v[109] = 0;
   } else {
      v[109] = v[109];
   }
   if( v[47] < v[98] ) {
      v[89] = v[109];
   } else {
      v[89] = 0;
   }
   if( v[83] < 0 ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[47] > v[66] ) {
      v[96] = 0;
   } else {
      v[96] = v[100];
   }
   if( v[83] < 0 ) {
      v[99] = v[99];
   } else {
      v[99] = 0;
   }
   if( v[47] > v[66] ) {
      v[93] = v[99];
   } else {
      v[93] = 0;
   }
   v[93] = ((0 - (x[51] * v[114] + v[91] * v[113] + v[121] * v[105] + x[50] * v[108]) * 1 / v[85] * v[88]) * 1 / v[85]) / 2. + v[126] + v[89] + v[96] + v[93];
   if( v[83] < 0 ) {
      v[96] = 0;
   } else {
      v[96] = v[93];
   }
   if( v[47] < v[86] ) {
      v[89] = v[96];
   } else {
      v[89] = 0;
   }
   if( v[47] < v[86] ) {
      v[96] = 0;
   } else {
      v[96] = v[96];
   }
   if( v[83] < 0 ) {
      v[93] = v[93];
   } else {
      v[93] = 0;
   }
   if( v[47] > v[66] ) {
      v[83] = v[93];
   } else {
      v[83] = 0;
   }
   if( v[47] > v[66] ) {
      v[93] = 0;
   } else {
      v[93] = v[93];
   }
   v[86] = 0 - v[89] + v[96] - v[83] + v[93];
   if( v[47] < v[98] ) {
      v[111] = v[111];
   } else {
      v[111] = 0;
   }
   if( v[47] < v[98] ) {
      v[109] = 0;
   } else {
      v[109] = v[109];
   }
   v[109] = v[111] + v[109];
   if( v[47] > v[66] ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[47] > v[66] ) {
      v[99] = 0;
   } else {
      v[99] = v[99];
   }
   v[99] = v[100] + v[99];
   v[100] = v[109] + v[99];
   v[111] = v[124] * v[82] + v[86] * v[61] + v[100] * v[34];
   v[126] = v[124] * v[80] + v[86] * v[64] + v[100] * v[43];
   v[121] = v[86] * v[33] + v[100] * v[46];
   v[91] = v[111] * -1 + v[126] * v[36] + v[121] * v[44];
   v[111] = v[111] * 2.06823107110214e-13 + v[126] * v[37] + v[121] * v[40];
   v[108] = v[126] * v[41] + v[121] * v[35];
   v[105] = v[91] * -1 + v[111] * v[18] + v[108] * v[38];
   v[91] = v[91] * 2.06823107110214e-13 + v[111] * v[25] + v[108] * v[28];
   v[113] = v[111] * v[29] + v[108] * v[17];
   v[114] = v[105] * 4.89663865010925e-12 + v[91] * v[19] + v[113] * v[22];
   v[88] = v[114] * v[14];
   v[85] = 0 - v[88];
   v[105] = v[105] * -1 + v[91] * v[20] + v[113] * v[26];
   v[128] = v[91] * v[23] + v[113] * v[19];
   v[112] = v[105] * v[11] + v[128] * v[3];
   if( v[47] > v[66] ) {
      v[95] = 0;
   } else {
      v[95] = v[95];
   }
   if( v[47] < v[92] ) {
      v[122] = 0;
   } else {
      v[122] = v[122];
   }
   v[122] = v[95] + v[122];
   if( v[47] > v[66] ) {
      v[84] = v[84];
   } else {
      v[84] = 0;
   }
   if( v[47] < v[98] ) {
      v[136] = v[136];
   } else {
      v[136] = 0;
   }
   v[136] = v[84] + v[136];
   v[84] = v[122] + v[136];
   v[99] = 0 - v[109] + v[99];
   v[109] = 0 - v[89] + v[96] + v[83] - v[93];
   v[98] = v[84] * v[82] + v[99] * v[61] + v[109] * v[34];
   v[47] = v[84] * v[80] + v[99] * v[64] + v[109] * v[43];
   v[66] = v[99] * v[33] + v[109] * v[46];
   v[95] = v[98] * -1 + v[47] * v[36] + v[66] * v[44];
   v[98] = v[98] * 2.06823107110214e-13 + v[47] * v[37] + v[66] * v[40];
   v[92] = v[47] * v[41] + v[66] * v[35];
   v[115] = v[95] * -1 + v[98] * v[18] + v[92] * v[38];
   v[95] = v[95] * 2.06823107110214e-13 + v[98] * v[25] + v[92] * v[28];
   v[102] = v[98] * v[29] + v[92] * v[17];
   v[153] = v[115] * 4.89663865010925e-12 + v[95] * v[19] + v[102] * v[22];
   v[107] = v[153] * v[14];
   v[115] = v[115] * -1 + v[95] * v[20] + v[102] * v[26];
   v[118] = v[95] * v[23] + v[102] * v[19];
   v[130] = v[115] * v[11] + v[118] * v[3];
   v[138] = v[107] * v[9] + v[130] * v[7];
   v[123] = v[115] * v[4] + v[118] * v[11];
   v[154] = v[88] * v[9] + v[112] * v[7];
   v[156] = v[105] * v[4] + v[128] * v[11];
   v[152] = 0 - v[130];
   jac[16] = 0 - (v[85] * v[7] + v[112] * v[9] + v[138] * v[5] + v[123] * v[1]) * v[133] + (v[154] * v[5] + v[156] * v[1] + v[107] * v[7] + v[152] * v[9]) * v[137];
   v[93] = v[89] + v[96] - v[83] - v[93];
   v[141] = v[101] + v[141];
   v[136] = 0 - v[122] + v[136];
   v[82] = v[93] * v[82] + v[141] * v[61] + v[136] * v[34];
   v[80] = v[93] * v[80] + v[141] * v[64] + v[136] * v[43];
   v[33] = v[141] * v[33] + v[136] * v[46];
   v[44] = v[82] * -1 + v[80] * v[36] + v[33] * v[44];
   v[82] = v[82] * 2.06823107110214e-13 + v[80] * v[37] + v[33] * v[40];
   v[41] = v[80] * v[41] + v[33] * v[35];
   v[38] = v[44] * -1 + v[82] * v[18] + v[41] * v[38];
   v[44] = v[44] * 2.06823107110214e-13 + v[82] * v[25] + v[41] * v[28];
   v[29] = v[82] * v[29] + v[41] * v[17];
   v[22] = v[38] * 4.89663865010925e-12 + v[44] * v[19] + v[29] * v[22];
   v[14] = v[22] * v[14];
   v[38] = v[38] * -1 + v[44] * v[20] + v[29] * v[26];
   v[23] = v[44] * v[23] + v[29] * v[19];
   v[3] = v[38] * v[11] + v[23] * v[3];
   jac[17] = 0 - (v[14] * v[9] + v[3] * v[7] + v[156] * v[8] + v[123] * v[0]) * v[144] + (0 - (v[38] * v[4] + v[23] * v[11]) + v[154] * v[8] + v[138] * v[0]) * v[145];
   jac[18] = 0 - (v[14] * v[1] + v[88] * v[49] + v[112] * v[0] + v[107] * v[6] + v[152] * v[8]) * v[135] + (v[3] * v[1] + v[85] * v[0] + v[112] * v[49] + v[107] * v[8] + v[130] * v[6]) * v[134];
   v[22] = v[22] * v[67] + v[114] * v[52] + v[153] * v[13];
   jac[19] = 0 - (v[38] * v[70] + v[105] * v[50] + v[115] * v[10] + v[23] * v[69] + v[128] * v[48] + v[118] * v[2] + v[22] - v[22] * -1 * -1) * v[127] + (v[23] * v[70] + v[128] * v[50] + v[118] * v[10] - (v[38] * v[69] + v[105] * v[48] + v[115] * v[2])) * -1 * v[116];
   jac[20] = 0 - (v[44] * v[68] + v[91] * v[53] + v[95] * v[15] + v[29] * v[72] + v[113] * v[55] + v[102] * v[21] + (v[44] * v[71] + v[91] * v[51] + v[95] * v[12]) * 4.89663865010925e-12) * v[140] + (v[29] * v[68] + v[113] * v[53] + v[102] * v[15] + (v[29] * v[71] + v[113] * v[51] + v[102] * v[12]) * 4.89663865010925e-12 - (v[44] * v[72] + v[91] * v[55] + v[95] * v[21])) * v[142];
   jac[21] = 0 - (v[41] * v[76] + v[108] * v[57] + v[92] * v[27] + (v[82] * v[74] + v[111] * v[54] + v[98] * v[16]) * -2.06823107110214e-13 + (v[82] * v[73] + v[111] * v[56] + v[98] * v[24]) * -1) * v[139] + ((v[41] * v[74] + v[108] * v[54] + v[92] * v[16]) * -2.06823107110214e-13 + (v[41] * v[73] + v[108] * v[56] + v[92] * v[24]) * -1 - (v[82] * v[76] + v[111] * v[57] + v[98] * v[27])) * v[87];
   jac[22] = 0 - (v[33] * v[78] + v[121] * v[62] + v[66] * v[39] + (v[80] * v[75] + v[126] * v[59] + v[47] * v[31]) * -2.06823107110214e-13 + (v[80] * v[77] + v[126] * v[58] + v[47] * v[30]) * -1) * v[106] + ((v[33] * v[75] + v[121] * v[59] + v[66] * v[31]) * -2.06823107110214e-13 + (v[33] * v[77] + v[121] * v[58] + v[66] * v[30]) * -1 - (v[80] * v[78] + v[126] * v[62] + v[47] * v[39])) * v[94];
   v[93] = v[93] * v[79] + v[124] * v[63] + v[84] * v[42] + (v[93] * v[81] + v[124] * v[60] + v[84] * v[32]) * 4.89663865010925e-12;
   jac[23] = 0 - (v[136] * v[103] + v[100] * v[65] + v[109] * v[45] + (v[141] * v[81] + v[86] * v[60] + v[99] * v[32]) * -1 + (v[141] * v[79] + v[86] * v[63] + v[99] * v[42]) * 4.89663865010925e-12 + v[93] - v[93] * -1 * -1) * v[90] + ((v[136] * v[81] + v[100] * v[60] + v[109] * v[32]) * -1 + (v[136] * v[79] + v[100] * v[63] + v[109] * v[42]) * 4.89663865010925e-12 - (v[141] * v[103] + v[86] * v[65] + v[99] * v[45])) * -1 * v[97];
}

