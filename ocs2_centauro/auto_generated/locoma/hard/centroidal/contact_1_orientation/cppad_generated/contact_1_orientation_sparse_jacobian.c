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

void contact_1_orientation_sparse_jacobian(double const *const * in,
                                           double*const * out,
                                           struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[173];

   v[0] = cos(x[9]);
   v[1] = cos(x[10]);
   v[2] = v[0] * v[1];
   v[3] = -1 * sin(x[12]);
   v[4] = 0 - v[3];
   v[5] = sin(x[10]);
   v[6] = v[0] * v[5];
   v[7] = sin(x[11]);
   v[8] = sin(x[9]);
   v[9] = cos(x[11]);
   v[10] = v[6] * v[7] - v[8] * v[9];
   v[11] = cos(x[12]);
   v[12] = v[2] * v[4] + v[10] * v[11];
   v[13] = cos(x[13]);
   v[14] = -1 * -1 * (1 - v[13]) + v[13];
   v[15] = -1 * v[14];
   v[16] = v[6] * v[9] + v[8] * v[7];
   v[17] = -1 * -1 * (1 - v[11]) + v[11];
   v[18] = v[16] * v[17];
   v[14] = 4.89663865010925e-12 * v[14];
   v[19] = v[12] * v[15] + v[18] * v[14];
   v[20] = cos(x[14]);
   v[21] = -2.06823107110214e-13 * v[20];
   v[22] = 4.89663865010925e-12 * v[13];
   v[23] = v[2] * v[11] + v[10] * v[3];
   v[24] = -1 * sin(x[13]);
   v[25] = 0 - v[24];
   v[26] = v[18] * v[13] + v[12] * v[22] + v[23] * v[25];
   v[27] = -1 * v[20];
   v[28] = 4.89663865010925e-12 * v[24];
   v[29] = v[18] * v[24] + v[12] * v[28] + v[23] * v[13];
   v[30] = -1 * sin(x[14]);
   v[31] = 0 - v[30];
   v[32] = v[19] * v[21] + v[26] * v[27] + v[29] * v[31];
   v[33] = cos(x[15]);
   v[34] = -1 * -1 * (1 - v[33]) + v[33];
   v[35] = 2.06823107110214e-13 * v[34];
   v[36] = -1 * -1 * (1 - v[20]) + v[20];
   v[37] = 2.06823107110214e-13 * v[36];
   v[36] = -1 * v[36];
   v[38] = v[26] * v[37] + v[19] * v[36];
   v[34] = -1 * v[34];
   v[39] = v[32] * v[35] + v[38] * v[34];
   v[40] = -1 * sin(x[16]);
   v[41] = -1 * v[40];
   v[42] = -2.06823107110214e-13 * v[33];
   v[43] = -1 * v[33];
   v[44] = -2.06823107110214e-13 * v[30];
   v[30] = -1 * v[30];
   v[45] = v[19] * v[44] + v[26] * v[30] + v[29] * v[20];
   v[46] = -1 * sin(x[15]);
   v[47] = 0 - v[46];
   v[48] = v[38] * v[42] + v[32] * v[43] + v[45] * v[47];
   v[49] = 4.89663865010925e-12 * v[40];
   v[50] = -2.06823107110214e-13 * v[46];
   v[46] = -1 * v[46];
   v[51] = v[38] * v[50] + v[32] * v[46] + v[45] * v[33];
   v[52] = cos(x[16]);
   v[53] = v[39] * v[41] + v[48] * v[49] + v[51] * v[52];
   v[54] = v[8] * v[1];
   v[55] = v[8] * v[5];
   v[56] = v[55] * v[7] + v[0] * v[9];
   v[57] = v[54] * v[4] + v[56] * v[11];
   v[58] = v[55] * v[9] - v[0] * v[7];
   v[59] = v[58] * v[17];
   v[60] = v[57] * v[15] + v[59] * v[14];
   v[61] = v[54] * v[11] + v[56] * v[3];
   v[62] = v[59] * v[13] + v[57] * v[22] + v[61] * v[25];
   v[63] = v[59] * v[24] + v[57] * v[28] + v[61] * v[13];
   v[64] = v[60] * v[21] + v[62] * v[27] + v[63] * v[31];
   v[65] = v[62] * v[37] + v[60] * v[36];
   v[66] = v[64] * v[35] + v[65] * v[34];
   v[67] = -1 * v[52];
   v[68] = v[60] * v[44] + v[62] * v[30] + v[63] * v[20];
   v[69] = v[65] * v[42] + v[64] * v[43] + v[68] * v[47];
   v[70] = 4.89663865010925e-12 * v[52];
   v[71] = v[65] * v[50] + v[64] * v[46] + v[68] * v[33];
   v[40] = 0 - v[40];
   v[72] = v[66] * v[67] + v[69] * v[70] + v[71] * v[40];
   v[73] = v[1] * v[9];
   v[74] = v[73] * v[17];
   v[75] = 0 - v[5];
   v[76] = v[1] * v[7];
   v[77] = v[75] * v[4] + v[76] * v[11];
   v[78] = v[75] * v[11] + v[76] * v[3];
   v[79] = v[74] * v[13] + v[77] * v[22] + v[78] * v[25];
   v[80] = v[77] * v[15] + v[74] * v[14];
   v[81] = v[79] * v[37] + v[80] * v[36];
   v[82] = v[74] * v[24] + v[77] * v[28] + v[78] * v[13];
   v[83] = v[80] * v[21] + v[79] * v[27] + v[82] * v[31];
   v[84] = v[80] * v[44] + v[79] * v[30] + v[82] * v[20];
   v[85] = v[81] * v[42] + v[83] * v[43] + v[84] * v[47];
   v[86] = -1 * -1 * (1 - v[52]) + v[52];
   v[87] = v[83] * v[35] + v[81] * v[34];
   v[88] = 4.89663865010925e-12 * v[86];
   v[89] = v[85] * v[86] + v[87] * v[88];
   v[90] = -1 * x[50];
   if( v[53] > v[72] ) {
      v[91] = 1 + v[53] - v[72] - v[89];
   } else {
      v[91] = 1 + v[72] - v[53] - v[89];
   }
   v[92] = 0 - v[72];
   if( v[53] < v[92] ) {
      v[93] = 1 + v[89] - v[53] - v[72];
   } else {
      v[93] = 1 + v[53] + v[72] + v[89];
   }
   if( v[89] < 0 ) {
      v[93] = v[91];
   } else {
      v[93] = v[93];
   }
   v[91] = sqrt(v[93]);
   v[94] = 0.5 / v[91];
   v[95] = v[90] * v[94];
   if( v[89] < 0 ) {
      v[96] = v[95];
   } else {
      v[96] = 0;
   }
   if( v[53] > v[72] ) {
      v[97] = 0;
   } else {
      v[97] = v[96];
   }
   v[98] = 0 - v[72];
   v[99] = x[51] * v[94];
   if( v[89] < 0 ) {
      v[100] = 0;
   } else {
      v[100] = v[99];
   }
   if( v[53] < v[98] ) {
      v[101] = v[100];
   } else {
      v[101] = 0;
   }
   v[101] = v[97] + v[101];
   v[97] = x[49] * v[94];
   if( v[89] < 0 ) {
      v[102] = v[97];
   } else {
      v[102] = 0;
   }
   if( v[53] > v[72] ) {
      v[103] = v[102];
   } else {
      v[103] = 0;
   }
   v[104] = 0 - v[72];
   v[105] = -1 * x[52];
   v[106] = v[105] * v[94];
   if( v[89] < 0 ) {
      v[107] = 0;
   } else {
      v[107] = v[106];
   }
   if( v[53] < v[104] ) {
      v[108] = 0;
   } else {
      v[108] = v[107];
   }
   v[108] = v[103] + v[108];
   v[103] = v[101] - v[108];
   v[109] = v[81] * v[50] + v[83] * v[46] + v[84] * v[33];
   v[110] = v[87] * v[67] + v[85] * v[70] + v[109] * v[40];
   v[111] = v[69] * v[86] + v[66] * v[88];
   v[112] = v[110] - v[111];
   v[113] = v[48] * v[86] + v[39] * v[88];
   v[114] = v[87] * v[41] + v[85] * v[49] + v[109] * v[52];
   v[115] = v[113] - v[114];
   if( v[53] > v[72] ) {
      v[116] = v[112];
   } else {
      v[116] = v[115];
   }
   v[117] = v[66] * v[41] + v[69] * v[49] + v[71] * v[52];
   v[118] = v[39] * v[67] + v[48] * v[70] + v[51] * v[40];
   v[119] = v[117] - v[118];
   if( v[53] < v[104] ) {
      v[120] = v[119];
   } else {
      v[120] = v[93];
   }
   if( v[89] < 0 ) {
      v[120] = v[116];
   } else {
      v[120] = v[120];
   }
   v[114] = v[113] + v[114];
   v[111] = v[110] + v[111];
   if( v[53] > v[72] ) {
      v[110] = v[114];
   } else {
      v[110] = v[111];
   }
   if( v[53] < v[104] ) {
      v[119] = v[93];
   } else {
      v[119] = v[119];
   }
   if( v[89] < 0 ) {
      v[119] = v[110];
   } else {
      v[119] = v[119];
   }
   v[118] = v[117] + v[118];
   if( v[53] > v[72] ) {
      v[117] = v[118];
   } else {
      v[117] = v[93];
   }
   if( v[53] < v[98] ) {
      v[111] = v[111];
   } else {
      v[111] = v[115];
   }
   if( v[89] < 0 ) {
      v[111] = v[117];
   } else {
      v[111] = v[111];
   }
   if( v[53] > v[72] ) {
      v[118] = v[93];
   } else {
      v[118] = v[118];
   }
   if( v[53] < v[104] ) {
      v[114] = v[114];
   } else {
      v[114] = v[112];
   }
   if( v[89] < 0 ) {
      v[114] = v[118];
   } else {
      v[114] = v[114];
   }
   if( v[89] < 0 ) {
      v[97] = 0;
   } else {
      v[97] = v[97];
   }
   if( v[53] < v[104] ) {
      v[118] = 0;
   } else {
      v[118] = v[97];
   }
   if( v[89] < 0 ) {
      v[95] = 0;
   } else {
      v[95] = v[95];
   }
   if( v[53] < v[104] ) {
      v[112] = v[95];
   } else {
      v[112] = 0;
   }
   if( v[89] < 0 ) {
      v[99] = v[99];
   } else {
      v[99] = 0;
   }
   if( v[53] > v[72] ) {
      v[93] = 0;
   } else {
      v[93] = v[99];
   }
   if( v[89] < 0 ) {
      v[106] = v[106];
   } else {
      v[106] = 0;
   }
   if( v[53] > v[72] ) {
      v[117] = v[106];
   } else {
      v[117] = 0;
   }
   v[117] = ((0 - (x[49] * v[120] + v[90] * v[119] + x[51] * v[111] + v[105] * v[114]) * 1 / v[91] * v[94]) * 1 / v[91]) / 2. + v[118] + v[112] + v[93] + v[117];
   if( v[89] < 0 ) {
      v[93] = 0;
   } else {
      v[93] = v[117];
   }
   if( v[53] < v[92] ) {
      v[112] = v[93];
   } else {
      v[112] = 0;
   }
   if( v[53] < v[92] ) {
      v[93] = 0;
   } else {
      v[93] = v[93];
   }
   if( v[89] < 0 ) {
      v[117] = v[117];
   } else {
      v[117] = 0;
   }
   if( v[53] > v[72] ) {
      v[118] = v[117];
   } else {
      v[118] = 0;
   }
   if( v[53] > v[72] ) {
      v[117] = 0;
   } else {
      v[117] = v[117];
   }
   v[105] = 0 - v[112] + v[93] - v[118] + v[117];
   if( v[53] < v[104] ) {
      v[97] = v[97];
   } else {
      v[97] = 0;
   }
   if( v[53] < v[104] ) {
      v[95] = 0;
   } else {
      v[95] = v[95];
   }
   v[95] = v[97] + v[95];
   if( v[53] > v[72] ) {
      v[99] = v[99];
   } else {
      v[99] = 0;
   }
   if( v[53] > v[72] ) {
      v[106] = 0;
   } else {
      v[106] = v[106];
   }
   v[106] = v[99] + v[106];
   v[99] = v[95] + v[106];
   v[97] = v[103] * v[88] + v[105] * v[67] + v[99] * v[41];
   v[90] = v[103] * v[86] + v[105] * v[70] + v[99] * v[49];
   v[115] = v[105] * v[40] + v[99] * v[52];
   v[110] = v[97] * v[34] + v[90] * v[42] + v[115] * v[50];
   v[113] = v[97] * v[35] + v[90] * v[43] + v[115] * v[46];
   v[116] = v[90] * v[47] + v[115] * v[33];
   v[121] = v[110] * v[36] + v[113] * v[21] + v[116] * v[44];
   v[122] = v[110] * v[37] + v[113] * v[27] + v[116] * v[30];
   v[123] = v[113] * v[31] + v[116] * v[20];
   v[124] = v[121] * v[14] + v[122] * v[13] + v[123] * v[24];
   v[125] = v[124] * v[17];
   v[126] = 0 - v[125];
   v[127] = v[121] * v[15] + v[122] * v[22] + v[123] * v[28];
   v[128] = v[122] * v[25] + v[123] * v[13];
   v[129] = v[127] * v[11] + v[128] * v[3];
   if( v[53] > v[72] ) {
      v[102] = 0;
   } else {
      v[102] = v[102];
   }
   if( v[53] < v[98] ) {
      v[100] = 0;
   } else {
      v[100] = v[100];
   }
   v[100] = v[102] + v[100];
   if( v[53] > v[72] ) {
      v[96] = v[96];
   } else {
      v[96] = 0;
   }
   if( v[53] < v[104] ) {
      v[107] = v[107];
   } else {
      v[107] = 0;
   }
   v[107] = v[96] + v[107];
   v[96] = v[100] + v[107];
   v[106] = 0 - v[95] + v[106];
   v[95] = 0 - v[112] + v[93] + v[118] - v[117];
   v[102] = v[96] * v[88] + v[106] * v[67] + v[95] * v[41];
   v[130] = v[96] * v[86] + v[106] * v[70] + v[95] * v[49];
   v[131] = v[106] * v[40] + v[95] * v[52];
   v[132] = v[102] * v[34] + v[130] * v[42] + v[131] * v[50];
   v[133] = v[102] * v[35] + v[130] * v[43] + v[131] * v[46];
   v[134] = v[130] * v[47] + v[131] * v[33];
   v[135] = v[132] * v[36] + v[133] * v[21] + v[134] * v[44];
   v[136] = v[132] * v[37] + v[133] * v[27] + v[134] * v[30];
   v[137] = v[133] * v[31] + v[134] * v[20];
   v[138] = v[135] * v[14] + v[136] * v[13] + v[137] * v[24];
   v[139] = v[138] * v[17];
   v[140] = v[135] * v[15] + v[136] * v[22] + v[137] * v[28];
   v[141] = v[136] * v[25] + v[137] * v[13];
   v[142] = v[140] * v[11] + v[141] * v[3];
   v[143] = v[139] * v[9] + v[142] * v[7];
   v[144] = v[140] * v[4] + v[141] * v[11];
   v[145] = sin(x[9]);
   v[146] = v[125] * v[9] + v[129] * v[7];
   v[147] = v[127] * v[4] + v[128] * v[11];
   v[148] = 0 - v[142];
   v[149] = cos(x[9]);
   jac[0] = 0 - (v[126] * v[7] + v[129] * v[9] + v[143] * v[5] + v[144] * v[1]) * v[145] + (v[146] * v[5] + v[147] * v[1] + v[139] * v[7] + v[148] * v[9]) * v[149];
   v[117] = v[112] + v[93] - v[118] - v[117];
   v[108] = v[101] + v[108];
   v[107] = 0 - v[100] + v[107];
   v[100] = v[117] * v[88] + v[108] * v[67] + v[107] * v[41];
   v[101] = v[117] * v[86] + v[108] * v[70] + v[107] * v[49];
   v[118] = v[108] * v[40] + v[107] * v[52];
   v[93] = v[100] * v[34] + v[101] * v[42] + v[118] * v[50];
   v[112] = v[100] * v[35] + v[101] * v[43] + v[118] * v[46];
   v[150] = v[101] * v[47] + v[118] * v[33];
   v[151] = v[93] * v[36] + v[112] * v[21] + v[150] * v[44];
   v[152] = v[93] * v[37] + v[112] * v[27] + v[150] * v[30];
   v[153] = v[112] * v[31] + v[150] * v[20];
   v[154] = v[151] * v[14] + v[152] * v[13] + v[153] * v[24];
   v[155] = v[154] * v[17];
   v[156] = v[151] * v[15] + v[152] * v[22] + v[153] * v[28];
   v[157] = v[152] * v[25] + v[153] * v[13];
   v[158] = v[156] * v[11] + v[157] * v[3];
   v[159] = sin(x[10]);
   v[160] = cos(x[10]);
   jac[1] = 0 - (v[155] * v[9] + v[158] * v[7] + v[147] * v[8] + v[144] * v[0]) * v[159] + (0 - (v[156] * v[4] + v[157] * v[11]) + v[146] * v[8] + v[143] * v[0]) * v[160];
   v[147] = sin(x[11]);
   v[146] = cos(x[11]);
   jac[2] = 0 - (v[155] * v[1] + v[125] * v[55] + v[129] * v[0] + v[139] * v[6] + v[148] * v[8]) * v[147] + (v[158] * v[1] + v[126] * v[0] + v[129] * v[55] + v[139] * v[8] + v[142] * v[6]) * v[146];
   v[154] = v[154] * v[73] + v[124] * v[58] + v[138] * v[16];
   v[138] = sin(x[12]);
   v[124] = cos(x[12]);
   jac[3] = 0 - (v[156] * v[76] + v[127] * v[56] + v[140] * v[10] + v[157] * v[75] + v[128] * v[54] + v[141] * v[2] + v[154] - v[154] * -1 * -1) * v[138] + (v[157] * v[76] + v[128] * v[56] + v[141] * v[10] - (v[156] * v[75] + v[127] * v[54] + v[140] * v[2])) * -1 * v[124];
   v[151] = (v[151] * v[74] + v[121] * v[59] + v[135] * v[18]) * 4.89663865010925e-12 + (v[151] * v[77] + v[121] * v[57] + v[135] * v[12]) * -1;
   v[135] = sin(x[13]);
   v[121] = cos(x[13]);
   jac[4] = 0 - (v[152] * v[74] + v[122] * v[59] + v[136] * v[18] + v[153] * v[78] + v[123] * v[61] + v[137] * v[23] + (v[152] * v[77] + v[122] * v[57] + v[136] * v[12]) * 4.89663865010925e-12 + v[151] - v[151] * -1 * -1) * v[135] + (v[153] * v[74] + v[123] * v[59] + v[137] * v[18] + (v[153] * v[77] + v[123] * v[57] + v[137] * v[12]) * 4.89663865010925e-12 - (v[152] * v[78] + v[122] * v[61] + v[136] * v[23])) * -1 * v[121];
   v[93] = (v[93] * v[80] + v[110] * v[60] + v[132] * v[19]) * -1 + (v[93] * v[79] + v[110] * v[62] + v[132] * v[26]) * 2.06823107110214e-13;
   v[132] = sin(x[14]);
   v[110] = cos(x[14]);
   jac[5] = 0 - (v[150] * v[82] + v[116] * v[63] + v[134] * v[29] + (v[112] * v[80] + v[113] * v[60] + v[133] * v[19]) * -2.06823107110214e-13 + (v[112] * v[79] + v[113] * v[62] + v[133] * v[26]) * -1 + v[93] - v[93] * -1 * -1) * v[132] + ((v[150] * v[80] + v[116] * v[60] + v[134] * v[19]) * -2.06823107110214e-13 + (v[150] * v[79] + v[116] * v[62] + v[134] * v[26]) * -1 - (v[112] * v[82] + v[113] * v[63] + v[133] * v[29])) * -1 * v[110];
   v[100] = (v[100] * v[81] + v[97] * v[65] + v[102] * v[38]) * -1 + (v[100] * v[83] + v[97] * v[64] + v[102] * v[32]) * 2.06823107110214e-13;
   v[102] = sin(x[15]);
   v[97] = cos(x[15]);
   jac[6] = 0 - (v[118] * v[84] + v[115] * v[68] + v[131] * v[45] + (v[101] * v[81] + v[90] * v[65] + v[130] * v[38]) * -2.06823107110214e-13 + (v[101] * v[83] + v[90] * v[64] + v[130] * v[32]) * -1 + v[100] - v[100] * -1 * -1) * v[102] + ((v[118] * v[81] + v[115] * v[65] + v[131] * v[38]) * -2.06823107110214e-13 + (v[118] * v[83] + v[115] * v[64] + v[131] * v[32]) * -1 - (v[101] * v[84] + v[90] * v[68] + v[130] * v[45])) * -1 * v[97];
   v[117] = v[117] * v[85] + v[103] * v[69] + v[96] * v[48] + (v[117] * v[87] + v[103] * v[66] + v[96] * v[39]) * 4.89663865010925e-12;
   v[96] = sin(x[16]);
   v[103] = cos(x[16]);
   jac[7] = 0 - (v[107] * v[109] + v[99] * v[71] + v[95] * v[51] + (v[108] * v[87] + v[105] * v[66] + v[106] * v[39]) * -1 + (v[108] * v[85] + v[105] * v[69] + v[106] * v[48]) * 4.89663865010925e-12 + v[117] - v[117] * -1 * -1) * v[96] + ((v[107] * v[87] + v[99] * v[66] + v[95] * v[39]) * -1 + (v[107] * v[85] + v[99] * v[69] + v[95] * v[48]) * 4.89663865010925e-12 - (v[108] * v[109] + v[105] * v[71] + v[106] * v[51])) * -1 * v[103];
   v[117] = x[49] * v[94];
   if( v[89] < 0 ) {
      v[107] = v[117];
   } else {
      v[107] = 0;
   }
   if( v[53] > v[72] ) {
      v[108] = 0;
   } else {
      v[108] = v[107];
   }
   v[95] = -1 * x[52];
   v[106] = v[95] * v[94];
   if( v[89] < 0 ) {
      v[99] = 0;
   } else {
      v[99] = v[106];
   }
   if( v[53] < v[98] ) {
      v[105] = v[99];
   } else {
      v[105] = 0;
   }
   v[105] = v[108] + v[105];
   v[108] = x[50] * v[94];
   if( v[89] < 0 ) {
      v[100] = v[108];
   } else {
      v[100] = 0;
   }
   if( v[53] > v[72] ) {
      v[118] = v[100];
   } else {
      v[118] = 0;
   }
   v[101] = -1 * x[51];
   v[131] = v[101] * v[94];
   if( v[89] < 0 ) {
      v[130] = 0;
   } else {
      v[130] = v[131];
   }
   if( v[53] < v[104] ) {
      v[115] = 0;
   } else {
      v[115] = v[130];
   }
   v[115] = v[118] + v[115];
   v[118] = v[105] - v[115];
   if( v[89] < 0 ) {
      v[108] = 0;
   } else {
      v[108] = v[108];
   }
   if( v[53] < v[104] ) {
      v[90] = 0;
   } else {
      v[90] = v[108];
   }
   if( v[89] < 0 ) {
      v[117] = 0;
   } else {
      v[117] = v[117];
   }
   if( v[53] < v[104] ) {
      v[93] = v[117];
   } else {
      v[93] = 0;
   }
   if( v[89] < 0 ) {
      v[106] = v[106];
   } else {
      v[106] = 0;
   }
   if( v[53] > v[72] ) {
      v[150] = 0;
   } else {
      v[150] = v[106];
   }
   if( v[89] < 0 ) {
      v[131] = v[131];
   } else {
      v[131] = 0;
   }
   if( v[53] > v[72] ) {
      v[112] = v[131];
   } else {
      v[112] = 0;
   }
   v[112] = ((0 - (x[50] * v[120] + x[49] * v[119] + v[95] * v[111] + v[101] * v[114]) * 1 / v[91] * v[94]) * 1 / v[91]) / 2. + v[90] + v[93] + v[150] + v[112];
   if( v[89] < 0 ) {
      v[150] = 0;
   } else {
      v[150] = v[112];
   }
   if( v[53] < v[92] ) {
      v[93] = v[150];
   } else {
      v[93] = 0;
   }
   if( v[53] < v[92] ) {
      v[150] = 0;
   } else {
      v[150] = v[150];
   }
   if( v[89] < 0 ) {
      v[112] = v[112];
   } else {
      v[112] = 0;
   }
   if( v[53] > v[72] ) {
      v[90] = v[112];
   } else {
      v[90] = 0;
   }
   if( v[53] > v[72] ) {
      v[112] = 0;
   } else {
      v[112] = v[112];
   }
   v[101] = 0 - v[93] + v[150] - v[90] + v[112];
   if( v[53] < v[104] ) {
      v[108] = v[108];
   } else {
      v[108] = 0;
   }
   if( v[53] < v[104] ) {
      v[117] = 0;
   } else {
      v[117] = v[117];
   }
   v[117] = v[108] + v[117];
   if( v[53] > v[72] ) {
      v[106] = v[106];
   } else {
      v[106] = 0;
   }
   if( v[53] > v[72] ) {
      v[131] = 0;
   } else {
      v[131] = v[131];
   }
   v[131] = v[106] + v[131];
   v[106] = v[117] + v[131];
   v[108] = v[118] * v[88] + v[101] * v[67] + v[106] * v[41];
   v[95] = v[118] * v[86] + v[101] * v[70] + v[106] * v[49];
   v[134] = v[101] * v[40] + v[106] * v[52];
   v[133] = v[108] * v[34] + v[95] * v[42] + v[134] * v[50];
   v[116] = v[108] * v[35] + v[95] * v[43] + v[134] * v[46];
   v[113] = v[95] * v[47] + v[134] * v[33];
   v[151] = v[133] * v[36] + v[116] * v[21] + v[113] * v[44];
   v[153] = v[133] * v[37] + v[116] * v[27] + v[113] * v[30];
   v[152] = v[116] * v[31] + v[113] * v[20];
   v[137] = v[151] * v[14] + v[153] * v[13] + v[152] * v[24];
   v[136] = v[137] * v[17];
   v[123] = 0 - v[136];
   v[122] = v[151] * v[15] + v[153] * v[22] + v[152] * v[28];
   v[154] = v[153] * v[25] + v[152] * v[13];
   v[157] = v[122] * v[11] + v[154] * v[3];
   if( v[53] > v[72] ) {
      v[100] = 0;
   } else {
      v[100] = v[100];
   }
   if( v[53] < v[98] ) {
      v[99] = 0;
   } else {
      v[99] = v[99];
   }
   v[99] = v[100] + v[99];
   if( v[53] > v[72] ) {
      v[107] = v[107];
   } else {
      v[107] = 0;
   }
   if( v[53] < v[104] ) {
      v[130] = v[130];
   } else {
      v[130] = 0;
   }
   v[130] = v[107] + v[130];
   v[107] = v[99] + v[130];
   v[131] = 0 - v[117] + v[131];
   v[117] = 0 - v[93] + v[150] + v[90] - v[112];
   v[100] = v[107] * v[88] + v[131] * v[67] + v[117] * v[41];
   v[156] = v[107] * v[86] + v[131] * v[70] + v[117] * v[49];
   v[141] = v[131] * v[40] + v[117] * v[52];
   v[140] = v[100] * v[34] + v[156] * v[42] + v[141] * v[50];
   v[128] = v[100] * v[35] + v[156] * v[43] + v[141] * v[46];
   v[127] = v[156] * v[47] + v[141] * v[33];
   v[158] = v[140] * v[36] + v[128] * v[21] + v[127] * v[44];
   v[155] = v[140] * v[37] + v[128] * v[27] + v[127] * v[30];
   v[148] = v[128] * v[31] + v[127] * v[20];
   v[142] = v[158] * v[14] + v[155] * v[13] + v[148] * v[24];
   v[139] = v[142] * v[17];
   v[129] = v[158] * v[15] + v[155] * v[22] + v[148] * v[28];
   v[126] = v[155] * v[25] + v[148] * v[13];
   v[125] = v[129] * v[11] + v[126] * v[3];
   v[144] = v[139] * v[9] + v[125] * v[7];
   v[143] = v[129] * v[4] + v[126] * v[11];
   v[161] = v[136] * v[9] + v[157] * v[7];
   v[162] = v[122] * v[4] + v[154] * v[11];
   v[163] = 0 - v[125];
   jac[8] = 0 - (v[123] * v[7] + v[157] * v[9] + v[144] * v[5] + v[143] * v[1]) * v[145] + (v[161] * v[5] + v[162] * v[1] + v[139] * v[7] + v[163] * v[9]) * v[149];
   v[112] = v[93] + v[150] - v[90] - v[112];
   v[115] = v[105] + v[115];
   v[130] = 0 - v[99] + v[130];
   v[99] = v[112] * v[88] + v[115] * v[67] + v[130] * v[41];
   v[105] = v[112] * v[86] + v[115] * v[70] + v[130] * v[49];
   v[90] = v[115] * v[40] + v[130] * v[52];
   v[150] = v[99] * v[34] + v[105] * v[42] + v[90] * v[50];
   v[93] = v[99] * v[35] + v[105] * v[43] + v[90] * v[46];
   v[164] = v[105] * v[47] + v[90] * v[33];
   v[165] = v[150] * v[36] + v[93] * v[21] + v[164] * v[44];
   v[166] = v[150] * v[37] + v[93] * v[27] + v[164] * v[30];
   v[167] = v[93] * v[31] + v[164] * v[20];
   v[168] = v[165] * v[14] + v[166] * v[13] + v[167] * v[24];
   v[169] = v[168] * v[17];
   v[170] = v[165] * v[15] + v[166] * v[22] + v[167] * v[28];
   v[171] = v[166] * v[25] + v[167] * v[13];
   v[172] = v[170] * v[11] + v[171] * v[3];
   jac[9] = 0 - (v[169] * v[9] + v[172] * v[7] + v[162] * v[8] + v[143] * v[0]) * v[159] + (0 - (v[170] * v[4] + v[171] * v[11]) + v[161] * v[8] + v[144] * v[0]) * v[160];
   jac[10] = 0 - (v[169] * v[1] + v[136] * v[55] + v[157] * v[0] + v[139] * v[6] + v[163] * v[8]) * v[147] + (v[172] * v[1] + v[123] * v[0] + v[157] * v[55] + v[139] * v[8] + v[125] * v[6]) * v[146];
   v[168] = v[168] * v[73] + v[137] * v[58] + v[142] * v[16];
   jac[11] = 0 - (v[170] * v[76] + v[122] * v[56] + v[129] * v[10] + v[171] * v[75] + v[154] * v[54] + v[126] * v[2] + v[168] - v[168] * -1 * -1) * v[138] + (v[171] * v[76] + v[154] * v[56] + v[126] * v[10] - (v[170] * v[75] + v[122] * v[54] + v[129] * v[2])) * -1 * v[124];
   v[165] = (v[165] * v[74] + v[151] * v[59] + v[158] * v[18]) * 4.89663865010925e-12 + (v[165] * v[77] + v[151] * v[57] + v[158] * v[12]) * -1;
   jac[12] = 0 - (v[166] * v[74] + v[153] * v[59] + v[155] * v[18] + v[167] * v[78] + v[152] * v[61] + v[148] * v[23] + (v[166] * v[77] + v[153] * v[57] + v[155] * v[12]) * 4.89663865010925e-12 + v[165] - v[165] * -1 * -1) * v[135] + (v[167] * v[74] + v[152] * v[59] + v[148] * v[18] + (v[167] * v[77] + v[152] * v[57] + v[148] * v[12]) * 4.89663865010925e-12 - (v[166] * v[78] + v[153] * v[61] + v[155] * v[23])) * -1 * v[121];
   v[150] = (v[150] * v[80] + v[133] * v[60] + v[140] * v[19]) * -1 + (v[150] * v[79] + v[133] * v[62] + v[140] * v[26]) * 2.06823107110214e-13;
   jac[13] = 0 - (v[164] * v[82] + v[113] * v[63] + v[127] * v[29] + (v[93] * v[80] + v[116] * v[60] + v[128] * v[19]) * -2.06823107110214e-13 + (v[93] * v[79] + v[116] * v[62] + v[128] * v[26]) * -1 + v[150] - v[150] * -1 * -1) * v[132] + ((v[164] * v[80] + v[113] * v[60] + v[127] * v[19]) * -2.06823107110214e-13 + (v[164] * v[79] + v[113] * v[62] + v[127] * v[26]) * -1 - (v[93] * v[82] + v[116] * v[63] + v[128] * v[29])) * -1 * v[110];
   v[99] = (v[99] * v[81] + v[108] * v[65] + v[100] * v[38]) * -1 + (v[99] * v[83] + v[108] * v[64] + v[100] * v[32]) * 2.06823107110214e-13;
   jac[14] = 0 - (v[90] * v[84] + v[134] * v[68] + v[141] * v[45] + (v[105] * v[81] + v[95] * v[65] + v[156] * v[38]) * -2.06823107110214e-13 + (v[105] * v[83] + v[95] * v[64] + v[156] * v[32]) * -1 + v[99] - v[99] * -1 * -1) * v[102] + ((v[90] * v[81] + v[134] * v[65] + v[141] * v[38]) * -2.06823107110214e-13 + (v[90] * v[83] + v[134] * v[64] + v[141] * v[32]) * -1 - (v[105] * v[84] + v[95] * v[68] + v[156] * v[45])) * -1 * v[97];
   v[112] = v[112] * v[85] + v[118] * v[69] + v[107] * v[48] + (v[112] * v[87] + v[118] * v[66] + v[107] * v[39]) * 4.89663865010925e-12;
   jac[15] = 0 - (v[130] * v[109] + v[106] * v[71] + v[117] * v[51] + (v[115] * v[87] + v[101] * v[66] + v[131] * v[39]) * -1 + (v[115] * v[85] + v[101] * v[69] + v[131] * v[48]) * 4.89663865010925e-12 + v[112] - v[112] * -1 * -1) * v[96] + ((v[130] * v[87] + v[106] * v[66] + v[117] * v[39]) * -1 + (v[130] * v[85] + v[106] * v[69] + v[117] * v[48]) * 4.89663865010925e-12 - (v[115] * v[109] + v[101] * v[71] + v[131] * v[51])) * -1 * v[103];
   v[112] = -1 * x[52];
   v[130] = v[112] * v[94];
   if( v[89] < 0 ) {
      v[115] = v[130];
   } else {
      v[115] = 0;
   }
   if( v[53] > v[72] ) {
      v[117] = 0;
   } else {
      v[117] = v[115];
   }
   v[131] = -1 * x[49];
   v[106] = v[131] * v[94];
   if( v[89] < 0 ) {
      v[101] = 0;
   } else {
      v[101] = v[106];
   }
   if( v[53] < v[98] ) {
      v[107] = v[101];
   } else {
      v[107] = 0;
   }
   v[107] = v[117] + v[107];
   v[117] = x[51] * v[94];
   if( v[89] < 0 ) {
      v[118] = v[117];
   } else {
      v[118] = 0;
   }
   if( v[53] > v[72] ) {
      v[99] = v[118];
   } else {
      v[99] = 0;
   }
   v[90] = x[50] * v[94];
   if( v[89] < 0 ) {
      v[105] = 0;
   } else {
      v[105] = v[90];
   }
   if( v[53] < v[104] ) {
      v[141] = 0;
   } else {
      v[141] = v[105];
   }
   v[141] = v[99] + v[141];
   v[99] = v[107] - v[141];
   if( v[89] < 0 ) {
      v[117] = 0;
   } else {
      v[117] = v[117];
   }
   if( v[53] < v[104] ) {
      v[156] = 0;
   } else {
      v[156] = v[117];
   }
   if( v[89] < 0 ) {
      v[130] = 0;
   } else {
      v[130] = v[130];
   }
   if( v[53] < v[104] ) {
      v[134] = v[130];
   } else {
      v[134] = 0;
   }
   if( v[89] < 0 ) {
      v[106] = v[106];
   } else {
      v[106] = 0;
   }
   if( v[53] > v[72] ) {
      v[95] = 0;
   } else {
      v[95] = v[106];
   }
   if( v[89] < 0 ) {
      v[90] = v[90];
   } else {
      v[90] = 0;
   }
   if( v[53] > v[72] ) {
      v[100] = v[90];
   } else {
      v[100] = 0;
   }
   v[100] = ((0 - (x[51] * v[120] + v[112] * v[119] + v[131] * v[111] + x[50] * v[114]) * 1 / v[91] * v[94]) * 1 / v[91]) / 2. + v[156] + v[134] + v[95] + v[100];
   if( v[89] < 0 ) {
      v[95] = 0;
   } else {
      v[95] = v[100];
   }
   if( v[53] < v[92] ) {
      v[134] = v[95];
   } else {
      v[134] = 0;
   }
   if( v[53] < v[92] ) {
      v[95] = 0;
   } else {
      v[95] = v[95];
   }
   if( v[89] < 0 ) {
      v[100] = v[100];
   } else {
      v[100] = 0;
   }
   if( v[53] > v[72] ) {
      v[89] = v[100];
   } else {
      v[89] = 0;
   }
   if( v[53] > v[72] ) {
      v[100] = 0;
   } else {
      v[100] = v[100];
   }
   v[92] = 0 - v[134] + v[95] - v[89] + v[100];
   if( v[53] < v[104] ) {
      v[117] = v[117];
   } else {
      v[117] = 0;
   }
   if( v[53] < v[104] ) {
      v[130] = 0;
   } else {
      v[130] = v[130];
   }
   v[130] = v[117] + v[130];
   if( v[53] > v[72] ) {
      v[106] = v[106];
   } else {
      v[106] = 0;
   }
   if( v[53] > v[72] ) {
      v[90] = 0;
   } else {
      v[90] = v[90];
   }
   v[90] = v[106] + v[90];
   v[106] = v[130] + v[90];
   v[117] = v[99] * v[88] + v[92] * v[67] + v[106] * v[41];
   v[156] = v[99] * v[86] + v[92] * v[70] + v[106] * v[49];
   v[131] = v[92] * v[40] + v[106] * v[52];
   v[112] = v[117] * v[34] + v[156] * v[42] + v[131] * v[50];
   v[114] = v[117] * v[35] + v[156] * v[43] + v[131] * v[46];
   v[111] = v[156] * v[47] + v[131] * v[33];
   v[119] = v[112] * v[36] + v[114] * v[21] + v[111] * v[44];
   v[120] = v[112] * v[37] + v[114] * v[27] + v[111] * v[30];
   v[94] = v[114] * v[31] + v[111] * v[20];
   v[91] = v[119] * v[14] + v[120] * v[13] + v[94] * v[24];
   v[108] = v[91] * v[17];
   v[150] = 0 - v[108];
   v[164] = v[119] * v[15] + v[120] * v[22] + v[94] * v[28];
   v[93] = v[120] * v[25] + v[94] * v[13];
   v[127] = v[164] * v[11] + v[93] * v[3];
   if( v[53] > v[72] ) {
      v[118] = 0;
   } else {
      v[118] = v[118];
   }
   if( v[53] < v[98] ) {
      v[101] = 0;
   } else {
      v[101] = v[101];
   }
   v[101] = v[118] + v[101];
   if( v[53] > v[72] ) {
      v[115] = v[115];
   } else {
      v[115] = 0;
   }
   if( v[53] < v[104] ) {
      v[105] = v[105];
   } else {
      v[105] = 0;
   }
   v[105] = v[115] + v[105];
   v[115] = v[101] + v[105];
   v[90] = 0 - v[130] + v[90];
   v[130] = 0 - v[134] + v[95] + v[89] - v[100];
   v[104] = v[115] * v[88] + v[90] * v[67] + v[130] * v[41];
   v[53] = v[115] * v[86] + v[90] * v[70] + v[130] * v[49];
   v[72] = v[90] * v[40] + v[130] * v[52];
   v[118] = v[104] * v[34] + v[53] * v[42] + v[72] * v[50];
   v[98] = v[104] * v[35] + v[53] * v[43] + v[72] * v[46];
   v[128] = v[53] * v[47] + v[72] * v[33];
   v[113] = v[118] * v[36] + v[98] * v[21] + v[128] * v[44];
   v[116] = v[118] * v[37] + v[98] * v[27] + v[128] * v[30];
   v[140] = v[98] * v[31] + v[128] * v[20];
   v[133] = v[113] * v[14] + v[116] * v[13] + v[140] * v[24];
   v[165] = v[133] * v[17];
   v[167] = v[113] * v[15] + v[116] * v[22] + v[140] * v[28];
   v[166] = v[116] * v[25] + v[140] * v[13];
   v[148] = v[167] * v[11] + v[166] * v[3];
   v[155] = v[165] * v[9] + v[148] * v[7];
   v[152] = v[167] * v[4] + v[166] * v[11];
   v[153] = v[108] * v[9] + v[127] * v[7];
   v[158] = v[164] * v[4] + v[93] * v[11];
   v[151] = 0 - v[148];
   jac[16] = 0 - (v[150] * v[7] + v[127] * v[9] + v[155] * v[5] + v[152] * v[1]) * v[145] + (v[153] * v[5] + v[158] * v[1] + v[165] * v[7] + v[151] * v[9]) * v[149];
   v[100] = v[134] + v[95] - v[89] - v[100];
   v[141] = v[107] + v[141];
   v[105] = 0 - v[101] + v[105];
   v[88] = v[100] * v[88] + v[141] * v[67] + v[105] * v[41];
   v[86] = v[100] * v[86] + v[141] * v[70] + v[105] * v[49];
   v[40] = v[141] * v[40] + v[105] * v[52];
   v[50] = v[88] * v[34] + v[86] * v[42] + v[40] * v[50];
   v[46] = v[88] * v[35] + v[86] * v[43] + v[40] * v[46];
   v[47] = v[86] * v[47] + v[40] * v[33];
   v[44] = v[50] * v[36] + v[46] * v[21] + v[47] * v[44];
   v[30] = v[50] * v[37] + v[46] * v[27] + v[47] * v[30];
   v[31] = v[46] * v[31] + v[47] * v[20];
   v[24] = v[44] * v[14] + v[30] * v[13] + v[31] * v[24];
   v[17] = v[24] * v[17];
   v[28] = v[44] * v[15] + v[30] * v[22] + v[31] * v[28];
   v[25] = v[30] * v[25] + v[31] * v[13];
   v[3] = v[28] * v[11] + v[25] * v[3];
   jac[17] = 0 - (v[17] * v[9] + v[3] * v[7] + v[158] * v[8] + v[152] * v[0]) * v[159] + (0 - (v[28] * v[4] + v[25] * v[11]) + v[153] * v[8] + v[155] * v[0]) * v[160];
   jac[18] = 0 - (v[17] * v[1] + v[108] * v[55] + v[127] * v[0] + v[165] * v[6] + v[151] * v[8]) * v[147] + (v[3] * v[1] + v[150] * v[0] + v[127] * v[55] + v[165] * v[8] + v[148] * v[6]) * v[146];
   v[24] = v[24] * v[73] + v[91] * v[58] + v[133] * v[16];
   jac[19] = 0 - (v[28] * v[76] + v[164] * v[56] + v[167] * v[10] + v[25] * v[75] + v[93] * v[54] + v[166] * v[2] + v[24] - v[24] * -1 * -1) * v[138] + (v[25] * v[76] + v[93] * v[56] + v[166] * v[10] - (v[28] * v[75] + v[164] * v[54] + v[167] * v[2])) * -1 * v[124];
   v[44] = (v[44] * v[74] + v[119] * v[59] + v[113] * v[18]) * 4.89663865010925e-12 + (v[44] * v[77] + v[119] * v[57] + v[113] * v[12]) * -1;
   jac[20] = 0 - (v[30] * v[74] + v[120] * v[59] + v[116] * v[18] + v[31] * v[78] + v[94] * v[61] + v[140] * v[23] + (v[30] * v[77] + v[120] * v[57] + v[116] * v[12]) * 4.89663865010925e-12 + v[44] - v[44] * -1 * -1) * v[135] + (v[31] * v[74] + v[94] * v[59] + v[140] * v[18] + (v[31] * v[77] + v[94] * v[57] + v[140] * v[12]) * 4.89663865010925e-12 - (v[30] * v[78] + v[120] * v[61] + v[116] * v[23])) * -1 * v[121];
   v[50] = (v[50] * v[80] + v[112] * v[60] + v[118] * v[19]) * -1 + (v[50] * v[79] + v[112] * v[62] + v[118] * v[26]) * 2.06823107110214e-13;
   jac[21] = 0 - (v[47] * v[82] + v[111] * v[63] + v[128] * v[29] + (v[46] * v[80] + v[114] * v[60] + v[98] * v[19]) * -2.06823107110214e-13 + (v[46] * v[79] + v[114] * v[62] + v[98] * v[26]) * -1 + v[50] - v[50] * -1 * -1) * v[132] + ((v[47] * v[80] + v[111] * v[60] + v[128] * v[19]) * -2.06823107110214e-13 + (v[47] * v[79] + v[111] * v[62] + v[128] * v[26]) * -1 - (v[46] * v[82] + v[114] * v[63] + v[98] * v[29])) * -1 * v[110];
   v[88] = (v[88] * v[81] + v[117] * v[65] + v[104] * v[38]) * -1 + (v[88] * v[83] + v[117] * v[64] + v[104] * v[32]) * 2.06823107110214e-13;
   jac[22] = 0 - (v[40] * v[84] + v[131] * v[68] + v[72] * v[45] + (v[86] * v[81] + v[156] * v[65] + v[53] * v[38]) * -2.06823107110214e-13 + (v[86] * v[83] + v[156] * v[64] + v[53] * v[32]) * -1 + v[88] - v[88] * -1 * -1) * v[102] + ((v[40] * v[81] + v[131] * v[65] + v[72] * v[38]) * -2.06823107110214e-13 + (v[40] * v[83] + v[131] * v[64] + v[72] * v[32]) * -1 - (v[86] * v[84] + v[156] * v[68] + v[53] * v[45])) * -1 * v[97];
   v[100] = v[100] * v[85] + v[99] * v[69] + v[115] * v[48] + (v[100] * v[87] + v[99] * v[66] + v[115] * v[39]) * 4.89663865010925e-12;
   jac[23] = 0 - (v[105] * v[109] + v[106] * v[71] + v[130] * v[51] + (v[141] * v[87] + v[92] * v[66] + v[90] * v[39]) * -1 + (v[141] * v[85] + v[92] * v[69] + v[90] * v[48]) * 4.89663865010925e-12 + v[100] - v[100] * -1 * -1) * v[96] + ((v[105] * v[87] + v[106] * v[66] + v[130] * v[39]) * -1 + (v[105] * v[85] + v[106] * v[69] + v[130] * v[48]) * 4.89663865010925e-12 - (v[141] * v[109] + v[92] * v[71] + v[90] * v[51])) * -1 * v[103];
}

