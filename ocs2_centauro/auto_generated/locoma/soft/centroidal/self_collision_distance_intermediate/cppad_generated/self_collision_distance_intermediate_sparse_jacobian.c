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

void self_collision_distance_intermediate_sparse_jacobian__1(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);
void self_collision_distance_intermediate_sparse_jacobian__2(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);

void self_collision_distance_intermediate_sparse_jacobian(double const *const * in,
                                                          double*const * out,
                                                          struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[876];
   double array[0];
   double sarray[0];
   unsigned long idx[0];

   self_collision_distance_intermediate_sparse_jacobian__1(in, out, atomicFun, v, array, sarray, idx);
   self_collision_distance_intermediate_sparse_jacobian__2(in, out, atomicFun, v, array, sarray, idx);
   // dependent variables without operations
   jac[0] = 0;
   jac[1] = 0;
   jac[2] = 0;
   jac[13] = 0;
   jac[14] = 0;
   jac[15] = 0;
   jac[28] = 0;
   jac[29] = 0;
   jac[30] = 0;
   jac[42] = 0;
   jac[43] = 0;
   jac[44] = 0;
   jac[58] = 0;
   jac[59] = 0;
   jac[60] = 0;
   jac[75] = 0;
   jac[76] = 0;
   jac[77] = 0;
   jac[88] = 0;
   jac[89] = 0;
   jac[90] = 0;
   jac[103] = 0;
   jac[104] = 0;
   jac[105] = 0;
   jac[117] = 0;
   jac[118] = 0;
   jac[119] = 0;
   jac[133] = 0;
   jac[134] = 0;
   jac[135] = 0;
   jac[145] = 0;
   jac[146] = 0;
   jac[147] = 0;
   jac[159] = 0;
   jac[160] = 0;
   jac[161] = 0;
   jac[172] = 0;
   jac[173] = 0;
   jac[174] = 0;
   jac[187] = 0;
   jac[188] = 0;
   jac[189] = 0;
   jac[198] = 0;
   jac[199] = 0;
   jac[200] = 0;
   jac[211] = 0;
   jac[212] = 0;
   jac[213] = 0;
   jac[225] = 0;
   jac[226] = 0;
   jac[227] = 0;
   jac[235] = 0;
   jac[236] = 0;
   jac[237] = 0;
   jac[247] = 0;
   jac[248] = 0;
   jac[249] = 0;
   jac[260] = 0;
   jac[261] = 0;
   jac[262] = 0;
   jac[273] = 0;
   jac[274] = 0;
   jac[275] = 0;
   jac[286] = 0;
   jac[287] = 0;
   jac[288] = 0;
   jac[298] = 0;
   jac[299] = 0;
   jac[300] = 0;
   jac[311] = 0;
   jac[312] = 0;
   jac[313] = 0;
   jac[326] = 0;
   jac[327] = 0;
   jac[328] = 0;
   jac[340] = 0;
   jac[341] = 0;
   jac[342] = 0;
   jac[356] = 0;
   jac[357] = 0;
   jac[358] = 0;
   jac[368] = 0;
   jac[369] = 0;
   jac[370] = 0;
   jac[382] = 0;
   jac[383] = 0;
   jac[384] = 0;
   jac[395] = 0;
   jac[396] = 0;
   jac[397] = 0;
   jac[410] = 0;
   jac[411] = 0;
   jac[412] = 0;
   jac[421] = 0;
   jac[422] = 0;
   jac[423] = 0;
   jac[434] = 0;
   jac[435] = 0;
   jac[436] = 0;
   jac[448] = 0;
   jac[449] = 0;
   jac[450] = 0;
   jac[458] = 0;
   jac[459] = 0;
   jac[460] = 0;
   jac[470] = 0;
   jac[471] = 0;
   jac[472] = 0;
   jac[483] = 0;
   jac[484] = 0;
   jac[485] = 0;
   jac[496] = 0;
   jac[497] = 0;
   jac[498] = 0;
   jac[508] = 0;
   jac[509] = 0;
   jac[510] = 0;
   jac[521] = 0;
   jac[522] = 0;
   jac[523] = 0;
   jac[536] = 0;
   jac[537] = 0;
   jac[538] = 0;
   jac[550] = 0;
   jac[551] = 0;
   jac[552] = 0;
   jac[566] = 0;
   jac[567] = 0;
   jac[568] = 0;
   jac[583] = 0;
   jac[584] = 0;
   jac[585] = 0;
   jac[596] = 0;
   jac[597] = 0;
   jac[598] = 0;
}

