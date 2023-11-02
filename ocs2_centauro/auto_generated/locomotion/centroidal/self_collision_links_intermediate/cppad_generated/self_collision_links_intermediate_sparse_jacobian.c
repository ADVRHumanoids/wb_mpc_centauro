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

void self_collision_links_intermediate_sparse_jacobian__1(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);
void self_collision_links_intermediate_sparse_jacobian__2(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);

void self_collision_links_intermediate_sparse_jacobian(double const *const * in,
                                                       double*const * out,
                                                       struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[944];
   double array[0];
   double sarray[0];
   unsigned long idx[0];

   self_collision_links_intermediate_sparse_jacobian__1(in, out, atomicFun, v, array, sarray, idx);
   self_collision_links_intermediate_sparse_jacobian__2(in, out, atomicFun, v, array, sarray, idx);
}

