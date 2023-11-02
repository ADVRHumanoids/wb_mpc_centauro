void contact_4_absolute_orientation_jacobian_sparsity(unsigned long const** row,
                                                      unsigned long const** col,
                                                      unsigned long* nnz) {
   static unsigned long const rows[32] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3};
   static unsigned long const cols[32] = {9,10,11,30,31,32,33,34,9,10,11,30,31,32,33,34,9,10,11,30,31,32,33,34,9,10,11,30,31,32,33,34};
   *row = rows;
   *col = cols;
   *nnz = 32;
}
