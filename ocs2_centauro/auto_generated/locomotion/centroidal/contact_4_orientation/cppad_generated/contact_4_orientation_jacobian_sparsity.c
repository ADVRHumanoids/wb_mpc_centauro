void contact_4_orientation_jacobian_sparsity(unsigned long const** row,
                                             unsigned long const** col,
                                             unsigned long* nnz) {
   static unsigned long const rows[24] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2};
   static unsigned long const cols[24] = {9,10,11,30,31,32,33,34,9,10,11,30,31,32,33,34,9,10,11,30,31,32,33,34};
   *row = rows;
   *col = cols;
   *nnz = 24;
}
