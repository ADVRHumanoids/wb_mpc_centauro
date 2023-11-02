void contact_1_orientation_jacobian_sparsity(unsigned long const** row,
                                             unsigned long const** col,
                                             unsigned long* nnz) {
   static unsigned long const rows[24] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2};
   static unsigned long const cols[24] = {9,10,11,12,13,14,15,16,9,10,11,12,13,14,15,16,9,10,11,12,13,14,15,16};
   *row = rows;
   *col = cols;
   *nnz = 24;
}
