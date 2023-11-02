void arm1_8_orientation_jacobian_sparsity(unsigned long const** row,
                                          unsigned long const** col,
                                          unsigned long* nnz) {
   static unsigned long const rows[30] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[30] = {9,10,11,36,37,38,39,40,41,42,9,10,11,36,37,38,39,40,41,42,9,10,11,36,37,38,39,40,41,42};
   *row = rows;
   *col = cols;
   *nnz = 30;
}
