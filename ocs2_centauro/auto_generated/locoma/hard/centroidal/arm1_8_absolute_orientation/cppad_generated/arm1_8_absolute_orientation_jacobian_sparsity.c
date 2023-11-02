void arm1_8_absolute_orientation_jacobian_sparsity(unsigned long const** row,
                                                   unsigned long const** col,
                                                   unsigned long* nnz) {
   static unsigned long const rows[40] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3};
   static unsigned long const cols[40] = {9,10,11,36,37,38,39,40,41,42,9,10,11,36,37,38,39,40,41,42,9,10,11,36,37,38,39,40,41,42,9,10,11,36,37,38,39,40,41,42};
   *row = rows;
   *col = cols;
   *nnz = 40;
}
