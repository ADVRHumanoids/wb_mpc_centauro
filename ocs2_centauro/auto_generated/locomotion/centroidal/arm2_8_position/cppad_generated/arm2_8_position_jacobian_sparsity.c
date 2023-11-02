void arm2_8_position_jacobian_sparsity(unsigned long const** row,
                                       unsigned long const** col,
                                       unsigned long* nnz) {
   static unsigned long const rows[32] = {0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[32] = {6,9,10,11,36,43,44,45,46,47,48,7,9,10,11,36,43,44,45,46,47,48,8,10,11,36,43,44,45,46,47,48};
   *row = rows;
   *col = cols;
   *nnz = 32;
}
