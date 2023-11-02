void contact_1_position_jacobian_sparsity(unsigned long const** row,
                                          unsigned long const** col,
                                          unsigned long* nnz) {
   static unsigned long const rows[26] = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2};
   static unsigned long const cols[26] = {6,9,10,11,12,13,14,15,16,7,9,10,11,12,13,14,15,16,8,10,11,12,13,14,15,16};
   *row = rows;
   *col = cols;
   *nnz = 26;
}
