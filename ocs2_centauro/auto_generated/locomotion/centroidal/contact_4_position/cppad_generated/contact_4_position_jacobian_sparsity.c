void contact_4_position_jacobian_sparsity(unsigned long const** row,
                                          unsigned long const** col,
                                          unsigned long* nnz) {
   static unsigned long const rows[26] = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2};
   static unsigned long const cols[26] = {6,9,10,11,30,31,32,33,34,7,9,10,11,30,31,32,33,34,8,10,11,30,31,32,33,34};
   *row = rows;
   *col = cols;
   *nnz = 26;
}
