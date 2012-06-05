

nA = size(A,1);
nB = size(B,1);

alpha = 500;
r = nB/nA;

W_ij = pdist2(A,B).^2;


clip(C_ij

cost = sum(vec(C_ij.*W_ij)) + opts.deficient_row_penalty*sum(r-sum(C_ij,2)) +opts.deficient_col_penalty*sum(1-sum(C_ij,1));


