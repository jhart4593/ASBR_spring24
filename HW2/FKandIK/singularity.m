syms t1 t2 t3 t4 t5 t6 t7 
theta = [0 0 t1 t2 -t2 t3 t4 t5 t6 t7];
symScrews = cellfun(@(S) sym(S), Screws, 'UniformOutput', false);
symBScrews = cellfun(@(S) sym(S), BScrewsSub, 'UniformOutput', false);

JacobianSpace = symScrews{1};
expProd = eye(4);
numJoints = 10; 
tic
for i = 2:numJoints-1
    expProd = simplify(expProd*ScrewAxisDist2MatExp(symScrews{i-1},theta(i-1)));
    JacobianSpace(:,i) = simplify(T2Adj(expProd)*symScrews{i});
    i

end
toc
save('JSpace', 'JacobianSpace')
tic 
sigmaSpace = svd(JacobianSpace(:,1:8));
toc
save('sSpace', 'sigmaSpace')
tic
JacobianBody(:,numJoints) = symBScrews{numJoints};
expProd = eye(4);

for i = numJoints-1:-1:2
    expProd = simplify(expProd*ScrewAxisDist2MatExp(-symBScrews{i+1},theta(i+1)));
    Jacobian(:,i) = simplify(T2Adj(expProd)*symBScrews{i});
    i
end
toc
save('JBody', 'JacobianBody')
tic
sigmaBody = svd(JacobianBody);
toc

save('sBody', 'sigmaBody')