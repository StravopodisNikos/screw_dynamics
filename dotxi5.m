function dotxi5 = dotxi5(qdot,xi1,xi2,xi3,xi4,xi5)
 dotxi5 = qdot(1)*LieProduct1(xi1,xi5) + qdot(2)*LieProduct1(xi2,xi5) + qdot(3)*LieProduct1(xi3,xi5) + qdot(4)*LieProduct1(xi4,xi5);
dotxi5=dotxi5';
end