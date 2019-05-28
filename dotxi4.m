function dotxi4 = dotxi4(qdot,xi1,xi2,xi3,xi4)
 dotxi4 = qdot(1)*LieProduct1(xi1,xi4) + qdot(2)*LieProduct1(xi2,xi4) + qdot(3)*LieProduct1(xi3,xi4);
 dotxi4=dotxi4';
end