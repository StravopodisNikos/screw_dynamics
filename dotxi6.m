function dotxi6 = dotxi6(qdot,xi1,xi2,xi3,xi4,xi5,xi6)
 dotxi6 = qdot(1)*LieProduct1(xi1,xi6) + qdot(2)*LieProduct1(xi2,xi6) + qdot(3)*LieProduct1(xi3,xi6) + qdot(4)*LieProduct1(xi4,xi6) + qdot(5)*LieProduct1(xi5,xi6);
 dotxi6=dotxi6';   
end