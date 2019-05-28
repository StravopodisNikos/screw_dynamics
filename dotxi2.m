function dotxi2 = dotxi2(qdot,xi1,xi2)
 dotxi2 = qdot(1)*LieProduct1(xi1,xi2);
 dotxi2 = dotxi2';
end