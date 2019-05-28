function dotxib1 = dotxib1(qdot,xib1,xib2,xib3,xib4,xib5,xib6)
 dotxib1 = qdot(2)*LieProduct1(xib1,xib2) + qdot(3)*LieProduct1(xib1,xib3) + qdot(4)*LieProduct1(xib1,xib4) + qdot(5)*LieProduct1(xib1,xib5) + qdot(6)*LieProduct1(xib1,xib6);
 dotxib1=dotxib1';   
end