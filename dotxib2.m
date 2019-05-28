function dotxib2 = dotxib2(qdot,xib2,xib3,xib4,xib5,xib6)
 dotxib2 = qdot(3)*LieProduct1(xib2,xib3) + qdot(4)*LieProduct1(xib2,xib4) + qdot(5)*LieProduct1(xib2,xib5) + qdot(6)*LieProduct1(xib2,xib6);
 dotxib2=dotxib2';   
end