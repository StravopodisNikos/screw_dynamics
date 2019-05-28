function dotxib3 = dotxib3(qdot,xib3,xib4,xib5,xib6)
 dotxib3 = qdot(4)*LieProduct1(xib3,xib4) + qdot(5)*LieProduct1(xib3,xib5) + qdot(6)*LieProduct1(xib3,xib6);
 dotxib3=dotxib3';   
end