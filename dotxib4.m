function dotxib4 = dotxib4(qdot,xib4,xib5,xib6)
 dotxib4 = qdot(5)*LieProduct1(xib4,xib5) + qdot(6)*LieProduct1(xib4,xib6);
 dotxib4=dotxib4';   
end