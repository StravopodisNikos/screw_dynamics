function dotxib5 = dotxib5(qdot,xib5,xib6)
 dotxib5 = qdot(6)*LieProduct1(xib5,xib6);
 dotxib5=dotxib5';   
end