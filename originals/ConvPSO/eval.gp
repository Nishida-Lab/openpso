set terminal x11
set xrange [0:628]
#set yrange [0:0.05]
plot "./e_convpso.dat" u 1:2 w l 
pause -1 "Hit return to save e_convpso.obj" 
set terminal tgif 
set output "e_convpso.obj" 
plot "./e_convpso.dat" u 1:2 w l 


