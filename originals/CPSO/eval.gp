set terminal x11
set xrange [0:628]
#set yrange [0:0.05]
plot "./e_cpso.dat" u 1:2 w l 
pause -1 "Hit return to save mse.obj" 
set terminal tgif 
set output "e_cpso.obj" 
plot "./e_cpso.dat" u 1:2 w l 


