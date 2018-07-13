set terminal x11
set xrange [0:628]
set yrange [0:1e-2]
plot "./e_dapso.dat" u 1:2 w l 
pause -1 "Hit return to save s_e_dapso.obj" 
set terminal tgif 
set output "s_e_dapso.obj" 
plot "./e_dapso.dat" u 1:2 w l 


