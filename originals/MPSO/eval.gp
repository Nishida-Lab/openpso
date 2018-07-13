set terminal x11
set xrange [0:628]
set yrange [0:1e-2]
plot "./e_mpso.dat" u 1:2 w l 
pause -1 "Hit return to save e_mpso.obj" 
set terminal tgif 
set output "e_mpso.obj" 
plot "./e_mpso.dat" u 1:2 w l 


