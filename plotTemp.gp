#!/usr/bin/gnuplot
set terminal x11 persist
set autoscale
set title "Temperaturas RTD"
set xlabel "Muestra"
set ylabel "Temp"
plot "/home/sena/uart/temps.dat" with lines
pause 0.1
reread
