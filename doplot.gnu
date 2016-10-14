#!/usr/local/bin/gnuplot
#
reset
set terminal png size 1024,768
set output 'audio.png'

# define axis
# remove border on top and right and set color to gray
set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror
# define grid
set style line 12 lc rgb '#808080' lt 0 lw 1
set grid back ls 12

# color definitions
set style line 1 lc rgb 'red' pt 1 ps 0.5 lt 1 lw 3 # --- red

set key top right

set title 'Audio'
set xlabel 'time'
set ylabel 'Ampl'

plot 'plot.dat' u 1:2 t 'Input' w lp ls 1
