FILEIN=simulationResults_OP_HC.txt
PLOT=performance_OP_HC.png
NRSIM=100

### Ploting and data process section
gnuplot << EOF

reset
set terminal pngcairo
set output "$PLOT"
set ylabel 'Reward'
set xlabel 'Duration of tree construction [ms]'
set title 'Simulation result'
set grid

set table "plotedDataHC.dat"
set print "data"
stats  "$FILEIN" using 0 nooutput

do for [i=0:STATS_blocks-1] {
   stats "$FILEIN" using 2:4 index i nooutput
   print sprintf("%e %e %e", STATS_max_x, STATS_mean_y, STATS_stddev_y*1.96/sqrt("$NRSIM"))
}
set autoscale xfix
set offsets 1,1,0,0
plot "data" using 1:2:3 w yerrorbars title "data"
unset table

plot "data" using 1:2:3 w yerrorbars title "data"

EOF

mv data plorData_OP_HC


################
################ ROCK SAMPLE

FILEIN=simulationResults_OP_RS.txt
PLOT=performance_OP_RS.png
### Ploting and data process section
gnuplot << EOF

reset
set terminal pngcairo
set output "$PLOT"
set ylabel 'Reward'
set xlabel 'Duration of tree construction [ms]'
set title 'Simulation result'
set grid

set table "plotedDataRS.dat"
set print "data"
stats  "$FILEIN" using 0 nooutput

do for [i=0:STATS_blocks-1] {
   stats "$FILEIN" using 2:4 index i nooutput
   print sprintf("%e %e %e", STATS_max_x, STATS_mean_y, STATS_stddev_y*1.96/sqrt("$NRSIM"))
}
set autoscale xfix
set offsets 1,1,0,0
plot "data" using 1:2:3 w yerrorbars title "data"
unset table

plot "data" using 1:2:3 w yerrorbars title "data"

EOF

mv data plorData_OP_RS

################
################ AUV

FILEIN=simulationResults_OP_AUV.txt
PLOT=performance_OP_AUV.png

### Ploting and data process section
gnuplot << EOF

reset
set terminal pngcairo
set output "$PLOT"
set ylabel 'Reward'
set xlabel 'Duration of tree construction [ms]'
set title 'Simulation result'
set grid

set table "plotedDataAUV.dat"
set print "data"
stats  "$FILEIN" using 0 nooutput

do for [i=0:STATS_blocks-1] {
   stats "$FILEIN" using 2:4 index i nooutput
   print sprintf("%e %e %e", STATS_max_x, STATS_mean_y, STATS_stddev_y*1.96/sqrt("$NRSIM"))
}
set autoscale xfix
set offsets 1,1,0,0
plot "data" using 1:2:3 w yerrorbars title "data"
unset table

plot "data" using 1:2:3 w yerrorbars title "data"

EOF

mv data plorData_OP_AUV


