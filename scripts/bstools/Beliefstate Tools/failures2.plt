set terminal pdf
set output "prediction_effects_robot.pdf"

set datafile separator ","

set boxwidth 1.0
set style fill solid
#set style histogram cluster gap 1
set style data histogram
set style fill solid border -1
set xtic scale 0

#set key Left left reverse
set key off

set xlabel " "#Amount of Experiment Trials per Evaluation or Model Training"
set ylabel "Nominal Occurrence"

Before = "#000000";
After = "#ff0000";

set title "Effect of Failure Predictions based on Episodic Memories"

set xrange [-0.5:2.5]
set yrange [0:101]

set palette model RGB defined (0 'grey', 1 'red', 2 'green')

plot "failures2.csv" using 2:xtic(1) title "Without Prediction"
