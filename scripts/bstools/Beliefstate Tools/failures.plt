set terminal pdf
set output "prediction_effects.pdf"

set datafile separator ","

set boxwidth 0.9
set style fill solid
set style histogram cluster gap 1
set style data histogram
set style fill solid border -1
set xtic scale 0

set key Left left reverse

set xlabel "Amount of Experiment Trials per Evaluation or Model Training"
set ylabel "Nominal Error Occurrence"

Before = "#000000";
After = "#ff0000";

set title "Effect of Failure Predictions based on Episodic Memories"

set xrange [0.5:10.5]
set yrange [0:65]

plot "failures.csv" using 2:xtic(1) title "Without Prediction", \
     "" u 3 title "Prediction based on Episodic Memories"
