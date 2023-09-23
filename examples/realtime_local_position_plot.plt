# Set up the initial plot
set datafile separator ','  # Set the CSV separator
set xlabel "Timestamp"
set ylabel "Values"
set title "Real-time CSV Plot"
set grid

# Specify the columns to plot
ts_col = 1  # Timestamp column
x_col = 2
y_col = 3
z_col = 4
vx_col = 5  # Column for vx
vy_col = 6  # Column for vy

plot "../build/gcs_local_position.csv" using ts_col:vx_col with lines title "vx", \
     "../build/gcs_local_position.csv" using ts_col:y_col with lines title "vy"

pause 1  # Pause for 1 second before updating
reread  # Reread the script to update the plot
