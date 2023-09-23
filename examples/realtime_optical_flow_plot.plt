# Set up the initial plot
set datafile separator ','  # Set the CSV separator
set xlabel "Timestamp"
set ylabel "Values"
set title "Real-time CSV Plot"
set grid

# Specify the columns to plot
ts_col = 1  # Timestamp column
vx_col = 2  # Column for vx
vy_col = 3  # Column for vy
xgyro_col = 4  # Column for xgyro
ygyro_col = 5  # Column for ygyro
zgyro_col = 6  # Column for zgyro

plot "../build/gcs_optical_flow.csv" using ts_col:vx_col with lines title "vx", \
     "../build/gcs_optical_flow.csv" using ts_col:vy_col with lines title "vy"

pause 1  # Pause for 1 second before updating
reread  # Reread the script to update the plot
