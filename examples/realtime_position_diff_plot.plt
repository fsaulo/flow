# Set up the initial plot
set datafile separator ','  # Set the CSV separator
set xlabel "Timestamp"
set ylabel "Values"
set title "Real-time CSV Plot"
set grid

# Specify the columns to plot
ts_col = 1  # Timestamp column
flow_vx_col = 2  # Column for vx
flow_vy_col = 3  # Column for vy
vx_col = 2
vy_col = 3

plot "../build/gcs_optical_flow.csv" using ts_col:flow_vx_col with lines title "flow_{x}", \
     "../build/gcs_optical_flow.csv" using ts_col:flow_vy_col with lines title "flow_{y}", \
     "../build/gcs_local_position.csv" using ts_col:vx_col with lines title "x", \
     "../build/gcs_local_position.csv" using ts_col:vy_col with lines title "y"


pause 1  # Pause for 1 second before updating
reread  # Reread the script to update the plot
