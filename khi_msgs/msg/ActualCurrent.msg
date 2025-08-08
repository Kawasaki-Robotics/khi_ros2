# Example
#   >ros2 topic echo /khi_controller0/khi_publisher/actual_current
#   actual_current:
#   - 0.07919596135616302
#   - 0.029698487371206284
#   - -0.010960155166685581
#   - 0.0036416000220924616
#   - -0.0036416000220924616
#   - 0.0036416000220924616
#
#   Note: The '0' in khi_controller0 changes according to controller_no specified in khi_bringup.launch.py.
#
#   Note: To publish the actual current value, you need to specify `actual_current:=true` when running `khi_bringup.launch.py`.

# Actual current values are stored.
float32[] actual_current
