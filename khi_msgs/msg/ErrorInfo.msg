# Example
#   >ros2 topic echo /khi_controller0/khi_publisher/error_info
#   time: '2024-06-26 18:13:49'
#   error_codes:
#   - -31135
#   - -21080
#   error_msgs:
#   - Motor power OFF
#   - Encoder battery low voltage.[Battery backup board 1
#
#   Note: The '0' in khi_controller0 changes according to controller_no specified in khi_bringup.launch.py.

# The date and time of the error occurence (year-month-day hour:minute:second) will be stored.
string time

# -1xxxx : Operation Error
# -2xxxx : Mechanical/Control Warning
# -3xxxx : Error
# -4xxxx : Fatal Error
int32[] error_codes

# The error messages corresponding to the error codes are stored.
# Please refer to the troubleshooting manual for details.
string[] error_msgs
