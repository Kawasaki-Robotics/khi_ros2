## Services

They can only be executed when the ROS2 node is Active or Inactive.  
[lifecycle](../khi_hardware/docs/lifecycle.md)

For details, please refer to the files in the srv directory.

## Publishers

They are only published when the ROS2 node is Active or Inactive.  
[lifecycle](../khi_hardware/docs/lifecycle.md)

### Error Publisher
Published only when an error occurs.
For details, please refer to ErrorInfo.msg.

### Actual Current Value Publisher
Published at regular intervals (2ms intervals for F controllers).
For details, please refer to ActualCurrent.msg.
