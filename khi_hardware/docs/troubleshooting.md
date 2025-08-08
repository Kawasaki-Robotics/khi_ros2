## Troubleshooting

### Error: Unable to reset error.
Refer to the [Troubleshooting Manual] for resolution.

### Error: AS ERROR controller_no:XX arm_no:XX error_code:XX
Error occurred during the real-time control.  
Please refer to the [AS Language Reference Manual] to resolve the error.

### Error: RTC SWITCH turned OFF controller_no:XX arm_no:XX
Real-time control of the robot controller turned OFF. It needs to restart to control again.

### Error: Program 'rb_rtcl' is not running.
Please perform one of the following actions.  
(a) After stopping real-time control with Ctrl+C, restart real-time control.  
(b) Transition to Activate using SetHardwareComponentState Service.
```
ros2 control set_hardware_component_state <robot_name> active
```

### Error: Communication with the robot controller has been lost.
Please follow the steps below to recover:  
1. Transition the state of the robot from `unconfigured` to `inactive`:
```
ros2 control set_hardware_component_state <robot_name> inactive
```
2. Transition the state of the `khi_controller` from `active` to `inactive` and then back to `active`:
```
ros2 control switch_controllers --deactivate khi_controller  
ros2 control switch_controllers --activate khi_controller
```
3. Transition the state of the robot to `active`:
```
ros2 control set_hardware_component_state <robot_name> active
```

### Error: XX returned -0xXXXX
API %s of KRNX returned the error code -0x%X. (e.g. `krnx_Open returned -0x%X`)  
Please refer to the KRNX Error section to resolve the error.

### Error: A commanded position exceeding the speed limit was sent. JTXX cmd:XX old_cmd:XX cmd_spd:XX act:XX
After releasing the emergency stop, if the above error occurs at the moment the robot transitions to `active`, please follow the steps below to address it.
1. Transition the state of the robot to `inactive`:
```
ros2 control set_hardware_component_state <robot_name> inactive
```
2. Transition the state of the `khi_controller` from `active` to `inactive` and then back to `active`:
```
ros2 control switch_controllers --deactivate khi_controller  
ros2 control switch_controllers --activate khi_controller
```
3. Transition the state of the robot to `active`:
```
ros2 control set_hardware_component_state <robot_name> active
```

### Warning: The current is saturated in JTXX. Please reduce the acceleration or change the motion.
To prevent failure of the robot, please ensure that this warning does not appear.  
If you have not performed a warm-up operation, please do so.  
If the weight setting has not been configured, please do so.  
If the load mass and load center of gravity position for the tool have not been set, please configure them.  
Instead of the joint experiencing current saturation, it may be improved by reducing the acceleration of the joint that is operated simultaneously.

<br>

## KRNX Error

Error code of KRNX API is defined in “khi_hardware/include/krnx.h”.   
Description and Value of the error codes are shown in the table below.

| Macro Definition          | Description                     | Value     |
| ------------------------- | ------------------------------- | --------- |
| KRNX_NOERROR              | No Error                        | (0x0000)  |
| KRNX_E_BADARGS            | Invalid Argument                | (-0x1000) |
| KRNX_E_INTERNAL           | Internal Error                  | (-0x1001) |
| KRNX_E_NOTSUPPORTED       | Not Supported API               | (-0x1002) |
| KRNX_E_TIMEOUT            | Timeout                         | (-0x1003) |
| KRNX_E_AUXNOTREADY        | AUX Monitor Not Ready           | (-0x1004) |
| KRNX_E_FOPENFAIL          | File Open Fail                  | (-0x1005) |
| KRNX_E_FILENOTREADY       | File Not Exist                  | (-0x1006) |
| KRNX_E_MATRIX             | Matrix Calculation Error        | (-0x1007) |
| KRNX_E_OUTOFRANGE         | Inverse Conversion Error        | (-0x1008) |
| KRNX_E_CANNOTCAL          | Inverse Jacobian Error          | (-0x1009) |
| KRNX_E_COMPDATA           | RTC Error                       | (-0x100a) |
| KRNX_E_BADUSERID          | Bad User ID                     | (-0x100c) |
| KRNX_E_NULLRESP           | Data Not Received               | (-0x100d) |
| KRNX_E_LOSTPROMPT         | Timeout for Prompt Receive      | (-0x100e) |
| KRNX_E_BUFSND             | Communication Send Error        | (-0x1010) |
| KRNX_E_BUFRCV             | Communication Receive Error     | (-0x1011) |
| KRNX_E_BUFTMO             | Communication Timeout           | (-0x1012) |
| KRNX_E_ASERROR            | AS Error                        | (-0x1020) |
| KRNX_E_NOROBOT            | No Robot Setting                | (-0x1021) |
| KRNX_E_SOCK               | Socket Create Error             | (-0x2000) |
| KRNX_E_NOHOST             | Bad Hostname                    | (-0x2001) |
| KRNX_E_IOCTLSOCK          | Socket Setting Error            | (-0x2002) |
| KRNX_E_SOCKWRITE          | Socket Write Error              | (-0x2003) |
| KRNX_E_SOCKREAD           | Socket Read Error               | (-0x2004) |
| KRNX_E_NODATA             | No Socket Data                  | (-0x2005) |
| KRNX_E_INVALIDPORT        | Invalid Port Number             | (-0x2006) |
| KRNX_E_CONNECT            | Socket Connect Fail             | (-0x2007) |
| KRNX_E_CANTLOGIN          | Login Fail                      | (-0x2008) |
| KRNX_E_ALREADYOPENED      | Socket Already Used             | (-0x2009) |
| KRNX_E_UNEXPECTED         | Received Data Error             | (-0x2010) |
| KRNX_E_KINENOTREADY       | Kinematics Not Initialized      | (-0x2011) |
| KRNX_E_ASDELAYED          | Communication Sync Error        | (-0x2012) |
| KRNX_E_BUFEMPTY           | Communication Buffer Error      | (-0x2013) |
| KRNX_E_BUFNO              | Invalid Buffer Number           | (-0x2014) |
| KRNX_E_BUFDATANUM         | Send Data Error                 | (-0x2015) |
| KRNX_E_RT_INTERNAL        | RT Communication Internal Error | (-0x2100) |
| KRNX_E_RT_CONNECT         | RT Communication Connect Error  | (-0x2101) |
| KRNX_E_RT_TIMEOUT         | RT Communication Timeout        | (-0x2102) |
| KRNX_E_RT_NOTCONNECT      | RT Communication Connect Error  | (-0x2103) |
| KRNX_E_RT_SEND            | RT Communication Send Error     | (-0x2104) |
| KRNX_E_PCASALREADYRUNNING | PC-AS Already Running           | (-0x2200) |
| KRNX_E_TOOMANYPROC        | Too Many Process                | (-0x2201) |
| KRNX_E_INVALIDFILENAME    | Invalid File Name               | (-0x2202) |
| KRNX_E_ILLCONTNO          | Invalid Controller Number       | (-0x2203) |
| KRNX_E_UNDEFF             | Undefined Error                 | (-0xFFFF) |
