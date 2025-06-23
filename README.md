# IMU Cpp interface for Reddog

xsens_mti_ros2_driver/src/xdainterface.cpp
```
std::vector<double> XdaInterface::spinForReddog(std::chrono::milliseconds timeout)
{
    ...
}
```

hardware_manager/src/main.cpp

```
auto data = xdaInterface->spinForReddog(milliseconds(10));
```
> data = {quaternion.x, quaternion.y, quaternion.z, quaternion.w, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z}