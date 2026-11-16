# Driver Development Guide

Drivers in APX interact with hardware peripherals and publish standard data types to uBus.

## Steps to Write a New Driver

1. **Create Driver Class**: Inherit from `apx::hal::DriverBase`.
2. **Implement Methods**: Implement `init()` for hardware setup and `update()` for periodic reads.
3. **Publish to uBus**: Push data to the standard sensor topics.
4. **Register**: Use the `APX_REGISTER_DRIVER` macro to make it discoverable.

## Example: Simple Temperature Sensor

```cpp
#include "hal/driver_base.hpp"
#include "ubus/ubus.hpp"
#include "ubus/topics/sensor_temp.hpp"

namespace apx::drivers {

class MyTempSensor : public apx::hal::DriverBase {
public:
    MyTempSensor() : DriverBase("MyTemp") {}

    bool init() override {
        // I2C init logic here
        return true;
    }

    void update() override {
        // Read from I2C
        float current_temp = 25.5f; // Stub

        // Publish to uBus
        ubus::SensorTemp msg{};
        msg.timestamp = get_time_usec();
        msg.temperature = current_temp;
        m_pub.publish(msg);
    }

private:
    ubus::Publisher<ubus::SensorTemp> m_pub{"sensor_temp_0"};
};

APX_REGISTER_DRIVER(MyTempSensor);

} // namespace apx::drivers
```

## Integrating

To include your driver in the build, add it to the corresponding `CMakeLists.txt` and ensure it gets linked. The initialization system will automatically instantiate and run registered drivers.
