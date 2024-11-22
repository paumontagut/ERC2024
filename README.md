# UJI ROBOTICS - ERC2024

Ver también: [Normas de Código](./normas_código.md)

## Compilación

Estar en ~/ERC2024/test_ws

rosdep install --from-paths src --ignore-src -r -y
- verifica que estén todas las dependencias

colcon build --packages-select custom_interfaces
colcon build --packages-select dynamixel_sdk
source install/setup.bash
colcon build --symlink-install --packages-select bringup
colcon build
