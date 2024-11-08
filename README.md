# Abrir el ambiente virtual

Desde ambiente_panel/src/

  `gz sim paneles.sdf`

Si se tiene el cortafuegos ```uwf``` activado es necesario asegurarse de que ```multicast``` está permitido. [Instalation troubleshooting. Enable multicast.](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)

```
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

# Gazebo Bridge

Instalar el puente para Gazebo:
```
sudo apt install ros-jazzy-ros_gz
```

Correr

  `ros2 run ros_gz_bridge parameter_bridge /commands/velocity@geometry_msgs/msg/Twist@gz.msgs.Twist`

# Correr script de movimiento

Desde ambiente_panel/ correr el nodo

  `ros2 run movement random_twist_publisher` 


# Demo de busqueda de paneles
```bash

# Simulacion 
cd ambiente_panel/src
gz sim paneles.sdf

# En diferentes terminales con source /opt/ros2/jazzy:

# 1) Puente comandos de movimiento
ros2 run ros_gz_bridge parameter_bridge /commands/velocity@geometry_msgs/msg/Twist@gz.msgs.Twist

# 2) Puente tópico de la camara
ros2 run ros_gz_bridge parameter_bridge '/camera@sensor_msgs/msg/Image@gz.msgs.Image'

# 3) Puente del "odometro" de la kobuki
ros2 run ros_gz_bridge parameter_bridge /model/kobuki_standalone/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V

# Dar play a la simulacion de gazeebo antes de ejecutar el demo

# Correr demo:
ros2 run movement path_rutine
```
