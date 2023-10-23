# Abrir el ambiente virtual

Desde ambiente_panel/src/

  `ign gazebo paneles.sdf`

# Gazebo Bridge

Desde el workspace del puente de gazebo correr

  `ros2 run ros_gz_bridge parameter_bridge /commands/velocity@geometry_msgs/msg/Twist@ignition.msgs.Twist`

# Correr script de movimiento

Desde ambiente_panel/ correr el nodo

  `ros2 run movement random_twist_publisher` 


# Demo de busqueda de paneles
```bash

# Simulacion 
cd ambiente_panel/src
ign gazebo paneles.sdf

# En diferentes terminales con source /opt/ros2/humble:

# 1) Puente comandos de movimiento
ros2 run ros_gz_bridge parameter_bridge /commands/velocity@geometry_msgs/msg/Twist@ignition.msgs.Twist 

# 2) Puente topico de la camara
ros2 run ros_gz_bridge parameter_bridge '/camera@sensor_msgs/msg/Image@ignition.msgs.Image'

# 3) Puente del "odometro" de la kobuki
ros2 run ros_gz_bridge parameter_bridge /model/kobuki_standalone/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V

# Dar play a la simulacion de gazeebo antes de ejecutar el demo

# Correr demo:
ros2 run movement path_rutine
```
