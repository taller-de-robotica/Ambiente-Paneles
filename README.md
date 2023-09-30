# Abrir el ambiente virtual

Desde ambiente_panel/src/

  `ign gazebo paneles.sdf`

# Gazebo Bridge

Desde el workspace del puente de gazebo correr

  `ros2 run ros_gz_bridge parameter_bridge /commands/velocity@geometry_msgs/msg/Twist@ignition.msgs.Twist`

# Correr script de movimiento

Desde ambiente_panel/ correr el nodo

  `ros2 run movement random_twist_publisher` 

