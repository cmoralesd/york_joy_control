# york_joy_control
Paquete de pruebas para el modelamiento y control del robot YORK mediante joystick.   
   
El paquete contiene un modelo URDF del robot YORK y un lanzador que permite su visualización en rviz2 y gazebo, además del control mediante un joystick y un control cinemático de los movimientos.
<image src="/images/york_joy_control.png" alt="Visualización del modelo en rviz2">   

Lanzar el paquete utilizando el archivo *sim.launch.py*.   
En esta versión inicial del paquete aún no está implementado:   
- La comunicación entre ros2 y gazebo, por lo cual los movimientos que se observan en rviz corresponden a simples cálculos cinemáticos y no a simulación en gazebo.
- Los plugin de ros2_control, que permiten el control considerando los parámetros dinámicos del modelo.