from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    ns = "my_robot"
    speedTopic = "motorspeed"
    pkg = "motorspeedcontroller"
    motorDriverExecutable = "motordriver"
    motorInterfaceExecutable = "speedinterface"

    motor_frontleft_driver = launch_ros.actions.Node(namespace=ns,
                                                     package=pkg,
                                                     executable=motorDriverExecutable,
                                                     remappings=[
                                                         (speedTopic, f"/frontleft/{speedTopic}")
                                                         ],
                                                     parameters=[
                                                         {"PWM_PIN": 18}
                                                     ])
    
    motor_frontright_driver = launch_ros.actions.Node(namespace=ns,
                                                      package=pkg,
                                                      executable=motorDriverExecutable,
                                                      remappings=[
                                                          (speedTopic, f"/frontright/{speedTopic}")
                                                          ],
                                                      parameters=[
                                                         {"PWM_PIN": 23}
                                                     ])
    
    motor_backleft_driver = launch_ros.actions.Node(namespace=ns,
                                                    package=pkg,
                                                    executable=motorDriverExecutable,
                                                    remappings=[
                                                         (speedTopic, f"/backleft/{speedTopic}")
                                                         ],
                                                    parameters=[
                                                         {"PWM_PIN": 24}
                                                     ])
    
    motor_backright_driver = launch_ros.actions.Node(namespace=ns,
                                                     package=pkg,
                                                     executable=motorDriverExecutable,
                                                      remappings=[
                                                          (speedTopic, f"/backright/{speedTopic}")
                                                          ],
                                                     parameters=[
                                                         {"PWM_PIN": 25}
                                                     ])
    
    driveInterface = launch_ros.actions.Node(napespace=ns,
                                     package=pkg,
                                     executable=motorInterfaceExecutable
                                     )

    return LaunchDescription([
        motor_frontleft_driver,
        motor_frontright_driver,
        motor_backleft_driver,
        motor_backright_driver,
        driveInterface
    ])