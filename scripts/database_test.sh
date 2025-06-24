ros2 topic pub --once /spotX/ns/global_task_allocation_topic triage_task_allocation_interface/msg/GlobalTaskAllocation "{task_allocation: [\
  {task_id: 1, robot_name: spotX, casualty_description: {\
    id: 1, geo_location: {x: 0, y: 1, z: 0}\
  }},\
  {task_id: 2, robot_name: spotX, casualty_description: {\
    id: 2, geo_location: {x: 1, y: 1, z: 0}\
  }},\
  {task_id: 3, robot_name: spotY, casualty_description: {\
    id: 3, geo_location: {x: 2, y: 1, z: 0}\
  }}\
  ]}"
