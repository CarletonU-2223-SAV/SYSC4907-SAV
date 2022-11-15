## Research

### Navigation Research
#### Care-O-bot
- Local navigation
  - Pros:
    - Capable of detecting the safety radius around the obstacle.
  - Cons:
    - Need to have specific immediate goals.
    - Can only plan to a target pose in the range of the costmap.
- SLAM navigation
  - Pros:
    - Can build a map as the robot explore the environment.
    - Can use 2D Nav Goal to specific a navigation goal, allowing the robot to move and build a map.
  - Cons:
    - Need a run on the environment first to explore it and build map.
    - 2D Nav Goal can only specify target poses that are in the range of the local costmap or in the previous explored map.
- Global navigation
  - Pros:
    - Work well if a have a pre-defined map
  - Cons:
    - Need to be used with both local and SLAM navigation if there is no pre-defined map.
#### TurtleBot
- Pros:
  - Can build a map and use it to autonomously navigate around the environment
  - Also work with a known map
- Cons:
  - Range of movement could be limited as well as the map size.
#### Husky
- Pros:
  - Have function to perform basic autonomous planning and movement
  - Basic planning and movement can be use simultaneously with localization and mapping
  - Have the exploration mode, allowing the robot to run without a predetermined map. 
  Also allow for resizing the map if there is a predetermined map.
- Cons:
  - Would require a laser scanner for boundary detection and movement.
#### MRP2
- Pros:
  - Can see the mapping process of the robot via RViz
  - Capable of exploring surrounding areas and make a map
  - Can do navigation through a known map
- Cons:
  - No mentioning on obstacle detection
  - Would require one exploration run on the map to map out the environment before it can be put to use.