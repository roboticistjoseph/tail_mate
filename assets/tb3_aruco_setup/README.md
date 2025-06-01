## ArUco Markers
- ArUco markers are like QR codes for robots—used for navigation, localization, and object tracking. They’re easy for cameras to spot and help your robot know exactly where it is in the world.

### Why ArUco Markers?
- **Navigation**: Robots use them as visual landmarks.
- **Localization**: Helps the robot know its position.
- **Object Tracking**: Great for pick-and-place or following tasks.
- **Easy Detection**: Fast, reliable, and open-source 
.
### Quick Start: Use the Pre-Configured Model
**Don’t want to do all the manual work?**
- Just use the provided model.sdf file!
- Here’s how:
  1. Copy the model.sdf from this folder into your TurtleBot3 WafflePi model directory.
  2. Make sure the marker image (aruco_marker_100mm.png) and material files are in the right places (see folder structure below).
  3. Update your package.xml as shown in the Package Configuration section.

## Adding ArUco marker to robot/object:
1. Created ArUco using this platform: [link](https://chev.me/arucogen/)
    - Remember your Dictionary, ID and marker_size
    - The platform creates a .svg or .pdf file.
    - convert to .png
2. Create a folder strcuture as below where the model.sdf is present [Materials -> scripts & textures]
    ```
    /turtlebot3_waffle_pi
    ├── aruco_marker
    │   └── materials
    │       ├── scripts
    │       │   └── aruco_marker.material
    │       └── textures
    │           └── aruco_marker.png
    ├── model-1_4_.sdf
    ├── model.config
    ├── model.sdf
    └── model.txt
    ```
3. Place the created aruco_marker.png in textures folder
4. Create a material script called aruco_marker.material in 'scripts' and place the below code
    ```
    material ArUcoMarker
    {
    technique
    {
        pass
        {
        texture_unit
        {
            texture aruco_marker_100mm.png
            filtering none  // Keeps those marker edges sharp!
            scale 1 1
        }
        }
    }
    }
    ```
5. inside the model.sdf file, place the below code under which link you want your marker to be attached.
    ```
    <visual name="aruco_marker_visual">
        <pose>-0.063 0 0.13  0 0 0</pose> <!-- Fine tune pose to hover slightly above the LiDAR surface -->
        <geometry>
          <box>
            <size>0.05 0.05 0.001</size> <!-- 50mm x 50mm x 1mm -->
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://turtlebot3_waffle_pi/aruco_marker/materials/scripts</uri>
            <uri>model://turtlebot3_waffle_pi/aruco_marker/materials/textures</uri>
            <name>ArUcoMarker</name>
          </script>
        </material>
      </visual>
      ```
    - Fine tune <pose> and box <size> to reflect accordingly.
6. Update package.xml to locate media
    ```
    <export>
        <build_type>ament_cmake</build_type>
        <gazebo_ros gazebo_model_path="${prefix}/models"/>
        <gazebo_ros gazebo_media_path="${prefix}/media"/> <!-- ADD THIS HERE -->
    </export>
    ```
