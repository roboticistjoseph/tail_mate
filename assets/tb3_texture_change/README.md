## 🦾 TurtleBot3 Waffle Pi — DAE Meshes & Sensor Config README

- **DAE Meshes:** All STL files (which had textures) are replaced with DAE files—so the robot is now gloriously texture-free, for this project.
- **Sensor Setup:** Camera visualization and always-on are set to `true` for camera and lidar. They have been set to `false`.
Perfect for when you want a clean, minimal simulation.

### 🛠️ Why DAE Instead of STL?

| Format | Texture Support | Why Use It Here?         |
|--------|----------------|-------------------------|
| STL    | Yes            | Had unwanted textures   |
| DAE    | No             | Clean, no textures!     |

### 📝 Example: model.sdf Snippet

```xml
<!-- Camera sensor config -->
<sensor name="camera" type="camera">
  <visualize>false</visualize>
  <always_on>false</always_on>
  <!-- ... -->
</sensor>

<!-- LDS sensor config (disabled) -->
<sensor name="hls_lfcd_lds" type="ray">
    <always_on>false</always_on>
    <visualize>false</visualize>
</sensor>
```
