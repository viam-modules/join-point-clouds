# Join Point Clouds
The `join_color_depth` model uses [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) to combine the PCD outputs of depth cameras already registered in your config to create a joined point cloud output. It uses the [Viam builtin frame system](https://docs.viam.com/services/frame-system/), and optionally the [iterative closest points (ICP) algorithm](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html), to align and combine the clouds within a single frame.

Note: combining PCDs with RGB data is currently unsupported.

## Configure your `join-point-clouds` component

Navigate to the **CONFIGURE** tab of your machine's page in [the Viam app](https://app.viam.com), searching for the modular component `join-point-clouds` and selecting the model `viam:join-camera:join-point-clouds`.

Fill in the attributes as applicable to the component, according to the example below.

```json
    {
      "name": "jpc-component",
      "namespace": "rdk",
      "type": "camera",
      "model": "viam:camera:join-point-clouds",
      "attributes": {
        "source_cameras": [
          "pcd-cam-1",
          "pcd-cam-2"  // other camera components to get PCDs to join from
        ],
        "target_frame": "world" | "<name-of-frame-in-frame-system>",
        "merge_method": "naive" | "icp",
        "proximity_threshold_mm": 0.1
      },
      "frame": {  // frame system setup
        "parent": "world",
        "translation": {
          "x": 0,
          "y": 0,
          "z": 0  // modify translation based on position in frame
        },
        "orientation": {
          "type": "quaternion",
          "value": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1  // modify quaternion based on rotation
          }
        }
      }
    },
```

Make sure to set up the `source_cam` components frame system data accurately for best results.

The following attributes are available for the `ultrasonic` component:

<!-- prettier-ignore -->
| Attribute | Type | Required? | Description |
| --------- | ---- | --------- | ----------- |
| `source_cameras` | []string | **Required** | The list of existing camera components in your machine to get input PCDs from |
| `target_frame` | string | **Required** | The target frame in the machine frame system to align the resultant PCD to |
| `merge_method`  | string | Optional | Either "naive" or "icp"— "icp" will use ICP to further align the PCDs and combine points below the proximity threshold. Naive will use only the frame system transformation with no further processing. |
| `proximity_threshold_mm`  | int | Optional | Only used when ICP is requested. ICP will combine points under this distance from each other. <br> Default: `0.05`. |
