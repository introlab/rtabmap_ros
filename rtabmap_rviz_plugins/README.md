

```wiki
'''''[[rtabmap_ros]]:''' [[rtabmap]] | [[rtabmap_conversions]] | [[rtabmap_costmap_plugins]] | [[rtabmap_demos]] | [[rtabmap_examples]] | [[rtabmap_launch]] | [[rtabmap_legacy]] | [[rtabmap_msgs]] | [[rtabmap_odom]] | [[rtabmap_python]] | [[rtabmap_rviz_plugins]] | [[rtabmap_slam]] | [[rtabmap_sync]] | [[rtabmap_util]] | [[rtabmap_viz]]''
```



## RVIZ plugins

### Map Cloud Display

![MapCloudType.png](doc/MapCloudType.png)

This rviz plugin subscribes to /mapData
([rtabmap_msgs/msg/MapData](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/MapData.html))
topic. A 3D map cloud will be created incrementally in RVIZ. When the
graph is changed, all point clouds added in RVIZ will be transformed to
new poses. It has the same properties as
[PointCloud](https://index.ros.org/p/rviz) display but with these new
ones:

  -- -------------------------------- -- ---------------------------------------------------------------------------------------------------------------------------------- -- ------------------ -- ------------- --
                                         **Properties**                                                                                                                                                            
     **Name**                            **Description**                                                                                                                       **Valid Values**      **Default**   
     Cloud decimation                    How the input depth and RGB images are decimated before creating the point cloud                                                      \[1-16\]              4             
     Cloud max depth (m)                 Maximum depth of each point cloud added to map (0 means no maximum)                                                                   \[0-9999\]            4             
     Cloud voxel size (m)                Voxel size of the generated point clouds (0 means no voxel)                                                                           \[0-1\]               0.01          
     Filter floor (m)                    Maximum height of the floor filtered (disabled=0).                                                                                    \[0-9999\]            0             
     Node filtering radius (m)           Only keep one node in the specified radius (disabled=0).                                                                              \[0-1\]               0.2           
     Node filtering angle (degrees)      Only keep one node in the specified angle in the filtering radius (disabled=0).                                                       \[0-180\]             30            
     Download Map                        Download the map from [rtabmap](https://index.ros.org/p/rtabmap_slam) node. This may take a while if the map is big, be patient!      .                     .             
     Download Graph                      Download the graph from [rtabmap](https://index.ros.org/p/rtabmap_slam) node.                                                         .                     .             
  -- -------------------------------- -- ---------------------------------------------------------------------------------------------------------------------------------- -- ------------------ -- ------------- --

### Map Graph Display

This rviz plugin subscribes to /mapGraph
([rtabmap_msgs/msg/MapGraph](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/MapGraph.html))
topic. It will show the RTAB-Map's graph with different colors
depending on the links' type. It has the same properties as
[Path](https://index.ros.org/p/rviz) display.

### Info Display

This rviz plugin subscribes to /info
([rtabmap_msgs/msg/Info](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/Info.html))
topic. Information about loop closures detected are shown in the
"Status".
