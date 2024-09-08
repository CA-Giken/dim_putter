# Saisun 3D

## Packages to add

1. ros_numpy
~~~
cd <<catkin work space>>/src
git clone https://github.com/eric-wieser/ros_numpy
~~~

## Launch
1. Launch "start.launch"
2. Launch "V-CAM" or "3DCAM" from the Dash-bar
3. Redraw scene by "Redraw icon" on the Dash-bar
4. To publish "sensors/X1" simulates to capture the scene

## Publishers
|Topic|Type|file|Description|
|:---|:---|:---|:---|
|/sensors/capt_pc2|PointCloud2|-|センサー点群出力。phoxiは"~pointcloud"をremap|
|/saisun/capt_p[0-9]|PointCloud2|-|面に分割した点群列|
|/saisun/capt_cn[0-9]|PoseArray|-|面の中心軸(面が円形の場合)|
|/saisun/capt_eg[0-9]|PoseArray|-|面のエッジ|
|/saisun/result|String|-|計測結果JSON形式|

## Subscribers
|Topic|Type|file|Description|
|:---|:---|:---|:---|
|/sensors/X1|Bool|-|センサーのトリガー|

## Result JSON format
Array of object below
~~~
{
  dimension:{
    value: 000,
    base: center
  }
  refs:{
    center: 0,
    plane: 0,
    edge: 0
  }
}
~~~
