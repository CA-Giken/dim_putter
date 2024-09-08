import open3d as o3d
import numpy as np
from PIL import Image as im 

K = np.array([[1000, 0, 500], [0, 1000, 500], [0, 0, 1]])
pcd=o3d.io.read_point_cloud("capture.ply")
depth=np.zeros((1024,1024),dtype='uint8')
uv=K.dot(np.array(pcd.points).T)
print(uv)
uvi=(uv[:2]/uv[2]).astype('int')
print(uvi.T,np.max(uvi[0]),np.max(uvi[1]))
depth[uvi[0],uvi[1]]=255
data = im.fromarray(depth)
data.save('capture.png')
