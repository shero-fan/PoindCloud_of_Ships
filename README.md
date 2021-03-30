# PoindCloud_of_Ships
Generate point clouds of the flow field of ships from their cross-sections which are perpendicular to x y z axis respectively.
# README

# About

This code is to generate point clouds by sets of cross-sections which are perpendicular to the x,  y and z axis respetively.

# Working directory

../PointCloud_of_ships.py

../showpc.m

../position_of_cross-section.xlsx

../Input_Origin/

sections_x/

sections_y/

sections_z/

```powershell
(python 3.6)>python PointCloud_of_ships.py
```

../Input_Pc/

sections_x/

sections_y/

sections_z/

../Output_Pc/

sections_x.pcd

sections_y.pcd

sections_z.pcd

This code will automatically generate dictories above to store the cross-sections after several interpolations and point clouds respectively.

## Input

'../Input_Origin/sections_{axis}/sections_{axis}{num}.png'

## Output

After interpolating:

'../Input_Pc/sections_{axis}/sections_{axis}{num}.png'

Generate point clouds:

'../Output_Pc/sections_{axis}.pcd'

[pcd file format](https://blog.csdn.net/renshengrumenglibing/article/details/9073763?ops_request_misc=&request_id=&biz_id=102&utm_term=.pcd%20binary%20header&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-9073763.pc_search_positive&spm=1018.2226.3001.4187)

Figure View (matlab): Point Clouds

# Import

Libraries, Modules, Package

## Codes

```powershell
import os
import sys
import shutil
import struct
import numpy as np
import cv2
from PIL import Image
from PIL import GifImagePlugin
import errno
from scipy.spatial.transform import Rotation as R
import scipy.ndimage
import matlab.engine
import time
```

## References

[Numpy](https://numpy.org/doc/stable/reference/)

[Scipy](https://docs.scipy.org/doc/scipy/reference/)

[OpenCV](https://blog.csdn.net/long_xuan123/article/details/105945628?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522161709814216780271527621%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=161709814216780271527621&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-3-105945628.pc_search_result_hbase_insert&utm_term=python+opencv&spm=1018.2226.3001.4187)

[Pillow](https://blog.csdn.net/swinfans/article/details/101989157?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522161598345616780357264136%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=161598345616780357264136&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-1-101989157.first_rank_v2_pc_rank_v29_10&utm_term=pillow)

# Defined Functions

## Interpolation

Set number  of interpolation

x,y,z

```python
def set_intrp(x,y,z):
    num={'sections_x':x,'sections_y':y,'sections_z':z}
    return num
```

Method of interpolation

```python
def addweighted(img1, img2, c):
    addweighted_img = cv2.addWeighted(img1, c, img2, 1-c, 0)
    return addweighted_img
```

Manipulation

```python
def intrp_sec(work_path,num_intrp):
	...
```

## Point Cloud Generation

Set parameters referr to 'position_of_cross-section.xlsx'

```python
def set_sec():
    scale_factorx=60
    scale_factory=10
    scale_factorz=20
    secStep={'sections_x':0.57/scale_factorx,'sections_y':-0.0728/scale_factory,'sections_z':0.08545/scale_factorz}
    initialDepth={'sections_x':0.0/scale_factorx,'sections_y':0.0/scale_factory,'sections_z':-0.3418/scale_factorz}
    return secStep,initialDepth
```

Preprocessing points

Only rotation at present.(from rotation vector)

```python
def dump_points(points, depth, rotatedegree):
	...
```

Write data in '.pcd' file

(binary)

```python
def write_points(points, fileName):
	...
```

Manipulation

```python
def generate_pc(initial_d,sec_step,num_intrp,work_path):
	...
```

## Show point cloud

Use matlab to show

Gray at present.

```python
def show_mlb():
eng = matlab.engine.start_matlab()
eng.showpc(nargout=0)
time.sleep(10)
```
