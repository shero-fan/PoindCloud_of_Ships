# python 3.6 python PointCloud_of_ships.py
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

Currentwork_path=os.getcwd()
print("Current Work path is ",Currentwork_path)


def show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)

def addweighted(img1, img2, c):
    addweighted_img = cv2.addWeighted(img1, c, img2, 1-c, 0)
    return addweighted_img


def mkdir(folder_path):    
	folder = os.path.exists(folder_path)
	if not folder:                   
		os.makedirs(folder_path)            
		print ("Create folder named",folder_path)

def deldir(folder_path):
    folder = os.path.exists(folder_path)
    if folder:
        shutil.rmtree(folder_path)
        print ("Delete folder named",folder_path)

def set_intrp(x,y,z):
    num={'sections_x':x,'sections_y':y,'sections_z':z}
    return num
 
def intrp_sec(work_path,num_intrp): 
    input_path=os.path.join(work_path,'Input_Origin')
    output_path=os.path.join(work_path,'Input_Pc')
    deldir(output_path)
    mkdir(output_path)
    sec_in=os.listdir(input_path)
    for inputSec in sec_in:
        current_path=os.path.join(input_path,inputSec)
        coutput_path=os.path.join(output_path,inputSec)
        mkdir(coutput_path)
                
        process_path=os.path.join(output_path,"process")
        mkdir(process_path)
        
        image_in=os.listdir(current_path)
        
        for j in range(1,num_intrp[inputSec]+1):
            for i in range(1,len(image_in)):
                if j==1:
                    image1_path=os.path.join(current_path,"{0}{1}.png".format(inputSec,str((i))))
                    image3_path=os.path.join(current_path,"{0}{1}.png".format(inputSec,str((i+1))))
                else:
                    image1_path=os.path.join(current_path,"num{0} section{1}.png".format(str(j-1),str((i))))
                    image3_path=os.path.join(current_path,"num{0} section{1}.png".format(str(j-1),str((i+1))))
                img_1 = cv2.imread(image1_path)
                img_3 = cv2.imread(image3_path)
                img_2=addweighted(img_1,img_3,0.5)
                newimg1=img_1
                newimg2=img_2
                newimg3=img_3
                #newimg1=cv2.applyColorMap(img_1, cv2.COLORMAP_JET);
                #newimg2=cv2.applyColorMap(img_2, cv2.COLORMAP_JET);
                #newimg3=cv2.applyColorMap(img_3, cv2.COLORMAP_JET);
                cv2.imwrite(os.path.join(process_path,"num{0} section{1}.png".format(str(j),str((i-1)*2+1))),newimg1)
                cv2.imwrite(os.path.join(process_path,"num{0} section{1}.png".format(str(j),str(i*2))),newimg2)
                if j==num_intrp[inputSec]:
                    cv2.imwrite(os.path.join(coutput_path,"{0}{1}.png".format(inputSec,str((i-1)*2+1))),newimg1)
                    cv2.imwrite(os.path.join(coutput_path,"{0}{1}.png".format(inputSec,str(i*2))),newimg2)
                i=i+1
                if i==len(image_in)-1:   #加上最后一张图片
                    cv2.imwrite(os.path.join(process_path,"num{0} section{1}.png".format(str(j),str(i*2+1))),newimg3)
                    if j==num_intrp[inputSec]:
                        cv2.imwrite(os.path.join(coutput_path,"{0}{1}.png".format(inputSec,str(i*2+1))),newimg3)
            else:
                print('End interpolation {}'.format(str(j)))
            current_path=process_path
            image_in=os.listdir(current_path)
            j=j+1
        else:
            deldir(process_path) 
            print("End all inerpolation of",inputSec)
    else:    
        print('END ALL INTERPOLATION ') 

def set_sec():
    scale_factorx=40
    scale_factory=10
    scale_factorz=20
    secStep={'sections_x':0.57/scale_factorx,'sections_y':-0.0728/scale_factory,'sections_z':0.08545/scale_factorz}
    initialDepth={'sections_x':0.0/scale_factorx,'sections_y':0.0/scale_factory,'sections_z':-0.3418/scale_factorz}
    return secStep,initialDepth

def getIfromRGB(rgb):#RGB信息存在一个二进制数RGBint中
    red = rgb[0]
    green = rgb[1]
    blue = rgb[2]
    RGBint = (red<<16) + (green<<8) + blue
    return RGBint

def dump_points(points, depth, rotatedegree):###########orientation  e.g. [1,0,0]
    calcPoints = []
    height = points.shape[0]
    width = points.shape[1]

    # setup rotation from euler angles
    
    rotation_vecx=np.radians(rotatedegree['x_axis'])*np.array([1, 0, 0]) 
    rotation_vecy=np.radians(rotatedegree['y_axis'])*np.array([0, 1, 0])
    rotation_vecz=np.radians(rotatedegree['z_axis'])*np.array([0, 0, 1])
    rotationx= R.from_rotvec(rotation_vecx)
    rotationy= R.from_rotvec(rotation_vecy)
    rotationz= R.from_rotvec(rotation_vecz)
    #rotation= R.from_euler('zyx', [
    #np.array([0, 0, rotatedegree['z_axis']]),
    #np.array([0, rotatedegree['y_axis'], 0]),
    #np.array([rotatedegree['x_axis'], 0, 0])], degrees=True)

    # write out the points
    for row in range(0, height):
        for col in range(0, width):
            xNorm = (col / width) - 0.5#归一化处理
            yNorm = (row / height) - 0.5
            pointsRgb = points[row, col]
            if pointsRgb[0] > 0 or pointsRgb[1] > 0 or pointsRgb[2] > 0:
                if pointsRgb[0] < 220 or pointsRgb[1] < 220 or pointsRgb[2] < 220:
                    rotatedPoint = rotationx.apply([xNorm, yNorm, depth])
                    rotatedPoint = rotationy.apply(rotatedPoint)
                    rotatedPoint = rotationz.apply(rotatedPoint)
                    calcPoints.append([rotatedPoint[0] * 0.5, rotatedPoint[1] * 0.5, rotatedPoint[2] * 0.5, getIfromRGB(pointsRgb)])#？？0.5
    return calcPoints
    
def write_points(points, fileName):
    file = open(fileName, "wb")
    sumpoint=len(sum(points, []))
    headers = [
        "VERSION .7",
        "FIELDS x y z rgb",
        "SIZE 4 4 4 4",
        "TYPE F F F I",
        "COUNT 1 1 1 1",
        "WIDTH %i" % (sumpoint),
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        "POINTS %i" % (sumpoint),
        "DATA binary"
    ]
    for header in headers:
        file.write(("%s\n" % (header)).encode('ASCII'))
    for pointRow in points:
        for point in pointRow:
            file.write(struct.pack("f", point[0]))
            file.write(struct.pack("f", point[1]))
            file.write(struct.pack("f", point[2]))
            file.write(struct.pack("I", point[3]))
    file.close()  

def generate_pc(initial_d,sec_step,num_intrp,work_path):
    frameCount = 0
    totalpoint = 0
    input_path=os.path.join(work_path,'Input_Pc')
    sec_in=os.listdir(input_path)
    degree={}
    degree['sections_x']={'x_axis':90,'y_axis':180,'z_axis':90}###check
    degree['sections_y']={'x_axis':-90,'y_axis':0,'z_axis':0}
    degree['sections_z']={'x_axis':0,'y_axis':0,'z_axis':0}
    for inputSec in sec_in:
        cinput_path=os.path.join(input_path,inputSec)
        output_path=os.path.join(work_path,'Output_Pc')
        mkdir(output_path)
        eDepth=initial_d[inputSec]
        e_STEP=sec_step[inputSec]/(2**num_intrp[inputSec])
        points = []
        for frame in range(1, len(os.listdir(cinput_path))+1):
            file_name = os.path.join(cinput_path,"{0}{1}.png".format(inputSec,str(frame)))#循环读入image
            imageObject = Image.open(file_name)
            print("Reading", file_name)
    
            img = imageObject.convert('RGB')     
            points.append(dump_points(np.array(img), eDepth,degree[inputSec]))          
            eDepth += e_STEP
            frameCount += 1
        output_file_name=os.path.join(output_path,'{}.pcd'.format(inputSec))    
        write_points(points, output_file_name)
        #show_pc(points,inputSec)
        totalpoint = totalpoint + len(sum(points, []))
        print("End generate point cloud of ",inputSec)
    print("Total points of the point cloud is",totalpoint)
    print("END ALL Point Cloud Generation")

def show_pc(m4,name):
    #列表解析x,y,z的坐标
    x=[k[0] for k in m4]
    y=[k[1] for k in m4]
    z=[k[2] for k in m4]
    #开始绘图
    fig=plt.figure(dpi=120)
    ax=fig.add_subplot(111,projection='3d')
    #标题
    plt.title(name)
    #利用xyz的值，生成每个点的相应坐标（x,y,z）
    ax.scatter(x,y,z,c='b',marker='.',s=2,linewidth=0,alpha=1,cmap=plt.cm.jet)
    ax.axis('scaled')          
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    #显示
    plt.show()
    
def show_mlb():
        eng = matlab.engine.start_matlab()
        eng.showpc(nargout=0)
        
def main():
    secStep,initialDepth=set_sec()
    Nums_intrp=set_intrp(1,2,1)
    #intrp_sec(Currentwork_path,Nums_intrp)
    generate_pc(initialDepth,secStep,Nums_intrp,Currentwork_path)
    show_mlb()
    
    
if __name__ == "__main__":
    main()
    
