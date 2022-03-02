import random
import bpy
import numpy as np
import os
import png
import open3d as o3d
import copy

#reading test imafe
def translate_depthmap(img, disparityX, depth_max_distance = 10, depth_min_distance = 0.1, width = 640, height = 640, cam_matrix = o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault), bitdepth = 16):
    # im_np = o3d.io.read_image('./test2.png')
    # depth_max_distance = 10
    # depth_min_distance = 0.1
    # cam_matrix = o3d.camera.PinholeCameraIntrinsic(
    #         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(img, cam_matrix, depth_scale = (2**bitdepth)/depth_max_distance)

    pcd_t = copy.deepcopy(pcd)
    pcd_t.translate((disparityX,0,0))

    recon_img_t = np.zeros((height, width))

    for i in np.array(pcd_t.points):
        x = i[0]*cam_matrix.get_focal_length()[0]/i[2] + cam_matrix.get_principal_point()[0]
        y = i[1]*cam_matrix.get_focal_length()[1]/i[2] + cam_matrix.get_principal_point()[1]
        z = i[2]
        if not (int(y) >= height or int(x) >= width):
            if(z > depth_max_distance):
                z = 0
            elif(z < depth_min_distance):
                z = 0
            recon_img_t[int(y), int(x)] = z * 2**bitdepth/depth_max_distance
   
    return recon_img_t

# if __name__ == "__main__":
#     im = o3d.io.read_image('./test2.png')
#     translate_depthmap(im)

# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# pcd_t.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# o3d.visualization.draw_geometries([pcd_t])

def export(filePath, fileName, data, depthMinDistance, depthMaxDistance, disparityX, width, height):
    print("Saving scene as depthmap...")

    bitdepth = 16
    scalingFactor = 2**bitdepth

    # is is possible to render a depthmap with Blenders compositing functions
    # see: https://blender.stackexchange.com/a/101600/95167
    # with that, we have no information about reflections etc. so we generate it from 
    # our own data

    pixels = np.full(width * height, 2**bitdepth-1) 
    print(pixels)

    for hit in data:
        distance = hit.distance

        # map the values the same way, the Kinect does it
        # 0 means outside range
        
        if distance < depthMinDistance:
            intensity = 0
        elif distance > depthMaxDistance:
            intensity = 1
        else:
            # else, map the values to the given interval
            intensity = distance / depthMaxDistance

        pixels[(hit.y * width) + hit.x] = intensity * (2**bitdepth-1)# scale to 16 bit

    pixels = pixels.reshape((height, width))
    depth_open3d = o3d.geometry.Image((pixels).astype(np.uint16))
    try:
        out = translate_depthmap(depth_open3d, disparityX, depthMaxDistance, depthMinDistance, width, height)
    except Exception as e:
        print(e)
    f = open(os.path.join(filePath, "%s_image_depthmap.png" % fileName), 'wb') 
    w = png.Writer(width, height, greyscale=True, bitdepth=bitdepth)
    out = out.reshape((height, width)).astype(np.uint16)
    w.write(f, out)
    f.close()   

    print("Done.")