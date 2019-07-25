import argparse
import cv2 as cv
import cv2.aruco
import numpy as np
import plyfile
import glob
import os
import OCDatasetLoader.OCDatasetLoader as OCDatasetLoader

from collections import namedtuple
from copy import deepcopy

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def keyPressManager():
    print('keyPressManager.\nPress "c" to continue or "q" to abort.')
    while True:
        key = cv.waitKey(15)
        if key == ord('c'):
            print('Pressed "c". Continuing.')
            break
        elif key == ord('q'):
            print('Pressed "q". Aborting.')
            exit(0)


def drawMask(img, imgpts):

    imgpts = np.int32(imgpts).reshape(-1, 2)

    img = cv.drawContours(img, [imgpts[:4]], -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[4:6]],[imgpts[1::-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[5:7]],[imgpts[2:0:-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[6:8]],[imgpts[3:1:-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[3::-3]],[imgpts[4::3]]]), -1, (255, 255, 255), -3)

    # img = cv.drawContours(img, [imgpts[4:]], -1, (255, 255, 255), -3)

    return img


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------
if __name__ == "__main__":

    # ---------------------------------------
    # --- Parse command line argument
    # ---------------------------------------
    ap = argparse.ArgumentParser()

    # Dataset loader arguments
    ap.add_argument("-p", "--path_to_images", help="path to the folder that contains the OC dataset", required=True)
    ap.add_argument("-ext", "--image_extension", help="extension of the image files, e.g., jpg or png", default='jpg')
    ap.add_argument("-m", "--mesh_filename", help="full filename to input obj file, i.e. the 3D model", required=True)
    ap.add_argument("-i", "--path_to_intrinsics", help="path to intrinsics yaml file", required=True)
    ap.add_argument("-ucci", "--use_color_corrected_images", help="Use previously color corrected images",
                    action='store_true', default=False)
    ap.add_argument("-si", "--skip_images", help="skip images. Useful for fast testing", type=int, default=1)
    ap.add_argument("-vri", "--view_range_image", help="visualize sparse and dense range images", action='store_true',
                    default=False)

    args = vars(ap.parse_args())
    print(args)

    # ---------------------------------------
    # --- INITIALIZATION
    # ---------------------------------------
    dataset_loader = OCDatasetLoader.Loader(args)
    dataset_cameras = dataset_loader.loadDataset()
    num_cameras = len(dataset_cameras.cameras)
    print("#########################################################################################################\n")
    print("Loaded " + str(num_cameras) + " cameras to the dataset!\n")

    # ---------------------------------------
    # --- Utility functions
    # ---------------------------------------
    def matrixToRodrigues(T):
        rods, _ = cv.Rodrigues(T[0:3, 0:3])
        rods = rods.transpose()
        return rods[0]


    def rodriguesToMatrix(r):
        rod = np.array(r, dtype=np.float)
        matrix = cv.Rodrigues(rod)
        return matrix[0]


    def traslationRodriguesToTransform(translation, rodrigues):
        R = rodriguesToMatrix(rodrigues)
        T = np.zeros((4, 4), dtype=np.float)
        T[0:3, 0:3] = R
        T[0:3, 3] = translation
        T[3, 3] = 1
        return T

    # ---------------------------------------
    # --- Detect ARUCOS
    # ---------------------------------------

    ArucoT = namedtuple('ArucoT', 'id center translation rodrigues aruco2camera camera2aruco')

    class ArucoConfiguration:
        def __init__(self):
            pass

    dataset_arucos = ArucoConfiguration()
    dataset_arucos.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
    dataset_arucos.parameters = cv.aruco.DetectorParameters_create()

    dataset_arucos.markerSize = 0.082
    dataset_arucos.distortion = np.array(dataset_cameras.cameras[0].rgb.camera_info.D)
    dataset_arucos.intrinsics = np.reshape(dataset_cameras.cameras[0].rgb.camera_info.K, (3, 3))
    dataset_arucos.world_T_aruco = {}
    dataset_arucos.aruco_T_world = {}

    # For each camera
    for i, camera in enumerate(dataset_cameras.cameras):
        camera.rgb.arucos = {}
        print("In camera " + camera.name + " there is:")

        image = cv.cvtColor(camera.rgb.image, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(image, dataset_arucos.aruco_dict,
                                                 parameters=dataset_arucos.parameters)

        if ids is None:
            print("\t\t\t No ArUco detected!")
            continue

        # Estimate pose of each marker
        rotationVecs, translationVecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, dataset_arucos.markerSize,
                                                                               dataset_arucos.intrinsics,
                                                                               dataset_arucos.distortion)
        # For each ArUco marker detected in the camera
        for j, id in enumerate(ids):

            # OpenCV format hack
            id = id[0]

            print("\t\t\t Aruco " + str(id) + ";")

            # OpenCV format hack
            my_corners = corners[j][0][:]

            # separate corners into x and y coordinates
            x = []
            y = []
            for corner in my_corners:
                x.append(corner[0])
                y.append(corner[1])

            # Get tuple with center of marker
            center = (np.average(x), np.average(y))

            rodrigues = np.array(rotationVecs[j][0])
            translation = np.array(translationVecs[j][0])
            aruco2camera = traslationRodriguesToTransform(translation, rodrigues)

            camera2aruco = np.linalg.inv(aruco2camera)

            # Create and assign the ArUco object to data structure
            camera.rgb.arucos[id] = (ArucoT(id, center, translation, rodrigues, aruco2camera, camera2aruco))

            # Add only if there is still not an estimate (made by another camera)
            if id not in dataset_arucos.world_T_aruco:
                dataset_arucos.world_T_aruco[id] = np.dot(camera.rgb.matrix, camera.rgb.arucos[id].camera2aruco)

            if id not in dataset_arucos.aruco_T_world:
                dataset_arucos.aruco_T_world[id] = np.dot(camera.rgb.matrix, camera.rgb.arucos[id].aruco2camera)

    # Display information over the ArUcos
    font = cv.FONT_HERSHEY_SIMPLEX
    for i, camera in enumerate(dataset_cameras.cameras):
        image = deepcopy(camera.rgb.image)
        corners, ids, _ = cv.aruco.detectMarkers(image, dataset_arucos.aruco_dict,
                                                 parameters=dataset_arucos.parameters)

        # Draw axis and write info
        for key, aruco in camera.rgb.arucos.items():
            cv.aruco.drawAxis(image, dataset_arucos.intrinsics, dataset_arucos.distortion, aruco.rodrigues,
                               aruco.translation, 0.05)
            cv.putText(image, "Id:" + str(aruco.id), aruco.center, font, 1, (0, 255, 0), 2, cv.LINE_AA)

        # Draw the outer square
        cv.aruco.drawDetectedMarkers(image, corners)

        # Show the image
        cv.namedWindow('cam' + str(i), cv.WINDOW_NORMAL)
        cv.imshow('cam' + str(i), image)

    print("Displaying aruco detections for all images.")
    keyPressManager()
    cv.destroyAllWindows()

    # ---------------------------------------
    # --- Inpainting
    # ---------------------------------------

    arucoBorder = 0.007
    arucoThickness = 0.006
    blurBorder = 0.030

    mask3DPoints = np.float32(
        [[-(dataset_arucos.markerSize / 2 + arucoBorder), dataset_arucos.markerSize / 2 + arucoBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder, dataset_arucos.markerSize / 2 + arucoBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder, -(dataset_arucos.markerSize / 2 + arucoBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), -(dataset_arucos.markerSize / 2 + arucoBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), dataset_arucos.markerSize / 2 + arucoBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder, dataset_arucos.markerSize / 2 + arucoBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder, -(dataset_arucos.markerSize / 2 + arucoBorder), -arucoThickness],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), -(dataset_arucos.markerSize / 2 + arucoBorder), -arucoThickness]])

    blurMask3DPoints = np.float32(
        [[-(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, -(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), -(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder + blurBorder, -(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), -arucoThickness],
         [-(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), -(dataset_arucos.markerSize / 2 + arucoBorder + blurBorder), -arucoThickness]])

    # Get paths to all the .ply files
    cloudFiles = sorted(glob.glob(args['path_to_images'] + '/*.' + 'ply'))

    # Create dir to save coloured point clouds
    directory = 'ColouredClouds'
    if not os.path.exists(directory):
        os.makedirs(directory)

    for i, camera in enumerate(dataset_cameras.cameras):

        print("Creating a mask for camera " + camera.name + "...")

        image = deepcopy(camera.rgb.image)
        height, width, channels = image.shape
        mask = np.zeros((height, width), dtype=np.uint8)
        ghostMask = np.zeros((height, width), dtype=np.uint8)
        blurMask = np.zeros((height, width), dtype=np.uint8)
        world_T_camera = np.linalg.inv(camera.rgb.matrix)

        # For each ArUco detected in the image
        for key, aruco in camera.rgb.arucos.items():

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(mask3DPoints, aruco.rodrigues, aruco.translation,
                                            dataset_arucos.intrinsics, dataset_arucos.distortion)

            maskpts, jac = cv2.projectPoints(blurMask3DPoints, aruco.rodrigues, aruco.translation,
                                             dataset_arucos.intrinsics, dataset_arucos.distortion)

            mask = drawMask(mask, imgpts)

            blurMask = drawMask(blurMask, maskpts)

            print("\t\t\t Added Aruco " + str(key) + " to the mask;")

        # Cross-Inpainting - For ArUcos not in the image
        for key in dataset_arucos.world_T_aruco.iterkeys():

            aruco_T_world = dataset_arucos.aruco_T_world[key]

            homogenous3DMask = np.ones(shape=(len(mask3DPoints), 4))

            homogenous3DMask[:, :-1] = mask3DPoints

            maskInNewCamera = np.zeros(shape=(len(mask3DPoints), 3))

            for k in range(len(mask3DPoints)):

                maskInNewCamera[k] = np.dot(world_T_camera, np.dot(aruco_T_world, homogenous3DMask[k]))[:3]

            # print("point in new camera = " + str(maskInNewCamera))

            # project 3D points to image plane
            ghostpts, jac = cv2.projectPoints(maskInNewCamera, (0,0,0), (0,0,0),
                                            dataset_arucos.intrinsics, dataset_arucos.distortion)

            ghostMask = drawMask(ghostMask, ghostpts)

        # Show the mask images
        # cv.namedWindow('cam mask ' + str(i), cv.WINDOW_NORMAL)
        # cv.imshow('cam mask ' + str(i), ghostMask)

        # Apply inpainting algorithm
        inpaintedImage = cv.inpaint(image, mask, 5, cv.INPAINT_TELEA)
        # inpaintedImage = cv.inpaint(image, mask, 5, cv.INPAINT_NS)

        # Show the masks over the original image
        # redImg = np.zeros(image.shape, image.dtype)
        # redImg[:, :] = (0, 0, 255)
        # redMask = cv2.bitwise_and(redImg, redImg, mask=mask)
        # redMaskImage = cv.addWeighted(redMask, 0.5, inpaintedImage, 0.5, 0.0)
        # cv.namedWindow('Red Mask image ' + str(i), cv.WINDOW_NORMAL)
        # cv.imshow('Red Mask image ' + str(i), redMaskImage)

        # Apply first blur to smooth inpainting
        blurredImage = cv.medianBlur(inpaintedImage, 201)
        inpaintedImage[np.where(mask == 255)] = blurredImage[np.where(mask == 255)]

        # Apply second blur to smooth out edges
        blurredImage = cv.medianBlur(inpaintedImage, 51)
        inpaintedImage[np.where(blurMask == 255)] = blurredImage[np.where(blurMask == 255)]

        # Show the blurred image
        # cv.namedWindow('Blurred image ' + str(i), cv.WINDOW_NORMAL)
        # cv.imshow('Blurred image ' + str(i), blurredImage)

        inpaintedImage = cv.bilateralFilter(inpaintedImage, 9, 75, 75)
        # inpaintedImage = cv.blur(inpaintedImage, (5, 5))

        # Show the final image
        cv.namedWindow('Inpainted image ' + str(i), cv.WINDOW_NORMAL)
        # cv.namedWindow('Inpainted image ' + str(i), cv.WINDOW_FULLSCREEN)
        cv.imshow('Inpainted image ' + str(i), inpaintedImage)

        ###################################################################
        # Show print .ply file with color

        # Read vertices from point cloud
        imgData = plyfile.PlyData.read(cloudFiles[i])["vertex"]
        numVertex = len(imgData['x'])

        # create array of 3d points                           add 1 to make homogeneous
        xyz = np.c_[imgData['x'], imgData['y'], imgData['z'], np.ones(shape=(imgData['z'].size, 1))]

        pointsInOpenCV = np.zeros(shape=(len(xyz), 3))

        pointColour = np.zeros(shape=(len(xyz), 3))

        print("#################################################")
        print("Camera " + camera.name + "\n")

        # The local point clouds (.ply files) are stored in OpenGL coordinates.
        # This matrix puts the coordinate frames back in OpenCV fashion
        camera.depth.matrix[0, :] = [1, 0, 0, 0]
        camera.depth.matrix[1, :] = [0, 0, 1, 0]
        camera.depth.matrix[2, :] = [0, -1, 0, 0]
        camera.depth.matrix[3, :] = [0, 0, 0, 1]

        world_T_camera = np.linalg.inv(camera.rgb.matrix)

        for j in range(len(xyz)):
            pointsInOpenCV[j] = np.dot(world_T_camera, np.dot(camera.depth.matrix, xyz[j]))[:3]

        # print("Points transformed from OpenGl to OpenCV coords = ")
        # print(pointsInOpenCV)

        # project 3D points from ArUco to image plane
        pointsInImage, jac = cv2.projectPoints(pointsInOpenCV, (0, 0, 0), (0, 0, 0),
                                               dataset_arucos.intrinsics, dataset_arucos.distortion)

        # print("Points projected to image = ")
        # print(pointsInImage)

        image = deepcopy(camera.rgb.image)

        # Figure out how many points project into the image
        nPointsWithColour = 0

        for j in range(len(pointsInImage)):
            row = int(round(pointsInImage[j][0][1]))
            col = int(round(pointsInImage[j][0][0]))

            # if it was projected within the image
            if 0 <= row < 1080 and 0 <= col < 1920:
                nPointsWithColour = nPointsWithColour + 1

        # Create the .ply file
        name = cloudFiles[i].split('/')[-1]
        name = name.split('.')[0]
        print(directory + '/' + name + '_with_colour.ply')
        file_object = open(directory + '/' + name + '_with_colour.ply', "w")

        # write file header information
        file_object.write('ply' + '\n')
        file_object.write('format ascii 1.0' + '\n')
        file_object.write('comment ---' + '\n')
        file_object.write('element vertex ' + str(nPointsWithColour) + '\n')
        file_object.write('property float x' + '\n')
        file_object.write('property float y' + '\n')
        file_object.write('property float z' + '\n')
        file_object.write('property uchar red' + '\n')
        file_object.write('property uchar green' + '\n')
        file_object.write('property uchar blue' + '\n')
        file_object.write('element face 0' + '\n')
        file_object.write('property list uchar uint vertex_indices' + '\n')
        file_object.write('end_header' + '\n')

        # Actually get the colours for the points projected
        for j in range(len(pointsInImage)):
            row = int(round(pointsInImage[j][0][1]))
            col = int(round(pointsInImage[j][0][0]))

            # if it was projected within the image
            if 0 <= row < 1080 and 0 <= col < 1920:
                pointColour[j] = inpaintedImage[row, col]

                file_object.write(str(imgData['x'][j]) + ' ' + str(imgData['y'][j]) + ' ' + str(imgData['z'][j]) +
                                  ' ' + str(int(pointColour[j][2])) + ' ' + str(int(pointColour[j][1])) +
                                  ' ' + str(int(pointColour[j][0])) + '\n')
        file_object.close()

    keyPressManager()
    cv.destroyAllWindows()