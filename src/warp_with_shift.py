import sys
import cv2
import numpy as np
import math

def sq(x):
    return x * x

def getRotationMat(H, fx, fy):
    # Flip Homography for convention sake
    F = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    H = np.dot(H, F)

    M = np.zeros((4, 4))
    M[0][0] = H[0][0] / fx
    M[0][1] = H[0][1] / fx
    M[0][3] = H[0][2] / fx
    M[1][0] = H[1][0] / fy
    M[1][1] = H[1][1] / fy
    M[1][3] = H[1][1] / fy
    M[2][0] = H[2][0]
    M[2][1] = H[2][1]
    M[2][3] = H[2][2]

    sf1 = math.sqrt(sq(M[0][0]) + sq(M[1][0] / fy) + sq(M[2][0]));

    sf2 = math.sqrt(sq(M[0][1]) + sq(M[1][1]) + sq(M[2][1]));

    scale_factor = math.sqrt(sf1 * sf2);

    M = M/scale_factor

    if M[2][3] > 0:
        M = M * -1;

    M[3][0] = 0
    M[3][1] = 0
    M[3][2] = 0
    M[3][3] = 1

    M[:3, 2] = np.cross(M[:3, 0], M[:3, 1])

    R = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            R[i][j] = M[i][j]

    U, s, V = np.linalg.svd(R)
    MR = np.dot(U, V)

    return np.dot(MR, F)

# Main script
im = cv2.imread(sys.argv[1])

H = np.loadtxt(sys.argv[2])
H = np.reshape(H, (3, 3))

tag_center = np.loadtxt(sys.argv[3])

tag_coords = np.loadtxt(sys.argv[4])
tag_coords = np.reshape(tag_coords, (4, 2))

tag_width = abs(tag_coords[0][0] - tag_coords[1][0])
tag_height = abs(tag_coords[0][1] - tag_coords[2][1])

imheight = im.shape[0]
imwidth = im.shape[1]

# Principle points
py = imheight / 2
px = imwidth / 2

# focal length
fx = 762
fy = 771

# warp_im = np.zeros(im.shape)

# x and y scale factors
sx = tag_width / 2.0
sy = tag_height / 2.0

#Scale matrix
Sc = np.array([[1/sx, 0, 0],[0, 1/sy, 0], [0, 0, 1]])
Tr = np.array([[1, 0, -tag_center[0]], [0, 1, -tag_center[1]], [0, 0, 1]])

Hinv = np.linalg.inv(H)

H = np.dot(H, Sc)
H = np.dot(H, Tr)

#Hinv = np.dot(Tr, Hinv)
#Hinv = np.dot(np.linalg.inv(Sc), Hinv)

warp_im = cv2.warpPerspective(im, H, (imwidth, imheight), flags=cv2.WARP_INVERSE_MAP)
#warp_im = cv2.warpPerspective(im, Hinv, (imwidth, imheight))#, flags=cv2.WARP_INVERSE_MAP)

# Rotation matrix
R = getRotationMat(H, 762, 771)
# Camera Matrix K
K = np.array([[2.4974e+03, 0., 1.6315e+03],
            [0., 2.4974e+03, 1.2235e+03],
            [0., 0., 1.]])
# Rotate image by KRKe-1
#Rfinal = np.dot(K, R)
#Rfinal = np.dot(Rfinal, np.linalg.inv(K))
#warp_im2 = cv2.warpPerspective(im, Rfinal, (2 * imwidth, 2 * imheight), flags=cv2.WARP_INVERSE_MAP)

#for j in range(imheight):
#    for i in range(imwidth):
#        x = np.dot(Hinv, [i, j, 1])
#        x = x / x[2]
#        # Multiply by scale and translate to center of apriltag
#        x[0] = (x[0] * sx)# + tag_center[0]
#        x[1] = (x[1] * sy)# + tag_center[1]
#        x = x.astype(int)
#        if (x[0] > 0) and (x[1] > 0) and (x[0] < imwidth) and (x[1] < imheight):
#            warp_im[x[1]][x[0]] = im[j][i]
#
#imclr = cv2.imread('src_157.jpg')
#warp_im = cv2.warpPerspective(im, R, (imwidth, imheight))
cv2.imwrite(sys.argv[5], warp_im)
#cv2.imwrite('warp_im2.png', warp_im2)
