import cv2
import numpy as np

set1 = [[615, 309],
 [626, 292],
 [681, 329],
 [670, 346]]

set2 = [[259, 293],
 [276, 281],
 [281, 287],
 [264, 299]]

set3 = [[491, 271],
 [496, 251],
 [565, 267],
 [560, 287]]

set4 = [[343, 252],
 [418, 243],
 [421, 267],
 [345, 276]]

pts = [set1, set2, set3, set4]

img = np.zeros((cv2.imread("TarmacCenter3ft10in.png").shape[0],cv2.imread("TarmacCenter3ft10in.png").shape[1],3), dtype=np.uint8)

for pt_set in pts:
    for i in range(4):
        img = cv2.circle(img, pt_set[i], 3, (255, 255, 255), 3)


cv2.imshow("win", img)
cv2.waitKey(0)


