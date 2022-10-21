import cv2
import numpy as np
import yaml

window_name = "Color Filter"
img = cv2.imread("TarmacCenter3ft10in.png")  # replace this with video feed
ellipse_pts_path = "ellipse_pts.txt"

try:

    with open('params.yaml') as f:
        dataMap = yaml.safe_load(f)
    lower_bound = np.array(dataMap["lowThresh"])
    upper_bound = np.array(dataMap["highThresh"])
    blur = dataMap["blur"]
    assert(lower_bound is not None and upper_bound is not None and blur is not None)
except Exception as e:
    print(e)
    lower_bound = [67, 227, 22]
    upper_bound = [93, 255, 255]
    blur = 5

overlay_opacity = 0.5

def lH_change(val):
    lower_bound[0] = val
def lS_change(val):
    lower_bound[1] = val
def lV_change(val):
    lower_bound[2] = val
def hH_change(val):
    upper_bound[0] = val
def hS_change(val):
    upper_bound[1] = val
def hV_change(val):
    upper_bound[2] = val
def blur_change(val):
    global blur
    blur = val;

cv2.imshow(window_name, img)

cv2.createTrackbar('lH', window_name, 0, 255, lH_change)
cv2.setTrackbarPos('lH', window_name, lower_bound[0])
cv2.createTrackbar('lS', window_name, 0, 255, lS_change)
cv2.setTrackbarPos('lS', window_name, lower_bound[1])
cv2.createTrackbar('lV', window_name, 0, 255, lV_change)
cv2.setTrackbarPos('lV', window_name, lower_bound[2])
cv2.createTrackbar('hH', window_name, 0, 255, hH_change)
cv2.setTrackbarPos('hH', window_name, upper_bound[0])
cv2.createTrackbar('hS', window_name, 0, 255, hS_change)
cv2.setTrackbarPos('hS', window_name, upper_bound[1])
cv2.createTrackbar('hV', window_name, 0, 255, hV_change)
cv2.setTrackbarPos('hV', window_name, upper_bound[2])
cv2.createTrackbar('blur', window_name, 0, 25, blur_change)
cv2.setTrackbarPos('blur', window_name, blur)

frame = None
while(1):
    if blur > 0:
        frame = cv2.blur(img.copy(), (blur, blur,))
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), np.array(lower_bound), np.array(upper_bound))
    # res = cv2.bitwise_and(cv2.cvtColor(frame, cv2.COLOR_HSV2RGB), mask=cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ellipse_pts = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # ellipse_pts.append((int(rect[0][0]), int(rect[0][1]+rect[1][1]/2),))
        # ellipse_pts.append((int(rect[0][0] + rect[1][0]), int(rect[0][1]+rect[1][1]/2),))

        ellipse_pts.append((int(rect[0][0]), int(rect[0][1]),))
        print(box)
        f = open(ellipse_pts_path, "a")
        f.write(str(box))
        f.close()
        # ellipse_pts.append((int(rect[0][0]), int(rect[0][1]),))
        cv2.drawContours(frame,[box],0,(0,0,255),2)
    
    cv2.drawContours(frame, contours, -1, (255, 255, 255), 2)
    for pt in ellipse_pts:
        cv2.circle(frame, pt, 5, (255, 255, 0), 5)

    cv2.imshow(window_name, cv2.addWeighted(frame, overlay_opacity, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), cv2.COLOR_HSV2RGB, 1-overlay_opacity, 0))
    key = cv2.waitKey(33)

    if key == ord('s'):
        if input("Would you like to save | yn: ") == "y":
            dicti = {}
            dicti['blur'] = blur
            dicti['lowThresh'] = np.array(lower_bound).tolist()
            dicti['highThresh'] = np.array(upper_bound).tolist()
            with open('params.yaml', "w") as f:
                yaml.dump(dicti, f)
            break
    elif key == ord('a'):
        overlay_opacity = max(min(overlay_opacity - 0.1, 1), 0)
    elif key == ord('d'):
        overlay_opacity = max(min(overlay_opacity + 0.1, 1), 0)


cv2.destroyAllWindows()
