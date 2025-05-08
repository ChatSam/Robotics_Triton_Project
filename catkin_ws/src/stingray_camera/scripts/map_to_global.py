import rospy
import cv2
from matplotlib import pyplot as plt

if __name__ == "__main__":
    rospy.init_node("map_to_global")

    print("x")
    # read in map
    map_raw = cv2.imread("../maps/map.pgm")
    print(map_raw[:10, :10])
    w = map_raw.shape[0]
    h = map_raw.shape[1]

    center = (w//2, h//2)
    print(center)

    # original corner value
    sample_gray = map_raw[0, 0]
    print(sample_gray)

    # set all black values to a little less than black
    map_raw[map_raw == 0] = 1

    # draw dot on center
    # cv2.circle(map_raw, center, 5, (0, 0, 255), -1)

    # cv2.imshow("raw map", map_raw)
    # cv2.waitKey(0)

    angle = -6

    M = cv2.getRotationMatrix2D(center, angle, scale=1.0)
    rotated = cv2.warpAffine(map_raw, M, (w, h))

    # set all black values to a sample_gray
    rotated[rotated == 0] = sample_gray[0]
    # set all almost black back to black
    rotated[rotated == 1] = 0


    # cv2.imshow("rotated map", rotated)
    # cv2.waitKey(0)

    # make sure map is still gray
    rotated = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)

    # save rotated map
    cv2.imwrite("../maps/map_rotated.pgm", rotated)
