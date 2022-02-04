import cv2 as cv
import numpy as np
import math

crop_img = 0
background_image = 0
xi, xf = 0, 0
yi, yf = 0, 0
drawing = False
close_paint = False

FLANN_INDEX_KDTREE = 1
FLANN_INDEX_LSH    = 6
flann_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2

def paint(event,x,y,flags,params):
    global xi, xf, yi, yf, drawing, close_paint

    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        xi,yi = x,y
    elif event == cv.EVENT_MOUSEMOVE and drawing == True:
        if drawing == True:
            background_image[:] = 0
            cv.rectangle(background_image, (xi,yi), (x,y), (0,0,255), 3)
            xf,yf = x,y
    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        close_paint = True


def crop_image():
    global close_paint, background_image, crop_img

    close_paint, drawing = False, False
    video = cv.VideoCapture(0)
    background_image = np.zeros((int(video.get(cv.CAP_PROP_FRAME_HEIGHT)) , int(video.get(cv.CAP_PROP_FRAME_WIDTH)), 3), dtype=np.uint8)
    while True:
        _, frame = video.read()
        #frame = cv.flip(frame, 1)
        cv.namedWindow("homography")
        cv.setMouseCallback("homography", paint)
        frame_crop = cv.addWeighted(frame, 1, background_image, 1, 0)
        cv.imshow("homography", frame_crop)
        # print("xi = ", xi, "xf = ", xf, "yi = ", yi, "yf = ", yf)
        stop_key = cv.waitKey(1)
        if stop_key == 27 or close_paint == True:
            if xi != xf and yi != yf:
                crop_img = frame[yi:yf, xi:xf]
                video.release()
                break
    try:
        # cv.imshow("correcta", crop_img)
        print("imagen seleccionada correcta")
    except:
        print("error en la imagen seleccionada")
    # cv.waitKey(0)


def homography():
    video = cv.VideoCapture(0)


    crop_img_gray = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY) # convierte la imagen de color, a blanco y negro
    # cv.imshow("keyimg", crop_img_gray)
    # cv.waitKey(0)
    orb = cv.ORB_create(nfeatures = 1000) # crea un obejto de tipo sift para hacer la localizacion de los puntos de interes
    kp_image, desc_image = orb.detectAndCompute(crop_img_gray, None) # encuentra los puntos de interes y los procesa para propocionar las coordenadas de cada uno de ellos
    # print("kp_image = ",kp_image)
    # cv.waitKey(0)
    # print("desc_image = ", desc_image)
    # cv.waitKey(0)
    keyimg = cv.drawKeypoints(crop_img_gray, kp_image, crop_img_gray) # dibuja cada punto de interes, sobre la imgen que se desea
    # cv.imshow("keyimg",keyimg)
    # cv.waitKey(0)
    # index_params = dict(algorithm = 0, trees = 5) # crea un diccionario
    # searh_params = dict() # crea un segundo diccionario
    flann = cv.FlannBasedMatcher(flann_params, {}) # Fast Library for Approximate Nearest Neighbors

    while True:
        _, frame = video.read() # leemmos el video de la camara web
        #frame = cv.flip(frame, 1) # aplicamos efecto espejo del video de la camara

        grayframe = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # aplicamos el metodo para cambiarlo a blaco y negro

        kp_grayframe, desc_grayframe = orb.detectAndCompute(grayframe, None) # detectamos los puntos de interes y los procesamos
        keygrayframe = cv.drawKeypoints(grayframe, kp_grayframe, grayframe) # dibujamos los puntos de interes

        matches = flann.knnMatch(desc_image, desc_grayframe, k = 2) # encuentra las mejores coincidecias

        good_point = [] # crea una lista
        try:
            for m, n in matches: #
                if m.distance < 0.6*n.distance: # distancia entre los puntos de interes, entre mas cercanos sean mejor va a hacer la coincidencia
                    good_point.append(m) # agrega paraetros al arreglo good_point
        except:
            print("error en la deteccion de imagen")

        kinect_intrinsic_param = np.array([[514.04093664, 0., 320], [0., 514.87476583, 240], [0., 0., 1.]])
        kinect_distortion_param = np.array([2.68661165e-01, -1.31720458e+00, -3.22098653e-03, -1.11578383e-03, 2.44470018e+00])
        if len(good_point) > 10:
            # print(len(good_point))
            query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_point]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_point]).reshape(-1, 1, 2)

            matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            h, w = crop_img_gray.shape
            pts = np.float32([[0, 0],[0, h],[w, h], [w, 0]]).reshape(-1, 1, 2)

            dst = cv.perspectiveTransform(pts, matrix)


            p4x = np.int32(dst[3 ,0 ,0]); p4y = np.int32(dst[3 ,0 ,1])
            p3x = np.int32(dst[2 ,0 ,0]); p3y = np.int32(dst[2 ,0 ,1])
            p2x = np.int32(dst[1 ,0 ,0]); p2y = np.int32(dst[1 ,0 ,1])
            p1x = np.int32(dst[0 ,0 ,0]); p1y = np.int32(dst[0 ,0 ,1])

            c_disp_x = np.int32(video.get(3)/2)
            c_disp_y = np.int32(video.get(4)/2)
            center_disp = (c_disp_x, c_disp_y)


            p4 = (p4x, p4y)
            p3 = (p3x, p3y)
            p2 = (p2x, p2y)
            p1 = (p1x, p1y)

            c_track_x = p1x + (np.int32(p3x-p1x)/2)
            c_track_y = p1y + (np.int32(p3y-p1y)/2)

            center_track = (c_track_x, c_track_y)

            homography = cv.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            homography = cv.circle(homography, center_track, 10, (255, 255, 255), -1)
            homography = cv.circle(homography, center_disp, 10, (0, 0, 255), -1)
            homography = cv.line(homography, center_disp, center_track, (255, 0, 0), 2)
            print(c_disp_x - c_track_x, c_disp_y - c_track_y)
            cv.imshow("homography",homography)
        else:
            cv.imshow("homography",frame)


        stop_key = cv.waitKey(1)
        if stop_key == 27:
            video.release()
            break


def main():
    crop_image()
    homography()


if __name__ == "__main__":

    main()
    print("homography closed")
    cv.destroyAllWindows()
