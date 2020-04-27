#!/usr/bin/python
#ETN-1040
#Proyecto de grado
#Juan Pablo Crespo Vargas
#2015
#Facultad de Ingenieria
#Universidad Mayor de San Andres 

#Importamos todas las librerias en python
import numpy as np
import sys  
import cv2.cv as cv
import smbus
import time
from  optparse import OptionParser


#Inicializamos el Bus en 1 (Rpi2)
bus = smbus.SMBus(1)
#Direccion ucontrolador predeterminada
address = 0x04

#Inicializamos variables de imagen
min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0

#funcion para enviar datos en i2c
def sendData(value):
    bus.write_byte(address, value)
    return -1

#funcion para recibir datos en i2c
def readData():
    state = bus.read_byte(address)
    return state

#funcion para la deteccion y encuadre del rostro
def detect_and_draw(img, cascade):
# Acondicionamiento de la imagen, imagenes temporales
    gray = cv.CreateImage((img.width,img.height), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
			    cv.Round (img.height / image_scale)), 8, 1)
# Crea la mascara
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
#Escala la imagen para procesarla homogeneamente
#Aplicacion de la Imagen Integral
    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
    cv.EqualizeHist(small_img, small_img)
# PROcesamiento de la Imagen
    if(cascade):
        t = cv.GetTickCount()
        faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        print "Tiempo de la deteccion = %gms" % (t/(cv.GetTickFrequency()*1000.))
	if faces:
            for ((x, y, w, h), n) in faces:
#calcula los puntos de deteccion
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
		posx = int(((x+(x+w))/2)* image_scale)
 		posy = int(((y+(y+h))/2)* image_scale)
#dibuja un circulo en el centro de la imagen y otro en el centro del rostro
		cv.Circle(img, (80,60), 5, cv.RGB(0,0,255), 1, 8, 0)
		cv.Circle(img, (posx, posy), 5, cv.RGB(0,255,0), 1, 8, 0)
#si el estado esta libre enviamos datos de posicion por i2c
		state = readData()
		time.sleep(0.005)
		if state == 1:
			sendData(posx)
            		sendData(posy)
		print 'posx: ' + str(posx) + ' posy: ' + str(posy)

	cv.ShowImage("video", img)


if __name__ == '__main__':
    #Entrada de datos del entrenador face.xml desde consola, cascada de Haars
    parser = OptionParser(usage = "usage: %prog [options] [filename|camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "../data/haarcascades/haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()

    cascade = cv.Load(options.cascade)

    if len(args) != 1:
        parser.print_help()
        sys.exit(1)

    input_name = args[0]

    if input_name.isdigit():
        capture = cv.CreateCameraCapture(int(input_name))

    else:
        capture = None

    cv.NamedWindow("video", 1)

#Tamano de la imagen de Video

    width = 160
    height = 120

    if width is None:
    	width = int(cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_WIDTH))
    else:
    	cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_WIDTH,width)

    if height is None:
	height = int(cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_HEIGHT))
    else:
	cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_HEIGHT,height)
#CAPTURAR FRAME POR IMAGEN DE ENTRADA
    if capture:
        frame_copy = None
        while True:

            frame = cv.QueryFrame(capture)
            if not frame:
                cv.WaitKey(0)
                break
            if not frame_copy:
                frame_copy = cv.CreateImage((frame.width,frame.height),
                                            cv.IPL_DEPTH_8U, frame.nChannels)

            if frame.origin == cv.IPL_ORIGIN_TL:
                cv.Copy(frame, frame_copy)
            else:
                cv.Flip(frame, frame_copy, 0)
#Funcion de analizar la imagen
            detect_and_draw(frame_copy, cascade)

#Salida del programa            
            if cv.WaitKey(10) >= 0:
                break
    else:
        
        image = cv.LoadImage(input_name, 1)
        detect_and_draw(image, cascade)
        cv.WaitKey(0)
    cv.DestroyWindow("video")


