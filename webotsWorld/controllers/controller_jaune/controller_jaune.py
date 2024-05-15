# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# août 2023

from vehicle import Driver
from controller import Lidar
import numpy as np
import time

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

#Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud() 

# vitesse en km/h
speed = 0
maxSpeed = 28 #km/h

# angle de la direction
angle = 0
maxangle_degre = 16


# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tableau_lidar_mm=[0]*360

def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s*3.6
    if speed > maxSpeed :
        speed = maxSpeed
    if speed < 0 :
        speed = 0
    driver.setCruisingSpeed(speed)
     
def set_direction_degre(angle_degre):
    if angle_degre > maxangle_degre:
        angle_degre = maxangle_degre
    elif angle_degre < -maxangle_degre:
        angle_degre = -maxangle_degre   
    angle = -angle_degre * 3.14/180
    driver.setSteeringAngle(angle)

def recule(): #sur la voiture réelle, il y a un stop puis un recul pendant 1s.
    driver.setCruisingSpeed(-1)  

# Main loop
while driver.step() != -1:
	# Get current steering angle and Lidar results
            donnees_lidar_brutes = lidar.getRangeImage()
            for i in range(360) :
                if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<20) :
                    tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
            else :
                tableau_lidar_mm[i] = 0
                
	# Set speed and angle
            angle_degre = 0.02*(tableau_lidar_mm[60]-tableau_lidar_mm[-60])
            set_direction_degre(angle_degre)
            vitesse_m_s = 0.5
            set_vitesse_m_s(vitesse_m_s)
