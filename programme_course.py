from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt
from rpi_hardware_pwm import HardwarePWM
import threading
# from luma.core.interface.serial import i2c
# from luma.core.render import canvas
# from luma.oled.device import sh1106
from gpiozero import LED, Button
# from pyPS4Controller.controller import Controller
from rpi_hardware_pwm import HardwarePWM
from stable_baselines3 import PPO
import smbus
import keyboard
import serial

bus = smbus.SMBus(1) 



#bp1 = Button("GPIO5")
#bp2 = Button("GPIO6")
#led1 = LED("GPIO17")
#led2 = LED("GPIO27")



def lecture_us_cm():
    try: 
        bus.write_byte_data(0x70, 0, 0x51)
        time.sleep(0.05)
        range1 = bus.read_byte_data(0x70, 2)
        range2 = bus.read_byte_data(0x70, 3)
        distance_cm = (range1 << 8) + range2
        if distance_cm < 1000:
            return distance_cm
        else:
            return 0
    except:
        return 0 
    

# serial = i2c(port=1, address=0x3C)
# device = sh1106(serial)
# 
# with canvas(device) as draw:
#     draw.rectangle(device.bounding_box, outline="white", fill="black")
#     draw.text((10, 10), "bp1 -> manette PS4", fill="white")
#     draw.text((10, 20), "manette X -> arret", fill="white")
#     draw.text((10, 35), "bp2 -> demo", fill="white")
#     draw.text((10, 45), "bp2 long pour arret", fill="white")

###################################################
#Intialisation des moteurs
##################################################

#################Constantes PWM###################


##############Démarrage des moteurs###############



###########Initialisation du tableau Lidar##########


#############Constante pour la manette PS4#################
#Classe de la voiture 
class Chassis():
    
    def __init__(self):
        
        #self.stop_prop = 8
        self.direction_prop = 1
        self.pwm_stop_prop = 7.42
        self.point_mort_prop = 0.4
        self.delta_pwm_max_prop = 1.4
        
        self.vitesse_max_m_s_hard = 10 #vitesse que peut atteindre la voiture
        self.vitesse_max_m_s_soft = 1.3 #vitesse maximale que l'on souhaite atteindre
        
        
        self.direction = 1
        self.angle_pwm_min = 6.5 #gauche
        self.angle_pwm_max = 8.9 #droite
        self.angle_degre_max = +12 #vers la gauche
        self.angle_pwm_centre= 7.75
        self.angle_degre=0
        self.port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=3.0)

            
        #Configuration des PWM
        self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
        
        self.pwm_prop.start(self.pwm_stop_prop)
        
        
        self.vitesse_consigne = 0
        self.direction_consigne =0
        
        
        
    def demarrage_voiture(self):
        print("demarrage voiture")
        #Démarrage des PWM     
        self.pwm_prop.start(self.pwm_stop_prop)
        time.sleep(1.0)
        self.pwm_prop.start(10)
        time.sleep(0.1)
        self.pwm_prop.start(5)
        time.sleep(0.1)
        print("test2")
        self.pwm_prop.start(self.pwm_stop_prop)
        self.pwm_dir.start(self.angle_pwm_centre)
        print
        self.vitesse_consigne = 0
        self.direction_consigne =0
        
        print("PWM activé")
        
            
    def get_direction(self,obs=False):
        if obs:
            return self.direction_consigne/self.angle_degre_max
        else:
            return self.direction_consigne
    
    def get_vitesse(self,obs=False):
        if obs: 
            return self.vitesse_consigne/self.vitesse_max_m_s_hard
        else:
            return self.vitesse_consigne
        
    def set_direction_degre(self,angle_degre,adresse=1) :
        #print("consigne in set_dir = ", angle_degre)
        self.direction_consigne = angle_degre
        if angle_degre>0:
            cmd = int(500 - angle_degre*8.9) # la relation direction / angle moteur n'est pas linéaire
        else :
            cmd = int(500 - angle_degre*11.1)
        if cmd > 610: # butée moteur
            cmd = 610
        elif cmd < 300:
            cmd = 300
        cmd_LSB = cmd & 0xFF
        cmd_MSB = (cmd & 0xFF00)>>8
        checksum = (~(39 + cmd_LSB + cmd_MSB))&0xff
        frame = bytearray(9)    
        frame[0] = np.uint8(255)
        frame[1] = np.uint8(255)
        frame[2] = np.uint8(adresse)
        frame[3] = np.uint8(5)
        frame[4] = np.uint8(3)
        frame[5] = np.uint8(30)
        frame[6] = np.uint8(cmd_LSB) #poids faible
        frame[7] = np.uint8(cmd_MSB) #poids fort
        frame[8] = np.uint8(checksum)
        self.port.write(frame)
    
    def set_vitesse_m_s(self,vitesse_m_s):
        if vitesse_m_s > self.vitesse_max_m_s_soft :
            vitesse_m_s = self.vitesse_max_m_s_soft
        elif vitesse_m_s < -self.vitesse_max_m_s_hard :
            vitesse_m_s = -self.vitesse_max_m_s_hard
        if vitesse_m_s == 0 :
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop)
        elif vitesse_m_s > 0 :
            vitesse = vitesse_m_s * (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop + self.direction_prop*(self.point_mort_prop + vitesse ))
        elif vitesse_m_s < 0 :
            vitesse = vitesse_m_s * (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop - self.direction_prop*(self.point_mort_prop - vitesse ))

            
        self.vitesse_consigne= vitesse_m_s
   
    def recule(self):
            # distance_arr=lecture_us_cm()
            # print("dist avant while = ", distance_arr)
            # debut = time.time()
            # temps = time.time()
            # while distance_arr<20 and temps-debut < 0.1:
            #     distance_arr=lecture_us_cm()
            #     print("dist dans while = ",distance_arr)
            #     temps = time.time()
            print("recule")
            self.set_vitesse_m_s(-self.vitesse_max_m_s_hard)
            time.sleep(0.2)
            self.set_vitesse_m_s(0)
            time.sleep(0.01)
            self.set_vitesse_m_s(-1)
            self.vitesse_consigne=0
            
    def arret_voiture(self):
        self.pwm_prop.stop()
        self.pwm_dir.stop()
        print("PWM arrêtées")



#Classe lidar
class Lidar_TT02():
    
    def __init__(self):
        
        #connexion et démarrage du lidar
        self.lidar = RPLidar("/dev/ttyUSB0",baudrate=256000)
        
        self.tableau_lidar_mm=np.zeros(360)
        self.acqui_lidar=np.zeros(360)
        
        self.drapeau_nouveau_scan = False
        self.scan_avant_en_cours = False
        self.Run_lidar = False
    
    def demarrage_lidar(self):
        self.lidar.connect()
        print (self.lidar.get_info())
        self.lidar.start_motor()
        time.sleep(2)
        
    def lidar_scan(self):
        while(self.Run_lidar == True):
            try:
                for _,_,angle_lidar,distance in self.lidar.iter_measures(scan_type='express'):
                    angle = min(359,max(0,359-int(angle_lidar)))
                    
                    if(angle >= 260) or (angle <= 100):
                        self.acqui_lidar[angle] = distance
                        
                        
                    if(angle<260) and (angle>110) and (self.scan_avant_en_cours == True):
                        self.drapeau_nouveau_scan = True
                        self.scan_avant_en_cours = False
                    if(angle >= 260) or (angle <= 100):
                        self.scan_avant_en_cours = True
                    if(self.Run_lidar == False):
                        break;
            except:
                print("souci acquisition Lidar")
                
    def get_values(self):
            for i in range (-100,101):
                self.tableau_lidar_mm[i] = self.acqui_lidar[i]
            self.acqui_lidar = np.zeros(360)
            
            for i in range(-98,99):
                if self.tableau_lidar_mm[i] == 0:
                    if self.tableau_lidar_mm[i-1] != 0 and self.tableau_lidar_mm[i+1] != 0:
                        self.tableau_lidar_mm[i] = (self.tableau_lidar_mm[i]+self.tableau_lidar_mm[i])/2
            self.drapeau_nouveau_scan =  False
            return self.tableau_lidar_mm
            
    def get_drapeau(self):
        return self.drapeau_nouveau_scan
    
    def set_drapeau(self,valeur):
        self.drapeau_nouveau_scan=valeur
          
    def get_run(self):
        return self.Run_lidar
    
    def set_run(self, valeur):
        self.Run_lidar = valeur
        
    def arret_lidar(self):
        self.lidar.stop_motor()
        self.lidar.stop()
        time.sleep(1)
        self.lidar.disconnect()
      
        

                             
            
            ###################################################
    
def conduite():
        global lidar
        global voiture
        print("début conduite")    
        tableau_lidar_mm = np.zeros(360)
        previous_lidar = np.zeros(360)
        while lidar.get_run() == True:
            if lidar.get_drapeau() == False:
                time.sleep(0.01)
            else:
                tableau_lidar_mm = lidar.get_values()
                for i in range(-99,100) :
                    if tableau_lidar_mm[i] == 0 :
                        tableau_lidar_mm[i] = tableau_lidar_mm[i-1]
                #print(tableau_lidar_mm)
                min_secteur = [0]*10
                               
                for index_secteur in range(0,10) :
                    angle_secteur = -90 + index_secteur*20
                    min_secteur[index_secteur] = 8000
                    for angle_lidar in range(angle_secteur-10,angle_secteur+10) :
                        if tableau_lidar_mm[angle_lidar] < min_secteur[index_secteur] and tableau_lidar_mm[angle_lidar] != 0 :
                            min_secteur[index_secteur] = tableau_lidar_mm[angle_lidar]
                    if min_secteur[index_secteur] == 8000 : #aucune valeur correcte
                        min_secteur[index_secteur] = 0
                #print("min_secteur[] :")
                #print(min_secteur)
                
                if (min_secteur[4]<=160 and min_secteur[4] !=0)\
                        or (min_secteur[3]<=160 and min_secteur[3] !=0)\
                            or (min_secteur[2]<=160 and min_secteur[2] !=0) :
                    angle_degre = -18
                    voiture.set_direction_degre(angle_degre)
                    print("mur à droite")
                    voiture.recule()
                    
                elif (min_secteur[5]<=160 and min_secteur[5] !=0)\
                        or (min_secteur[6]<=160 and min_secteur[6] !=0)\
                            or (min_secteur[7]<=160 and min_secteur[7] !=0) :
                    angle_degre = +18
                    voiture.set_direction_degre(angle_degre)
                    print("mur à gauche")
                    voiture.recule()
                    
                else :
                    
                    current_lidar=tableau_lidar_mm.astype("float64")/8000
                    previous_speed=np.array([float(voiture.get_vitesse(obs=True))])
                    previous_angle=np.array([float(voiture.get_direction(obs=True))])
                    if previous_angle[0] > 1 :
                        previous_angle[0] = 1
                    elif previous_angle[0] < -1 :
                        previous_angle[0] = -1
                    
                    
                    
                    obs={"current_lidar":current_lidar,
                         "previous_lidar":previous_lidar,
                         "previous_speed":previous_speed,
                         "previous_angle":previous_angle,
                         }
                    print("angle_obs = ",previous_angle)
                    
                    action,_= modele.predict(obs,deterministic=True)
                    angle=float(voiture.get_direction(obs=True))+action[1]*18.0
                    #print("angle = ",angle)
                    
                    voiture.set_direction_degre(angle)
                    nouvelle_vitesse = float(voiture.get_vitesse())+action[0]
                    if (nouvelle_vitesse <0.1) :
                        vitesse_m_s = 0.1
                    else : 
                        vitesse_m_s = nouvelle_vitesse
                    print("vitesse  = ", vitesse_m_s)
                    voiture.set_vitesse_m_s(vitesse_m_s)
        
    
lidar= Lidar_TT02()    
voiture = Chassis()

#modele= PPO.load("/home/voitureBasile/Documents/PPO_results_12022024hugo.zip")
#modele= PPO.load("/home/voitureBasile/Documents/version_basile/PPO_results_25032024_4_train2_9.zip")
#modele= PPO.load("/home/voitureBasile/Documents/version_basile/PPO_results_27032024_train4.zip")
modele= PPO.load("/home/voitureBasile/Documents/version_basile/ter-covapsy_3003_train3.zip")


print("Chargé")

# voiture.set_vitesse_m_s(3)
# time.sleep(3)
# voiture.set_vitesse_m_s(1)
# time.sleep(3)
# voiture.recule()
# time.sleep(1)
# voiture.set_vitesse_m_s(0)

while True :
    
#     while (bp1.is_pressed) or (bp2.is_pressed) :
#         pass
#     
#     while (not bp1.is_pressed) and (not bp2.is_pressed) :
#         led1.on()
#         time.sleep(0.25)
#         led1.off()
#         time.sleep(0.25)
        
    print("Appuyer sur une touche 'm' pour commencer: ")
    print(" 't' : programme test")
    print(" 'c' : programme course")
    
    x = input("Appuyer sur une touche pour commencer: ")
    while True:
        
        if x=='t' :
           
            #connexion et démarrage du lidar
            print("début test")
            voiture.demarrage_voiture()
            time.sleep(1)
            voiture.set_vitesse_m_s(1)
            time.sleep(1)
    #         voiture.set_direction_degre(-18)
    #         time.sleep(1)
    #         voiture.set_direction_degre(18)

            voiture.recule()
            time.sleep(1)
            voiture.arret_voiture()
            print("fin test")


        if x=="c":
           
            #connexion et démarrage du lidar
            lidar.demarrage_lidar()
            lidar.set_run(True)
            print("lidar lancée")
            
            voiture.demarrage_voiture()
            print("voiture lancée")

            thread_scan_lidar = threading.Thread(target=lidar.lidar_scan)
            thread_scan_lidar.start()
            time.sleep(1)
            
            print("thread lidar lancée")
            
            x = input("Appuyer sur une touche 'g' pour commencer la course: ")
            if x=='g':

                thread_conduite_autonome = threading.Thread(target = conduite)
                #print("thread configurée")
                thread_conduite_autonome.start()
                #print("thread conduite lancée")

            while True :
                try :
                    
                    x=input("Appuyer sur 'a' pour arrêter la voiture\n")
                    time.sleep(1)
                    if x=='a' :
                        lidar.set_run(False)
                        voiture.set_vitesse_m_s(0)
                        break
                except KeyboardInterrupt: #récupération du CTRL+C
                    print("arrêt du programme")
                    lidar.set_run(False)
                    voiture.set_vitesse_m_s(0)
                    break
            
        #thread_conduite_autonome.join()
            thread_scan_lidar.join()
            thread_conduite_autonome.join()



            #arrêt et déconnexion du lidar

            lidar.arret_lidar()
            voiture.arret_voiture()

        # #affichage des données acquises sur l'environnement
        # print(len(tableau_lidar_mm))
        # print(tableau_lidar_mm)
        # teta = [0]*360 #création d'un tableau de 360 zéros
        # for i in range(360) :
        #     teta[i]=i*np.pi/180
        # fig = plt.figure()
        # ax = plt.subplot(111, projection='polar')
        # line = ax.scatter(teta, tableau_lidar_mm, s=5)
        # line.set_array(tableau_lidar_mm)
        # ax.set_rmax(3000)
        # ax.grid(True)
        # plt.show()
    
        else:
            x=input("erreur, recommencez")
#         controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
#         try :
#             controller.listen()
#         except :
#             "fin de l'usage de la manette bp1 pour recommencer"
