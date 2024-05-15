# Python imports
import random
import time
import struct

# Webots imports
from controller import Supervisor

# Global constants
PI = 3.141592653589793
RECEIVER_SAMPLING_PERIOD = 64	# In milliseconds

# /!\ Set the number of sparring partner cars
NB_SPARRINGPARTNER_CARS = 3

# Clipping function
def value_clip(x, low, up):
	return low if x < low else up if x > up else x

# Angle normalization function (Webots angle values range between -pi and pi)
def angle_clip(x):
	a = x % (2*PI)
	return a if a < PI else a - 2*PI

def add_random_obstacle(supervisor, position, rotation, obstacle_name="SimpleObstacle"):
        root_node = supervisor.getRoot()
        children_field = root_node.getField('children')
        
        
        # Randomize position and size
        x,y = position
        z = 0.1
        theta = rotation
        size = 0.3  # Fixed size as per your requirement
        
        # Create a PROTO string with random parameters
        proto_string = f'SimpleObstacle {{ translation {x} {y} {z} rotation {0} {0} {1} {theta} size {size} {size} {size} name "{obstacle_name}"}}'
        
        # Add the PROTO instance to the simulation
        children_field.importMFNodeFromString(-1, proto_string)

def clear_obstacles(supervisor):
        root_node = supervisor.getRoot()
        children_field = root_node.getField('children')
        for i in reversed(range(children_field.getCount())):
            node = children_field.getMFNode(i)
            if node.getTypeName() == "SimpleObstacle":
                children_field.removeMF(i)

def send_time_message():
    message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time)
    LapEmitter.send(message)

# Positions intiales aléatoires 
# (plusieurs positions initiales auxquelles s'ajoute un peu d'aléatoire)
# 	[[x_min, x_max], [y_min, y_max], rotation]
# Les angles correspondents au même sens de rotation pour les voitures

"""
# Train track
starting_positions = [
 	[[ 0.00,  4.00], [ 5.10,  5.30],  0.00],
 	[[ 4.95,  5.15], [ 0.30,  4.00], -PI/2],
 	[[ 2.70,  4.50], [-1.25, -0.75],  PI  ],
 	[[ 3.20,  3.45], [ 2.00,  2.40],  PI/2],
 	[[ 1.90,  2.70], [ 3.10,  3.30],  PI  ],
 	[[ 0.00,  0.25], [-0.50,  1.50], -PI/2],
 	[[-2.20, -1.90], [-0.50,  2.30],  PI/2]
]
"""

# Test track
starting_positions = [
	[[ 0.60,  4.20], [ 5.40,  5.60],  0.00],
	[[ 5.40,  5.60], [-4.30,  4.30], -PI/2],
	[[ 1.40,  1.60], [-5.00, -4.30],  PI/2],
	[[ 3.40,  3.60], [-2.85, -1.45],  PI/2],
	[[ 3.40,  3.60], [ 2.20,  2.50],  PI/2],
	[[-0.60, -0.40], [ 0.30,  0.90], -PI/2],
	[[-3.20, -2.00], [-4.60, -4.40],  PI],
	[[-2.60, -2.40], [-0.80,  2.30],  PI/2],
]

obstacles_positions = [
    [-2.21, 0.5, 0],
    [-2.3, 3.82, 0.262],
    [-0.2, 4.6, -0.262],
    [2.7, 5.22, 0],
    [0.8, 5.8, 0],
    [3.5, 5.8, 0],
    [5.04, 4.96, -0.785],
    [5.24, 2.95, 0],
    [5.2, -3, 0],
    [5.79, -1.56, 0],
    [5.08, -4.93, -0.524],
    [2.56, -5.2, 0],
    [3.84, -5.78, 0],
    [1.82, -4.31, -0.262],
    [3.21, -1.87, 0],
    [3.75, -1, 0],
    [2.94, 0, -0.785],
    [2.94, 1, -0.785],
    [2.9, 1.9, 0.5],
    [2.9, 3.1, -0.555],
    [2, 3, -0.785],
    [0.895, 1.84, -0.524],
    [-0.05, 1.95, -0.785],
    [-0.951, -3.95, 0.785],
    [-4, -4, 0.785],
    [-4.06, -2.04, -2.36]
]

obstacles_rotations = [
    0,
    0,
    -0.262,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    -0.785,
    -0.785,
    0.5,
    -0.555,
    -0.785,
    0,
    -0.785,
    0,
    0,
    0
]
       



#### Initialisation ####
supervisor = Supervisor()
basicTimeStep = int(supervisor.getBasicTimeStep())
print("basic time step : "+str(basicTimeStep) + "ms")
add_obstacle = True
print("add obstacle : ", add_obstacle)
random_direction = True
print("random direction : ", random_direction)

# Reset Receiver et emitter
ResetReceiver = supervisor.getDevice("ResetReceiver")
ResetReceiver.enable(RECEIVER_SAMPLING_PERIOD)
ResetReceiver.setChannel(1)
ResetEmitter = supervisor.getDevice("ResetEmitter")
ResetEmitter.setChannel(2)
packet_number = 0
line1_t, line2_t, line3_t = 0, 0, 0
lap_time, sector1_t, sector2_t, sector3_t = 0, 0, 0, 0

# Lap and sector times emitter
LapEmitter = supervisor.getDevice("LapEmitter")
LapEmitter.setChannel(3)

# Initialize crossing-line times
line1_t, line2_t, line3_t = 0,0,0
direction = 1

# Recuperation des liens vers les noeuds voitures
tt_02 = supervisor.getFromDef("TT02_2023b_RL")
tt_02_translation = tt_02.getField("translation")
tt_02_rotation = tt_02.getField("rotation")

# Create a struct (Python object) for the TT-02 car node
tt_02.sector1_t = 0
tt_02.sector2_t = 0
tt_02.sector3_t = 0
tt_02.lap_time = 0
tt_02.nb_sectors1_completed = 0
tt_02.nb_sectors2_completed = 0
tt_02.nb_sectors3_completed = 0
tt_02.nb_laps_completed = 0
tt_02.best_lap_time = 0
tt_02.best_sector1_t = 0
tt_02.best_sector2_t = 0
tt_02.best_sector3_t = 0


sparringpartner_car_nodes = [supervisor.getFromDef(f"sparringpartner_car_{i}") for i in range(1,NB_SPARRINGPARTNER_CARS+1)]
sparringpartner_car_translation_fields = [sparringpartner_car_nodes[i].getField("translation") for i in range(NB_SPARRINGPARTNER_CARS)]
sparringpartner_car_rotation_fields = [sparringpartner_car_nodes[i].getField("rotation") for i in range(NB_SPARRINGPARTNER_CARS)]

# Clear obstacles
clear_obstacles(supervisor)

erreur_position = 0
# Main loop
while supervisor.step(basicTimeStep) != -1:
        # Extraire les valeurs de position pour éviter les appels multiples
    x, y = tt_02_translation.getSFVec3f()[:2]
    current_time = supervisor.getTime()

    # Définition des positions des lignes
    is_line1_crossed = 4.8 < x < 6.2 and 1.95 < y < 2
    is_line2_crossed = -3.5 < x < -1.5 and 1.25 < y < 1.3
    is_line3_crossed = 2.7 < x < 4 and 2.15 < y < 2.2

    if direction==1:
        # Ligne 1
        if is_line1_crossed:
            if abs(line1_t - current_time) >1: # Pour éviter de franchir la ligne plusieurs fois en restant dans la zone
                print("line 1 crossed")
                if line3_t!=0 : 
                    tt_02.sector3_t = current_time - line3_t
                    if tt_02.best_sector3_t == 0 or tt_02.sector3_t < tt_02.best_sector3_t:
                        tt_02.best_sector3_t = tt_02.sector3_t
                    print("sector 3 time : ", tt_02.sector3_t)
                if line1_t!= 0 : # Si on a passe la ligne il y a plus d'une sec
                    tt_02.lap_time = current_time - line1_t
                    if tt_02.best_lap_time == 0 or tt_02.lap_time < tt_02.best_lap_time:
                        tt_02.best_lap_time = tt_02.lap_time
                    print("lap time : ", tt_02.lap_time)
                send_time_message()
                line1_t = current_time
                line2_t, line3_t = 0, 0 # A chaque tour on reset les temps des secteurs
        # Ligne 2
        elif is_line2_crossed:
            if abs(line2_t - current_time) >1:
                if line1_t!=0:
                    tt_02.sector1_t = current_time - line1_t
                    if tt_02.best_sector1_t == 0 or tt_02.sector1_t < tt_02.best_sector1_t:
                        tt_02.best_sector1_t = tt_02.sector1_t
                    send_time_message()
                    print("sector 1 time = ", tt_02.sector1_t)
                line2_t = current_time      
        # Ligne 3
        elif is_line3_crossed:
            if abs(line3_t - current_time) >1:
                print("line 3 crossed")
                if line2_t!=0 :
                    tt_02.sector2_t = current_time - line2_t
                    if tt_02.best_sector2_t == 0 or tt_02.sector2_t < tt_02.best_sector2_t:
                        tt_02.best_sector2_t = tt_02.sector2_t
                    send_time_message()
                    print("sector 2 time = ", tt_02.sector2_t)
                line3_t = current_time
        
    if direction==0:
        # Ligne 1
        if is_line1_crossed:
            if abs(line1_t - current_time) >1:
                print("line 1 crossed")
                if line2_t!=0 : 
                    tt_02.sector1_t = current_time - line2_t
                    if tt_02.best_sector1_t == 0 or tt_02.sector1_t < tt_02.best_sector1_t:
                        tt_02.best_sector1_t = tt_02.sector1_t
                    print("sector 1 time : ", tt_02.sector1_t)
                if line1_t!= 0 : # Si on a passe la ligne il y a plus d'une sec
                    tt_02.lap_time = current_time - line1_t
                    if tt_02.best_lap_time == 0 or tt_02.lap_time < tt_02.best_lap_time:
                        tt_02.best_lap_time = tt_02.lap_time
                    print("lap time : ", tt_02.lap_time)
                send_time_message()
                line1_t = current_time
                line2_t, line3_t = 0, 0 # A chaque tour on reset les temps des secteurs 
        # Ligne 2
        elif is_line2_crossed:
            if abs(line2_t - current_time) >1:
                print("line 2 crossed")
                if line3_t!=0:
                    tt_02.sector2_t = current_time - line3_t
                    send_time_message()
                    if tt_02.best_sector2_t == 0 or tt_02.sector2_t < tt_02.best_sector2_t:
                        tt_02.best_sector2_t = tt_02.sector2_t
                    print("sector 2 time = ", tt_02.sector2_t)
                line2_t = current_time      
        # Ligne 3
        elif is_line3_crossed:
            if abs(line3_t - current_time) >1:
                print("line 3 crossed")
                if line1_t!=0 :
                    tt_02.sector3_t = current_time - line1_t
                    send_time_message()
                    if tt_02.best_sector3_t == 0 or tt_02.sector3_t < tt_02.best_sector3_t:
                        tt_02.best_sector3_t = tt_02.sector3_t            
                    print("sector 3 time = ", tt_02.sector3_t)
                line3_t = current_time

    # detection de positions incoherentes. Ne devrait pas servir...
    if (abs(tt_02_translation.getSFVec3f()[0]) > 30) or (abs(tt_02_translation.getSFVec3f()[1]) > 30) or (abs(tt_02_translation.getSFVec3f()[2]) > 10) or ((abs(tt_02_rotation.getSFVec3f()[3]) > PI/2) and ((abs(tt_02_rotation.getSFVec3f()[0]) > 0.5) or (abs(tt_02_rotation.getSFVec3f()[1]) > 0.5))):
        start_rot = [0.0, 0.0, 1.0,0]
        tt_02_rotation.setSFRotation(start_rot)
        tt_02.setVelocity([0,0,0,0,0,0])
        tt_02_translation.setSFVec3f([4.59, 0, 0.039]) #devant un mur
        erreur_position += 1
        supervisor.step(basicTimeStep)
        print("erreur position numero : ",erreur_position)     
    # If reset signal : replace the cars
    if ResetReceiver.getQueueLength() > 0:
	# Get the data off the queue    
        
        if add_obstacle :
            # Create obstacles
            clear_obstacles(supervisor)
            indexes = random.sample(range(0,len(obstacles_positions)), 3)
            for i in indexes:
                position = obstacles_positions[i][0:2]
                rotation = obstacles_positions[i][2]
                add_random_obstacle(supervisor, position, rotation, f"SimpleObstacle{i}")
        
        if random_direction:
            # Choose random direction
            direction = random.choice([0,1])

        try :
            data = ResetReceiver.getString()
            ResetReceiver.nextPacket()

    	# Select random starting points
            coordinates_idx = random.sample(range(len(starting_positions)), 1+NB_SPARRINGPARTNER_CARS)
    
    		# Replace TT-02 avec une position pseudo aleatoire
    		# print("replacement de la voiture violette")
            coords = starting_positions[coordinates_idx[0]]
            start_x = random.uniform(coords[0][0], coords[0][1])
            start_y = random.uniform(coords[1][0], coords[1][1])
            start_z = 0.039
    		
            angle = coords[2]
            start_angle = random.uniform(angle - PI/12, angle + PI/12)
            if direction:
                start_angle = start_angle + PI
            start_rot = [0.0, 0.0, 1.0, angle_clip(start_angle)]
            tt_02_rotation.setSFRotation(start_rot)		
            tt_02_translation.setSFVec3f([start_x, start_y, start_z])
            tt_02.setVelocity([0,0,0,0,0,0])
            packet_number += 1

            supervisor.step(basicTimeStep)
            ResetEmitter.send("voiture replacee num : " + str(packet_number))

            # Replace sparring partner cars
            for i in range(NB_SPARRINGPARTNER_CARS):
                sparringpartner_car_nodes[i].setVelocity([0,0,0,0,0,0])
                coords = starting_positions[coordinates_idx[i + 1]]
                start_x = random.uniform(coords[0][0], coords[0][1])
                start_y = random.uniform(coords[1][0], coords[1][1])
                start_z = 0.039
                sparringpartner_car_translation_fields[i].setSFVec3f([start_x, start_y, start_z])
                # Rotate sparringpartner cars
                angle = coords[2]
                start_angle = random.uniform(angle - PI/12, angle + PI/12)
                if direction:
                    start_angle = start_angle + PI
                start_rot = [0, 0, 1, angle_clip(start_angle)]
                sparringpartner_car_rotation_fields[i].setSFRotation(start_rot)
                sparringpartner_car_nodes[i].setVelocity([0,0,0,0,0,0])

            # Reset every timing
            start_t = supervisor.getTime()
            line1_t, line2_t, line3_t = 0, 0, 0		
            tt_02.lap_time, tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t = 0, 0, 0, 0 # Reset lap time and sector times
            # Send lap time and sector times
            message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time)
            #print("super sends : ", message)
            LapEmitter.send(message)
            
        except :
            print("souci de réception")
	
