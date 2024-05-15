# Python imports
import gymnasium as gym
import numpy as np
import time
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
import os


import struct
import json
import sys
#sous Linux, chemin a adapter export WEBOTS_HOME=/usr/local/webots
#sys.path.append('/usr/local/webots/lib/controller/python/')

#sous windows, chemin a adapter
sys.path.append('C:/Program Files/Webots/lib/controller/python/')

print("Version de Python :", sys.version)

# Webots imports
from vehicle import Driver

# Global constants
RECEIVER_SAMPLING_PERIOD = 64	# In milliseconds
PI = 3.141592653589793
MAXSPEED = 36 #km/h
VITESSE_MAX_M_S = MAXSPEED/3.6
MAXANGLE_DEGRE = 18
RESET_STEP = 4097	# Number of steps between 2 forced resets to avoid overfitting

# Pour extraire des données vers un json (pour l'analyse)
def save_data_to_json(data, filename="lidar_data.json"):
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)

# Pour ajouter une metrics personnalisée à TensorBoard: temps de secteur et temps de tour
class CustomCallback(BaseCallback):
	def __init__(self, verbose=0):
		super(CustomCallback, self).__init__(verbose)
		# Initialize any necessary variables for custom metrics


	def _on_step(self) -> bool:
		

		wrapped_env = self.model.env.envs[0]
		# Calculate your custom metric
		sector1_t = wrapped_env.sector1_t
		sector2_t = wrapped_env.sector2_t
		sector3_t = wrapped_env.sector3_t
		lap_time = wrapped_env.lap_time
		best_sector1_t = wrapped_env.best_sector1_t
		best_sector2_t = wrapped_env.best_sector2_t
		best_sector3_t = wrapped_env.best_sector3_t
		best_lap_time = wrapped_env.best_lap_time
		nb_sectors1_completed = wrapped_env.nb_sectors1_completed
		nb_sectors2_completed = wrapped_env.nb_sectors2_completed
		nb_sectors3_completed = wrapped_env.nb_sectors3_completed
		nb_laps_completed = wrapped_env.nb_laps_completed
		nb_crashes = wrapped_env.numero_crash
		nb_sect1_completed_in_lap = wrapped_env.nb_sect1_completed_in_lap
		nb_sect2_completed_in_lap = wrapped_env.nb_sect2_completed_in_lap
		nb_sect3_completed_in_lap = wrapped_env.nb_sect3_completed_in_lap
		nb_laps_completed_in_lap = wrapped_env.nb_laps_completed_in_lap


		# Times
		self.logger.record("Times/Sector1_t", sector1_t)
		self.logger.record("Times/Sector2_t", sector2_t)
		self.logger.record("Times/Sector3_t", sector3_t)
		self.logger.record("Times/Lap_time", lap_time)

		# Best Times
		self.logger.record("Best Times/Best_Sector1_t", best_sector1_t)
		self.logger.record("Best Times/Best_Sector2_t", best_sector2_t)
		self.logger.record("Best Times/Best_Sector3_t", best_sector3_t)
		self.logger.record("Best Times/Best_Lap_time", best_lap_time)

		# Completion Counts
		self.logger.record("Completion Counts/Sectors1_Completed", nb_sectors1_completed)
		self.logger.record("Completion Counts/Sectors2_Completed", nb_sectors2_completed)
		self.logger.record("Completion Counts/Sectors3_Completed", nb_sectors3_completed)
		self.logger.record("Completion Counts/Laps_Completed", nb_laps_completed)
		self.logger.record("Completion Counts without crash/Sector1_Completed_In_Lap", nb_sect1_completed_in_lap)
		self.logger.record("Completion Counts without crash/Sector2_Completed_In_Lap", nb_sect2_completed_in_lap)
		self.logger.record("Completion Counts without crash/Sector3_Completed_In_Lap", nb_sect3_completed_in_lap)
		self.logger.record("Completion Counts without crash/Laps_Completed_In_Lap", nb_laps_completed_in_lap)

		# Miscellaneous
		self.logger.record("Crashes", nb_crashes)
	
		return True
	
# Custom Gym environment
class WebotsGymEnvironment(Driver, gym.Env):
	def __init__(self):
		super().__init__()
				
		#valeur initiale des actions
		self.consigne_angle = 0.0 #en degres
		self.consigne_vitesse = 0.1 #en m/s

		#compteur servant à la supervision de l'apprentissage
		self.numero_crash = 0 		# compteur de collisions
		self.nb_pb_lidar = 0 		# compteur de problèmes de communication avec le lidar
		self.nb_demarrage_lidar=0        # compteur de démarrages de lidar (il faut parfois le démarrer plusieurs fois)
		self.nb_pb_acqui_lidar=0	# compteur de problèmes d'acquisition lidar (il renvoie 0 ce qui n'est pas possible)
		self.reset_counter = 0 		# compteur de pas d'apprentissage pour arrêter un épisode après RESET_STEP pas
		self.packet_number = 0 		# compteur de messages envoyés


		# Receiver for reset signal
		self.ResetEmitter = super().getDevice("ResetEmitter")
		self.ResetReceiver = super().getDevice("ResetReceiver")
		self.ResetReceiver.enable(RECEIVER_SAMPLING_PERIOD)
		self.ResetReceiver.setChannel(2)
		self.ResetEmitter.setChannel(1)

		# Receiver for lap and sector times
		self.LapReceiver = super().getDevice("LapReceiver")
		self.LapReceiver.enable(RECEIVER_SAMPLING_PERIOD)
		self.LapReceiver.setChannel(3)
		
		# Lidar initialisation
		self.lidar = super().getDevice("RpLidarA2")
		self.lidar.enable(int(super().getBasicTimeStep()))
		self.lidar.enablePointCloud()

		# Action space
		self.action_space = gym.spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
		
		# Observation space
		#self.observation_space = gym.spaces.Box(np.ones(360)*-1, np.ones(360), dtype=np.float64)
		
		self.observation_space = gym.spaces.Dict({
		"current_lidar":gym.spaces.Box(np.zeros(360), np.ones(360), dtype=np.float64),
		"previous_lidar":gym.spaces.Box(np.zeros(360), np.ones(360), dtype=np.float64),
		"previous_speed":gym.spaces.Box(np.zeros(1), np.ones(1), dtype=np.float64),
		"previous_angle":gym.spaces.Box(np.array([-1.0]), np.ones(1), dtype=np.float64),
		})

		# Times
		self.sector1_t = 0
		self.sector2_t = 0
		self.sector3_t = 0
		self.lap_time = 0

		# Best times
		self.best_lap_time = 0
		self.best_sector1_t = 0
		self.best_sector2_t = 0
		self.best_sector3_t = 0

		# Nb of sectors and laps
		self.nb_sectors1_completed = 0
		self.nb_sectors2_completed = 0
		self.nb_sectors3_completed = 0
		self.nb_laps_completed = 0

		# Nb of sectors and laps in one episode
		self.nb_sect1_completed_in_lap = 0
		self.nb_sect2_completed_in_lap = 0
		self.nb_sect3_completed_in_lap = 0
		self.nb_laps_completed_in_lap = 0

		# Add randomness to the environment
		self.add_randomness = False
		self.random_speed_bias = 0 # en m/s
		self.random_speed_prop = 1
		self.random_dir_bias = 0 # en degres
		self.random_dir_prop = 1
		self.max_random_speed_bias = 0.5
		self.max_random_speed_prop = 2
		self.max_random_dir_bias = 0.5
		self.max_random_dir_prop = 1.5
		self.random_angle_max_bias = 0.5
		self.random_point_mort_max = 0.1
		self.min_random_speed_prop = 0.8
		self.min_random_dir_prop = 0.5

		

	def get_lidar_mm(self):
		tableau_lidar_mm = np.zeros(360)
		try :
			donnees_lidar_brutes = np.array(self.lidar.getRangeImage()) #lidar en mm
			for i in range(0,101) :
				if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<12) :
					tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
				else :
					tableau_lidar_mm[i] = 0
			for i in range(101,260) : #zone correspondant à l'habitacle de la voiture
				tableau_lidar_mm[i] = 0	
			for i in range(260,360) :
				if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<12) :
					tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
				else :
					tableau_lidar_mm[i] = 0
		except : 
			self.nb_pb_lidar+=1
			print("souci lidar"+str(self.nb_pb_lidar))
		return tableau_lidar_mm
	
	def set_vitesse_m_s(self,vitesse_m_s):
		if vitesse_m_s < 0 : # Commande d'arret
			speed = 0
		else :
			if self.add_randomness :
				vitesse_m_s = self.random_speed_prop*vitesse_m_s+self.random_speed_bias
			speed = vitesse_m_s*3.6
			if speed > MAXSPEED :
				speed = MAXSPEED
			if speed < self.point_mort :
				speed = 0
		super().setCruisingSpeed(speed)

	def set_direction_degre(self,consigne_angle_degre):
		if self.add_randomness:
			angle_degre = (self.random_dir_bias/324)*consigne_angle_degre**2 + 1*consigne_angle_degre + self.random_dir_bias # Fonction qui passe par (-18,-18) (0,random_dir_bias) et (18,18)
		if angle_degre > MAXANGLE_DEGRE:
			angle_degre = MAXANGLE_DEGRE
		elif angle_degre < -MAXANGLE_DEGRE:
			angle_degre = -MAXANGLE_DEGRE   
		angle = -angle_degre * PI/180
		super().setSteeringAngle(angle)
	
	def replacer_voiture(self):
		super().setCruisingSpeed(0)
		super().setSteeringAngle(0)
		super().step()

	# Get Lidar observation
	def get_observation(self,init=False):
		tableau_lidar_mm = self.get_lidar_mm()
		i = 0
		while(tableau_lidar_mm[0] == 0) and (i<50) :
			#on essaie d'avoir un tableau de valeur correct !
			self.nb_pb_acqui_lidar+=1
			#print("souci d'aquisition lidar"+str(self.nb_pb_acqui_lidar))
			i = i+1
			tableau_lidar_mm = self.get_lidar_mm() #lidar en mm
            	
  	
		if init:
			current_lidar=tableau_lidar_mm.astype("float64")/12000
			previous_lidar=tableau_lidar_mm.astype("float64")/12000
			previous_speed=np.zeros(1)
			previous_angle=np.zeros(1)

		else:
			#grandeurs normalisées pour observation
			previous_lidar=self.observation["current_lidar"]
			current_lidar=tableau_lidar_mm.astype("float64")/12000
			previous_speed=np.array([self.consigne_vitesse/VITESSE_MAX_M_S])
			# si on a un capteur de vitesse sur la voiture relle : 
			# previous_speed=[(super().getTargetCruisingSpeed()/3.6)/VITESSE_MAX_M_S]
			previous_angle=np.array([self.consigne_angle/MAXANGLE_DEGRE])
			# si on a un capteur de vitesse sur la voiture relle : 
			# previous_speed=[(super().getSteeringAngle()*180/PI)/MAXANGLE_DEGRE]
 
		observation = {
                        "current_lidar": current_lidar,
                        "previous_lidar":previous_lidar,
                        "previous_speed":previous_speed,
                        "previous_angle":previous_angle,
                      }
		# print(observation["current_lidar"])
		self.observation=observation
		return observation
		
	def update_lap_times(self):

		# Get lap and sector times
		while(self.LapReceiver.getQueueLength() > 0) : 
			data = self.LapReceiver.getBytes()
			sector1_t, sector2_t, sector3_t, lap_time = struct.unpack("ffff", data) # Unpack the data
			if self.sector1_t != sector1_t and sector1_t != 0:
				self.nb_sectors1_completed += 1
				self.nb_sect1_completed_in_lap += 1
				self.sector1_t = sector1_t
			if self.sector2_t != sector2_t and sector2_t != 0:
				self.nb_sectors2_completed += 1
				self.nb_sect2_completed_in_lap += 1
				self.sector2_t = sector2_t
			if self.sector3_t != sector3_t and sector3_t != 0:
				self.nb_sectors3_completed += 1
				self.nb_sect3_completed_in_lap += 1
				self.sector3_t = sector3_t
			if self.lap_time != lap_time and lap_time != 0:
				self.nb_laps_completed += 1
				self.nb_laps_completed_in_lap += 1
				self.lap_time = lap_time
			self.LapReceiver.nextPacket()

		# Update best times. A time of 0 means the sector or lap was not completed
		if self.lap_time!=0 and (self.best_lap_time==0 or (self.lap_time != 0 and (self.lap_time < self.best_lap_time))):
			self.best_lap_time = self.lap_time
			print("best_lap_time = ", self.best_lap_time)
			
		if self.sector1_t!=0 and (self.best_sector1_t==0 or (self.sector1_t!=0 and (self.sector1_t < self.best_sector1_t))):
			self.best_sector1_t = self.sector1_t
			print("best_sector1_t = ", self.best_sector1_t)
		if self.sector2_t !=0 and (self.best_sector2_t==0 or (self.sector2_t != 0 and (self.sector2_t < self.best_sector2_t))):			
			self.best_sector2_t = self.sector2_t
			print("best_sector2_t = ", self.best_sector2_t)
		if self.sector3_t !=0 and (self.best_sector3_t==0 or (self.sector3_t != 0 and (self.sector3_t < self.best_sector3_t))):		
			self.best_sector3_t = self.sector3_t
			print("best_sector3_t = ", self.best_sector3_t)

	# Reward function
	def get_reward(self, obs):
		done = False

		self.update_lap_times()

		reward = 0
		done = False
		mini = 1

		# Get lap and sector times
		while(self.LapReceiver.getQueueLength() > 0) : 
			data = self.LapReceiver.getBytes()
			print("tt02 received : ", data)
			self.sector1_t, self.sector2_t, self.sector3_t, self.lap_time = struct.unpack("ffff", data) # Unpack the data
			self.LapReceiver.nextPacket()

		#recherche de la distance la plus faible mesuree par le lidar entre -15 et +15°
		for i in range(-15,15) :
			if (obs["current_lidar"][i] < mini and obs["current_lidar"][i]!=0) :
				mini = obs["current_lidar"][i]
		# print(mini)
				
		#si le lidar touche un mur ou si la voiture va trop vite (chute en dehors du sol de la piste)
		if mini < 0.014 or super().getCurrentSpeed() > 30.0 : #0.015 <-> 170 mm
			# Crash
			self.numero_crash += 1
			speed_penalty = super().getCurrentSpeed()*10
			reward = -400 - speed_penalty
			done = True
			
		else:
			#Récompense pour une grande distance aux obstacles et une grande vitesse
			reward = 18 * (mini-0.018) + 2 * super().getTargetCruisingSpeed()
			#print("reward : "+str(reward))
		
		return reward, done

	# Reset the simulation
	def reset(self, seed=None):

		global MAXANGLE_DEGRE
		
		# Reset speed and steering angle et attente de l'arrêt de la voiture
		self.replacer_voiture()
		for i in range(20) :
				super().step()	
		self.reset_counter = 0
		# print("crash num : " +str(self.numero_crash))

		# reset nb of sectors and laps in one episode
		self.nb_sect1_completed_in_lap = 0
		self.nb_sect2_completed_in_lap = 0
		self.nb_sect3_completed_in_lap = 0
		self.nb_laps_completed_in_lap = 0
		
		if(self.numero_crash != 0):         
			#attente de l'arrêt de la voiture
			while abs(super().getTargetCruisingSpeed()) >= 0.001 :    		           	
				print("voiture pas encore arrêtée")
				super().step()
            		
			# Return an observation
			self.packet_number += 1
			# Envoi du signal de reset au Superviseur pour replacer les voitures
			self.ResetEmitter.send("voiture crash numero " + str(self.packet_number))
			super().step()
			#attente de la remise en place des voitures
			while(self.ResetReceiver.getQueueLength() == 0) :        	
				self.set_vitesse_m_s(self.consigne_vitesse )
				super().step()

			data = self.ResetReceiver.getString()
			self.ResetReceiver.nextPacket()
				
			self.replacer_voiture()
			# on fait quelques pas à l'arrêt pour stabiliser la voiture si besoin
			while abs(super().getTargetCruisingSpeed()) >= 0.001 :    		           	
				print("voiture pas arrêtée")
				self.set_vitesse_m_s(self.consigne_vitesse )
				super().step()
		
		#quelques pas de simulations pour que le lidar fasse 2 tours (200 ms) avant la première observation
		mini=130000
		i=0
		while((mini <= 120) or (mini == 130000)) and (i<20) :
			#on essaie d'avoir un tableau de valeur correct (au moins 170 mm par mesure), 20 essais max
			super().step()
			tableau_lidar_mm = self.get_lidar_mm()
			self.nb_demarrage_lidar +=1
			i = i+1
			for j in range (-40,+40) :
				if tableau_lidar_mm[j] < mini :
					mini = tableau_lidar_mm[j]
		# print("démarrage lidar num "+str(self.nb_demarrage_lidar)+ " mini = " +str(mini))
					
		# Ajout d'aleatoire aux actions
		if self.add_randomness:
			self.random_speed_bias = np.random.uniform(-self.max_random_speed_bias, self.max_random_speed_bias)
			self.random_speed_prop = np.random.uniform(self.min_random_speed_prop, self.max_random_speed_prop)
			self.random_dir_bias = np.random.uniform(0, self.max_random_dir_bias)
			self.random_dir_prop = np.random.uniform(self.min_random_dir_prop, self.max_random_dir_prop)
			MAXANGLE_DEGRE = np.random.uniform(18-self.random_angle_max_bias,18+ self.random_angle_max_bias) # Angle max autour de 18°
			self.point_mort = np.random.uniform(0,self.random_point_mort_max)

		return self.get_observation(True), {}

	
	# Step function
	def step(self, action):

		self.reset_counter += 1
		if self.reset_counter > RESET_STEP:
			print("Reset because of too many steps without crash")
			return self.reset()

		self.consigne_vitesse=((action[0]+1)*VITESSE_MAX_M_S) # L'action est normalisée entre -1 et 1
		self.consigne_angle=(action[1]*MAXANGLE_DEGRE)
		
		# saturations
		if self.consigne_angle > MAXANGLE_DEGRE  :
			self.consigne_angle = MAXANGLE_DEGRE 
		elif self.consigne_angle < -MAXANGLE_DEGRE  :
			self.consigne_angle = -MAXANGLE_DEGRE 
		
		if self.consigne_vitesse > VITESSE_MAX_M_S :
			self.consigne_vitesse = VITESSE_MAX_M_S
		# if self.consigne_vitesse < 0.1:
		# 	self.consigne_vitesse = 0.1
			
		self.set_vitesse_m_s(self.consigne_vitesse)
		self.set_direction_degre(self.consigne_angle)
		super().step()
		
		obs = self.get_observation()
		reward, done = self.get_reward(obs)
		truncated = False # it must be a boolean
		info = {}
		
		#print(f"REWARD: {reward}")
		return obs, reward, done, truncated, info


# Main function
def main():
	t0 = time.time()
	env = WebotsGymEnvironment()
	print("environnement créé")
	env.add_randomness = True # Ajoute des biais aux actions
	if env.add_randomness:
		env.max_random_dir_bias = 2
		env.max_random_dir_prop = 1.5
		env.max_random_speed_bias = 0.5
		env.max_random_speed_prop = 1.5
	mode = "train_from_scratch" # "train_from_scratch", "load_and_train", "show_results"
	file_name = "ter-covapsy"
	file_to_load_path = "PPO_results_25032024_4_train2_9"
	save_freq = 10000
	total_timesteps = 100000
	batch_size = 128
	n_epochs = 20
	policy_kwargs = dict(
    	net_arch=[dict(pi=[128, 128, 128], vf=[128, 128, 128])])  # Increase the number of layers and neurons
	print("add_randomness = ", env.add_randomness)
	print("mode : " + mode)
	check_env(env)
	print("vérification de l'environnement")
	
	if torch.cuda.is_available():
		print(torch.cuda.get_device_name(0))
                 	
	match mode:
		case "train_from_scratch":
			#	Model definition
			model = PPO(policy="MultiInputPolicy",
				env=env, 
				learning_rate=5e-4,
				verbose=1,
				device='cuda:0',
				tensorboard_log='./PPO_Tensorboard',
				#	Additional parameters
				n_steps=2048,
				batch_size=batch_size, # Factor of n_steps
				n_epochs=n_epochs, # Number of policy updates per iteration of n_steps
				gamma=0.99,
				gae_lambda=0.95,
				clip_range=0.2,
				vf_coef=1,
				ent_coef=0.01,
				policy_kwargs=policy_kwargs
				)
			
			# Training
			print("début de l'apprentissage")
			customCallback=CustomCallback() # Pour ajouter lap_time à TensorBoard
			# Pour sauvegarder le modèle à chaque 10000 pas
			checkpoint_callback = CheckpointCallback(
				save_path='./logs/',
				save_freq=save_freq,  # Save the model every 100000 steps
				name_prefix=file_name
			)
			combined_callback = CallbackList([customCallback, checkpoint_callback])
			model.learn(total_timesteps, callback=combined_callback)
			PPO_result_name = file_name
			model.save(PPO_result_name)
			print("sauvegarde du modèle : " + PPO_result_name)
			t1 = time.time()
			print("fin de l'apprentissage après " + str(t1-t0) + "secondes")
			print("nombre de collisions : " +str(env.numero_crash))
			print("nombre de problèmes de communication avec le lidar : " +str(env.nb_pb_lidar))
			print("nombre de problèmes d'acquisition lidar : " +str(env.nb_pb_acqui_lidar))
		
		case "load_and_train":
			# Load learning data
			path = file_to_load_path
			model = PPO.load(path, env=env, device='cpu')
			print(f"Chargement du modèle depuis le chemin : {path}")

			# Training
			print("début de l'apprentissage")
			customCallback=CustomCallback() # Pour ajouter lap_time à TensorBoard
			# Pour sauvegarder le modèle à chaque 10000 pas
			checkpoint_callback = CheckpointCallback(
				save_path='./logs/',
				save_freq=save_freq,  # Save the model every 100000 steps
				name_prefix=file_name
			)
			combined_callback = CallbackList([customCallback, checkpoint_callback])
			model.learn(total_timesteps, callback=combined_callback)
			PPO_result_name = file_name
			model.save(PPO_result_name)
			print("sauvegarde du modèle : " + PPO_result_name)
			t1 = time.time()
			print("fin de l'apprentissage après " + str(t1-t0) + "secondes")
			print("nombre de collisions : " +str(env.numero_crash))
			print("nombre de problèmes de communication avec le lidar : " +str(env.nb_pb_lidar))
			print("nombre de problèmes d'acquisition lidar : " +str(env.nb_pb_acqui_lidar))
		
		case "show_results":
			# Load learning data
			path = file_name
			model = PPO.load(path, env=env, device='cpu')
			print(f"Chargement du modèle depuis le chemin : {path}")

			# Demo of the results
			obs, info = env.reset()
			print("Demo of the results.")
			c = 0
			for _ in range(10000):
				# Play the demo
				action, _ = model.predict(obs)
				obs, reward, done, _, info = env.step(action)
				c += reward
				print(reward)
				if done:
					obs, info = env.reset()
			print("Exiting.")
			print(c)


if __name__ == '__main__' :
	main()
