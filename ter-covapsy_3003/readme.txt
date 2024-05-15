STABLE
25 03
La version 25 03 corrige les bugs lié au passage des lignes de secteur. Les temps de secteurs fonctionnent
La version 25 03_2 ajoute le changement de sens de circulation à  chaque reset
La version 25 03_4 permet de regarder depuis tensorboard l'évolution des temps au secteurs/tour
La version 26 03 ajoute des checkpoints pendant l'entraînement. Il ajoute des case pour changer facilement de mode "entrainement" "démo" etc.
La version 27 03 permet d'ajouter des biais/coeff prop aléatoires aux consignes de vitesse/angle
	Elle corrige les temps au tours/secteurs
	Elle corrige le changement de sens pour permettre de compter les temps au tour
	Elle corrige l'ajout d'obstacles en les orientant mieux
	Ajoute des paramètres à analyser sur tensorboard :
		- temps de secteur conservé si non mis à jour
		- meilleurs temps par secteurs
		- nombre de secteurs validés
		- nombre de secteurs validés avant le prochain reset
		- nombre de crash
	Correction des passages de ligne : il faut 1s entre deux passages pour être compté comme valide
Version 28 03 modifie le modèle pour inclure 3 couches de 128 neuronnes (au lieu de 2 de 64)
Version 30 03 corrige les ajouts de biais pour que la direction soit toujours entre -18 et 18°.
	Ajuste les paramètres/seuils avec les paramètres de a voiture réelle
	Ajout d'un point mort pour la propulsion
	Vitesse max à 10m/s
	modèle plus gros

TRAIN
2503_4_train1. Avec génération d'obstacles. 1e7 Timesteps. Tourne à l'envers donc seul lap_time est pertinent
2503_4_train2. Sans génération d'obstacles. 1e7 Timesteps. Tourne à l'envers donc seul lap_time est pertinent
2503_4_train3. Avec génération d'obstacles. Pas de changement de sens. 3e7 Timesteps # PAS FONCTIONNEL
2503_4_train4. Avec génération d'obstacles. 1e7 TimeSteps. Ajoute best_times aux metrics. Nouvelle fonction reward.
2603_train1. Sans génération d'obstacles. 1e7 steps. Repars de train2. Avec checkpoints.

2603_train3. Sans génération d'obstacles. 1e7 steps. Repars avec train2. RewardFuntion2. Pas de reset si crash : done=False dans get_reward
2703_train1. Sans génération d'obstacles. 1e7 steps. Random_direction. Add_randomness. RewardFunction1
2703_train2. Sans génération d'obstacles. 1e7 steps. Random_direction. Add_randomness. RewardFunction2
2703_train3. Avec génération d'obstacles. 3e7 steps. Random_direction. Add_randomness. RewardFunction1. n_epoch=30
2803_train1. AddObstacles. 3e7 steps. Random_direction. Add_randomness. n_epoch=20. batch_size=128



ATTENTION : Le changement de sens doit être complété avec un changement de sens des lignes 1 2 et 3 : 
en l'état il faut franchir les lignes dans le bon sens, et ce sens ne change pas même si la voiture tourne dans l'autre sens




FONCTIONS REWARD
### Reward 1 ###
Fonction de kevin de base
### RewardFunction2 ###
def get_reward(self, obs):
    # Initialize the reward
    reward = 0
    
    # Safety Margin Reward
    min_distance = np.min(obs["current_lidar"][obs["current_lidar"] > 0])  # Minimum non-zero distance
    safety_reward = -100 if min_distance < 0.014 else 0  # Penalize if too close to an obstacle
    
    # Efficiency Reward for Sector and Lap Times
    # Adjust reward calculations to only proceed if sector_times and lap_time are not None
    sector_reward = 0
    if hasattr(self, 'sector_times') and self.sector_times is not None:
        sector_reward = sum([500 - min(time, 500) for time in self.sector_times])  # Reward for fast sector times
    
    lap_reward = 0
    if hasattr(self, 'lap_time') and self.lap_time is not None and self.lap_time > 0:
        lap_reward = 1000 - min(self.lap_time, 1000)  # Reward for fast lap times
    
    # Steering Smoothness
    # Assuming there's a method to calculate steering change accurately
    steering_change = np.abs(obs["previous_angle"][0] - self.consigne_angle / MAXANGLE_DEGRE)
    steering_penalty = -50 if steering_change > 0.2 else 0  # Penalize excessive steering oscillation
    
    # Calculate total reward
    total_reward = reward + safety_reward + sector_reward + lap_reward + steering_penalty
    
    # Check if the episode should end (if a crash is detected or too close to an obstacle)
    done = min_distance < 0.014  # End the episode if the car gets too close to an obstacle
    
    return total_reward, done
