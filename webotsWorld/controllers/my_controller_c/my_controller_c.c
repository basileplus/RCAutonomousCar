/*
 * File:          my_controller_c.c
 * Date:          23 mai 2023
 * Description:
 * Author: Bruno Larnaudie, Anthony Juton
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 32
#define SIZE_TABLEAU 200
#define MAX_SPEED 6.28  // Vitesse maximale des moteurs

const float* range_donnees;
//float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto=0;

// prototype des fonctions
void affichage_consigne();
void set_direction_degres(float angle_degre);
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
 
//vitesse en km/h
float speed = 0;
float maxSpeed = 28; //km/h

// angle max de la direction
float maxangle_degre = 16; 

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
int main(int argc, char **argv) 
{
  unsigned int i;
  signed int data_lidar_mm_main[360];
  float angle_degre, vitesse_m_s;
  /* necessary to initialize webots stuff */
  //intialisation du conducteur de voiture
  wbu_driver_init();
  //enable keyboard
  wb_keyboard_enable(TIME_STEP);
  // enable lidar
  WbDeviceTag lidar = wb_robot_get_device("RpLidarA2");
  wb_lidar_enable(lidar,TIME_STEP);
  // affichage des points lidar sur la piste
  wb_lidar_enable_point_cloud(lidar);
 
  affichage_consigne();
  set_direction_degres(0);
  set_vitesse_m_s(0);
  while (wbu_driver_step() != -1) 
  {
    float distance;
    /* lire le lidar et traiter les données :   */
    range_donnees=wb_lidar_get_range_image(lidar);
    distance = range_donnees[0];
    if((distance > 0.0) && (distance <20.0))
      data_lidar_mm_main[0]=1000*distance;
      else data_lidar_mm_main[0] = 0;
    for(i = 1; i<360 ; i++)
    {
      distance = range_donnees[360-i];
      if((distance > 0.0) && (distance <20.0))
        data_lidar_mm_main[i]=1000*distance;
      else data_lidar_mm_main[i] = 0;
    }       
    gestion_appui_clavier();     
    if(modeAuto)
    {
        /****************************************/
        /* Programme etudiant avec              */
        /*  - le tableau data_lidar_mm_main     */
        /*  - la fonction set_direction_degre(.)*/
        /*  - la fonction set_vitesse_m_s(...)  */
        /*  - la fonction recule()              */
        /****************************************/
        angle_degre = 0.02*(data_lidar_mm_main[60]-data_lidar_mm_main[300]); //distance à 60° - distance à -60°
        set_direction_degres(angle_degre);
        vitesse_m_s = 0.5;
        set_vitesse_m_s(vitesse_m_s);
    }
  }
  /* This is necessary to cleanup webots resources */
  wbu_driver_cleanup();
  return 0;
}

unsigned char gestion_appui_clavier(void)
{
  int key;
  key=wb_keyboard_get_key();
  switch(key)
  {
    case -1:
      break;
         
    case 'n':
    case 'N':
      if (modeAuto)
      {
        modeAuto = 0;
        printf("--------Mode Auto Désactivé-------");
      }
      break;
    
    case 'a':
    case 'A':
      if (!modeAuto)
      {
        modeAuto = 1;
        printf("------------Mode Auto Activé-----------------");
      }
      break;
      
    default:
      break; 
  }
  return key;    
}

void affichage_consigne()
{
  printf("cliquer sur la vue 3D pour commencer\n");
  printf("a pour mode auto, n pour stop\n");
}

void set_direction_degres(float angle_degre)
{
  float angle=0; 
  if(angle_degre > maxangle_degre)
    angle_degre = maxangle_degre;
  else if(angle_degre < -maxangle_degre)
    angle_degre = -maxangle_degre;   
  angle = -angle_degre * 3.14/180; 
  wbu_driver_set_steering_angle(angle);
}

void set_vitesse_m_s(float vitesse_m_s){
  float speed;
  speed = vitesse_m_s*3.6;
  if(speed > maxSpeed)
    speed = maxSpeed;
  if(speed < 0)
    speed = 0;
  wbu_driver_set_cruising_speed(speed);
}

void recule(void){
    wbu_driver_set_cruising_speed(-1);
}

