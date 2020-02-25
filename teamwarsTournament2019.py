#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# multirobot_teamwars.py
# Contact (ce fichier uniquement): nicolas.bredeche(at)upmc.fr
# Ce code utilise pySpriteWorld, développé par Yann Chevaleyre (U. Paris 13)
#
# Historique:
# 	2018-04-09__16:35 - version pour examen 2018
#   2019-04-02__11:42 - passage Python 3.x
#   2019-04-15__21:49 - version pour examen 2018
#
# Dépendances:
#   Python 3.x
#   Matplotlib
#   Pygame
#
# Description:
#   Template pour projet multi-robots "MULTIROBOT WARS"
#   Les équipes "verte" et "bleue" ont chacune une stratégie d'exploration
#
# Mode d'emploi pour l'évaluation:
#   Mettre votre fonction stepController() à la place de celle de AgentTypeA
#   Tout autre modification est interdite
#   Vous êtes l'équipe verte
#   Pour lancer le code: "python teamwarsTournament.py <numero_de_l'arene>" avec <numero_de_l'arene> = 0, 1, 2, 3 ou 4
#
# Mode d'emploi pour un tournoi entre deux équipes:
#   Une équipe copie sa fonction stepController dans AgentTypeA
#   L'autre équipe fait de même avec AgentTypeB
#   Evaluation sur 10 matchs:
#       1. testez avec chacune des 5 arènes (variable "arena" ci-dessous, valeurs de 0 à 4)
#       2. pour chaque arène, testez deux fois, en échangeant la position initiale entre chaque essais (variable "invertInitPop" ci-dessous)
#   Pour lancer le code: "python teamwarsTournament.py <numero_de_l'arene> <position_standard>"
#       avec <numero_de_l'arene> = 0, 1, 2, 3 ou 4
#       avec <position_standard> = True ou False (False (par défaut): l'équipe verte commence à gauche, sinon, l'inverse)
#   => Comptez le nombre de victoires de chacun. En cas d'égalité: +0.5 point chacun.
#
#

##############################
# import et choix de stratégie
##############################
from StrategieBase import StrategieBase
from StratAvoidH import StratAvoidH
from StratAvoidIA import StratAvoidIA
from StratFollowH import StratFollowH
from StratExploreIA import StratExploreIA
avoidH=StratAvoidH()
avoidIA=StratAvoidIA()
followH=StratFollowH()
exploreIA=StratExploreIA()

base1=StrategieBase(avoidH)
base2=StrategieBase(followH)
##################################
# fin import et choix de stratégie
##################################

from robosim import *
from random import random, shuffle, randint
import time
import sys
import atexit
import numpy as np
import math

''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
''' Paramètres du Tournoi  '''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''

# Quelle arêne? (valeurs de 0 à 4)
arena = 0

# Position de départ? (si la valeur est vraie, alors les deux équipes échangent leur position de départ)
invertInitPop = False

''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
'''  variables globales    '''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''

game = Game()
agents = []

running = False

nbAgents = 8 # doit être pair et inférieur a 32
maxSensorDistance = 30              # utilisé localement.
maxRotationSpeed = 5
maxTranslationSpeed = 1
SensorBelt = [-170,-80,-40,-20,+20,40,80,+170]  # angles en degres des senseurs

screen_width=512 #512,768,... -- multiples de 32  
screen_height=512 #512,768,... -- multiples de 32

maxIterations = 6000 # infinite: -1
showSensors = False
frameskip = 4   # 0: no-skip. >1: skip n-1 frames
verbose = False # permet d'afficher le suivi des agents (informations dans la console)

occupancyGrid = []
for y in range(screen_height//16):
    l = []
    for x in range(screen_width//16):
        l.append("_")
    occupancyGrid.append(l)

iteration = 0

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Agent "A"            '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

class AgentTypeA(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    agentType = "A"
    etat = 0
    
    translationValue = 0 # ne pas modifier directement
    rotationValue = 0 # ne pas modifier directement


    def __init__(self,robot):
        self.id = AgentTypeA.agentIdCounter
        AgentTypeA.agentIdCounter = AgentTypeA.agentIdCounter + 1
        #print ("robot #", self.id, " -- init")
        self.robot = robot
        self.robot.teamname = self.teamname

    def getType(self):
        return self.agentType

    def getRobot(self):
        return self.robot

   # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-= JOUEUR A -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= 
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-= pour l'évaluation, seul le teamname et la fct  stepController(.)  =-=
    # =-=-=-=-= seront utilisés. Assurez-vous donc que tout votre code utile est  =-=
    # =-=-=-=-= auto-contenu dans la fonction stepControlelr. Vous pouvez changer =-=
    # =-=-=-=-= teamname (c'est même conseillé si vous souhaitez que vos robots   =-=
    # =-=-=-=-= se reconnaissent entre eux.                                       =-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    teamname = "team-fire-heart" # A modifier avec le nom de votre équipe

    def stepController(self):
        import math
        import random 
        
    	# cette méthode illustre l'ensemble des fonctions et informations que vous avez le droit d'utiliser.
    	# tout votre code doit tenir dans cette méthode. La seule mémoire autorisée est la variable self.etat
    	# (c'est un entier).
        self.teamname = "team-fire-heart"
        color( ( 254, 95, 251) )
        circle( *self.getRobot().get_centroid() , r = 22) # je dessine un rond bleu autour de ce robot
        coord = self.getRobot().get_centroid()
        #===========================================================================================#
        if(self.id==0):
            t=1#valeur de translation
            acc=0
            demi_tour=False
            fact=[1,-1,-1]#on evite les murs et les autres joueurs peu importe l'équipe 
            for i in range(len(SensorBelt)):
                
                if((i==0 or i==len(SensorBelt)-1)):#si on est poursuivit on tourne 
                    if(self.getObjectTypeAtSensor(i)==2 and self.getRobotInfoAtSensor(i)["teamname"]!=self.teamname):
                        self.etat+=1
                    else:
                        self.etat=0
                    if(self.etat>2):
                        demi_tour=True #quand quelqu'un le suit il tourne en rond
                        break;
                acc+=fact[self.getObjectTypeAtSensor(i)]*((SensorBelt[i])/170)
            if(demi_tour==True):
                rot=-1
            else:
                rot=math.tanh (acc+random.randint(-1,+1))
        
        #===========================================================================================#
        elif(self.id==1):#On suit l'adversaire tant que c'est possible
            t=1#valeur de translation
            acc=0
            demi_tour=False
            fact=[0.5,-1,1]
            for i in range(len(SensorBelt)):
                fact=[0.5,-1,1] # si aucune condition n'est verifiée on suit les agents
                if((i==0 or i==len(SensorBelt)-1)):#si on est poursuivit on tourne 
                    if(self.getObjectTypeAtSensor(i)==2 and self.getRobotInfoAtSensor(i)["teamname"]!=self.teamname):
                        self.etat+=1
                    else:
                        self.etat=0
                    if(self.etat>2):
                        demi_tour=True #quand quelqu'un le suit il tourne en rond
                        break;
                elif(self.getObjectTypeAtSensor(i)==2):#on a un agent devant 
                    if (self.getRobotInfoAtSensor(i)["teamname"]==self.teamname):# si c'est un agent de notre équipe on l'évite
                        fact=[0.5,-1,-1]
                    elif(self.getDistanceAtSensor(i)<0.05):#si c'est un adversaire et qu'on est bloqué on s'éloigne
                        fact=[0.5,-1,-0.6]
                
                acc+=(fact[self.getObjectTypeAtSensor(i)])*((SensorBelt[i])/100)
            if(demi_tour==True):
                rot=-1
            else:
                rot=math.tanh (acc+random.randint(-1,+1))
            
        #===========================================================================================#
        elif(self.id==2):
            t=1#valeur de translation
            acc=0
            demi_tour=False
            fact=[1,-1,0] #si aucune condition vérifiée on fait un parcours en evitant les murs et ignorant les robots
            for i in range(len(SensorBelt)):
                fact=[1,-1,0] 
                if((i==0 or i==len(SensorBelt)-1)):# si les senseurs de l'arriere détectent un adversaire qui suit l'agent on fait demi-tour pour lui echapper
                    if(self.getObjectTypeAtSensor(i)==2 and self.getRobotInfoAtSensor(i)["teamname"]!=self.teamname):
                        self.etat+=1
                    else:
                        self.etat=0
                    if(self.etat>2):
                        demi_tour=True #quand quelqu'un le suit il tourne en rond
                        break;
                elif(self.getObjectTypeAtSensor(i)==2 and self.getRobotInfoAtSensor(i)["teamname"]==self.teamname):# si c'est un agent de notre équipe on l'évite
                        fact=[0.5,-1,-1]
                acc+=fact[self.getObjectTypeAtSensor(i)]*((SensorBelt[i])/170)
            if(demi_tour==True):
                rot=-1
            else:
                rot=math.tanh (acc+random.randint(-1,+1))
        
        #===========================================================================================#
        else:
            if(self.etat ==0): #première itration
                self.etat=[]
                fitness=0
                #generer gnome et le transformer en int en considérant -1 comme 2
                f=1
                s=0
                for i in range(14):  # taille du genome 
                    s+=f*(randint(0,+2)) 
                    f=f*10
                self.etat=s
            else :
                #transformer state en un tableau
                params=[]
                l=self.etat
                for i in range(14):
                    if(l%10 == 2):
                        params.append(-1)
                    else:
                        params.append(l%10)
                    l=l//10
                #les cases à consulter selon le genome
                cases=[(1,0),(1,1),(-1,1),(-1,0),(0,0),(1,0),(1,1),(-1,1),(-1,0),(0,0),(-1,-1),(1,-1),(-1,-1),(1,-1)]
               
                """ Evaluation : generation des enfant et test """
                select=params
                fitnessEnf=0
                ######################
                #variant la population 
                ######################
                l=[]
                for j in range(8): #genere aleatoirement des indice à flip
                    l.append(randint(0,len(params)-1)) 
                enfants=[]
                for i in l : #genere des enfants
                    temp=params.copy()
                    if (temp[i] == 0):
                        temp[i]=-1
                    elif (temp[i] == 1):
                        temp[i]=0
                    else :
                        temp[i]=1
                    enfants.append(temp)
                enfants.append(params)
                #####################
                
                ######## Evaluer les enfants #########
                for e in enfants :
                    fitness = 0
                    for k in range(len(e)):
                        case= occupancyGrid[(int(coord[0])+cases[k][0]*e[k])//16][(int(coord[1])+cases[k][1]*e[k])//16]
                        if( case != "A"):
                            fitness+=1
                    if(fitnessEnf < fitness):
                        fitnessEnf = fitness
                        select = e
                params=select
                ##########################
                
                #Transforme la liste en un int
                f=1
                s=0
                for i in range(14):  # taille du genome 
                    if(params[i] == -1):
                        s+=f*(2)
                    else:
                        s+=f*(params[i])
                    f=f*10
                self.etat=s
                """             """"""""""""""""""""""""""""               """
            #Amelioration 
            if(fitness!=0):
                sensorMinus170 = self.getDistanceAtSensor(0)
                sensorMinus40 = self.getDistanceAtSensor(2)
                sensorMinus20 = self.getDistanceAtSensor(3)
                sensorPlus20 = self.getDistanceAtSensor(4)
                sensorPlus40 = self.getDistanceAtSensor(5)
                sensorPlus170 = self.getDistanceAtSensor(7)   
                
                t =  math.tanh( sensorMinus170 * params[10] + sensorMinus40 * params[0] + sensorMinus20 * params[1] + sensorPlus20 * params[2] + sensorPlus40 * params[3]+ sensorPlus170 * params[11] + params[4] ) 
                rot =  math.tanh( sensorMinus170 * params[12] + sensorMinus40 * params[5] + sensorMinus20 * params[6] + sensorPlus20 * params[7] + sensorPlus40 * params[8] + sensorPlus170 * params[13] + params[9] )
            #Exploration
            else :
                t=1
                acc=0
                demi_tour=False
                fact=[1,-1,-1]#on evite les murs et les autres joueurs peu importe l'équipe 
                for i in range(len(SensorBelt)):
                    
                    if((i==0 or i==len(SensorBelt)-1)):#si on est poursuivit on tourne 
                        if(self.getObjectTypeAtSensor(i)==2 and self.getRobotInfoAtSensor(i)["teamname"]!=self.teamname):
                            self.etat+=1
                        else:
                            self.etat=0
                        if(self.etat>2):
                            demi_tour=True #quand quelqu'un le suit il tourne en rond
                            break;
                    acc+=fact[self.getObjectTypeAtSensor(i)]*((SensorBelt[i])/170)
                if(demi_tour==True):
                    rot=-1
                else:
                    rot=math.tanh (acc+random.randint(-1,+1))
        
        #===========================================================================================#
        self.setRotationValue(rot)
        self.setTranslationValue(t) 

        
		# monitoring (optionnel - changer la valeur de verbose)
        if verbose == True:
	        print ("Robot #"+str(self.id)+" [teamname:\""+str(self.teamname)+"\"] [variable mémoire = "+str(self.etat)+"] :")
	        for i in range(len(SensorBelt)):
	            print ("\tSenseur #"+str(i)+" (angle: "+ str(SensorBelt[i])+"°)")
	            print ("\t\tDistance  :",self.getDistanceAtSensor(i))
	            print ("\t\tType      :",self.getObjectTypeAtSensor(i)) # 0: rien, 1: mur ou bord, 2: robot
	            print ("\t\tRobot info:",self.getRobotInfoAtSensor(i)) # dict("id","centroid(x,y)","orientation") (si pas de robot: renvoi "None" et affiche un avertissement dans la console

        return

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    def step(self):
        self.stepController()
        self.move()

    def move(self):
        self.robot.forward(self.translationValue)
        self.robot.rotate(self.rotationValue)

    def getDistanceAtSensor(self,id):
        sensor_infos = sensors[self.robot] # sensor_infos est une liste de namedtuple (un par capteur).
        return min(sensor_infos[id].dist_from_border,maxSensorDistance) / maxSensorDistance

    def getObjectTypeAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border > maxSensorDistance:
            return 0 # nothing
        elif sensors[self.robot][id].layer == 'joueur':
            return 2 # robot
        else:
            return 1 # wall/border

    def getRobotInfoAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border < maxSensorDistance and sensors[self.robot][id].layer == 'joueur':
            otherRobot = sensors[self.robot][id].sprite
            info = {'id': otherRobot.numero, 'centroid': otherRobot.get_centroid(), 'orientation': otherRobot.orientation(), 'teamname': otherRobot.teamname }
            return info
        else:
            #print ("[WARNING] getPlayerInfoAtSensor(.): not a robot!")
            return None

    def setTranslationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxTranslationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
        self.translationValue = value

    def setRotationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxRotationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxRotationSpeed
        else:
            value = value * maxRotationSpeed
        self.rotationValue = value


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Agent "B"            '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

class AgentTypeB(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    agentType = "B"
    etat = 0
    
    translationValue = 0
    rotationValue = 0

    def __init__(self,robot):
        self.id = AgentTypeB.agentIdCounter
        AgentTypeB.agentIdCounter = AgentTypeB.agentIdCounter + 1
        #print ("robot #", self.id, " -- init")
        self.robot = robot
        self.robot.teamname = self.teamname


    def getType(self):
        return self.agentType

    def getRobot(self):
        return self.robot


    teamname = "Equipe BLEUE" # A modifier avec le nom de votre équipe

    def stepController(self):

        color( (0,0,255) )
        circle( *self.getRobot().get_centroid() , r = 22) # cercle bleu autour du robot
        
        #
        # SECONDE EQUIPE
        # COPIER-COLLER ICI LE CODE DE VOTRE FONCTION STEPCONTROLLER()
        #
        # vvvvvvvvvvvvv

        distGauche = self.getDistanceAtSensor(2)
        distGauche2= self.getDistanceAtSensor(3)
        distGauche3= self.getDistanceAtSensor(1)
        distDroite = self.getDistanceAtSensor(5)
        distDroite2= self.getDistanceAtSensor(4)
        distDroite3= self.getDistanceAtSensor(6)
        
        if distGauche < distDroite:
            self.setRotationValue( +1 )
        elif distGauche > distDroite:
            self.setRotationValue( -1 )
        elif distGauche2 < distDroite2:
            self.setRotationValue( +0.5 )
        elif distGauche2 > distDroite2:
            self.setRotationValue( -0.5 )
        elif distGauche3 < distDroite3:
            self.setRotationValue( +0.75 + 0.25*random() )
        elif distGauche3 > distDroite3:
            self.setRotationValue( -0.75 - 0.25*random() )
        else:
            self.setRotationValue( 0.1*random()-.05 )
    
        self.etat = self.etat + 1
        if self.etat % 1000 < 5:
            self.setTranslationValue(-1)
            if self.etat % 2000 == 0:
                self.setRotationValue( +1 )
            else:
                self.setRotationValue( -1 )
        else:
            self.setTranslationValue(+1)
        
        # ^^^^^^^^^^^^^
        #
        #
        #

        return

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


    def step(self):
        self.stepController()
        self.move()

    def move(self):
        self.robot.forward(self.translationValue)
        self.robot.rotate(self.rotationValue)

    def getDistanceAtSensor(self,id):
        sensor_infos = sensors[self.robot] # sensor_infos est une liste de namedtuple (un par capteur).
        return min(sensor_infos[id].dist_from_border,maxSensorDistance) / maxSensorDistance

    def getObjectTypeAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border > maxSensorDistance:
            return 0 # nothing
        elif sensors[self.robot][id].layer == 'joueur':
            return 2 # robot
        else:
            return 1 # wall/border

    def getRobotInfoAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border < maxSensorDistance and sensors[self.robot][id].layer == 'joueur':
            otherRobot = sensors[self.robot][id].sprite
            info = {'id': otherRobot.numero, 'centroid': otherRobot.get_centroid(), 'orientation': otherRobot.orientation(), 'teamname': otherRobot.teamname }
            return info
        else:
            #print ("[WARNING] getPlayerInfoAtSensor(.): not a robot!")
            return None


    def setTranslationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxTranslationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
        self.translationValue = value

    def setRotationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxRotationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxRotationSpeed
        else:
            value = value * maxRotationSpeed
        self.rotationValue = value


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions init/step  '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''


def setupAgents():
    global screen_width, screen_height, nbAgents, agents, game

    # Make agents

    nbAgentsTypeA = nbAgentsTypeB = nbAgents // 2
    nbAgentsCreated = 0

    for i in range(nbAgentsTypeA):
        p = game.add_players( (16 , 200+32*i) , None , tiled=False)
        p.oriente( 0 )
        p.numero = nbAgentsCreated
        nbAgentsCreated = nbAgentsCreated + 1
        if invertInitPop == True:
            agents.append(AgentTypeB(p))
        else:
            agents.append(AgentTypeA(p))

    for i in range(nbAgentsTypeB):
        p = game.add_players( (486 , 200+32*i) , None , tiled=False)
        p.oriente( 180 )
        p.numero = nbAgentsCreated
        nbAgentsCreated = nbAgentsCreated + 1
        if invertInitPop == True:
            agents.append(AgentTypeA(p))
        else:
            agents.append(AgentTypeB(p))

    game.mainiteration()


def setupArena0(): # the void
    return

def setupArena1(): # the check point
    for i in range(6,13):
        addObstacle(row=6,col=i)
    for i in range(3,10):
        addObstacle(row=9,col=i)
    for i in range(0,6):
        addObstacle(row=i,col=8)
    for i in range(10,16):
        addObstacle(row=i,col=6)
    addObstacle(row=4,col=12)
    addObstacle(row=5,col=12)
    addObstacle(row=6,col=12)
    addObstacle(row=11,col=3)
    addObstacle(row=10,col=3)
    addObstacle(row=9,col=3)
    return

def setupArena2(): # the presque-wall
    for i in range(1,15):
        addObstacle(row=i,col=7)
    return

def setupArena3(): # the fortress
    for i in range(3,12):
        if i != 7:
            addObstacle(row=i,col=6)
            addObstacle(row=i,col=9)
    for i in range(2,14):
        addObstacle(row=0,col=i)
        addObstacle(row=15,col=i)
    for i in range(2,14):
        addObstacle(row=i,col=2)
        addObstacle(row=i,col=13)
    return

def setupArena4(): # the corridors
    for i in range(2,14):
        for j in range(1,16,2):
            addObstacle(row=j,col=i)
    return

'''
def setupArena5(): # the vault
    for i in range(0,5):
        addObstacle(row=11,col=i)
        addObstacle(row=4,col=i)
        addObstacle(row=11,col=11+i)
        addObstacle(row=4,col=11+i)
    for i in range(1,3):
        addObstacle(row=11-i,col=11)
        addObstacle(row=4+i,col=4)
    addObstacle(row=5,col=11)
    addObstacle(row=10,col=4)
    return
'''

def updateSensors():
    global sensors 
    # throw_rays...(...) : appel couteux (une fois par itération du simulateur). permet de mettre à jour le masque de collision pour tous les robots.
    sensors = throw_rays_for_many_players(game,game.layers['joueur'],SensorBelt,max_radius = maxSensorDistance+game.player.diametre_robot() , show_rays=showSensors)

def stepWorld():
    efface()
    
    updateSensors()

    # chaque agent se met à jour. L'ordre de mise à jour change à chaque fois (permet d'éviter des effets d'ordre).
    shuffledIndexes = [i for i in range(len(agents))]
    shuffle(shuffledIndexes)
    for i in range(len(agents)):
        agents[shuffledIndexes[i]].step()
        # met à jour la grille d'occupation
        coord = agents[shuffledIndexes[i]].getRobot().get_centroid()
        occupancyGrid[int(coord[0])//16][int(coord[1])//16] = agents[shuffledIndexes[i]].getType() # first come, first served
    return


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions internes   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

def addObstacle(row,col):
    # le sprite situe colone 13, ligne 0 sur le spritesheet
    game.add_new_sprite('obstacle',tileid=(0,13),xy=(col,row),tiled=True)

class MyTurtle(Turtle): # also: limit robot speed through this derived class
    maxRotationSpeed = maxRotationSpeed # 10, 10000, etc.
    def rotate(self,a):
        mx = MyTurtle.maxRotationSpeed
        Turtle.rotate(self, max(-mx,min(a,mx)))

def displayOccupancyGrid():
    global iteration
    nbA = nbB = nothing = 0

    for y in range(screen_height//16):
        for x in range(screen_width//16):
            sys.stdout.write(occupancyGrid[x][y])
            if occupancyGrid[x][y] == "A":
                nbA = nbA+1
            elif occupancyGrid[x][y] == "B":
                nbB = nbB+1
            else:
                nothing = nothing + 1
        sys.stdout.write('\n')

    sys.stdout.write('Time left: '+str(maxIterations-iteration)+'\n')
    sys.stdout.write('Summary: \n')
    sys.stdout.write('\tType A: ')
    sys.stdout.write(str(nbA))
    sys.stdout.write('\n')
    sys.stdout.write('\tType B: ')
    sys.stdout.write(str(nbB))
    sys.stdout.write('\n')
    sys.stdout.write('\tFree  : ')
    sys.stdout.write(str(nothing))
    sys.stdout.write('\n')
    sys.stdout.flush() 

    return nbA,nbB,nothing

def onExit():
    if running == True:
        ret = displayOccupancyGrid()
        print ("\n\n=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
        if ret[0] > ret[1]:
            print ("Robots type A (\"" + str(AgentTypeA.teamname) + "\") wins!")
        elif ret[0] < ret[1]:
            print ("Robots type B (\"" + str(AgentTypeB.teamname) + "\") wins!")
        else:
            print ("Nobody wins!")
        print ("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n")
        print ("\n[Simulation::stop]")


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Main loop            '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

init('empty',MyTurtle,screen_width,screen_height) # display is re-dimensioned, turtle acts as a template to create new players/robots
game.auto_refresh = False # display will be updated only if game.mainiteration() is called
game.frameskip = frameskip
atexit.register(onExit)

print ('Number of arguments:', len(sys.argv), 'arguments.')
print ('Argument List:', str(sys.argv))

if len(sys.argv) > 1:
    arena = int(sys.argv[1])
    print ("Arena: ", str(arena), "(user-selected)")
    if len(sys.argv) == 3:
        if sys.argv[2] == "True" or sys.argv[2] == "true":
            invertInitPop = True
        if invertInitPop == False:
            print ("Equipe BLEUE commence à gauche.")
        else:
            print ("Equipe VERTE commence à gauche.")
else:
    print ("Arena: ", str(arena), "(default), équipe BLEUE commence à gauche.")

if arena == 0:
    setupArena0()
elif arena == 1:
    setupArena1()
elif arena == 2:
    setupArena2()
elif arena == 3:
    setupArena3()
elif arena == 4:
    setupArena4()
else:
    print ("labyrinthe inconnu.")
    exit (-1)

running = True

setupAgents()
game.mainiteration()

iteration = 0
while iteration != maxIterations:
    stepWorld()
    game.mainiteration()
    if iteration % 200 == 0:
        displayOccupancyGrid()
    iteration = iteration + 1

