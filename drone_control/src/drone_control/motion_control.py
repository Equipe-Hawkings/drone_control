#!/usr/bin/env python

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 02/12/2021                                         ##
############################################################################
## Modulo ROS que agrupa os status principais para o controle de um drone ##
## de forma "paralela". Servirah de base para conseguir movimentar drones ##
## via ROS.                                                               ##
############################################################################

import rospy    # Biblioteca padrao do ROS para python
from mavros_msgs.srv import *    # Tipagens ROS de messages nos Services
# Tipagens ROS de messages usadas como variaveis
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

# Classe das principais funcoes de movimentacao de um drone
class MotionControl:
    def __init__(self):
        ### Variaveis ###
        ## Instancia os objetos que armazenarao parametros utilizados nos metodos ##

        self.modoDeVoo = State()    # Modo de voo atual
        self.destino = PoseStamped()    # Coordenas x, y e z -> destino.pose.position.(x, y ou z)
        self.rate = rospy.Rate(60)    # Frenquencia de envio dos dados (Hz)
        self.posicao = PoseStamped()    # Posicao atual do drone

        ### Definicao de Parametros ###
        ## Abrange os Services e os Publishers, que servem para "setar" parametros ao drone ##

            ### Services ###
                ### Checagem de Servicos ###
                ## Tenta conectar com os servicos ##

        service_timeout = 30    # Tempo para conectar aos Services
        rospy.loginfo("Conectando ROS Services...")
        try:
            rospy.wait_for_service('/mavros/set_mode',service_timeout)
            rospy.wait_for_service('/mavros/cmd/land',service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming',service_timeout)
            rospy.loginfo("Services conectadas com sucesso!")
        except:
            rospy.logerr("Erro ao conectar ROS Services!")

                ### Atribuicao de Servicos ###

        self.set_modoDeVoo = rospy.ServiceProxy('/mavros/set_mode', SetMode)    # Altera o modo de voo
        self.pouso = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)    # Inicia o modo de pouso (land_mode)
        self.droneArmado = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)    # Bool para drone armado ou nao

            ### Publishers ###

        self.set_destino = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)    # Publica as coordenadas de destino
        
        ### Retorno de Parametros ###
        ## Abrange os Services, que retornam parametros referentes ao drone ##

            ### Subscribers ###

        self.modoDeVoo_atual = rospy.Subscriber('/mavros/state', State, self.get_modoDeVoo)    # Retorna o modo de voo
        self.posicao_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_posicao)    # Retorna a posicao atual do drone
       

    ### Callbacks ###
    ## Funcoes de "callbacks" dos subscribers ##

    def get_modoDeVoo(self, modoVoo):
        self.modoDeVoo = modoVoo
    
    
    def get_posicao(self, posicao):
        self.posicao.pose.position.x = posicao.pose.position.x
        self.posicao.pose.position.y = posicao.pose.position.y
        self.posicao.pose.position.z = posicao.pose.position.z
    

    ### Funcoes de movimentacao ###
    ## Define acoes para a movimentacao do drone ##

    def armar(self, state):
        armado = self.droneArmado(state)
        if armado.success == True:
            rospy.loginfo("Drone Armado.")
        else:
            rospy.logwarn("Erro ao armar drone.")

    def pousar(self):
        self.pouso(altitude = 0, latitude = 0, longitude = 0, yaw = 0)
        rospy.loginfo("Pousando Drone...")
     
    # Verifica se o drone chegou aa posicao determinada, para controla o envio das coordenadas de sertino
    def chegou(self, setMax_erro):
        erro = setMax_erro
        if (abs(self.destino.pose.position.x - self.posicao.pose.position.x) < erro) and (abs(self.destino.pose.position.y - self.posicao.pose.position.y) < erro) and (abs(self.destino.pose.position.z - self.posicao.pose.position.z) < erro):
            return True
        else:
            return False

    # Define o modo de voo
    def setModoDeVoo(self, modoVoo):
        self.set_modoDeVoo(custom_mode=modoVoo)
        if self.modoDeVoo.mode == modoVoo:
            rospy.loginfo(f'Modo de voo atual: {self.modoDeVoo.mode}.')
        else:
            rospy.logwarn("Erro ao alterar modo e voo.")

    def setpoint(self, x, y, z):
        self.destino.pose.position.x = x
        self.destino.pose.position.y = y
        self.destino.pose.position.z = z
        self.destino.header.stamp = rospy.Time.now()
        self.set_destino.publish(self.destino)
        self.rate.sleep()
