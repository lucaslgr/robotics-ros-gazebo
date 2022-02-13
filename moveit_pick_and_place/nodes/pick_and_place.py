#!/usr/bin/env python3

# importacao das bibliotecas padrão do Python
import sys
import math
import time

# importacao ROS
import rospy

# importacao MoveIt e TF
import moveit_commander
from tf.transformations import quaternion_from_euler

# importacao msgs and srvs
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import tf2_geometry_msgs
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, Grasp, PlaceLocation
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Quaternion

## constantes com os valores de posicao do objeto a ser definido como objetivo do pick and place
# posicao central: x = 0.5, y = 0, z = 0.5
# range de posicoes possiveis na coordenada X: 0.47 ~ 0.65
# range de posicoes possiveis na coordenada Y: 0.15 ~ -0.14
# coordenada Z = 0.5 (fixo)

# classe que define um objeto posicao
class Position:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

# constroi um vetor de posicoes
target_object_positions = []
CURRENT_POSITION_INDEX = 0

# carrega todas posicoes a serem testadas dentro do vetor
def load_all_positions():
  target_object_positions.append(Position(0.5, 0, 0.5)) #11.669291496276855
  target_object_positions.append(Position(0.45, 0, 0.5)) #10.466451168060303
  target_object_positions.append(Position(0.48, -0.14, 0.5)) #16.2263286113739
  target_object_positions.append(Position(0.47, 0.14, 0.5)) #13.280328750610352
  target_object_positions.append(Position(0.47, -0.14, 0.5)) #11.447410821914673
  target_object_positions.append(Position(0.48, 0.16, 0.5)) #13.890687942504883
  target_object_positions.append(Position(0.48, -0.16, 0.5)) #16.01171636581421
  target_object_positions.append(Position(0.50, 0.13, 0.5)) #FALHA
  target_object_positions.append(Position(0.45, 0.13, 0.5)) #FALHA
  target_object_positions.append(Position(0.48, 0.17, 0.5)) #11.45394492149353
  target_object_positions.append(Position(0.48, 0.15, 0.5)) #10.12423523523432
  target_object_positions.append(Position(0.51, 0.16, 0.5)) #FALHA
  target_object_positions.append(Position(0.50, 0.15, 0.5)) #11.195968389511108
  target_object_positions.append(Position(0.46, 0.12, 0.5)) #FALHA
  target_object_positions.append(Position(0.48, 0.12, 0.5)) #8.676451683044434
  target_object_positions.append(Position(0.50, 0.11, 0.5)) #FALHA
  target_object_positions.append(Position(0.51, 0.12, 0.5)) #FALHA
  target_object_positions.append(Position(0.48, 0.13, 0.5)) #10.576608180999756
  target_object_positions.append(Position(0.59, 0.13, 0.5)) #12.647433042526245
  target_object_positions.append(Position(0.59, 0.15, 0.5)) #11.828290939331055
  target_object_positions.append(Position(0.59, 0.12, 0.5)) #FALHA
  target_object_positions.append(Position(0.58, 0.15, 0.5)) #13.841835975646973
  target_object_positions.append(Position(0.58, 0.14, 0.5)) #FALHA
  target_object_positions.append(Position(0.58, 0.16, 0.5)) #13.096974849700928
  target_object_positions.append(Position(0.58, 0.13, 0.5)) #11.303464412689209
  target_object_positions.append(Position(0.58, -0.14, 0.5)) #FALHA
  target_object_positions.append(Position(0.59, 0.17, 0.5)) #10.808913469314575
  target_object_positions.append(Position(0.46, 0.17, 0.5)) #FALHA
  target_object_positions.append(Position(0.46, 0.13, 0.5)) #FALHA
  target_object_positions.append(Position(0.47, 0.11, 0.5)) #13.088848114013672
  target_object_positions.append(Position(0.47, 0.15, 0.5)) #FALHA
  target_object_positions.append(Position(0.45, 0.16, 0.5)) #FALHA


# constante com o nome do objeto que sera manipulado
OBJECT_TARGET_NAME = "object_target"

# metodo que e responsavel por abrir a garra (abre o dedo1 e o dedo2)
def open_gripper(posture):
    """
    - Abrir a garra -
    Parametros: Posicao da Garra
        posture : trajectory_msgs.msg.JointTrajectory
    """

    # adiciona as duas juntas dos dedos da garra do panda
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    # seta os dedos para abrir com uma largura suficiente para caber o objeto desejado
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.04
    posture.points[0].positions[1] = 0.04
    posture.points[0].time_from_start = rospy.Duration(0.5)

# metodo responsavel por fechar a garra (fecha o dedo1 e o dedo2)
def close_gripper(posture):
    """
    - Fechar a garra -
    Parametros: Posicao da Garra
        posture : trajectory_msgs.msg.JointTrajectory
    """

    # adiciona as duas juntas dos dedos da garra do panda
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    # seta os dedos para fechar
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.00
    posture.points[0].positions[1] = 0.00
    posture.points[0].time_from_start = rospy.Duration(0.5)

# metodo responsavel por pegar o objeto
def pick(move_group):
    """
    - Pegar o objeto -
    Parametros: 
    ----------
    Group : moveit_commander.RobotCommander
                    Moveit_commander move group.
    """

    # cria um vetor de pegadas a serem tentadas mas atualmente apenas uma unica pegada é executada
    grasps = [Grasp() for i in range(1)]

    ## Setando a posicao da pegada
    # essa e a posicao do panda_link8.
    # do panda_link8 até a palma do efetor final a distancia e 0.058, o cubo comeca 0.01 antes 5.0 (metade do comprimento do cubo).
    # daí a posicao do panda_link8 = (comprimento do cubo/2 - distancia entre panda_link8 e a palma do efetor final - alguma espaco extra)
    grasps[0].grasp_pose.header.frame_id = "panda_link0"
    orientation = quaternion_from_euler(-math.pi/2, -math.pi/4, -math.pi/2)
    grasps[0].grasp_pose.pose.orientation.x = orientation[0]
    grasps[0].grasp_pose.pose.orientation.y = orientation[1]
    grasps[0].grasp_pose.pose.orientation.z = orientation[2]
    grasps[0].grasp_pose.pose.orientation.w = orientation[3]
    
    grasps[0].grasp_pose.pose.position.x = target_object_positions[CURRENT_POSITION_INDEX].x - 0.085
    grasps[0].grasp_pose.pose.position.y = target_object_positions[CURRENT_POSITION_INDEX].y
    grasps[0].grasp_pose.pose.position.z = target_object_positions[CURRENT_POSITION_INDEX].z

    ## Setando a aplicacao de pre pegada
    # definindo com relação ao frame_id
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0"
    # a direcao e setada como eixo x positivo
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0
    grasps[0].pre_grasp_approach.min_distance = 0.095
    grasps[0].pre_grasp_approach.desired_distance = 0.115

    ## Setando o recuo pos pegada
    # definindo com relação ao frame_id
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0"
    # a direcao e setada como eixo z positivo
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0
    grasps[0].post_grasp_retreat.min_distance = 0.1
    grasps[0].post_grasp_retreat.desired_distance = 0.25

    ## Setando a postura do efetor final antes da pegada
    open_gripper(grasps[0].pre_grasp_posture)

    ## Setando a postura do efetor final durante a pegada
    close_gripper(grasps[0].grasp_posture)

    ## Setando a superfície de suporte como table1
    move_group.set_support_surface_name("table1")

    ## Chama o metodo pick para pegar o objeto usando as pegadas fornecidas como parametro
    move_group.pick(OBJECT_TARGET_NAME, grasps)

def place(group):
    """
    - Coloca o objeto -
    Parametros: 
    ----------
    Group : moveit_commander.RobotCommander
                    Moveit_commander move group.
    """

    # cria um vetor de lugares para tentar colocar o objeto manipulado, porém aqui será somente um lugar
    place_location = [PlaceLocation() for i in range(1)]

    ###############################################################################
    # defindo a pose para se colocar o objeto, i.e, onde e como colocá-lo
    place_location[0].place_pose.header.frame_id = "panda_link0"
    orientation = quaternion_from_euler(0, 0, math.pi / 2)
    place_location[0].place_pose.pose.orientation.x = orientation[0]
    place_location[0].place_pose.pose.orientation.y = orientation[1]
    place_location[0].place_pose.pose.orientation.z = orientation[2]
    place_location[0].place_pose.pose.orientation.w = orientation[3]

    place_location[0].place_pose.pose.position.x = 0
    place_location[0].place_pose.pose.position.y = 0.5
    place_location[0].place_pose.pose.position.z = 0.5
    ################################################################################

    # estabelecendo a abordagem pre-place em relação ao frame_id com a direção em z negativa 
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
    place_location[0].pre_place_approach.direction.vector.z = (
        -1.0
    ) 
    place_location[0].pre_place_approach.min_distance = 0.095
    place_location[0].pre_place_approach.desired_distance = 0.115

    # estabelecendo o recuo post-grasp em relaçaõ ao frame_id
    # direção x é negativa
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
    place_location[
        0
    ].post_place_retreat.direction.vector.y = (
        -1.0
    )  
    place_location[0].post_place_retreat.min_distance = 0.1
    place_location[0].post_place_retreat.desired_distance = 0.25

    ## Setando a postura do efetor final apos colocar o objeto
    open_gripper(place_location[0].post_place_posture)
 
    ## Estabelece suporte para a superfície da mesa 2, i.e, a mesa que será colocada o objeto manipulado
    group.set_support_surface_name("table2")

    ## Chama o place para colocar o objeto com a localização dada
    group.place(OBJECT_TARGET_NAME, place_location[0])


def add_collision_objects(planning_scene_interface):

    # Aqui será criado os três objetos que estão no cenário. Duas "mesas" e um "paralelepídeo" que será o objeto que se deseja
    # mover
    collision_objects_names = [str for i in range(3)]
    collision_object_sizes = [str for i in range(3)]
    collision_objects = [PoseStamped() for i in range(3)]

    ## Esse trecho de codigo e responsavel por setar nome, frame_id, tamanho e posicao de um objeto no cenário
    # adiciona a primeira mesa
    collision_objects_names[0] = "table1"
    collision_objects[0].header.frame_id = "panda_link0"
    # define as dimensões da primeira mesa
    collision_object_sizes[0] = (0.2, 0.4, 0.4)  
    # define a pose da mesa
    collision_objects[0].pose.position.x = 0.5
    collision_objects[0].pose.position.y = 0
    collision_objects[0].pose.position.z = 0.2

    ##########################################################
    # aqui será repetido o processo acima para inserir no cenário a segunda mesa
    collision_objects_names[1] = "table2"
    collision_objects[1].header.frame_id = "panda_link0"
    collision_object_sizes[1] = (0.4, 0.2, 0.4)   
    collision_objects[1].pose.position.x = 0
    collision_objects[1].pose.position.y = 0.5
    collision_objects[1].pose.position.z = 0.2
    ##########################################################

    ## Inserindo o objeto que vai ser manipulado pelo braço no cenário
    collision_objects_names[2] = OBJECT_TARGET_NAME
    collision_objects[2].header.frame_id = "panda_link0"
    # define as dimensões dele
    collision_object_sizes[2] = (0.02, 0.02, 0.2)  # Box size
    # define a pose
    collision_objects[2].pose.position.x = target_object_positions[CURRENT_POSITION_INDEX].x
    collision_objects[2].pose.position.y = target_object_positions[CURRENT_POSITION_INDEX].y
    collision_objects[2].pose.position.z = target_object_positions[CURRENT_POSITION_INDEX].z

    # Adicionando todos os objetos no cenário no Rviz
    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)


if __name__ == "__main__":

    # inicializa o no do ROS
    rospy.init_node("pick_and_place_using_panda_arm")

    # carrega o vetor de posicoes possiveis para o objeto alvo que será deslocado
    load_all_positions()

    # inicializa o moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # conecta moveit services
    rospy.loginfo(
        "Conneting moveit default moveit 'apply_planning_scene' service.")
    rospy.wait_for_service("apply_planning_scene")
    try:
        planning_scene_srv = rospy.ServiceProxy(
            "apply_planning_scene", ApplyPlanningScene
        )
        rospy.loginfo("Moveit 'apply_planning_scene' service found!")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Moveit 'apply_planning_scene' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            planning_scene_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    # cria um robot commander
    robot = moveit_commander.RobotCommander(
        robot_description="robot_description", ns="/"
    )
    rospy.logdebug("Robot Groups: %s", robot.get_group_names())

    # pegando informação sobre o mundo e atualizando o entendimento que o robo tem do mundo
    move_group = robot.get_group("panda_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")

    # especificando o planejador de trajetorias que queremos usar
    move_group.set_planner_id("TRRTkConfigDefault")

    # cria um DisplayTrajectory que é um no publicador do ROS para mostrar o planejamento no RViz
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )

    # espera um pouco para o ROS incializar o planning_scene_interface
    rospy.sleep(1.0)

    # adiciona os objetos de colisao
    add_collision_objects(planning_scene_interface)

    # espera um pouco para carregar os objetos
    rospy.sleep(1.0)

    #iniciando a contagem de tempo
    start_time = time.time()

    # pega o objeto ja pre definido passando como argumento o move_group do panda_arm
    pick(move_group)

    # espera 1seg
    delay = 1.0
    rospy.sleep(delay)

    # coloca o objeto 
    place(move_group)

    # imprimindo o tempo que levou a execucao menos o delay forçado de 1 segundo
    print("position %s: x=%s, y=%s, z=%s | --- %s seconds ---" % (
      CURRENT_POSITION_INDEX,
      target_object_positions[CURRENT_POSITION_INDEX].x,
      target_object_positions[CURRENT_POSITION_INDEX].y,
      target_object_positions[CURRENT_POSITION_INDEX].z,
      time.time() - start_time - delay)
    )