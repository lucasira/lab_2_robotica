
import rospy
import sys
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

#Relative paths
line = "path_line.txt"
sine = "path_sine.txt"
sqrt = "path_sqrt.txt"



posicion = []
with open(sys.argv[1]) as f: #lee el archivo
    linea = f.readlines()
    posicion = []
    for i in range(len(linea)):
        lista = linea[i].split(',')
        posicion.append([float(lista[0]),float(lista[1])])



def nodo(): #Crea el nodo que manda el path
    rospy.init_node( 'lectura') 
    pub = rospy.Publisher('/goal_list', Path, queue_size=10 )
    r = rospy.Rate(60.0)
    cuenta = 0
    while not rospy.is_shutdown() and cuenta <=30:
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        for i in posicion:
            pose = PoseStamped()
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            msg.poses.append(pose)
        pub.publish(msg)
        #print('mandando pose', cuenta)
        r.sleep()   
        cuenta += 1
    #print(msg.poses)
    #for i in msg.poses:
    #    print(i.pose.position.x)



nodo()

