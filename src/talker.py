#!/usr/bin/env python
#Ros ne fonctionne qu en Python 2 
import rospy
# Attention a bien inclure chaque type de message
from std_msgs.msg import String

def talker():
	# Creation d un publisher avec
	# - le nom du topic
	# - le type de message
	# - la taille du buffer de messages
	pub = rospy.Publisher('chatter', String, queue_size=10)

	# Initialisation du node avec
	rospy.init_node('talker')

	# Frequence de la boucle
	rate = rospy.Rate(10) # 10hz

	# Boucle principale du programme (tant que pas de Ctrl-C)
	while not rospy.is_shutdown():
		# Creation d un message de type "String"
		hello_str = String()

		# Remplissage du contenu du message
		hello_str.data = "hello world %s" % rospy.get_time()

		# Affichage dans le terminal (equivalent d un print())
		rospy.loginfo(hello_str)

		# Publication du message
		pub.publish(hello_str)

		# Pause pour respecter la frequence de la boucle
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
