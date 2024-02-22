# pip install opencv-python
# pip install imutils
# pip install sockets


# This is client code to receive video frames over UDP
import cv2, imutils, socket
import numpy as np
import time
import base64

# Se crea un socket UDP (socket.SOCK_DGRAM) y se configura su tamaño de búfer para manejar grandes cantidades de datos.
BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)

# Envío de un mensaje de prueba al servidor:
host_name = socket.gethostname()
host_ip = '192.168.1.89'#  socket.gethostbyname(host_name)
print('Connected to the server:',host_ip) # print(host_ip)
port = 5000
message = b'Hello'
client_socket.sendto(message,(host_ip,port))


fps,st,frames_to_count,cnt = (0,0,20,0)
while True:
	packet,_ = client_socket.recvfrom(BUFF_SIZE)
	# Se recibe un paquete UDP y se decodifica desde base64. Se decodifica la imagen utilizando cv2.imdecode. Se muestra la imagen recibida en una ventana utilizando OpenCV
	data = base64.b64decode(packet,' /')
	npdata = np.fromstring(data,dtype=np.uint8)
	frame = cv2.imdecode(npdata,1)
	frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
	cv2.imshow("RECEIVING VIDEO",frame)
	key = cv2.waitKey(1) & 0xFF

	# Salida del bucle y cierre del socket
	if key == ord('q'):
		client_socket.close()
		break

	# Actualización del contador de FPS
	if cnt == frames_to_count:
		try:
			fps = round(frames_to_count/(time.time()-st))
			st=time.time()
			cnt=0
		except:
			pass
	cnt+=1


# Nota: Al salir con q tambien se debe reiniciar el server
