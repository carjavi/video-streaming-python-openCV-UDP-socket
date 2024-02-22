<p align="center"><img src="./img/streaming.gif" width="400" alt=" " /></p>
<h1 align="center"> Video Streaming Using Python OpenCV/ UDP protocol/ Socket Programming </h1> 
<h4 align="right">February 24</h4>

![Windows 11](https://img.shields.io/badge/Windows%2011-%230079d5.svg?style=for-the-badge&logo=Windows%2011&logoColor=white)

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
<br>

```cv2:``` Biblioteca OpenCV para procesamiento de imágenes y vídeo.<br>
```imutils:``` Ofrece algunas funciones útiles para el procesamiento de imágenes.<br>
```socket:``` Para la comunicación de red.<br>
```numpy as np:``` Para manipulación de matrices.<br>
```time:``` Para medir el tiempo.<br>
```base64:``` Para decodificar los datos recibidos.<br>

## install Library (server/client)
```
sudo apt update
sudo apt install python3-pip
sudo apt-get install python3-opencv -y
pip install imutils
pip install sockets
pip install numpy
```


# Server (RPI with Ubuntu 20.04)
```
# This is server code to send video frames over UDP
import cv2, imutils, socket
import numpy as np
import time
import base64

# Se crea un socket UDP (socket.SOCK_DGRAM) y se configura su tamaño de búfer para manejar grandes cantidades de datos.
BUFF_SIZE = 65536
server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)

# Asignación del host y el puerto
host_name = socket.gethostname()
host_ip = '192.168.1.89' #socket.gethostbyname(host_name)
print(host_ip)
port = 5000 # 
socket_address = (host_ip,port)
server_socket.bind(socket_address)
print('Listening at:',socket_address)

# Inicialización de la captura de video
vid = cv2.VideoCapture(0) #  replace 'rocket.mp4' with 0 for webcam

fps,st,frames_to_count,cnt = (0,0,20,0)

# Bucle principal para recibir el mensaje de conexión y transmitir fotogramas de vídeo
while True:
	# El servidor espera recibir un mensaje de conexión.
	msg,client_addr = server_socket.recvfrom(BUFF_SIZE)
	print('GOT connection from ',client_addr)
	WIDTH=400
	# Bucle interno para transmitir fotogramas de video
	while(vid.isOpened()):
		_,frame = vid.read()
		frame = imutils.resize(frame,width=WIDTH)
		encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
		message = base64.b64encode(buffer)
		server_socket.sendto(message,client_addr)
		frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
		#cv2.imshow('TRANSMITTING VIDEO',frame) # hace que se visualice en el server. si no hay GUI debe estar desabilitada
		key = cv2.waitKey(1) & 0xFF

		# Salida del bucle y cierre del socket
		if key == ord('q'):
			server_socket.close()
			break

		# Cálculo de FPS
		if cnt == frames_to_count:
			try:
				fps = round(frames_to_count/(time.time()-st))
				st=time.time()
				cnt=0
			except:
				pass
		cnt+=1


```

# Client (PC windows)
```
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


```

<br>

---
Copyright &copy; 2022 [carjavi](https://github.com/carjavi). <br>
```www.instintodigital.net``` <br>
carjavi@hotmail.com <br>
<p align="center">
    <a href="https://instintodigital.net/" target="_blank"><img src="./img/developer.png" height="100" alt="www.instintodigital.net"></a>
</p>


