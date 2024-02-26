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


# Server (RPI with Ubuntu 20.04) serverUDP.py
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

# Client (PC windows) clienteUPD.py
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

# Transmitiendo video y data del sensor de distancia Benewake sobre el video

### server-video-benewake.py
inf Benewake: https://github.com/carjavi/Benewake-TF03-LiDAR-Testing-UART

```
# This is server code to send video frames over UDP
import cv2, imutils, socket
import numpy as np
import base64

# Sensor TF03
import serial.tools.list_ports
import time
import numpy as np
ser = serial.Serial()
ser.port = '/dev/ttyUSB1'
ser.baudrate = 115200

# Salida del bucle, cierre del socket, serial port
try:
    if ser.is_open == False:
        try:
            ser.open()
        except:
            print('Open COM failed')
except KeyboardInterrupt:
    print('Interrupted, pressed ctrl-c button')
    if ser != None:
        ser.close()
		#server_socket.close()

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
		
		# Codificar fotograma
		message = base64.b64encode(buffer)
		
        # Obtener datos del Sensor de distancia TFmini 
		count = ser.in_waiting
		if count > 8:
			recv = ser.read(9)
			ser.reset_input_buffer()
			if recv[0] == 0x59 and recv[1] == 0x59:
				distance = np.int16(recv[2] + np.int16(recv[3]<<8))
				tfmini = distance
				#print('distance = %5d' % (distance))
				ser.reset_input_buffer()
		else:
			time.sleep(0.050) # 50ms
			
		# Enviar fotograma y Dato
		server_socket.sendto(message + b'|' + str(tfmini).encode(),client_addr)
		
        # ***** Anotar FPS y el Dato en el fotograma **************
		# frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
		# frame = cv2.putText(frame, 'Elapsed Time: ' + str(elapsed_time), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        # hace que se visualice el video en el server
        # cv2.imshow('TRANSMITTING VIDEO',frame)  #si no hay GUI debe estar desabilitada
		key = cv2.waitKey(1) & 0xFF

```

### client-video-benewake
```
import cv2
import numpy as np
import socket
import base64

# Configurar el socket UDP
BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)

# Conectar al servidor
host_ip = '192.168.1.89'  # Cambiar por la dirección IP del servidor
port = 5000
server_address = (host_ip, port)

# Envío de un mensaje de prueba al servidor
message = b'Hello'
client_socket.sendto(message, server_address)

# Inicializar ventana de visualización
cv2.namedWindow('RECEIVING VIDEO', cv2.WINDOW_NORMAL)

# Variables para el cálculo de FPS y contador
fps, st, frames_to_count, cnt = (0, 0, 20, 0)
counter_value = 0

# Bucle para recibir y mostrar el video
while True:
    # Recibir datos del servidor
    packet, _ = client_socket.recvfrom(BUFF_SIZE)
    
    # Decodificar el paquete desde base64
    data = packet.split(b'|')
    frame_data = data[0]
    distance_value = int(data[1])
    
    # Decodificar la imagen utilizando cv2.imdecode
    frame = base64.b64decode(frame_data)
    npdata = np.frombuffer(frame, dtype=np.uint8)
    frame = cv2.imdecode(npdata, 1)
    
    # Anotar el valor de la Distancia en el fotograma
    frame = cv2.putText(frame, 'Distance: ' + str(distance_value), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Mostrar el fotograma
    cv2.imshow("RECEIVING VIDEO", frame)
    key = cv2.waitKey(1) & 0xFF
    
    # Salir del bucle con la tecla 'q'
    if key == ord('q'):
        cv2.destroyAllWindows()
        client_socket.close()
        break

# Cerrar la ventana y el socket
# cv2.destroyAllWindows()
# client_socket.close()

```


<br>

---
Copyright &copy; 2022 [carjavi](https://github.com/carjavi). <br>
```www.instintodigital.net``` <br>
carjavi@hotmail.com <br>
<p align="center">
    <a href="https://instintodigital.net/" target="_blank"><img src="./img/developer.png" height="100" alt="www.instintodigital.net"></a>
</p>


