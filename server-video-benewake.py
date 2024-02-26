# python3
# sudo apt-get install python3-opencv -y
# pip install imutils
# pip install sockets


# This is server code to send video frames over UDP
import cv2, imutils, socket
import numpy as np
import base64

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