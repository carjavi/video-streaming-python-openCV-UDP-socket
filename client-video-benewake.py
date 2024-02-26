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