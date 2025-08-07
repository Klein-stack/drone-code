import socket

raspA_ip = "192.2.2.2"  #giving raspA ip address
raspA_port = 9999       #giving raspA port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((raspA_ip, raspA_port))

print("Socket has been successfully created!")

try:
	while True:
		data, addr = sock.recvfrom(1024)
		print(f"data has been received from {addr}")
		decoded = data.decode()
except:
	print("Stopped by user!")
finally:
	sock.close()
		
	

