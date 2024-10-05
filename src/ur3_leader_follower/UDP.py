import socket
import struct

class UDPSender(object):
    def __init__(self, udp_ip, udp_port):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        #rospy.loginfo("UDP sender sending to ip {}, port {}, timestamp {}".format(self.udp_ip, self.udp_port, rospy.Time.now()))
        #creating UDP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_msg_bytes(self, data):
        #trying to send the packet
        self.client_socket.sendto(data, (self.udp_ip, self.udp_port))

    def close(self):
        self.client_socket.close()
        
class UDPReceiver(object):
    def __init__(self, udp_ip, udp_port, udp_timeout, format='<fffffff'):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.udp_timeout = udp_timeout
        self.format = format
        #rospy.loginfo("UDP Receiver binding to ip {}, port {}, timeout {}".format(self.udp_ip, self.udp_port, self.udp_timeout))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(self.udp_timeout)

    def wait_for_bytes(self, callback):
        while True:
            # buffer size is 1024 bytes
            data, from_addr = self.sock.recvfrom(124) 
            #rospy.loginfo("Received packet")
            unpacked_data = list(struct.unpack(self.format, data))
            callback(unpacked_data)
 
