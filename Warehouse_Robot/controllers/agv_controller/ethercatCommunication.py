import pyads

ip = "254.254.254.254"

class Communication():
    def set_ip(self, ip):
        self.ip = ip
        
    def get_ip(self):
        return self.ip
        