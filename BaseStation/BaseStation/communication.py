import socket
import threading

class WiFiHandler:
    def __init__(self, remote_ip, remote_port, local_listen_port, on_receive_callback):
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.local_listen_port = local_listen_port # Port for this handler to listen on
        self.socket = None
        self.connected = False
        self.lock = threading.Lock()
        self.receive_thread = None
        self.on_receive_callback = on_receive_callback
        self.is_listening = False

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Bind to a specific port for receiving messages for this robot
            self.socket.bind(('', self.local_listen_port))
            self.connected = True # Indicates socket is ready for sending
            self.is_listening = True
            self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.receive_thread.start()
            print(f"WiFiHandler for robot at {self.remote_ip} listening on port {self.local_listen_port}, sending to port {self.remote_port}")
            return True
        except Exception as e:
            print(f"Failed to bind/listen on port {self.local_listen_port} for robot {self.remote_ip}: {e}")
            if self.socket:
                self.socket.close()
            self.socket = None
            return False

    def disconnect(self):
        self.is_listening = False
        self.connected = False
        if self.socket:
            try:
                # To unblock recvfrom, send a dummy packet to itself
                # This is a common workaround for UDP socket blocking on close
                if self.local_listen_port > 0: # Ensure port is valid
                    dummy_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    dummy_socket.sendto(b'shutdown', ('127.0.0.1', self.local_listen_port))
                    dummy_socket.close()
            except Exception as e:
                print(f"Error sending dummy packet for shutdown: {e}")
            
            self.socket.close()
            self.socket = None
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0) # Add timeout
        print(f"Disconnected WiFiHandler for robot {self.remote_ip}")


    def send(self, message):
        if self.socket and self.connected: # Check 'connected' for ability to send
            try:
                self.socket.sendto(message.encode(), (self.remote_ip, self.remote_port))
                # print(f"Sent to {self.remote_ip}:{self.remote_port}: {message}") # Optional: for debugging
                return True
            except Exception as e:
                print(f"Failed to send message to {self.remote_ip}:{self.remote_port}: {e}")
                return False
        else:
            print(f"Not connected to send to {self.remote_ip}")
            return False

    def receive_loop(self):
        while self.is_listening:
            if not self.socket:
                break
            try:
                data, addr = self.socket.recvfrom(1024) # Buffer size
                if not self.is_listening: # Check again after blocking call
                    break
                if data:
                    if self.on_receive_callback:
                        with self.lock: # Ensure thread-safe callback execution
                            self.on_receive_callback(data.decode())
                    # else:
                        # print(f"Data from unexpected IP {addr[0]} on port {self.local_listen_port}")
            except socket.timeout: # If socket timeout is set
                continue
            except OSError as e: # Handle socket closed errors
                if self.is_listening: # Only print if we weren't expecting to close
                    print(f"Socket error in receive_loop for {self.remote_ip} on port {self.local_listen_port}: {e}")
                break # Exit loop if socket is closed or error
            except Exception as e:
                if self.is_listening:
                    print(f"Error receiving data for {self.remote_ip} on port {self.local_listen_port}: {e}")
                break
        print(f"Receive loop stopped for robot {self.remote_ip} on port {self.local_listen_port}.")


class RefBoxHandler:
    def __init__(self, ip, port, on_receive_callback, on_disconnect_callback):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        self.running = False
        self.on_receive_callback = on_receive_callback
        self.on_disconnect_callback = on_disconnect_callback

    def connect(self):
        if not self.connected:
            self.running = True
            # Ensure the listening thread is only started once if connect is called multiple times
            if not hasattr(self, 'listen_thread') or not self.listen_thread.is_alive():
                self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
                self.listen_thread.start()

    def _listen_loop(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.ip, self.port))
                self.socket = s
                self.connected = True
                print(f"Connected to RefBox at {self.ip}:{self.port}")
                if self.on_receive_callback: # Initial connection message
                    self.on_receive_callback("Connection Established with RefBox.")

                while self.running:
                    data = s.recv(1024)
                    if not data:
                        break
                    message = data.decode("utf-8").strip()
                    if message and self.on_receive_callback:
                        self.on_receive_callback(message)
        except ConnectionRefusedError:
            print(f"RefBox connection refused at {self.ip}:{self.port}.")
            if self.on_receive_callback:
                 self.on_receive_callback(f"RefBox connection refused at {self.ip}:{self.port}.")
        except Exception as e:
            print(f"RefBox connection error: {e}")
            if self.on_receive_callback:
                 self.on_receive_callback(f"RefBox connection error: {e}")
        finally:
            self.connected = False
            # self.running = False # Keep running true unless stop() is called, to allow reconnect attempts if desired
            if self.socket:
                self.socket.close()
                self.socket = None
            if self.on_disconnect_callback:
                self.on_disconnect_callback()
            print("RefBox connection closed or failed.")

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR) # Gracefully shutdown
                self.socket.close()
            except OSError as e:
                print(f"Error closing RefBox socket: {e}")
            finally:
                self.socket = None
        self.connected = False # Ensure connected is false
        if hasattr(self, 'listen_thread') and self.listen_thread.is_alive():
            self.listen_thread.join(timeout=1.0)
        print("RefBox handler stopped.")