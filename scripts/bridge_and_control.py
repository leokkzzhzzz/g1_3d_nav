import json, socket, time, os
os.environ["G1_INTERFACE"] = "enP8p1s0"
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(0, "enP8p1s0")
robot = LocoClient()
robot.SetTimeout(10.0)
robot.Init()
robot.Start()
print("SDK2 ready", flush=True)
sock = None
while True:
    if not sock:
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                sock.connect(("127.0.0.1", 7777))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Bridge connected", flush=True)
                break
            except:
                time.sleep(3)
    try:
        chunk = sock.recv(4096)
        if not chunk: raise Exception()
        for line in chunk.decode().strip().split("\n"):
            try:
                data = json.loads(line)
                if data["type"] == "geometry_msgs/Twist":
                    l = data["msg"]["linear"]
                    a = data["msg"]["angular"]
                    vx, vy, wz = float(l["x"]), float(l["y"]), float(a["z"])
                    if abs(vx) > 0.03 or abs(wz) > 0.03:
                        robot.Move(vx=vx, vy=vy, vyaw=wz, continous_move=True)
            except: pass
    except socket.timeout: continue
    except:
        sock = None
