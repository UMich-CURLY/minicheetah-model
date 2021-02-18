import lcm
from lcm_type import groundtruth_t

# subscribe to ground truth channel to verify it's working

def my_handler(channel, data):
    msg = groundtruth_t.decode(data)
    print("timestamp = {}".format(msg.mocap_timestamp))
    print("labels: {}".format(msg.contact))
    print("")

lc = lcm.LCM()
subscription = lc.subscribe("ground_truth", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass