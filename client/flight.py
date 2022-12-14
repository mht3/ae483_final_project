import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from skimage import transform
from edge_detector import *

# Radio Channel
uri = 'radio://0/54/2M/E7E7E7E7E7'

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # State estimates (from default EKF)
    'ae483log.o_x',
    'ae483log.o_y',
    'ae483log.o_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.a_z',
    'ae483log.x_meas',
    'ae483log.y_meas',
    'ae483log.z_meas',
    # Setpoint
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=False):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.fully_connected.add_callback(self.fully_connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def connected(self, uri):
        print(f'Connected to {uri}')
    
    def fully_connected(self, uri):
        print(f'Fully connected to {uri}')
        self.is_fully_connected = True

        # Reset the default EKF
        self.cf.param.set_value('kalman.resetEstimation', 1)

        # Enable the controller (1 for default controller, 4 for ae483 controller)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 4)
            self.cf.param.set_value('powerDist.motorSetEnable', 1)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)
            self.cf.param.set_value('powerDist.motorSetEnable', 0)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_fully_connected = False

    def log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
    
    def move_smooth(self, p1, p2, yaw, speed):
        print(f'Move smoothly from {p1} to {p2} with yaw {yaw} degrees at {speed} meters / second')
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        # Compute distance from p1 to p2
        distance_from_p1_to_p2 = np.sqrt(np.sum((p2-p1)**2))
        
        # Compute time it takes to move from p1 to p2 at desired speed
        time_from_p1_to_p2 = distance_from_p1_to_p2 / speed
        
        start_time = time.time()
        while True:
            current_time = time.time()
            
            # Compute what fraction of the distance from p1 to p2 should have
            # been travelled by the current time
            s = (current_time - start_time) / time_from_p1_to_p2
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p = (1.- s) * p1 + s*p2
            
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], yaw)
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Arguments for epochs and batch size. 
    parser.add_argument("-t", "--threshold", type=float, help="Enter lower bound threshold.", default=100)
    parser.add_argument("-s", "--sigma", type=float, help="Standard deviation for gaussian blur.", default=0.)
    parser.add_argument("-f", "--filename", default='data/perry.png')
    parser.add_argument("--height", type=int, help="Image height.", default=None)
    parser.add_argument("--width", type=int, help="Image width.", default=None)
    parser.add_argument("--option", type=int, help="Skeletonization (0) or Image Closing (1)", default=0)

    args = parser.parse_args()
    # Read in image as black and white
    image = cv2.cvtColor(cv2.imread(args.filename), cv2.COLOR_BGR2RGB)

    if args.height is None:
        height = image.shape[0]
    else:
        height = args.height

    if args.width is None:
        width = image.shape[1]
    else:
        width = args.width

    image = transform.resize(image, (height, width), order=0)

    print("Running Edge Detector...")
    canny = Canny(filepath=args.filename, sigma=args.sigma, threshold=args.threshold, size=(height, width))

    canny_output = canny()
    uint8_output = (canny_output / 255.).astype(np.uint8)
    # final post-processing 
    output = final_postprocess(uint8_output, option=args.option)
    # Plot the image to see what the edges look like
    plot(image, output, vmax=1.)

    print("Converting to move commands...")

    # whole image size of 1 meter
    image_size = 1.2
    # Divide by fixed image size
    data = image_size * (get_move_data(output) / max(args.height, args.width))

    print("Preparing for flight...")
    # Initialize everything
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=True, use_observer=True)
    while not client.is_fully_connected:
        time.sleep(0.1)

    # Activate crossing beams method
    client.cf.param.set_value('lighthouse.method', 0)
    # Set default observer to kalman estimator if use_observer=False
    client.cf.param.set_value('stabilizer.estimator', 2)
    # Leave time at the start to initialize
    client.stop(1.0)

    # LED Ring solid color effect
    client.cf.param.set_value('ring.effect', '7')
    # Set the RGB values to off
    client.cf.param.set_value('ring.solidRed', '0')
    client.cf.param.set_value('ring.solidGreen', '0')
    client.cf.param.set_value('ring.solidBlue', '0')

    height = 0.7

    client.move(0.0, 0.0, 0.25, 0.0, 1.0)

    prev_x, prev_y = data[0]
    
    client.move_smooth([0., 0., 0.25], [prev_x, prev_y, height], 0.0, 0.25)

    # Set the RGB values to green
    client.cf.param.set_value('ring.solidRed', '0')
    client.cf.param.set_value('ring.solidGreen', '100')
    client.cf.param.set_value('ring.solidBlue', '0')

    for i in range(1, len(data)):
        x, y = data[i]
        # Move forward smoothly at 0.25 meters / second
        client.move_smooth([prev_x, prev_y, height], [x, y, height], 0.0, 0.25)
        client.move(x, y, height, 0.0, 0.1)
        prev_x = x
        prev_y = y

    # Turn Light off
    client.cf.param.set_value('ring.solidRed', '0')
    client.cf.param.set_value('ring.solidGreen', '0')
    client.cf.param.set_value('ring.solidBlue', '0')
    # Go back to hover (with zero yaw) and prepare to land
    client.move_smooth([prev_x, prev_y, height], [0., 0., height], 0.0, 0.25)

    client.move_smooth([0., 0., height], [0., 0., 0.25], 0.0, 0.2)

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('hardware_data.json')

#