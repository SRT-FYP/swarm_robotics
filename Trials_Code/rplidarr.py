from rplidar import RPLidar, RPLidarException


# Testing the lidar baud rate (which one works):
# ----------------------------------------------
# lidar = None

# for baud in [115200, 256000]:
#     try:
#         print(f'Trying baudrate: {baud}')
#         lidar = RPLidar(PORT_NAME, baudrate=baud)
#         print(f'Connected successfully with baudrate {baud}')
#         break
#     except Exception as e:
#         print(f'Failed with baudrate {baud}: {e}')
# ----------------------------------------------

# Setting port and baud rate
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME, baudrate=115200)

if lidar is None:
    print('Unable to connect to the RPLidar. Exiting.')
    exit(1)

# Scanning 
try:
    print('Starting scan...')
    target_angle = 180 
    tolerance = 1.0  # degrees

    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan: # (quality, angle, distance)
            if abs(angle - target_angle) <= tolerance:
                print(f'Measurement near {target_angle}°: Distance = {(distance)/10:.2f} cm')

except KeyboardInterrupt:
    print('Stopping...')
except RPLidarException as e:
    print(f'Lidar exception: {e}')
finally:
    if lidar:
        print('Cleaning up...')
        lidar.stop()
        lidar.disconnect()
