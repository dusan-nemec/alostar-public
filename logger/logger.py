import udp
import time
import os

streams = [
    {
        'name': 'ROBOT', 
        'port': 5000, 
        'head': 'Timestamp;PosX;PosY;PosZ;Roll;Pitch;Yaw;Q1;Q2;Q3;Q4;M1;M2;M3;M4;AccelX;AccelY;AccelZ;GyroX;GyroY;GyroZ\r\n'
    },
    {
        'name': 'SLAM',
        'port': 5001,
        'head': 'Timestamp;PosX;PosY;PosZ;Roll;Pitch;Yaw\r\n'
    },
    {
        'name': 'GNSS',
        'port': 5002,
        'head': 'Timestamp;Fix;Latitude;Longitude;Altitude;GeoidSeparation;Satellites;HDOP\r\n'
    },
    {
        'name': 'IMU1', 
        'port': 5101,
        'head': 'DeviceID;Timestamp;Temperature;GyroX;GyroY;GyroZ;AccelX;AccelY;AccelZ;Dist1;Dist2;Dist3;Err1;Err2;Err3\r\n'
    },
    {
        'name': 'IMU2', 
        'port': 5102,
        'head': 'DeviceID;Timestamp;Temperature;GyroX;GyroY;GyroZ;AccelX;AccelY;AccelZ;Dist1;Dist2;Dist3;Err1;Err2;Err3\r\n'
    },
    {
        'name': 'IMU3', 
        'port': 5103,
        'head': 'DeviceID;Timestamp;Temperature;GyroX;GyroY;GyroZ;AccelX;AccelY;AccelZ;Dist1;Dist2;Dist3;Err1;Err2;Err3\r\n'
    },
    {
        'name': 'IMU4', 
        'port': 5104,
        'head': 'DeviceID;Timestamp;Temperature;GyroX;GyroY;GyroZ;AccelX;AccelY;AccelZ;Dist1;Dist2;Dist3;Err1;Err2;Err3\r\n'
    }
]
0
servers = []
try:
    dir = 'logs/' + time.strftime("%Y%m%dT%H%M%S")
    os.mkdir(dir)

    for s in streams:
        csv = open(dir + '/' + s['name'] + '.csv', 'wb')
        csv.write(s['head'].encode())

        srv = udp.Server(s['port'])
        srv.name = s['name']
        srv.msgTim = time.time()
        srv.msgRate = 0.0

        srv.start()
        #print('Stream ' + s['name'] + ' is listening on port ' + str(s['port']) + ' ...')

        servers.append((srv, csv))
    
    while True:
        print("\033[{}A\033[2K".format(len(servers)), end='')
        for srv, csv in servers:
            data = srv.read()
            t = time.time()

            if len(data) > 0:
                csv.write(data)
                if t > srv.msgTim:
                    srv.msgRate = 0.99 * srv.msgRate + 0.01 / (t - srv.msgTim)
                srv.msgTim = t
            else:
                if t > srv.msgTim + 1.0:
                    srv.msgRate = 0.0

            print("{} on port {}: {:.1f} samples/s".format(srv.name, srv.port, srv.msgRate))
                
except KeyboardInterrupt:
    pass

except Exception as ex:
    print(ex)
    
for srv, csv in servers:
    try:
        csv.close()
        srv.stop()
    except:
        pass
    
os.system('cls')
print('Done')
