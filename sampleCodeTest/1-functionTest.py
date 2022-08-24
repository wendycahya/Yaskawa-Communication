import math

def convert_mm(x, y, z, rx, ry, rz, re):
    str_x = "{:4d}.{:03d}".format(x // 1000, x % 1000)
    str_y = "{:4d}.{:03d}".format(y // 1000, y % 1000)
    str_z = "{:4d}.{:03d}".format(z // 1000, z % 1000)
    str_rx = "{:4d}.{:04d}".format(rx // 10000, rx % 10000)
    str_ry = "{:4d}.{:04d}".format(ry // 10000, ry % 10000)
    str_rz = "{:4d}.{:04d}".format(rz // 10000, rz % 10000)
    str_re = "{:4d}.{:04d}".format(re // 10000, re % 10000)

    x = float(str_x)
    y = float(str_y)
    z = float(str_z)
    rx = float(str_rx)
    ry = float(str_ry)
    rz = float(str_rz)
    re = float(str_re)

    input = [x, y, z, rx, ry, rz, re]

    return input

def move_distance(post1, post2):
    x_coor = (post2[0] - post1[0]) * 1000
    y_coor = (post2[1] - post1[1]) * 1000
    z_coor = (post2[2] - post1[2]) * 1000
    rx_coor = (post2[3] - post1[3]) * 1000
    ry_coor = (post2[4] - post1[4]) * 1000
    rz_coor = (post2[5] - post1[5]) * 1000
    re_coor = (post2[6] - post1[6]) * 1000

    dist = (math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2)))*1000

    move_coor = [int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor)]
    return move_coor, int(dist)

def time_robot(speed, distance):
    distance = distance / 1000
    speed = speed / 10
    time = (distance / speed) + 0.5
    return time

post1 = convert_mm(4550033, 302201, -678008, 12453, 12122, 12434, 1214345)
post2 = convert_mm(4550033, 352201, -678008, 12453, 12122, 12434, 1214345)

print(post1)
print(post2)
print("==========================\n")
post1_move, distance = move_distance(post1, post2)
print(post1_move)
print(distance)
print("==========================\n")
speed = 500
time = time_robot(speed, distance)
print("time estimation: ", time)