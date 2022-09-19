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

    input = (x, y, z, rx, ry, rz, re)
    return input

def move_convert(post_original):
    x = post_original[0] * 1000
    y = post_original[1] * 1000
    z = post_original[2] * 1000
    rx = post_original[3] * 10000
    ry = post_original[4] * 10000
    rz = post_original[5] * 10000
    re = post_original[0] * 10000
    post_robot = (x, y, z, rx, ry, rz, re)
    return post_robot


def move_distance(post1, post2):
    print("nilai post akhir", post2[2])
    print("nilai post awal", post1[2])

    post1 = list(post1)
    post2 = list(post2)

    x_coor = (post2[0] - post1[0])
    y_coor = (post2[1] - post1[1])
    z_coor = (post2[2] - post1[2])
    rx_coor = (post2[3] - post1[3])
    ry_coor = (post2[4] - post1[4])
    rz_coor = (post2[5] - post1[5])
    re_coor = (post2[6] - post1[6])

    dist = math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2))

    move_coor = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    return move_coor, int(dist)

def time_robot(speed, distance):
    distance = distance / 1000
    speed = speed / 10
    time = (distance / speed) + 0.5
    return time

def rob_command(post1):
    x_coor = post1[0] * 1000
    y_coor = post1[1] * 1000
    z_coor = post1[2] * 1000
    rx_coor = post1[3] * 1000
    ry_coor = post1[4] * 1000
    rz_coor = post1[5] * 1000
    re_coor = post1[6] * 1000

    robot_command = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    return robot_command

# print(post1)
# print(post2)
# print("==========================\n")
# post1_move, distance = move_distance(post1, post2)
# print(post1_move)
# print(distance)
# print("==========================\n")
# post_robot = rob_command(post1)
# print(post_robot)
# speed = 150
# time = time_robot(speed, distance)
# print("time estimation: ", time)
# print("==========================\n")
#===== point movement mm =====
pointHome = [457.413, -5.049, 293.318, 178.6969, -0.0019, -0.0283, 0]
point1 = [405.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point2 = [405.929, -387.592, -201.391, 178.7000, 0.0002, -0.0261, 0]
point4 = [405.919, 253.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point5 = [405.932, 253.783, -201.391, 178.6961, -0.0045, -0.0303, 0]


# ===== convert robot command =====
robHome = rob_command(pointHome)
rob1 = rob_command(point1)
rob2 = rob_command(point2)
rob4 = rob_command(point4)
rob5 = rob_command(point5)

# ===== move and distance =========
post1_move, distance1 = move_distance(robHome, rob1)
post2_move, distance2 = move_distance(rob1, rob2)
post3_move, distance3 = move_distance(rob2, rob1)
post4_move, distance4 = move_distance(rob1, rob4)
post5_move, distance5 = move_distance(rob4, rob5)
post6_move, distance6 = move_distance(rob5, rob4)
post7_move, distance7 = move_distance(rob4, robHome)

print("post move 1: ", post1_move)
print("post move 2: ", post2_move)
print("post move 3: ", post3_move)
print("post move 4: ", post4_move)
print("post move 5: ", post5_move)
print("post move 6: ", post6_move)
print("post move 7: ", post7_move)

# ===== list movement task ========
postMove = [post1_move, post2_move, post3_move, post4_move, post5_move, post6_move, post7_move]
