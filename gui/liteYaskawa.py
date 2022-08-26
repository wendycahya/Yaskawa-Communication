from fs100 import FS100


class liteYaskawa:
    def __init__(self, ipConn):
        self.robot = FS100.connect(ipConn)

    def readPos(self, pos_info, robot_no):
        self.pos_info = {}
        self.robot_no = 1

        if FS100.ERROR_SUCCESS == self.robot.read_position(pos_info, robot_no):
            self.x, self.y, self.z, self.rx, self.ry, self.rz, self.re = pos_info['pos']
            self.str = "CURRENT POSITION\n" + \
                  "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                  "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                     self.x // 1000, self.x % 1000, self.rx // 10000,
                                                                                     self.rx % 10000) + \
                  "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                      self.y // 1000, self.y % 1000, self.ry // 10000, self.ry % 10000) + \
                  "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                      self.z // 1000, self.z % 1000, self.rz // 10000, self.rz % 10000) + \
                  "                            Re   {:4d}.{:04d} deg.\n".format(
                      self.re // 10000, self.re % 10000)

        print(self.str)

LocalIP = '192.168.255.1'
robot = liteYaskawa(LocalIP)
