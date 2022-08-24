from fs100 import FS100


class liteYaskawa:
    def __init__(self, ipConn):
        self.ip = ipConn
        self.robot = FS100.connect(self.ip)

    def readPos(self, pos_info, robot_no):
        self.pos_info = {}
        self.robot_no = 1

        if FS100.ERROR_SUCCESS == self.robot.read_position(pos_info, robot_no):
            x, y, z, rx, ry, rz, re = pos_info['pos']
            str = "CURRENT POSITION\n" + \
                  "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                  "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                     x // 1000, x % 1000, rx // 10000,
                                                                                     rx % 10000) + \
                  "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                      y // 1000, y % 1000, ry // 10000, ry % 10000) + \
                  "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                      z // 1000, z % 1000, rz // 10000, rz % 10000) + \
                  "                            Re   {:4d}.{:04d} deg.\n".format(
                      re // 10000, re % 10000)

        print(str)