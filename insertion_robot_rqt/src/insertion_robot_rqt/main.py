import sys

from rqt_gui.main import Main


def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='insertion_robot_rqt.robot_control.RobotControl'))


if __name__ == '__main__':
    main()