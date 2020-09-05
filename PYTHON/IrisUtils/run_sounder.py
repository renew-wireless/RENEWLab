from wrapper import *
import sys

if __name__ == '__main__':

    filename = "bsconfig.json"
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    sounder = Sounder(filename)
    sounder.start()
    print("Recorded %d Frames in %s" % (sounder.getMaxFrame(), sounder.getTraceFileName()))
