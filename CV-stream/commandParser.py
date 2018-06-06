from io import StringIO
import csv


def parseCommand(command):
    'Takes a csv string and converts it into a list of floating point variables. Requires StringIO and csv modules'
    #command = "1;32;150;0"
    f = StringIO(command)
    reader = csv.reader(f, delimiter=';')
    comList = list(reader)
    commandNumbers = list(map(float, comList[0]))
    # debug:
    #for number in commandNumbers:
    #    print(number)
    return commandNumbers
