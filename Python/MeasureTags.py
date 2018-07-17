import pandas as pd
import math

'''
Floor Orientation.  Point 1 is (0,0)
  4  E3   3
  _________
  |       |
  |       |
E4|       |E2
  |       |
  |       |
  _________
  1   E1  2
'''

c1 = [0,0]
c2 = [13.411,0]
c3 = [13.411,26.223]
c4 = [0,26.223]

E1 = c2[0]
E2 = c3[1]
E3 = c2[0]
E4 = c3[1]
print E1, E2, E3, E4

d1 = 4.293
d2 = 9.374
d3 = 26.676
d4 = 25.341

'''
cos(x) = (d1^2 + d2^2 -E^2) / 2*d1*d2
'''

# General Formula
def solvecoordinate (d1,d2,E,edge):
    #print d1, d2, E
    #b = (d1^2 + E^2 - d2^2)
    #c = (2*d1*E)
    costheta =  (d1**2.0 + E**2.0 - d2**2.0) / (2.0*d1*E)
    #print b,c, (b/c)
    theta = math.acos(costheta)
    vert = d1*math.sin(theta)
    hor = d1*math.cos(theta)
    if edge == 1:
        x = hor
        y = vert
        return [x,y]
    if edge == 2:
        x = E1 - vert
        y = hor
        return [x, y]
    if edge == 3:
        x = E3 - hor
        y = E2 - vert
        return [x, y]
    if edge == 4:
        x = vert
        y = E2 - hor
        return [x,y]

# returns true if over a centimeter of error
def checkError(coord1, coord2, coord3, coord4, thresh):
    error = 0
    if          abs(coord1[0] - coord2[0]) > error:
        error = abs(coord1[0] - coord2[0])
    if        abs(coord1[1] - coord2[1]) > error:
        error = abs(coord1[1] - coord2[1])
    if        abs(coord1[0] - coord3[0]) > error:
        error = abs(coord1[0] - coord3[0])
    if        abs(coord1[1] - coord3[1]) > error:
        error = abs(coord1[1] - coord3[1])
    if        abs(coord1[0] - coord4[0]) > error:
        error = abs(coord1[0] - coord4[0])
    if        abs(coord1[1] - coord4[1]) > error:
        error = abs(coord1[1] - coord4[1])
    if        abs(coord2[0] - coord3[0]) > error:
        error = abs(coord2[0] - coord3[0])
    if        abs(coord2[1] - coord3[1]) > error:
        error = abs(coord2[1] - coord3[1])
    if        abs(coord2[0] - coord4[0]) > error:
        error = abs(coord2[0] - coord4[0])
    if        abs(coord2[1] - coord4[1]) > error:
        error = abs(coord2[1] - coord4[1])
    if        abs(coord3[0] - coord4[0]) > error:
        error = abs(coord3[0] - coord4[0])
    if        abs(coord3[1] - coord4[1]) > error:
        error = abs(coord3[1] - coord4[1])

    return error

def avgCoords(coord1, coord2, coord3, coord4):
    COORDx = (coord1[0] + coord2[0] + coord3[0] + coord4[0])/4.0
    COORDy = (coord1[1] + coord2[1] + coord3[1] + coord4[1]) / 4.0
    return ["%.3f"%COORDx,"%.3f"%COORDy]

def determinePosition(d1,d2,d3,d4):
    d1 = d1 + .027
    d2 = d2 + .027
    d3 = d3 + .027
    d4 = d4 + .027
    coord1 = solvecoordinate(d1, d2, E1, 1)  # Edge1
    coord2 = solvecoordinate(d2, d3, E2, 2)  # Edge2
    coord3 = solvecoordinate(d3, d4, E3, 3)  # Edge3
    coord4 = solvecoordinate(d4, d1, E4, 4)  # Edge4

    print coord1, '\n', coord2,'\n', coord3, '\n', coord4, '\n'

    return avgCoords(coord1, coord2, coord3, coord4)

def determineError(d1,d2,d3,d4):
    d1 = d1 + .027
    d2 = d2 + .027
    d3 = d3 + .027
    d4 = d4 + .027
    coord1 = solvecoordinate(d1, d2, E1, 1)  # Edge1
    coord2 = solvecoordinate(d2, d3, E2, 2)  # Edge2
    coord3 = solvecoordinate(d3, d4, E3, 3)  # Edge3
    coord4 = solvecoordinate(d4, d1, E4, 4)  # Edge4

    error = checkError(coord1, coord2, coord3, coord4, .010)
    return checkError(coord1, coord2, coord3, coord4, .010)

data = pd.read_csv('TagMeasurements.csv')
data['Coord'] = data.apply(lambda row: determinePosition(row['D1'], row['D2'], row['D3'], row['D4']), axis=1)
data['Error'] = data.apply(lambda row: determineError(row['D1'], row['D2'], row['D3'], row['D4']), axis=1)

print data
