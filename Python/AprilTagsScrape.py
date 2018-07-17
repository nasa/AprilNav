import pandas as pd
from TagCoordinates import decodeTags, getX, getY
from termcolor import colored
import math

'''
CAMERA ORIENTATION:
X is distance from camera
Y is right(+) to left(-)
Z is up(+) and down(-)'''

count = 0
index = 0

# Import April Tags CSV data for processing
#launch = pd.read_csv("C:/Users/tkschule/Desktop/AprilTagsPython/FloorTHREE.csv", header = 0 )
launch = pd.read_csv("C:/Users/tkschule/PycharmProjects/AprilTags/three_tags.csv", header = 0 )

# Convert for vertical testing instead of ceiling Testing.
ColsTest = ['Count', 'Time', 'Tag ID', 'Distance', 'Z', 'X', 'Y', 'Pitch', 'Roll', 'Yaw']
launch.columns = ColsTest
launch.drop(launch.index[0], inplace = True)  # C++ Data Aquisition is exporting NAN values for index 1 currently

# Add Coordinates to Data frame given a Tag ID
launch['QR X'] = launch.apply(lambda row: getX(decodeTags(row["Tag ID"])), axis=1)
launch['QR Y'] = launch.apply(lambda row: getY(decodeTags(row["Tag ID"])), axis=1)
launch['QR Z'] = 4.6  # Z is constant on the ceiling
launch.Yaw = launch.Yaw * 180/math.pi # Convert Yaw Column from Radians to Degree
launch.Roll = launch.Roll * 180/math.pi
launch.Pitch = launch.Pitch * 180/math.pi

# ----------------------------------------------------------------------------------
# Convert to Real Camera Coordinates
def CameraX(RelX, QRX):
    return QRX - RelX

def CameraY(RelY, QRY):
    return QRY - RelY

def CameraY(RelZ, QRZ):
    return QRZ - RelZ

launch['Camera X'] = launch.apply(lambda row: CameraX(row["X"],row["QR X"]), axis=1)
launch['Camera Y'] = launch.apply(lambda row: CameraX(row["Y"],row["QR Y"]), axis=1)
launch['Camera Z'] = launch.apply(lambda row: CameraX(row["Z"],row["QR Z"]), axis=1)

# ----------------------------------------------------------------------------------

#Create new empty DataFrame with the same Header as the imported CSV
Cols = list(launch)
optimized = pd.DataFrame(columns=Cols)

print colored(("Total QR Codes Found: " + str(launch.Count.size)), "yellow"), "\n"

# Add groupings of QR Codes to a dataframe with averaged vectors to minimize error in Camera position accuracy

c1 = 0
for i in range (1,launch.Count.size):
    if launch.iloc[i]['Count'] != launch.iloc[i-1]['Count']:
        ctr = int((i-launch.iloc[i]['Count']))
        optimized.ix[str(index)] = launch[i-c1:i].mean()
        index+=1
        c1 = 0
    c1+=1

#Drop all but these columns in the new data structure
col_list = ["Camera X", "Camera Y", "Pitch", "Roll", "Yaw"]
optimized = optimized[col_list]


print launch[3:8], "\n"
print colored(optimized,"grey")
print colored("Repeats Found: " + str(index), "red")

