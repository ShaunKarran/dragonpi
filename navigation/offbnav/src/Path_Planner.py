"""
Path Planning software
Generates x,y,z cordinates for UAV to fly and search area given dimensions of
searching room and specifications of camera.

Cordinates are writen into a wayPoints.txt file and displayed using
the matplot lib

Author: Mitchell Sainty
"""
import sys
import math
#import numpy as np
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib.patches import Rectangle
#from matplotlib.lines import Line2D




## HELPER FUNCTIONS ##

#Function takes input, returns true if input can be cast to float #
def inputIsValidNumber(input):
        try:
                float(input)
        except ValueError:
                print("Input must be a number!")
        else:
                return True

# returns true is number postive else negative
def numberIsPostive(number):
        if(number > 0):
                return True
        else:
                print('The number must be postive')

#continually prompts user until accetable input is given as defined by inputISValidNumber
def promptNumber(prompt):
        userInputIsValidNumber = False
        while(not userInputIsValidNumber):
                myInput = input(prompt)
                userInputIsValidNumber = inputIsValidNumber(myInput)

        return float(myInput)


## FUNCTIONS FOR GETTING USER INPUT ##

#function that gets dimensions of room by prompting user, will continue to prompt until gets values that are suitable
# suitable values are blah conditions
def getRoomDimensions():
        #Make sure user inputs number, number is postive, number is within range of UAV searching capabilties
        minX = promptNumber('Enter in the Min x value of the room in metres: ') # prompts user continually until number is entered
        maxX = promptNumber('Enter in the Max x value of the room in metres: ')
        while(minX >= maxX):
                print('min x must be greater then max x')
                minX = promptNumber('Enter in the Min x value of the room in metres: ') # prompts user continually until number is entered
                maxX = promptNumber('Enter in the Max x value of the room in metres: ')


        minY = promptNumber('Enter in the Min y value of the room in metres: ')
        maxY = promptNumber('Enter in the Max y value of the room in metres: ')
        while(minY >= maxY):
                print('min y must be greater then max y')
                minY = promptNumber('Enter in the Min y value of the room in metres: ')
                maxY = promptNumber('Enter in the Max y value of the room in metres: ')

        return minX,maxX,minY,maxY

#Prompts user for camera specifications, will not accept negative values
def getCameraSpecifications():
        #Prompt User for Camera FOV foward direction
        FOVFD = promptNumber('Enter in the field of vision angle of the camera in the foward direction in degrees: ')
        while(not numberIsPostive(FOVFD)):
                FOVFD = promptNumber('Enter in the field of vision angle of the camera in the foward direction in degrees: ') # angle needs to be positive

        #Prompt User for Camera FOV Adjacent Direction
        FOVAD = promptNumber('Enter in the field of vision angle of the camera in the adjacent direction in degrees: ')
        while(not numberIsPostive(FOVAD)):
                FOVAD = promptNumber('Enter in the field of vision angle of the camera in the adjacent direction in degrees: ') # angle needs to be positive

        return FOVFD, FOVAD

#prompts user for searching altitude, will not accept negative values
def getAltitude():
        altitude = promptNumber('Enter in the desired altitude in metres for the UAV to search at: ')
        while(not numberIsPostive(altitude)):
                altitude = promptNumber('Enter in the desired altitude in metres for the UAV to search at: ')
        return altitude


#simple function that obtains the desired image overlap for stiching and to make sure there will be an image with entire target available for image detection
def getImageOverlap():
        valid = False
        while(not valid):
                overlapFD =promptNumber('Enter in the desired overlap % in the forward direction: ')
                if (not numberIsPostive(overlapFD)):
                        print('overlap must be postive')
                elif(overlapFD>=100): # overlap cant be 100% or more or theres no reason for the UAV to travel...
                        print('overlap cannot be greater then or equal to 100% otherwise there is no reason for the UAV to travel')
                else:
                        valid = True #input is fine



        valid = False
        while(not valid):
                overlapAD = promptNumber('Enter in the desired overlap % in the adjacent direction: ')
                if(not numberIsPostive(overlapAD)):
                        print('overlap must be postive')

                elif(overlapAD>=100): # overlap cant be 100% or more or theres no reason for the UAV to travel...
                        print('overlap cannot be greater then or equal to 100% otherwise there is no reason for the UAV to travel')
                else:
                        valid = True #input is fine

        return overlapFD, overlapAD

## 					##

#functions calculates both the legnth of area covered in foward and adjacent directiosn using fov angles and altitude of camera
def calcImgArea(FOVFD, FOVAD, altitude):
        lenFD = (altitude*math.tan(math.radians(0.5*FOVFD)))*2 # math.tan is in radians
        lenAD = (altitude*math.tan(math.radians(0.5*FOVAD)))*2
        return lenFD, lenAD

#return true if surface area at given x will not extend x boundaries
def inRangeX(x, lenAD, minX, maxX, direction):
        inRange = True
        if(direction==1): # right
                distanceFromWayPointX = x+(0.5*lenAD)
        else:
                distanceFromWayPointX = x-(0.5*lenAD)

        if(distanceFromWayPointX<minX or distanceFromWayPointX>maxX): # the cordinate would have the photo capturing outside of boarder
                inRange = False

        return inRange

#return true if surface area at given y will not extend y boundaries
def inRangeY(y, lenFD, minY, maxY, direction):
        inRange = True
        if(direction == 1):
                distanceFromWayPointY = y+(0.5*lenFD)
        else:
                distanceFromWayPointY = y-(0.5*lenFD)
        if(distanceFromWayPointY<minY or distanceFromWayPointY>maxY): # the cordinate would have the photo capturing outside of boarder
                inRange = False

        return inRange

#returns true if there if cordinate did not include up to boundary line
def isRemainderY(minY, maxY, y, lenFD):
        a = round(y+(0.5*lenFD), 3)
        b = round(y-(0.5*lenFD), 3)
        if(a == maxY or b == minY): #the image taken covers the boarder    ### We will have to look at this later as theres no point covering remainder that is to small to have a significant ammount of the target  ###
                return False
        else:
                return True

def isRemainderX(maxX, x, lenAD, minX):
        a = round(x+(0.5*lenAD), 3)
        b = round(x-(0.5*lenAD), 3)
        if(a == maxX or b == minX): #the image taken covers the boarder    ### We will have to look at this later as theres no point covering remainder that is to small to have a significant ammount of the target  ###
                return False
        else:
                return True



# function to generate cordinates for ground search
def generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minY, maxY, altitude):
        #convert ovelap from percent
        overlapAD = (100-overlapAD)/100
        overlapFD = (100-overlapFD)/100


        #calculate the first way point
        firstWayPointX = minX + (0.5*lenAD)
        firstWayPointY = minY + (0.5*lenFD)
        firstWayPointZ = altitude

        groundWayPoints = [[firstWayPointX, firstWayPointY, firstWayPointZ]]
        x=0
        y=1
        z=2

        up = 1
        down = -1
        left= -1
        right =1

        directiony = up
        directionx = right

        moveRight = False
        complete = False
        i = 0
        while(not complete):
                if(moveRight): #generate a new way point in the positive x direction
                        newX = groundWayPoints[i][x] + (lenAD*overlapAD)
                        newWayPoint = [newX, groundWayPoints[i][y], altitude]
                        moveRight = False
                else: #generate a new waypoint in y direction
                        newY = groundWayPoints[i][y] + (directiony*(lenFD*overlapFD))
                        newWayPoint = [groundWayPoints[i][x], newY, altitude]

                if(not inRangeX(newWayPoint[x],lenAD, minX, maxX, directionx)): # waypoint not in range over boundary in x direction, we will need to add more to this for instances where surface area of uav is larger then surface area of image
                        oldX = groundWayPoints[i][x]
                        if(isRemainderX(maxX, oldX, lenAD, minX)):
                                boarderSearchWayPointX = maxX-0.5*lenAD
                                newWayPoint = [boarderSearchWayPointX, groundWayPoints[i][y], altitude]
                                groundWayPoints.append(newWayPoint)
                                i+= 1
                        else:
                                complete = True

                elif(not inRangeY(newWayPoint[y], lenFD, minY, maxY, directiony)): # waypoint not in range over boundary in y direction
                        oldY = groundWayPoints[i][y]
                        if(isRemainderY(minY, maxY, oldY, lenFD)):
                                if(directiony == up):
                                        boarderSearchWayPointY = maxY-0.5*lenFD
                                else:
                                        boarderSearchWayPointY = minY+0.5*lenFD
                                newWayPoint = [groundWayPoints[i][x], boarderSearchWayPointY, altitude]
                                groundWayPoints.append(newWayPoint)
                                i+= 1
                        else:
                                 #flip y direction
                                if(directiony==up):
                                     directiony = down
                                else: # was down
                                     directiony = up
                                moveRight = True # need to generate next cordinate in pos x direction

                else: #newWayPoint is in range add
                        groundWayPoints.append(newWayPoint)
                        i+= 1
        return groundWayPoints

# function to generate cordinates for wall search
def generateWallWaypoints(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minY, maxY, minAlt, maxAlt, distanceFromTarget):
        x = 0
        y = 1
        z = 2
        #recyle code, need to swap the x,y,z to the correct perspective and set all z values to 1 as wall target will be at 1m
        rightSideWayPoints = generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minY, maxY, minAlt, maxAlt, maxX-distanceFromTarget)

        for wayPoint in  rightSideWayPoints:
             wayPoint[x], wayPoint[y], wayPoint[z] = wayPoint[z], wayPoint[x], maxAlt
        rightSideWayPoints = list(reversed(rightSideWayPoints))

        bottomSideWayPoints = generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minAlt, maxAlt, minY+distanceFromTarget)

        for wayPoint in  bottomSideWayPoints:
             wayPoint[x], wayPoint[y], wayPoint[z] = wayPoint[x], wayPoint[z], maxAlt
        bottomSideWayPoints = list(reversed(bottomSideWayPoints))

        leftSideWayPoints = generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minAlt, maxAlt, minX+distanceFromTarget)
        for wayPoint in  leftSideWayPoints:
             wayPoint[x], wayPoint[y], wayPoint[z] = wayPoint[z], wayPoint[x], maxAlt

        topSideWayPoints = generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minAlt, maxAlt, maxY-distanceFromTarget)
        for wayPoint in  topSideWayPoints:
             wayPoint[x], wayPoint[y], wayPoint[z] = wayPoint[x], wayPoint[z], maxAlt

        return rightSideWayPoints,  bottomSideWayPoints, leftSideWayPoints, topSideWayPoints




def main():

        #Prompt User for Min x, Max x, Min y, Max y
        choice = input('Enter "custom" for custom values or will go to default values for dragonPI prototype: ')
        if(choice == 'custom'):
                print('We will now get some custom values')
                #proceed with getting values from user

                #Get dimensions for room
                minX, maxX, minY, maxY = getRoomDimensions()

                #Get camera specifications
                FOVFD, FOVAD = getCameraSpecifications()

                #get altitude
                altitude = getAltitude()

                #get overlap
                overlapFD, overlapAD = getImageOverlap()

        else:
                #we set some default values
                print('You selected default values')
                minX = -2
                maxX = 2
                minY = -2
                maxY = 2

                FOVFD = 45 #Field of view foward direction for rasberry pi camera
                FOVAD = 60 #Field of view adjacent direction for rasberry pi camera

                altitude = 1 #distance from target

                overlapFD = 20 #overlap Foward direction
                overlapAD = 15 #overlap Adjacent direction




        print('Custom values for determining flight path are as follows...', 'minX: ', minX, 'maxX: ', maxX, 'minY: ', minY, 'maxY: ', maxY, 'FOVFD: ', FOVFD, 'FOVAD: ', FOVAD, 'altitude: ', altitude, 'overlapFD: ', overlapFD, 'overlapAD: ', overlapAD)

        #determine surface area covered by image with given altitude
        lenFD, lenAD = calcImgArea(FOVFD, FOVAD, altitude)


        print('The length covered in the foward direction with: ', 'FOVFD: ', FOVFD, ' altitdude: ', altitude, ' is ', lenFD)
        print('The length covered in the adjacent direction with: ', 'FOVAD: ', FOVAD, ' altitdude: ', altitude, ' is ', lenAD)




        #Generate Ground Way Points
        groundWayPoints = generateWayPoints2D(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minY, maxY, altitude)

        #Write ground waypoints to txt file
        thefile = open('groundWayPoints.txt', 'w')

        for wayPoint in groundWayPoints:
            thefile.write("%s\n" % wayPoint)
        thefile.close()

        distanceFromTarget = altitude # 1 m same as when doing ground search
        minAlt = 0
        maxAlt = 1


	    #Generate wall search points
        rightSideWayPoints,  bottomSideWayPoints, leftSideWayPoints, topSideWayPoints = generateWallWaypoints(lenFD, lenAD, overlapAD, overlapFD, minX, maxX, minY, maxY, minAlt, maxAlt, distanceFromTarget)

        #Write ground waypoints to txt file
        thefile = open('rightSideWayPoints.txt', 'w')

        for wayPoint in rightSideWayPoints:
            thefile.write("%s\n" % wayPoint)
        thefile.close()

        #Write ground waypoints to txt file
        thefile = open('bottomSideWayPoints.txt', 'w')

        for wayPoint in bottomSideWayPoints:
            thefile.write("%s\n" % wayPoint)
        thefile.close()

        #Write ground waypoints to txt file
        thefile = open('leftSideWayPoints.txt', 'w')

        for wayPoint in leftSideWayPoints:
            thefile.write("%s\n" % wayPoint)
        thefile.close()

        #Write ground waypoints to txt file
        thefile = open('topSideWayPoints.txt', 'w')

        for wayPoint in topSideWayPoints:
            thefile.write("%s\n" % wayPoint)
        thefile.close()

        #values set for readability when plotting waypoints
        x = 0
        y = 1
        z = 2










main()
