import math
import json
from random import randint
import random
from socket import SO_EXCLUSIVEADDRUSE

#################################
####  INITIALISER FUNCTIONS  ####
# These functions are used to   #
# initialise the data that will #
# be passed to the simulation.  #
#################################

numFluidParticlesPerCell = 4

def InitGridSize(solids):
    
    minX = minY = 1000.0
    maxX = maxY = -1000.0

    for solid in solids:
        for i in range(0, len(solid[0])):
            if i % 3 == 0:
                if solid[0][i] < minX: minX = solid[0][i]
                if solid[0][i] > maxX : maxX = solid[0][i]
            elif i % 3 == 1:
                if solid[0][i] < minY: minY = solid[0][i]
                if solid[0][i] > maxY : maxY = solid[0][i]

    return (minX, maxX, minY, maxY)

# Initialise the positions of the grid Cells
def InitGridCellPositions(numCellsWidth, numCellsHeight, domainMinX, domainMinY, gridCellSize):
    gridCellPositions = []

    for i in range(0, numCellsWidth):
        x = domainMinX + i * gridCellSize
        for j in range(0, numCellsHeight):
            y = domainMinY + j * gridCellSize
            pos = [x, y]
            gridCellPositions.append(pos)

    return gridCellPositions

# Initialise the velocities of the particles
def InitParticleVelocities(numParticles):
    # Currently initialise all velocities to zero.
    particleVelocities = []

    for i in range(0, numParticles):
        particleVelocities.append([0.0, 0.0])

    return particleVelocities

# Initialise the mass of the particles
def InitParticleMasses(fluidDensity, cellSize):
    # Initialise all masses
    cellVolume = cellSize * cellSize

    fluidParticleMass = cellVolume * fluidDensity / numFluidParticlesPerCell

    return fluidParticleMass

def InitCellTypes(gridCellPositions, solids, liquids):
    cellTypes = [-1] * len(gridCellPositions)

    for i in range(0, len(gridCellPositions)):
        for j in range(0, len(solids)):
            if PointInSolid(solids[j], gridCellPositions[i]):
                cellTypes[i] = 1
                break
            for j in range(0, len(liquids)):
                if PointInSolid(liquids[j], gridCellPositions[i]):
                    cellTypes[i] = 2
                    break

    return cellTypes

def PointInSolid(solid, position):
    contained = False

    # Assume solid is a cube TODO:
    minX = minY = 10000.0
    maxX = maxY = -10000.0

    for i in range(0, len(solid[0])):
        if i % 3 == 0:
            if solid[0][i] < minX: minX = solid[0][i]
            if solid[0][i] > maxX : maxX = solid[0][i]
        elif i % 3 == 1:
            if solid[0][i] < minY: minY = solid[0][i]
            if solid[0][i] > maxY : maxY = solid[0][i]

    if position[0] >= minX and position[0] <= maxX and position[1] >= minY and position[1] <= maxY:
        contained = True

    return contained


def GetClosestCell(position, domainMinX, domainMinY, numCellsHigh, gridCellSize):
    x = round((position[0] - domainMinX) / gridCellSize)
    y = round((position[1] - domainMinY) / gridCellSize)

    return GetIndex(x, y, numCellsHigh)

def GetIndex(x, y, numCellsHigh):
    return int(y + x * numCellsHigh)

def GetXYZ(index, numCellsWide, numCellsHigh):
    y = int(index % numCellsHigh)
    x = int(index / numCellsHigh) % numCellsWide

    return x, y

def LoadFromObj(path):
    solids = []      # fixed rigid objects
    liquids = []   # fluid objects

    try:
        with open(path, 'r') as file:
            # Read file contents
            content = file.read()
            objects = content.split('o ')

            del objects[0] ## First item is some blender jargon, so bin it

            # For each line in the file
            for object in objects:
                if object[0:5] == 'Fixed':
                    solids.append(FixedFromString(object))
                elif object[0:5] == 'Fluid':
                    liquids.append(FixedFromString(object)) 

    except FileNotFoundError:
        print(f"File {path} not found.")

    return solids, liquids

# Assumes string is a blender exported format of obj.
def FixedFromString(objectString):

    solidVertices = []
    solidNormals = []

    data = objectString.split('\n')
    for line in data:
        if line == '': continue     

        line = line.replace('vn', 'n') # replace this for to simplify finding vertices and normals.
        line = line.replace('\n', '')

        if(line[0] == 'v'): # if vertex
            #split the line into floats
            for value in line[1:].split():
                solidVertices.append(float(value))
        if(line[0] == 'n'): # if normal
            #split the line into floats
            for value in line[1:].split():
                solidNormals.append(float(value))

    return [solidVertices, solidNormals]

def SeedParticles(gridCellPositions, gridCellTypes, cellSize):
    liquidParticlePositions = []
    halfCell = cellSize * 0.5

    for i in range(0, len(gridCellPositions)):
        if gridCellTypes[i] != 1:
            for p in range(0, numFluidParticlesPerCell):
                particlePosX = gridCellPositions[i][0] + random.uniform(-halfCell, halfCell)
                particlePosY = gridCellPositions[i][1] + random.uniform(-halfCell, halfCell)
                if gridCellTypes[i] == 2:
                    liquidParticlePositions.append([particlePosX, particlePosY])

    return liquidParticlePositions

#################################
# Initialise the values to be   #
# passed to the simulation.     #
#################################

print("~~~~~ Initialising Scene ~~~~~")

deltaTime = 0.005
duration = 10.0 #seconds

print("~~~ Initialising fluid parameters ~~~")
fluidDensity = 1000.0
 
print("~~~ Initialising scene from obj ~~~")
gridCellSize = 0.5

solids, liquids = LoadFromObj('scenes/dambreak_2d.obj')

print("~~~ Initialising grid values ~~~")
 
domainMinX, domainMaxX, domainMinY, domainMaxY = InitGridSize(solids) # Assume domain is surrounded by solid walls
numCellsWide = round((domainMaxX - domainMinX) / gridCellSize)
numCellsHigh = round((domainMaxY - domainMinY) / gridCellSize)
numGridCells = numCellsWide * numCellsHigh

print("~~~~~ Initialising Grid Cell Positions ~~~~~")

gridCellPositions = InitGridCellPositions(numCellsWide, numCellsHigh, domainMinX, domainMinY, gridCellSize)

print("~~~~~ Initialising Grid Cell Types ~~~~~")

gridCellTypes = InitCellTypes(gridCellPositions, solids, liquids)

particlePositions = SeedParticles(gridCellPositions, gridCellTypes, gridCellSize)

numParticles = len(particlePositions)
particleVelocities = InitParticleVelocities(numParticles)
fluidParticleMass = InitParticleMasses(fluidDensity, gridCellSize)

print("----- Initialised " + str(numParticles) + " particles -----")
print("----- Initialised " + str(numGridCells) + " grid cells -----")
print("-------- " + str(numCellsWide) + " wide, " + str(numCellsHigh) + " high --------")
print("----- Initialised " + str(len(solids)) + " fixed solids -----")



#################################
# Create a dictionary from the  #
# initialised values.           #
#################################


# Create dictionary of data to be written
dictionary = {
    "simulationDuration": duration,
    "deltaTime": deltaTime,
    "numParticles": numParticles,
    "numGridCells": numGridCells,
    "numCellsWidth": numCellsWide,
    "numCellsHeight": numCellsHigh,
    "gridCellPositions": gridCellPositions,
    "cellSize": gridCellSize,
    "domainMinX": domainMinX,
    "domainMaxX": domainMaxX,
    "domainMinY": domainMinY,
    "domainMaxY": domainMaxY,
    "particlePositions": particlePositions,
    "particleVelocities": particleVelocities,
    "particleMass": fluidParticleMass,
    "solids": solids,
    "fluidDensity": fluidDensity,
}

#################################
# Serialize the dictionary to a #
# JSON and write to a file.     #
#################################

# Serializing json
json_object = json.dumps(dictionary, indent=1)

# Writing json_object to file.
with open("Scenes/MPM_scene.json", "w") as output:
    output.write(json_object)