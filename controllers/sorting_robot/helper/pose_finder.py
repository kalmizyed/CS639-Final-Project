# TODO create getObjectPosition() function
# Get the task-space position of the desied object.
def getObjectPosition() -> list[float]:
    return [0.0, 0.67, 0.16]

# TODO figure this out
# Get the task-space position best suited to grabbing the desired object.
def getOptimalPosition() -> list[float]:
    return getObjectPosition()

# TODO create getOptimalRotation() function
# Get the task-space rotation best suited to grabbing the desired object.
def getOptimalRotation() -> list[float]:
    return [0.0, 0.0, 0.0]

def getOptimalPose() -> list[float]:
    return getOptimalPosition() + getOptimalRotation()