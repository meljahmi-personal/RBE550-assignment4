# env/parking.py
import math

def parking_bays_se(n, cell, trailer=False):
    """
    Create a parking rectangle in the SE 3x3-cell corner.
    Returns: {"bays": [poly, ...], "goal": {"pose": (x,y,theta), "tol_xy": m, "tol_yaw": rad}}
    """
    # map size in meters
    X = n * cell
    Y = n * cell

    # SE 3x3 cells
    x0 = X - 3*cell
    y0 = Y - 3*cell
    x1 = X
    y1 = Y

    bay = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    

    goal = {
        "pose": ((x0+x1)/2.0, (y0+y1)/2.0, 0.0),
        "tol_xy": 2.0,                      # TEMP generous
        "tol_yaw": math.radians(30)         # TEMP generous
    }


    data = {"bays": [bay], "goal": goal}

    
    #if trailer:
        # (optional) add a second adjacent bay if you like; not required
    #    pass

    return data
    
    


