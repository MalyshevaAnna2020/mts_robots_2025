from math import pi, sin, cos, atan2, tan

LIDAR_STEP = pi / 180 / 4
lidar = [] # Данные с лидара (/scan)
 
def angles():
    """Calculate angles from lidar data"""
    d = 20
    angle_list = []
    
    for i in range(d, len(lidar)):
        al = i * LIDAR_STEP - pi / 4
        ar = (i - d) * LIDAR_STEP - pi / 4
        
        xl = lidar[i] * sin(al)
        yl = lidar[i] * cos(al)
        xr = lidar[i - d] * sin(ar)
        yr = lidar[i - d] * cos(ar)
        
        a = atan2(yr - yl, xr - xl)
        if a < 0:
            a += 2 * pi
            
        angle_list.append(a)
    
    dist = [0] * 128
    for a in angle_list:
        idx = int(a / pi * 2 * len(dist)) % len(dist)
        dist[idx] += 1
    
    peak = 0
    for i in range(len(dist)):
        if dist[i] > dist[peak]:
            peak = i
    
    shift = (len(dist) // 2 if peak < len(dist) // 2 else 3 * len(dist) // 2) - peak
    shift = shift * pi / 2 / len(dist)
    
    total = 0
    count = 0
    
    for a in angle_list:
        ad = a / pi * 2 * len(dist)
        d1 = (ad + len(dist) - peak) % len(dist)
        d2 = (peak + len(dist) - ad) % len(dist)
        
        if d1 > 5 and d2 > 5:
            continue
            
        total += ((a + shift) * 2000000 / pi) % 1000000
        count += 1
    
    if count == 0:
        return 0
        
    return (total / count) * pi / 2000000 - shift
 
 
def find_break(lidar, direction):
    """Find break in lidar data"""
    start = 80
    if direction < 0:
        start = len(lidar) - start - 1
    
    diff = angles() / LIDAR_STEP
    start -= diff
    
    current = lidar[int(start)]
    if current > 0.8:
        return 1
    
    step = abs(direction)
    for i in range(80, 180, step):
        idx = int(start)
        if idx < len(lidar) and lidar[idx] - current > 0.35:
            a = abs(180 - (start + diff)) * LIDAR_STEP
            print(f"   {a}")
            return int((0.25 / tan(a) - 0.15) * 2) + 1
        
        current = lidar[idx]
        start += direction
    
    return 0
 
 
def right_break(lidar):
    return find_break(lidar, 1)
 
 
def left_break(lidar):
    return find_break(lidar, -1)
 
 
def next_waypoint(lidar):
        rb, lb = (right_break(lidar), left_break(lidar))
        print(f"breaks: R-{rb}, L-{lb}")
        if rb == 0 and lb == 0:
            return (0, 0, pi/2)
        elif rb > 0:
            return (rb * 0.5, 0, -pi/2)
        else:
            return (lb * 0.5, 0, 0)