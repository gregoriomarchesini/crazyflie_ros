#!/usr/bin/env python3.10

import numpy as np

NB_BOT = 10
RADIUS = 2
PERIOD_SIZE = 25
PARAM = [
    "DIM: 2",
    "MANAGER_TIMER: 0.8",
    "AGENT_TIMER: 0.05",
    "SPEED: 0.4",
    "BOX_WEIGHT: 10",
    "COMM_DISTANCE: 3.",
    "HOOVERING_HEIGHT: 1."
]

POS_Z = .3
BOT_PARAM = [
    "radio: 1",
    "channel: 80"
]

NB_PERIOD = 5

# A circle and the drones cross it
with open ("circle.yaml", "w", encoding = "utf-8") as file:
    print("/**:", file = file)
    print("  ros__parameters:", file = file)
    nb_tab = 2
    for param in PARAM :
        print("  "*nb_tab + param, file = file)
    
    print("", file = file)

    angle = 0
    goals = []
    for i in range(NB_BOT) :
        print("  " * nb_tab + "agent_" + str(i) + ":", file = file)
        nb_tab += 1
        pos = [RADIUS*np.cos(angle), RADIUS*np.sin(angle), POS_Z]
        angle += 2*np.pi/NB_BOT
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        for p in BOT_PARAM :
            print("  "*nb_tab + p, file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, 2) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    angle = 2*np.pi/NB_BOT + np.pi
    pos_0 = np.array([-RADIUS, 0])
    for i in range(1, NB_BOT) :
        print("  " * nb_tab + f"task_{i}:", file = file)
        nb_tab+=1
        goal = np.array([RADIUS*np.cos(angle), RADIUS*np.sin(angle)])
        angle += 2*np.pi/NB_BOT
        pos_rel = [float(d) for d in goal-pos_0]
        print("  "*nb_tab + "period_num: 0", file = file)
        print("  "*nb_tab + "edges: " + str([0, i]), file = file)
        print("  "*nb_tab + "rel_position: " + str(pos_rel), file = file)
        print("  "*nb_tab + "size: 0.2", file = file)
        nb_tab-=1

# A circle and the drones goes to random points on it
with open ("random.yaml", "w", encoding = "utf-8") as file:
    print("/**:", file = file)
    print("  ros__parameters:", file = file)
    nb_tab = 2
    for param in PARAM :
        print("  "*nb_tab + param, file = file)
    
    print("", file = file)

    angle = 0
    goals = []
    for i in range(NB_BOT) :
        print("  " * nb_tab + "agent_" + str(i) + ":", file = file)
        nb_tab += 1
        pos = [RADIUS*np.cos(angle), RADIUS*np.sin(angle), POS_Z]
        angle += 2*np.pi/NB_BOT
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        for p in BOT_PARAM :
            print("  "*nb_tab + p, file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, NB_PERIOD+1) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    angle = 2*np.pi/NB_BOT + np.pi
    for p in range(NB_PERIOD) :
        angle = np.random.random() * 2*np.pi
        pos_0 = np.array([float(RADIUS*np.cos(angle)), float(RADIUS*np.sin(angle))])
        for i in range(1, NB_BOT) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i}:", file = file)
            nb_tab+=1
            angle = np.random.random() * 2*np.pi
            goal = np.array([float(RADIUS*np.cos(angle)), float(RADIUS*np.sin(angle))])
            pos_rel = [float(d) for d in goal-pos_0]
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "edges: " + str([0, i]), file = file)
            print("  "*nb_tab + "rel_position: " + str(pos_rel), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

PARAM = [
    "DIM: 3",
    "MANAGER_TIMER: 0.8",
    "AGENT_TIMER: 0.05",
    "SPEED: 0.4",
    "BOX_WEIGHT: 10",
    "COMM_DISTANCE: 3.",
    "HOOVERING_HEIGHT: 1."
]
GOLDEN_RATIO = (1+np.sqrt(5))/2
angles_3d = []
for i in range(NB_BOT) :
    theta = 2*np.pi*i*GOLDEN_RATIO
    phi = np.arccos(1-2*i/NB_BOT)
    angles_3d.append(RADIUS*np.array((np.cos(theta)*np.sin(phi), np.sin(phi)*np.sin(theta), np.cos(phi))))

SPHERE_CENTER = np.array((0, 0, RADIUS+1))

# A sphere and the drones cross it
with open ("sphere.yaml", "w", encoding = "utf-8") as file:
    print("/**:", file = file)
    print("  ros__parameters:", file = file)
    nb_tab = 2
    for param in PARAM :
        print("  "*nb_tab + param, file = file)
    
    print("", file = file)

    angle = 0
    goals = []
    for i in range(NB_BOT) :
        print("  " * nb_tab + "agent_" + str(i) + ":", file = file)
        nb_tab += 1
        pos = [RADIUS*np.cos(angle), RADIUS*np.sin(angle), POS_Z]
        angle += 2*np.pi/NB_BOT
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        for p in BOT_PARAM :
            print("  "*nb_tab + p, file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, 3) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    for p in range(2) :
        pos_0 = (SPHERE_CENTER + angles_3d[0]) if p%2 else (SPHERE_CENTER - angles_3d[0])
        for i in range(1, NB_BOT) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i}:", file = file)
            nb_tab+=1
            goal = (SPHERE_CENTER + angles_3d[i]) if p%2 else (SPHERE_CENTER - angles_3d[i])
            pos_rel = [float(d) for d in goal-pos_0]
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "edges: " + str([0, i]), file = file)
            print("  "*nb_tab + "rel_position: " + str(pos_rel), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

# A sphere and the drones goes to random point on it
with open ("rsphere.yaml", "w", encoding = "utf-8") as file:
    print("/**:", file = file)
    print("  ros__parameters:", file = file)
    nb_tab = 2
    for param in PARAM :
        print("  "*nb_tab + param, file = file)
    
    print("", file = file)

    angle = 0
    goals = []
    for i in range(NB_BOT) :
        print("  " * nb_tab + "agent_" + str(i) + ":", file = file)
        nb_tab += 1
        pos = [RADIUS*np.cos(angle), RADIUS*np.sin(angle), POS_Z]
        angle += 2*np.pi/NB_BOT
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        for p in BOT_PARAM :
            print("  "*nb_tab + p, file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, NB_PERIOD+1) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    for p in range(NB_PERIOD) :
        phi = np.random.random()*np.pi - np.pi/2
        theta = np.random.random()*2*np.pi
        pos = RADIUS * np.array((np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta), np.sin(phi)))
        pos_0 = (SPHERE_CENTER + angles_3d[0]) if p==0 else (SPHERE_CENTER + pos)
        for i in range(1, NB_BOT) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i}:", file = file)
            nb_tab+=1
            phi = np.random.random()*np.pi - np.pi/2
            theta = np.random.random()*2*np.pi
            pos = RADIUS * np.array((np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta), np.sin(phi)))
            goal = SPHERE_CENTER + pos
            pos_rel = [float(d) for d in goal-pos_0]
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "edges: " + str([0, i]), file = file)
            print("  "*nb_tab + "rel_position: " + str(pos_rel), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1
