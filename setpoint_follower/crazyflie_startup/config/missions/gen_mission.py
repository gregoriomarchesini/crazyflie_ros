#!/usr/bin/env python3.10

import numpy as np
import sys
import re

NB_BOT = 10
RADIUS = .5
PERIOD_SIZE = 20
BOTS = []
if len(sys.argv) > 1 :
    for arg in sys.argv[1:] :
        split = re.split(r"[\D]+", arg)
        if arg.startswith("nb_bot=") :
            NB_BOT = int(split[1])
        elif arg.startswith("radius=") :
            RADIUS = int(split[1])
        elif arg.startswith("period_size=") :
            PERIOD_SIZE = int(split[1])
        else :
            for bot in  split :
                if bot.isdigit() :
                    BOTS.append(int(bot))

if not BOTS :
    BOTS = [i for i in range(NB_BOT)]
else :
    NB_BOT = len(BOTS)

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
    "radio: \"0\"",
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
    for i, bot in enumerate(BOTS) :
        print("  " * nb_tab + "agent_" + str(bot) + ":", file = file)
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
    angle = np.pi
    for i, bot in enumerate(BOTS) :
        print("  " * nb_tab + f"task_{i+1}:", file = file)
        nb_tab+=1
        goal = np.array([RADIUS*np.cos(angle), RADIUS*np.sin(angle)])
        angle += 2*np.pi/NB_BOT
        print("  "*nb_tab + "period_num: 0", file = file)
        print("  "*nb_tab + "bot: " + str(bot), file = file)
        print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
        print("  "*nb_tab + "size: 0.2", file = file)
        nb_tab-=1

print("circle.yaml generated")

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
    for bot in BOTS :
        print("  " * nb_tab + "agent_" + str(bot) + ":", file = file)
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
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        for i, bot in enumerate(BOTS) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i+1}:", file = file)
            nb_tab+=1
            angle = np.random.random() * 2*np.pi
            goal = np.array([float(RADIUS*np.cos(angle)), float(RADIUS*np.sin(angle))])
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("random.yaml generated")

with open ("test_real.yaml", "w", encoding = "utf-8") as file:
    print("/**:", file = file)
    print("  ros__parameters:", file = file)
    nb_tab = 2
    for param in PARAM :
        print("  "*nb_tab + param, file = file)
    
    print("", file = file)

    goals = []
    for i, bot in enumerate(BOTS) :
        print("  " * nb_tab + "agent_" + str(bot) + ":", file = file)
        nb_tab += 1
        pos = [0.5, (1-2*(i%2)) * RADIUS*((i+1)//2), POS_Z]
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
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        for i,bot in enumerate(BOTS) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i}:", file = file)
            nb_tab+=1
            goal = [-0.5 if p == 0 else 0.5, (1-2*(i%2)) * RADIUS*((i+1)//2)]
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("test_real.yaml generated")

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
    for i in BOTS :
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
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        for i, bot in enumerate(BOTS) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i+1}:", file = file)
            nb_tab+=1
            goal = (SPHERE_CENTER + angles_3d[i]) if p%2 else (SPHERE_CENTER - angles_3d[i])
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("sphere.yaml generated")

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
    for i in BOTS :
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
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        for i,bot in enumerate(BOTS) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i}:", file = file)
            nb_tab+=1
            phi = np.random.random()*np.pi - np.pi/2
            theta = np.random.random()*2*np.pi
            pos = RADIUS * np.array((np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta), np.sin(phi)))
            goal = SPHERE_CENTER + pos
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("rsphere.yaml generated")
