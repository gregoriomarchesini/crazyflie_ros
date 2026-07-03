#!/usr/bin/env python3.10

import re
import os
import sys
import numpy as np

rel_path = os.path.dirname(os.path.relpath(__file__))
abs_path = os.path.dirname(os.path.abspath(__file__))

NB_BOT = 10
RADIUS = 1
PERIOD_SIZE = 20
CHANNELS = []
BOTS = []
CPP = False
seen_index = set()
imposed_order = {}

if len(sys.argv) > 1 :
    for i, arg in enumerate(sys.argv[1:]) :
        if re.fullmatch(r"\d+:\d+:?\d*", arg) :
            split = re.split(r":", arg)
            channel = "\"" + split[1] + "\""
            if not channel in CHANNELS :
                CHANNELS.append(channel)
            if len(split) == 3 and split[2] and split[1] :
                radio = int(split[2])
                for key, value in imposed_order.items() :
                    if (key == radio and value != channel) or (key != radio and value == channel) :
                        raise ValueError(f"The radio {radio} is attributed to two frequencies, this is not possible")
                imposed_order[radio] = channel
        if arg == "cpp" :
            seen_index.add(i)
            CPP = True
            DEFAULT_RADIO = "\"*\""

for radio, channel in imposed_order.items() :
    if radio >= len(CHANNELS) :
        raise ValueError(f"There are not enough radio to have a radio number {radio}")
    CHANNELS[CHANNELS.index(channel)] = CHANNELS[radio]
    CHANNELS[radio] = channel

bot_channel = {}
bot_radio = {}
if not CHANNELS :
    CHANNELS.append("\"80\"")

if len(sys.argv) > 1 :
    for i, arg in enumerate(sys.argv[1:]) :
        if arg.startswith("nb_bot=") :
            NB_BOT = int(re.split(r"[^\d]+", arg)[1])
        elif arg.startswith("radius=") :
            RADIUS = float(re.split(r"[^\d.]+", arg)[1])
        elif arg.startswith("period_size=") :
            PERIOD_SIZE = int(re.split(r"[^\d]+", arg)[1])
        elif re.fullmatch(r"\d+:?\d*:?\d*", arg) :
            split = re.split(r":", arg)
            for idx, value in enumerate(re.split(r":", arg)) :
                match idx :
                    case 0 :
                        BOTS.append(int(value))
                    case 1 :
                        bot_channel[BOTS[-1]] = "\"" + value + "\"" if value else (CHANNELS[int(split[2])] if len(split) == 3 and split[2] and int(split[2]) < len(CHANNELS) else CHANNELS[0])
                    case 2 :
                        bot_radio[BOTS[-1]] = "\"" + value + "\"" if value and not CPP else ("\"*\"" if CPP else "\"" + str(CHANNELS.index(bot_channel[BOTS[-1]])) + "\"")
            if len(split) < 2 :
                bot_channel[BOTS[-1]] = CHANNELS[0]
            if len(split) < 3 :
                bot_radio[BOTS[-1]] = "\"*\"" if CPP else "\"" + str(CHANNELS.index(bot_channel[BOTS[-1]])) + "\""
        elif not i in seen_index :
            raise ValueError(f"The arg {arg} is not recognised")

if not BOTS :
    BOTS = [i for i in range(NB_BOT)]
else :
    NB_BOT = len(BOTS)

PARAM = [
    "DIM: 2",
    "MANAGER_TIMER: 0.8",
    "AGENT_TIMER: 0.05",
    "SPEED: 0.2",
    "BOX_WEIGHT: 10",
    "COMM_DISTANCE: 3.",
    "HOOVERING_HEIGHT: 1."
]

POS_Z = .3

NB_PERIOD = 5

# A circle and the drones cross it
with open (f"{abs_path}/circle.yaml", "w", encoding = "utf-8") as file:
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
        pos = [0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2), POS_Z]
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        print("  "*nb_tab + "radio: " + bot_radio[bot], file = file)
        print("  "*nb_tab + "channel: " + bot_channel[bot], file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, 4) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    for p in range(3) :
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        angle = np.pi*(p%2)
        for i, bot in enumerate(BOTS) :
            if p == 2 :
                goal = [0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2)]
            else :
                goal = np.array([RADIUS*np.cos(angle), RADIUS*np.sin(angle)])
                angle += 2*np.pi/NB_BOT
            print("  " * nb_tab + f"task_{p*NB_BOT + i+1}:", file = file)
            nb_tab+=1
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("circle.yaml generated")

# A circle and the drones goes to random points on it
with open (f"{abs_path}/random.yaml", "w", encoding = "utf-8") as file:
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
        print("  "*nb_tab + "radio: " + bot_radio[bot], file = file)
        print("  "*nb_tab + "channel: " + bot_channel[bot], file = file)
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

with open (f"{abs_path}/test_real.yaml", "w", encoding = "utf-8") as file:
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
        pos = [0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2), POS_Z]
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        print("  "*nb_tab + "radio: " + bot_radio[bot], file = file)
        print("  "*nb_tab + "channel: " + bot_channel[bot], file = file)
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
            goal = [-0.5*(i//6) if p == 0 else 0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2)]
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
    "SPEED: 0.2",
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

SPHERE_CENTER = np.array((0, 0, RADIUS+.5))

# A sphere and the drones cross it
with open (f"{abs_path}/sphere.yaml", "w", encoding = "utf-8") as file:
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
        pos = [0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2), POS_Z]
        pos = [float(p) for p in pos]
        print("  "*nb_tab + "pos: " + str(pos), file = file)
        print("  "*nb_tab + "radio: " + bot_radio[bot], file = file)
        print("  "*nb_tab + "channel: " + bot_channel[bot], file = file)
        nb_tab -= 1

    print("\n" + "  "*nb_tab + "PERIODS:", file = file)
    nb_tab+= 1
    for i in range(1, 4) :
        print("  "*nb_tab + "period_" + str(i-1) + ": " + str([PERIOD_SIZE*i-5, PERIOD_SIZE*i]), file = file)
    nb_tab -= 1

    print("\n" + "  "*nb_tab + "TASKS:", file = file)
    nb_tab += 1
    for p in range(3) :
        print("\n" +  "  " * nb_tab + f"# Period {p} \n", file = file)
        for i, bot in enumerate(BOTS) :
            print("  " * nb_tab + f"task_{p*NB_BOT + i+1}:", file = file)
            nb_tab+=1
            if p == 2 :
                goal = [0.5*(1-i//6), (1-2*((i%6)%2)) * 0.5*(((i%6)+1)//2), 1]
            else :
                goal = (SPHERE_CENTER + angles_3d[i]) if p%2 else (SPHERE_CENTER - angles_3d[i])
            print("  "*nb_tab + f"period_num: {p}", file = file)
            print("  "*nb_tab + "bot: " + str(bot), file = file)
            print("  "*nb_tab + "goal: " + str([float(x) for x in goal]), file = file)
            print("  "*nb_tab + "size: 0.2", file = file)
            nb_tab-=1

print("sphere.yaml generated")

# A sphere and the drones goes to random point on it
with open (f"{abs_path}/rsphere.yaml", "w", encoding = "utf-8") as file:
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
        print("  "*nb_tab + "radio: " + bot_radio[i], file = file)
        print("  "*nb_tab + "channel: " + bot_channel[i], file = file)
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
print(f"see folder {rel_path} :)")
