#!/usr/bin/env python3.10

import numpy as np
import sys

agents = [1, 2, 3, 4]
agents = [1, 2, 3, 4, 5, 6, 7, 9 ,10, 11, 12, 13]
radio = {1:60, 2:80, 3:80, 4:80, 5:80, 6:80, 7:60, 9:60, 10:60, 11:80, 12:60, 13:60}
angles = {}
nb_period_rota = 15
nb_period_transi_1 = 10
nb_period_ring = 2
nb_period_transi_2 = 30
nb_period_modif_theta = 15
nb_period_land = 10
nb_period = nb_period_rota + nb_period_transi_1 + nb_period_transi_2 + nb_period_ring + nb_period_modif_theta + nb_period_land
speed = 0.2
hoov_height = 1.3
big_radius = 1.2
small_radius = big_radius * np.sqrt(2)

param = [
    "DIM: 3",
    "MANAGER_TIMER: 1.",
    "AGENT_TIMER: 0.05",
    f"SPEED: {speed}",
    "BOX_WEIGHT: 10",
    "COMM_DISTANCE: 3.",
    f"HOOVERING_HEIGHT: {hoov_height}"
]

with open("pretty.yaml", mode="w", encoding="utf-8") as file :
    sys.stdout = file
    print("/**:")
    tab = 1

    # Parameters
    print("  "*tab + "ros__parameters:")
    tab+=1
    for p in param :
        print("  "*tab + p, file = file)
    print(file = file)

    # Agents
    for i, a in enumerate(agents) :
        print("  "*tab + f"agent_{a}:", file = file)
        tab += 1
        radius = 1/np.sqrt(2) if i < 4 else (1 if i < 8 else np.sqrt(2))
        angle = 1/2 * (i-4) * np.pi if 3 < i < 8 else (1/4 + 1/2 * (i%4))*np.pi
        angles[a] = angle
        print("  "*tab + f"pos: {[float(e) for e in radius * np.array([np.cos(angle), np.sin(angle), 0.3/radius])]}", file = file)
        print("  "*tab + 'radio: "*"', file = file)
        print("  "*tab + f'channel: "{radio[a]}"', file = file)
        tab-=1
    print(file = file)

    # Periods
    print("  "*tab + "PERIODS:", file = file)
    tab += 1
    for i in range(nb_period) :
        print("  "*tab + f"period_{i}: {[2*i, 2*i+2]}", file = file)
    tab -= 1
    print(file = file)

    # Tasks
    print("  "*tab + "TASKS:", file = file)
    tab += 1

    # Rotation
    for p in range(nb_period_rota) :
        print()
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{p*len(agents)+i+1}:", file = file)
            tab += 1
            print("  "*tab + f"period_num: {p}")
            print("  "*tab + f"bot: {a}")
            radius = 1/np.sqrt(2) - (1/np.sqrt(2)-big_radius)/(nb_period_rota-1)*p if i < 4 else (1 - (1-big_radius)/(nb_period_rota-1)*p if i < 8 else np.sqrt(2) - (np.sqrt(2)-big_radius)/(nb_period_rota-1)*p)
            angle = angles[a]
            angle += 1.5*speed/radius if 3<i<8 else -1.5*speed/radius
            angles[a] = angle%(2*np.pi)
            dh = (0.05*p if p < 10 else 0.5) if i < 4 else (-(0.05*p if p < 10 else 0.5) if i > 7 else 0)
            print("  "*tab + f"goal: {[float(e) for e in radius * np.array([np.cos(angle), np.sin(angle), (hoov_height+dh)/radius])]}")
            print("  "*tab + "size: 0.2")
            tab -= 1

    print()

    # Going to 2 ring
    for p in range(nb_period_transi_1) :
        print()
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{(p+nb_period_rota)*len(agents)+i+1}:", file = file)
            tab += 1
            print("  "*tab + f"period_num: {p+nb_period_rota}")
            print("  "*tab + f"bot: {a}")
            dh = (0.5 + (big_radius*np.sqrt(2)-1)*p/2/(nb_period_transi_1-1)) if i < 4 else (-(0.5 + (big_radius*np.sqrt(2)-1)*p/2/(nb_period_transi_1-1)) if i > 7 else 0)
            if i in (5, 7) :
                dh = big_radius/np.sqrt(2)*p/(nb_period_transi_1-1)
            if i in (4, 6) :
                dh = -big_radius/np.sqrt(2)*p/(nb_period_transi_1-1)
            radius = np.sqrt(big_radius**2 - (0.5 + (big_radius*np.sqrt(2)-1)*p/2/(nb_period_transi_1-1))**2) if 3 < i < 8 else np.sqrt(big_radius**2 - dh**2)
            angle = angles[a]
            if i == 1 :
                angle = angles[agents[0]] + np.pi/3
            elif i == 2 :
                angle = angles[agents[0]] + np.pi
            elif i == 3 :
                angle = angles[agents[0]] - 2*np.pi/3
            elif i == 4 :
                angle = angles[agents[0]] - np.pi/2
            elif i == 5 :
                angle = angles[agents[0]] - np.pi/3
            elif i == 6 :
                angle = angles[agents[4]] + np.pi
            elif i == 7 :
                angle = angles[agents[0]] + 2*np.pi/3
            elif i == 8 :
                angle = angles[agents[4]] - 2*np.pi/3
            elif i == 9 :
                angle = angles[agents[4]] - np.pi/3
            elif i == 10 :
                angle = angles[agents[4]] + np.pi/3
            elif i == 11 :
                angle = angles[agents[4]] + 2*np.pi/3
            else :
                angle -= 1.4*speed/radius
            angles[a] = angle%(2*np.pi)
            x, y = radius*np.cos(angle), radius*np.sin(angle)
            print("  "*tab + f"goal: {[float(e) for e in np.array([x, y, (hoov_height+dh)])]}")
            print("  "*tab + "size: 0.2")
            tab -= 1

    nb_period_done = nb_period_rota + nb_period_transi_1
    print()

    # Rotating a bit
    for p in range(nb_period_ring) :
        print()
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{(p+nb_period_done)*len(agents)+i+1}:", file = file)
            tab += 1
            print("  "*tab + f"period_num: {p+nb_period_done}")
            print("  "*tab + f"bot: {a}")
            dh = (big_radius/np.sqrt(2)) if i < 4 or i in (5, 7) else (-(big_radius/np.sqrt(2)))
            radius = (big_radius/np.sqrt(2))
            angle = angles[a]
            if i == 1 :
                angle = angles[agents[0]] + np.pi/3
            elif i == 2 :
                angle = angles[agents[0]] + np.pi
            elif i == 3 :
                angle = angles[agents[0]] - 2*np.pi/3
            elif i == 4 :
                angle = angles[agents[0]] - np.pi/2
            elif i == 5 :
                angle = angles[agents[0]] - np.pi/3
            elif i == 6 :
                angle = angles[agents[4]] + np.pi
            elif i == 7 :
                angle = angles[agents[0]] + 2*np.pi/3
            elif i == 8 :
                angle = angles[agents[4]] - 2*np.pi/3
            elif i == 9 :
                angle = angles[agents[4]] - np.pi/3
            elif i == 10 :
                angle = angles[agents[4]] + np.pi/3
            elif i == 11 :
                angle = angles[agents[4]] + 2*np.pi/3
            else :
                angle -= speed/radius
            angles[a] = angle%(2*np.pi)
            x, y = radius*np.cos(angle), radius*np.sin(angle)
            print("  "*tab + f"goal: {[float(e) for e in np.array([x, y, (hoov_height+dh)])]}")
            print("  "*tab + "size: 0.2")
            tab -= 1
    
    nb_period_done += nb_period_ring
    omega = 1.5*speed/big_radius
    ring = [abs(a) < omega/2 for a in angles]
    alphas = [np.pi/2] * len(agents)

    print()

    # crossing the rings
    for p in range(nb_period_transi_2) :
        print()
        # print(alphas)
        # print(angles)
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{(p+nb_period_done)*len(agents)+i+1}:")
            tab += 1
            print("  "*tab + f"period_num: {p + nb_period_done}")
            print("  "*tab + f"bot: {a}")
            angle = angles[a]
            angle -= omega
            angles[a] = angle%(2*np.pi)
            if ring[i] :
                alpha = alphas[i] + omega
                alphas[i] = alpha%(2*np.pi)  # pyright: ignore[reportArgumentType, reportCallIssue]
                x = big_radius*np.sin(alpha)/np.sqrt(2)
                # x = big_radius*np.sin(alpha)
                y = big_radius*np.cos(alpha)
                z = hoov_height + x if i < 4 or i in (5, 7) else hoov_height - x
                # z = hoov_height
                print("  "*tab + f"goal: {[float(e) for e in [x, y, z]]}")
            else :
                radius = big_radius/np.sqrt(2)
                x, y = radius*np.cos(angle), radius*np.sin(angle)
                dh = radius if i < 4 or i in (5, 7) else -radius
                print("  "*tab + f"goal: {[float(e) for e in np.array([x, y, (hoov_height+dh)])]}")
                if abs(angle) < omega/2 :
                    for j, b in enumerate(ring) :
                        if b :
                            alphas[i] = alphas[j] - angles[agents[i]] + angles[agents[j]]
                            break
                    ring[i] = True
            print("  "*tab + "size: 0.2")
            tab -= 1

    nb_period_done += nb_period_transi_2
    theta = np.pi/4
    print()

    # CHanging theta
    for p in range(nb_period_modif_theta) :
        print()
        theta -= np.pi/4/nb_period_modif_theta
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{(p+nb_period_done)*len(agents)+i+1}:")
            tab += 1
            print("  "*tab + f"period_num: {p + nb_period_done}")
            print("  "*tab + f"bot: {a}")
            alpha = alphas[i] + omega
            alphas[i] = alpha%(2*np.pi)  # pyright: ignore[reportArgumentType, reportCallIssue]
            x = big_radius*np.sin(alpha)*np.cos(theta)
            y = big_radius*np.cos(alpha)
            z = hoov_height + big_radius*np.sin(alpha)*np.sin(theta) if i < 4 or i in (5, 7) else hoov_height - big_radius*np.sin(alpha)*np.sin(theta)
            print("  "*tab + f"goal: {[float(e) for e in [x, y, z]]}")
            print("  "*tab + "size: 0.2")
            tab -= 1

    nb_period_done += nb_period_modif_theta
    print()

    # Landing
    for p in range(nb_period_land) :
        print()
        for i, a in enumerate(agents) :
            print("  "*tab + f"task_{(p+nb_period_done)*len(agents)+i+1}:")
            tab += 1
            print("  "*tab + f"period_num: {p + nb_period_done}")
            print("  "*tab + f"bot: {a}")
            alpha = alphas[i] + 1.5*speed/big_radius
            alphas[i] = alpha%(2*np.pi)  # pyright: ignore[reportArgumentType, reportCallIssue]
            x = big_radius*np.sin(alpha)
            y = big_radius*np.cos(alpha)
            z = hoov_height + (.3-hoov_height)/(nb_period_land-1)*p
            print("  "*tab + f"goal: {[float(e) for e in [x, y, z]]}")
            print("  "*tab + "size: 0.2")
            tab -= 1
