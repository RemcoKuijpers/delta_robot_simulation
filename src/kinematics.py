#!/usr/bin/env python

from math import sqrt, atan, cos, sin, degrees, radians

# Arm lengtes in mm
L = 320
l = 850
z_offset = -920

# Basis driehoek afmetingen in mm
Wb = 200
Sb = 2*sqrt(3)*Wb
Ub = 2*Wb

# Eindeffector driehoek afmetingen in mm
Up = 50
Sp = sqrt(3)*Up
Wp = 0.5*Up

#----- Omgekeerde kinematica -----
def reverse(x,y,z):
    # Dummy variabelen
    a = Wb-Up
    b = (Sp/2)-((sqrt(3)/2)*Wb)
    c = Wp-(0.5*Wb)
    y = -y

    # ---Weierstrass variabelen---
    # Arm 1
    E1 = 2*L*(y+a)
    F1 = 2*z*L
    G1 = x**2+y**2+z**2+a**2+L**2+(2*y*a)-l**2

    # Arm 2
    E2 = -L*(sqrt(3)*(x+b)+y+c)
    F2 = 2*z*L
    G2 = x**2+y**2+z**2+b**2+c**2+L**2+2*(x*b+y*c)-l**2

    # Arm 3
    E3 = L*(sqrt(3)*(x-b)-y-c)
    F3 = 2*z*L
    G3 = x**2+y**2+z**2+b**2+c**2+L**2+2*(-x*b+y*c)-l**2

    # Zet Weierstrass variabelen in list om er doorheen te kunnen  loopen bij het vinden van de t waardes
    Wl = [[E1,F1,G1], [E2,F2,G2], [E3,F3,G3]]

    try:
        #--- Vind t-waardes ---
        ti = []
        for v in Wl:
            # abc-formule + variant
            t1 = (sqrt(v[1]**2-v[2]**2+v[0]**2)-v[1])/(v[2]-v[0])
            # abc formule - variant
            t2 = (sqrt(v[1]**2-v[2]**2+v[0]**2)+v[1])/(v[2]-v[0])
            # Zet resultaten in list
            ti.append([t1,t2])
        
        # Bereken oplossingen voor theta
        theta1 = [2*atan(ti[0][0]), 2*atan(ti[0][1])]
        theta3 = [2*atan(ti[1][0]), 2*atan(ti[1][1])]
        theta2 = [2*atan(ti[2][0]), 2*atan(ti[2][1])]

        # Testen blijkt dat de - variant de juiste oplossing geeft

        return [theta1[1], theta2[1], theta3[1]] # Geef oplossing in radialen

    except ValueError:
        print("Position is unreachable")
        return None

# ----- Voorwaartse kinematica -----
def forward(theta1, theta2, theta3):
    #--- Middenpunten bollen ---
    # Arm 1 (x1 = 0)
    y1 = -Wb-(L*cos(theta1))+Up
    z1 = -L*sin(theta1)

    # Arm 2
    x2 = (sqrt(3)/2)*(Wb+L*cos(theta2))-(0.5*Sp)
    y2 = 0.5*(Wb+(L*cos(theta2)))-Wp
    z2 = -L*sin(theta2)

    # Arm 3
    x3 = -(sqrt(3)/2)*(Wb+L*cos(theta3))+(0.5*Sp)
    y3 = 0.5*(Wb+L*cos(theta3))-Wp
    z3 = -L*sin(theta3)

    # Dummy variabelen
    d = ((y2-y1)*x3)-((y3-y1)*x2)
    w1 = y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    # x = (a1*z + b1)/d
    a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
    b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

    # y = (a2*z + b2)/d
    a2 = -(z2-z1)*x3+(z3-z1)*x2
    b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

    # a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + d*d
    b = 2*(a1*b1 + a2*(b2-y1*d) - z1*d*d)
    c = (b2-y1*d)*(b2-y1*d) + b1*b1 + d*d*(z1*z1 - l*l)

    # Vind z door vergelijking op te lossen met abc-formule
    z11 = (-b+sqrt(b**2-4*a*c))/(2*a)
    z12 = (-b-sqrt(b**2-4*a*c))/(2*a)
    z = [z11, z12]
    # Uit testen blijkt dat de juiste z gevonden wordt met de + variant van de abc-formule
    # Z invullen om x en y te vinden
    x = -(a1*z[0]+b1)/d
    y = -(a2*z[0]+b2)/d

    return [x,y,-z[0]-z_offset]

#reverse(0,0,-920+240)
#reverse(-167,-77,-920+230)
#reverse(61.18,-74.38,-920+153)
#reverse(56,-74,-920+800)
#m1, m2, m3 = radians(-23), radians(-27), radians(7)
#forward(m1,m2,m3)
#reverse(185.5129397890076, -79.97139654144587, -767.6459232727889)