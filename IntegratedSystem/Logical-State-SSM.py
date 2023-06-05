import time as t
def SSM_calculation(Vr, Vh, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr*Ts + ((ac*pow(Ts, 2))/2)
    Sp   = Vh * (Tr + Ts) + (Vr * Tr) + Ss + Ctot
    return Sp
def SpMax(Vr_Max, Vh_Max, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_Max*Ts + ((ac*pow(Ts, 2))/2)
    SpMax   = (Vh_Max * (Tr + Ts)) + (Vr_Max * Tr) + Ss + Ctot
    return SpMax

def Spmin(C, Zd, Zr):
    Ctot = C + Zd + Zr
    SpminVal = Ctot
    return SpminVal

def SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts,2))/2)
    SpPFL   = Vh * ( Tr + Ts ) + (Vr_PFL * Tr) + Ss + Ctot
    return SpPFL
def SpSafe(Vr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr*Ts + ((ac*pow(Ts, 2))/2)
    SpSafeVal = Ss + Ctot
    return SpSafeVal

def Vr_SSM(D, Vh, Tr, Ts, ac, C, Zd, Zr, Vr_PFL):
    T = Tr + Ts
    Ctot = C + Zd + Zr
    VrSSM = ((D - (Vh*T) - Ctot) / T) - (ac*pow(Ts, 2)/(2*T))
    if VrSSM < Vr_PFL:
        Reduce_Value = 0
    else:
        Reduce_Value = VrSSM
    return Reduce_Value

def Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    VrSSM2 = (D / Ts) - ((ac*Ts)/2) - (Ctot/Ts)
    if VrSSM2 > 0:
        Stop_Value = VrSSM2
    else:
        Stop_Value = 0
    return Stop_Value

vrstop = 0
D = 600
Vrinitial = 1500
Vr = Vrinitial
Vr_PFL = 400
Vh_max = 1600
Vh = 0
Tr = 0.1
Ts = 0.08
ac = 3000
C_SSM = 200
Zd = 106.7
Zr = 1
Ss = 0
RobotVrmax = 1500

## SSM Preparation Calculation
Sp = SSM_calculation(Vr, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
Spfull = SpMax(Vr, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)
SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)
SpPFLVal = SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
# ===== SSM calculation ======
print("Safety Separation Distance: ", Sp, ",", Spfull, ",", SpminVal, ", ", SpSafeVal, ",", SpPFLVal)

if D <= SpminVal:
    Vr = 0
    # server.pause()
    print("Robot harus berhenti", Vr)
    mode_collab = 4
    # mode SSM ori stop = 3
    mode_SSMori = 3
    VrOriSSM = 0
    # pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
    # text_stop = font_reg.render("Stop", True, (242, 242, 247))
    # window.blit(text_stop, (467, 555))
    # pygame.draw.rect(window, red, (460, 588, 166, 81), border_radius=5)

    t.sleep(0.5)
elif D > SpminVal and D <= SpSafeVal:
    # server.resume()
    # print("Robot speed reduction")
    speed = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
    Vr = round(speed, 2)

    mode_collab = 2
    # calculate the Vmax allowable
    # print("Vmax allowable in this workspace: ", Vr_max_command)
    # Vr = Vr_max_command

    print("change value speed safe: ", Vr)
    # jacoRobot.setSpeed(Vr, vrot)

    # mode SSM ori reduce speed = 2
    # mode_SSMori = 2
    # VrOriSSM = speed

    # print("Succes send speed Vr Mid")
    # pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
    # text_coll = font_reg.render("Collaboration", True, (242, 242, 247))
    # window.blit(text_coll, (467, 555))
    # pygame.draw.rect(window, blue, (460, 588, 166, 81), border_radius=5)
    # #jacoRobot.message("Robot speed reduction")
    t.sleep(0.5)
elif D > SpSafeVal and D <= SpPFLVal:
    # server.resume()
    # print("Robot speed reduction")
    mode_collab = 2
    # calculate the Vmax allowable
    # print("Vmax allowable in this workspace: ", Vr_max_command)
    # Vr = Vr_max_command
    speed = Vr_PFL
    Vr = round(speed, 2)
    print("change value speed PFL: ", Vr_PFL)
    # jacoRobot.setSpeed(Vr, vrot)

    # pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
    # text_coll = font_reg.render("Collaboration", True, (242, 242, 247))
    # window.blit(text_coll, (467, 555))
    # pygame.draw.rect(window, blue, (460, 588, 166, 81), border_radius=5)
    # jacoRobot.message("Robot speed reduction")
    t.sleep(0.5)

elif D > SpPFLVal and D <= Spfull:
    # server.resume()
    # print("Robot speed reduction")
    # mode_collab = 2
    # calculate the Vmax allowable
    # print("Vmax allowable in this workspace: ", Vr_max_command)
    # Vr = Vr_max_command
    speed = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
    Vr = round(speed, 2)
    # print("change value speed Reduce: ", speed)
    # jacoRobot.setSpeed(Vr, vrot)

    # mode SSM ori reduce speed = 2
    # mode_SSMori = 2
    # VrOriSSM = speed
    print("Kecepatan rreduce speed:", Vr)
    # pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
    # text_reduce = font_reg.render("Reduce Speed", True, (242, 242, 247))
    # window.blit(text_reduce, (467, 555))
    # pygame.draw.rect(window, yellow, (460, 588, 166, 81), border_radius=5)
    # jacoRobot.message("Robot speed reduction")
    t.sleep(0.5)
else:
    # server.resume()
    # print("Robot bekerja maximal")
    # mode_collab = 1
    Vr = RobotVrmax

    print("change value speed maximum: ", Vr)
    # jacoRobot.setSpeed(Vr, vrot)

    # mode SSM ori full speed = 1
    # mode_SSMori = 1
    # VrOriSSM = speed
    # print("Succes send speed Vr Full Speed")
    # pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
    # text_freespeed = font_reg.render("Full Speed", True, (242, 242, 247))
    # window.blit(text_freespeed, (467, 555))
    # pygame.draw.rect(window, green, (460, 588, 166, 81), border_radius=5)
    # jacoRobot.message("Robot free speed")
    t.sleep(0.5)