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


def Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    VrSSM2 = (D / Ts) - ((ac*Ts)/2) - (Ctot/Ts)
    if VrSSM2 > 0:
        Stop_Value = VrSSM2
    else:
        Stop_Value = 0
    return Stop_Value

D = 348
Vrinitial = 1500
Vr = Vrinitial
Vr_PFL = 400
Vh_max = 1600
Vh = 0
Tr = 0.1
Ts = 0.08
ac = 3000
C = 200
Zd = 106.7
Zr = 1
Ss = 0

Spmax = SpMax(Vrinitial,Vh_max,Tr, Ts, ac, C, Zd, Zr)
print("Nilai Sp max", Spmax)
Spmin = Spmin(C, Zd, Zr)
print("Nilai Sp min", Spmin)
SpPFL = SpPFL(Vr_PFL,Vh_max,Tr, Ts, ac, C, Zd, Zr)
print("Nilai Sp PFL", SpPFL)
SpSafer = SpSafe(Vr_PFL, Ts, ac, C, Zd, Zr)
print("Nilai Sp Safer", SpSafer)


velSSM2 = Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr)
print("Nilai VelSSM2", velSSM2)

