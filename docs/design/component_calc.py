pfet_R1 = 1.5e3
pfet_R2 = 1e3

V_max = 30
V_typ = 18
V_min = 9

print("--- P_FET VGS ---")
print("R1={:.0f} R2={:.0f}".format(pfet_R1, pfet_R2))

Vgs_max = V_max*pfet_R1/(pfet_R1+pfet_R2)
print("V={:.0f}, V_GS={:.2f}".format(V_max, Vgs_max))

Vgs_typ = V_typ*pfet_R1/(pfet_R1+pfet_R2)
print("V={:.0f}, V_GS={:.2f}".format(V_typ, Vgs_typ))

Vgs_min = V_min*pfet_R1/(pfet_R1+pfet_R2)
print("V={:.0f}, V_GS={:.2f}".format(V_min, Vgs_min))

print("--- P_FET VGS MAX (TOL) ---")

Vgs_tol_max = V_max*(pfet_R1*1.01)/((pfet_R1*1.01)+(pfet_R2*0.99))
Vgs_tol_min = V_max*(pfet_R1*0.99)/((pfet_R1*0.99)+(pfet_R2*1.01))
print("MAX: V_GS={:.2f}   MIN: V_GS={:.2f}".format(Vgs_tol_max, Vgs_tol_min))

print("--- STANDING CURRENT ---")

I_standing = V_max/(pfet_R1+pfet_R2)
print("I_stand MAX={:.2f}mA".format(I_standing*1000))

I_standing = V_typ/(pfet_R1+pfet_R2)
print("I_stand TYP={:.2f}mA".format(I_standing*1000))

print("--- N_FET GATE CURRENT ---")
V_IO = 3.3
R_gate = 470
I_gate = V_IO/R_gate
print("I_gate: R={:.2f}, I={:.2f}".format(R_gate, I_gate*1000))

print("--- LPF ---")
R = 10e3
C = 100e-9
Fc = 1/(2*3.14159*R*C)
print("Cutoff frequency = {:.2f}Hz".format(Fc))

print("--- VOLTAGE SENSING ---")
V_R1 = 100e3
V_R2 = 12e3
V_ref = 27

steps = 4096
resolution = 3.3/4096

V_out = (V_ref*V_R2)/(V_R1+V_R2)
adc_value = round(V_out/resolution)

print("V_out={:.3f}, ADC={:.0f}".format(V_out, adc_value))