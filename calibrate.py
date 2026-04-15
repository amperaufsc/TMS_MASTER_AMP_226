import math

kB = 8.617e-5

T_K = [5.0 + 273.15, 25.0 + 273.15, 40.0 + 273.15]
T5 = T_K[0]
T25 = T_K[1]
T40 = T_K[2]

rs_5c  = [27.1, 26.3, 26.4, 26.9, 27.2, 27.3, 28.3, 30.5, 31.3]
rs_25c = [19.1, 17.0, 16.5, 17.0, 17.3, 17.4, 17.3, 17.9, 18.4]
rs_40c = [14.1, 13.2, 13.5, 13.7, 13.2, 13.0, 13.0, 14.2, 13.8]

# convert to ohms
rs_5c = [x * 1e-3 for x in rs_5c]
rs_25c = [x * 1e-3 for x in rs_25c]
rs_40c = [x * 1e-3 for x in rs_40c]

def theoretical_ratio(ea):
    num = math.exp(ea / (kB * T5)) - math.exp(ea / (kB * T25))
    den = math.exp(ea / (kB * T25)) - math.exp(ea / (kB * T40))
    return num / den

num_soc = 9
r0_list = []
r1_list = []
ea_list = []

for i in range(num_soc):
    target_ratio = (rs_5c[i] - rs_25c[i]) / (rs_25c[i] - rs_40c[i])
    
    # Bisection
    low = 0.01
    high = 1.0
    for _ in range(100):
        mid = (low + high) / 2
        ratio = theoretical_ratio(mid)
        if ratio > target_ratio:
            high = mid
        else:
            low = mid
            
    ea = (low + high) / 2
    r1 = (rs_5c[i] - rs_25c[i]) / (math.exp(ea / (kB * T5)) - math.exp(ea / (kB * T25)))
    r0 = rs_25c[i] - r1 * math.exp(ea / (kB * T25))
    
    ea_list.append(ea)
    r1_list.append(r1)
    r0_list.append(r0)

# We want 11 points for SOC 0.0 to 1.0, wait! We have 9 points [0.1, 0.2 ... 0.9].
# For 0.0 and 1.0 we can extrapolate or just repeat 0.1 and 0.9.
def pad(lst):
    return [lst[0]] + lst + [lst[-1]]

r0_final = pad(r0_list)
r1_final = pad(r1_list)
ea_final = pad(ea_list)

print("static const float r0Lut[BMS_TEMP_EST_SOC_POINTS] = {")
print(", ".join(f"{r:.6f}f" for r in r0_final))
print("};")

print("static const float r1Lut[BMS_TEMP_EST_SOC_POINTS] = {")
print(", ".join(f"{r:.6e}f" for r in r1_final))
print("};")

print("static const float eaLut[BMS_TEMP_EST_SOC_POINTS] = {")
print(", ".join(f"{r:.6f}f" for r in ea_final))
print("};")
