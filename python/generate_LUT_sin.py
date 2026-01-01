import math

SIN_LUT_SIZE = 128
Q15_SCALE = 32767

lut = []

for i in range(SIN_LUT_SIZE + 1):
    angle = i * math.pi / (2.0 * SIN_LUT_SIZE)
    val = int(round(math.sin(angle) * Q15_SCALE))
    lut.append(val)

# Vérifications de sécurité
assert len(lut) == SIN_LUT_SIZE + 1
assert lut[0] == 0
assert lut[-1] == 32767
assert all(-32768 <= v <= 32767 for v in lut)

# Impression format C
print("static const int16_t sin_lut[SIN_LUT_SIZE + 1] = {")
for i in range(0, len(lut), 8):
    row = ", ".join(f"{v:6d}" for v in lut[i:i+8])
    print("   " + row + ",")
print("};")
