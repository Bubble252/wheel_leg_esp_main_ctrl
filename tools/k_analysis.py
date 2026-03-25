#!/usr/bin/env python3
"""K matrix physical intuition analysis"""

coeff_list = [
    ("K0  theta->T  ", [-85.9446, 51.8245, -16.0279, 0.0171]),
    ("K1  dtheta->T  ", [-0.7067, 0.2248, -1.2956, 0.0074]),
    ("K2  x->T       ", [-91.1227, 47.5999, -8.8752, -0.0779]),
    ("K3  v->T       ", [-76.6760, 40.6210, -8.1250, -0.0850]),
    ("K4  phi->T     ", [-124.8101, 110.1666, -34.2998, 4.6626]),
    ("K5  dphi->T    ", [-6.0142, 5.2943, -1.6627, 0.2328]),
    ("K6  theta->Tp  ", [278.8236, -95.8756, 2.7879, 3.3673]),
    ("K7  dtheta->Tp ", [27.2014, -11.0475, 1.3292, 0.3141]),
    ("K8  x->Tp      ", [-101.9070, 89.9506, -28.0057, 3.8070]),
    ("K9  v->Tp      ", [-106.9091, 87.5343, -26.1997, 3.6044]),
    ("K10 phi->Tp    ", [2232.0404, -1165.9537, 217.3977, 1.9079]),
    ("K11 dphi->Tp   ", [112.8738, -59.0739, 11.0640, 0.0073]),
]

def pe(c, L):
    return c[0]*L**3 + c[1]*L**2 + c[2]*L + c[3]

# Table at multiple leg lengths
legs = [0.10, 0.12, 0.14, 0.16, 0.18, 0.20]
print("=" * 100)
hdr = "%-18s" % "K index"
for L in legs:
    hdr += "  L=%.2fm " % L
print(hdr)
print("=" * 100)
for name, c in coeff_list:
    line = "%-18s" % name
    for L in legs:
        line += "  %+8.4f " % pe(c, L)
    print(line)
print("=" * 100)

# Trend analysis
print("\n--- Leg length trend (L0: 0.10 -> 0.20) ---")
for name, c in coeff_list:
    v10 = pe(c, 0.10)
    v20 = pe(c, 0.20)
    pct = (v20 - v10) / abs(v10) * 100 if abs(v10) > 0.01 else 9999
    print("  %s: %+.4f -> %+.4f (%+.1f%%)" % (name.strip(), v10, v20, pct))

# Scenario analysis at L0=0.14
print("\n\n" + "=" * 70)
print("  Physical scenario analysis @ L0=0.14m (MATLAB frame, u=-K*x)")
print("=" * 70)

K = [pe(c, 0.14) for _, c in coeff_list]

scenarios = [
    ("theta=+0.05 (leg backward 2.9deg)", [0.05, 0, 0, 0, 0, 0]),
    ("dtheta=+0.5 (leg swinging backward)", [0, 0.5, 0, 0, 0, 0]),
    ("x=+0.1 (forward 10cm)", [0, 0, 0.1, 0, 0, 0]),
    ("v=+0.3 (moving forward 0.3m/s)", [0, 0, 0, 0.3, 0, 0]),
    ("phi=+0.1 (forward lean 5.7deg)", [0, 0, 0, 0, 0.1, 0]),
    ("phi=-0.1 (backward lean 5.7deg)", [0, 0, 0, 0, -0.1, 0]),
    ("dphi=+0.5 (tilting forward)", [0, 0, 0, 0, 0, 0.5]),
    ("combined: lean fwd+overshoot", [0, 0, 0.05, 0.1, 0.08, 0.2]),
]

for desc, state in scenarios:
    T = sum(-K[i] * state[i] for i in range(6))
    Tp = sum(-K[i + 6] * state[i] for i in range(6))
    print("\n  %s" % desc)
    print("    T  = %+.4f Nm  (>0=forward, <0=backward)" % T)
    print("    Tp = %+.4f Nm  (>0=leg backward, <0=leg forward)" % Tp)

# Magnitude comparison
print("\n\n" + "=" * 70)
print("  Contribution magnitude ranking @ L0=0.14m")
print("=" * 70)
labels = ["theta", "dtheta", "x", "v", "phi", "dphi"]
print("\n  T contributions (|K[i]|):")
T_contribs = [(abs(K[i]), labels[i], K[i]) for i in range(6)]
T_contribs.sort(reverse=True)
for mag, lab, val in T_contribs:
    print("    %-8s: |K| = %.4f  (K = %+.4f)" % (lab, mag, val))

print("\n  Tp contributions (|K[i+6]|):")
Tp_contribs = [(abs(K[i + 6]), labels[i], K[i + 6]) for i in range(6)]
Tp_contribs.sort(reverse=True)
for mag, lab, val in Tp_contribs:
    print("    %-8s: |K| = %.4f  (K = %+.4f)" % (lab, mag, val))
