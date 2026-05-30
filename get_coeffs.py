from scipy.signal import butter
b_bp, a_bp = butter(4, [0.5/128, 10/128], btype='band')
b_hp, a_hp = butter(4, 20/128, btype='high')
print("b_bp =", list(b_bp))
print("a_bp =", list(a_bp))
print("b_hp =", list(b_hp))
print("a_hp =", list(a_hp))
