import math

print('='*60)
print('场景: 腿垂直(alpha=-90), 身体前倾')
print('='*60)

# ======= 参考代码 (右腿) =======
print()
print('--- 参考代码 (右腿) ---')

phi0_eq = -math.pi/2  # 腿垂直向下时 phi0 = -pi/2
theta_eq = math.pi/2 - 0 - phi0_eq  # = pi
print('腿垂直: phi0 = %.4f = -pi/2' % phi0_eq)
print('平衡: theta_R = pi/2 - 0 - (-pi/2) = pi = %.4f' % theta_eq)

# 前倾: PitchR = +0.05
theta_fp = math.pi/2 - 0.05 - phi0_eq
print('前倾(PitchR=+0.05): theta_R=%.4f, delta=%.4f' % (theta_fp, theta_fp - math.pi))

# ======= 我们的代码 (右腿) =======
print()
print('--- 我们的代码 (右腿) ---')
print('theta_right = DEG2RAD(-pitch + body_angle + 90)')

# 平衡
theta_ours_eq = math.radians(0 + (-90) + 90)
print('平衡: theta = DEG2RAD(0+(-90)+90) = %.4f' % theta_ours_eq)

# 前倾 pitch=+3
pitch = 3.0
theta_ours_fp = math.radians(-pitch + (-90) + 90)
print('前倾(pitch=+3): theta = %.4f rad = %.1f deg' % (theta_ours_fp, math.degrees(theta_ours_fp)))

print()
print('--- 关系分析 ---')
print('参考: theta_R(eq)=pi, 前倾后 theta_R=pi-0.05 (减小)')
print('我们: theta_ours(eq)=0, 前倾后 theta_ours=-0.05 (减小)')
print('两者变化方向一致!')
print()
print('theta_R = theta_ours + pi')
print('验证: pi - 0.05 = (0 - 0.05) + pi = pi - 0.05 ✓')

# ======= K矩阵验证 =======
print()
print('--- K矩阵验证 ---')
L0 = 0.07
K0 = -213.69*(L0**3) + 153.33*(L0**2) + (-50.98)*L0 + (-0.13)
K4 = -246.36*(L0**3) + 173.91*(L0**2) + (-47.66)*L0 + 6.13
print('K[0](theta->T)=%.4f  K[4](phi->T)=%.4f' % (K0, K4))
print()

# 参考代码: T = K[0]*theta_R + K[4]*PitchR
# 我们代码: T = K[0]*theta_ours + K[4]*pitch
# 
# theta_R = theta_ours + pi
# T_ref = K[0]*(theta_ours + pi) + K[4]*PitchR
#        = K[0]*theta_ours + K[0]*pi + K[4]*PitchR
# T_ours = K[0]*theta_ours + K[4]*pitch
# 
# 动态部分 (去掉常数):
# T_ref_dyn = K[0]*delta_theta_ours + K[4]*delta_pitch
# T_ours_dyn = K[0]*delta_theta_ours + K[4]*delta_pitch
# 
# 它们相同! (如果 theta_R = theta_ours + pi)

delta_theta = -0.05
delta_pitch = 0.05
T_ref_dyn = K0 * delta_theta + K4 * delta_pitch
T_ours_dyn = K0 * delta_theta + K4 * delta_pitch
print('delta_theta=%.4f, delta_pitch=%.4f' % (delta_theta, delta_pitch))
print('T_ref_dyn  = K[0]*dtheta + K[4]*dpitch = %.4f' % T_ref_dyn)
print('T_ours_dyn = K[0]*dtheta + K[4]*dpitch = %.4f' % T_ours_dyn)
print('动态部分完全相同! (因为 theta_R = theta_ours + pi, 偏差相同)')

print()
print('='*60)
print('重要结论: theta_R = theta_ours + pi (不是 pi - theta_ours!)')
print()
print('之前的分析说 theta_R = pi - theta_ours 是错误的!')
print('正确关系: 两者只差一个常数偏移 pi')
print('动态变化方向完全一致, 不需要取反!')
print('='*60)

# 但是等等... 让我再仔细验证
print()
print('--- 仔细验证: 如果只有alpha变化(pitch不变) ---')
print()

# 假设 pitch=0, alpha 从 -90 变到 -85 (腿向后摆5度)
# 我们: body_angle从-90变到-85
# theta_ours = DEG2RAD(-0 + (-85) + 90) = DEG2RAD(5) = 0.0873
theta_ours_back = math.radians(0 + (-85) + 90)
print('腿向后摆5度: body_angle=-85')
print('  theta_ours = DEG2RAD(0+(-85)+90) = DEG2RAD(5) = %.4f' % theta_ours_back)
print('  theta_ours 增大了 (从0到+0.0873)')

# 参考代码: 
# phi0 = atan2(YC, XC-l5/2)
# 当腿向后摆(从右边看逆时针), alpha增大, phi0怎么变?
# alpha = pi/2 - phi0 → phi0 = pi/2 - alpha
# alpha从-pi/2变到-pi/2+5deg → phi0从-pi/2变到-pi/2-5deg(减小)
phi0_back = -math.pi/2 - math.radians(5)
theta_R_back = math.pi/2 - 0 - phi0_back
print()
print('参考代码:')
print('  alpha: -90 -> -85 deg (向后摆)')
print('  phi0: -pi/2 -> %.4f (减小)' % phi0_back)
print('  theta_R = pi/2 - 0 - (%.4f) = %.4f' % (phi0_back, theta_R_back))
print('  theta_R 增大了 (从pi=%.4f 到 %.4f)' % (math.pi, theta_R_back))
print()
print('两者都增大! 再次验证 theta_R = theta_ours + pi')
print('  %.4f = %.4f + pi = %.4f ✓' % (theta_R_back, theta_ours_back, theta_ours_back + math.pi))
