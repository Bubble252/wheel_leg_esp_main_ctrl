import math

print('=== 参考代码: phi1/phi4 与前后的对应 ===')
print()
print('从底盘上往下看(上=前, 下=后):')
print()
print('右腿:')
print('  右上角(前) id=6 -> joint_motor[0] -> phi1 (A点)')
print('  右下角(后) id=8 -> joint_motor[1] -> phi4 (E点)')
print('  -> phi1=前, phi4=后, X轴: A->E = 前->后')
print()
print('左腿:')
print('  左上角(前) id=8 -> joint_motor[3] -> phi4 (E点)')
print('  左下角(后) id=6 -> joint_motor[2] -> phi1 (A点)')
print('  -> phi1=后, phi4=前, X轴: A->E = 后->前')
print()
print('两腿五连杆X轴方向 (对机器人): 右腿前->后, 左腿后->前')
print()

print('=== theta与摆动方向 ===')
print()

# 右腿向前摆
theta_ours_R_fwd = math.radians(0 + (-95) + 90)  # alpha=-95
print('右腿向前摆5度(alpha:-90->-95):')
print('  theta_ours = DEG2RAD(-5) = %.4f (减小)' % theta_ours_R_fwd)

phi0_R_fwd = -math.pi/2 + math.radians(5)
theta_R_fwd = math.pi/2 - 0 - phi0_R_fwd
print('  参考: theta_R = %.4f, delta = %.4f (减小)' % (theta_R_fwd, theta_R_fwd - math.pi))

# 右腿向后摆
theta_ours_R_bwd = math.radians(0 + (-85) + 90)  # alpha=-85
print()
print('右腿向后摆5度(alpha:-90->-85):')
print('  theta_ours = DEG2RAD(5) = %.4f (增大)' % theta_ours_R_bwd)

phi0_R_bwd = -math.pi/2 - math.radians(5)
theta_R_bwd = math.pi/2 - 0 - phi0_R_bwd
print('  参考: theta_R = %.4f, delta = %.4f (增大)' % (theta_R_bwd, theta_R_bwd - math.pi))

print()
print('=== 左腿分析 ===')
print()
print('左腿五连杆: A(phi1)=后方, E(phi4)=前方, X轴: 后->前')
print('左腿 PitchL = -Pitch, PithGyroL = -Gyro')
print()
print('腿向前摆(从左边看,腿往前):')
print('  从左边看, 腿顺时针转')
print('  五连杆X轴指向前方, C点偏向X正 = 前方')
print('  phi0 = atan2(YC, XC-l5/2)')
print('  C点向X正方向偏: XC增大, YC大致不变(接近垂直时)')

# 左腿向前摆时, 在五连杆坐标系中:
# X轴向前, 腿向前摆 = C点的极角phi0变化
# 垂直时 phi0=-pi/2 (Y向下)
# 向前摆 = C点从正下方向X正(前)方偏
# phi0从-pi/2向0方向增大
phi0_L_fwd = -math.pi/2 + math.radians(5)
theta_L_fwd = math.pi/2 - 0 - phi0_L_fwd  # PitchL=0 when Pitch=0
print('  phi0: -pi/2 -> %.4f (增大)' % phi0_L_fwd)
print('  theta_L = pi/2 - 0 - (%.4f) = %.4f' % (phi0_L_fwd, theta_L_fwd))
print('  theta_L 减小 (从pi=%.4f 到 %.4f)' % (math.pi, theta_L_fwd))
print()
print('腿向后摆:')
phi0_L_bwd = -math.pi/2 - math.radians(5)
theta_L_bwd = math.pi/2 - 0 - phi0_L_bwd
print('  phi0: -pi/2 -> %.4f (减小)' % phi0_L_bwd)
print('  theta_L = pi/2 - 0 - (%.4f) = %.4f' % (phi0_L_bwd, theta_L_bwd))
print('  theta_L 增大 (从pi=%.4f 到 %.4f)' % (math.pi, theta_L_bwd))

print()
print('='*50)
print('结论:')
print('  右腿: theta增大 = 向后摆')
print('  左腿: theta增大 = 向后摆')
print('  两腿一致! theta变大都是向后摆!')
print('='*50)
