import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Inisialisasi Kanvas 3D
fig = plt.figure(figsize=(14, 8.5))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.05, bottom=0.22, right=0.72, top=0.95)

g_const = 9.80665  # Gravitasi standar (m/s^2)

# ==========================================
# 1. DEFINISI GEOMETRI 3D DRONE FPV
# ==========================================
def create_drone_fpv_geometry():
    """Membuat mesh drone FPV: Frame X, Kamera Moncong, Baterai, Motor, Propeller"""
    body_plate = np.array([[ 0.8, -0.25, 0.0], [ 0.8,  0.25, 0.0], [-0.8,  0.25, 0.0], [-0.8, -0.25, 0.0]])
    
    arms = [
        np.array([[0, 0, 0], [ 1.3,  1.3, 0.0]]),
        np.array([[0, 0, 0], [ 1.3, -1.3, 0.0]]),
        np.array([[0, 0, 0], [-1.3,  1.3, 0.0]]),
        np.array([[0, 0, 0], [-1.3, -1.3, 0.0]])
    ]
    
    bx, by, bz = 0.5, 0.2, 0.25
    batt_vertices = np.array([
        [-bx, -by, -bz], [bx, -by, -bz], [bx, by, -bz], [-bx, by, -bz],
        [-bx, -by, 0.0], [bx, -by, 0.0], [bx, by, 0.0], [-bx, by, 0.0]
    ])
    batt_vertices[:, 2] -= 0.02

    cam_box = np.array([
        [0.7, -0.15, -0.1], [1.05, -0.12, -0.22], [1.05, 0.12, -0.22], [0.7, 0.12, -0.1],
        [0.7, -0.15,  0.0], [1.05, -0.12, -0.12], [1.05, 0.12, -0.12], [0.7, 0.12,  0.0]
    ])
    lens_center = np.array([1.06, 0.0, -0.17])
    
    t = np.linspace(0, 2*np.pi, 16)
    prop_r = 0.45
    circle_pts = np.stack([prop_r*np.cos(t), prop_r*np.sin(t), np.zeros_like(t)], axis=1)
    
    motor_pos = [
        np.array([ 1.3,  1.3, -0.05]), np.array([ 1.3, -1.3, -0.05]),
        np.array([-1.3,  1.3, -0.05]), np.array([-1.3, -1.3, -0.05])
    ]
    props = [circle_pts + pos for pos in motor_pos]
    
    return body_plate, arms, batt_vertices, cam_box, lens_center, motor_pos, props

body_plate, arms, batt_vertices, cam_box, lens_center, motor_pos, props = create_drone_fpv_geometry()

# ==========================================
# 2. MATRIKS ROTASI DIRGANTARA (FRD)
# ==========================================
def get_rotation_matrix(pitch_deg, roll_deg, yaw_deg):
    pitch = np.radians(pitch_deg)
    roll  = np.radians(roll_deg)
    yaw   = np.radians(yaw_deg)
    
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)], [ 0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll),  np.cos(roll)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw),  np.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# ==========================================
# 3. FUNGSI UPDATE VISUALISASI INTERAKTIF
# ==========================================
def update(val=None):
    yaw   = slider_yaw.val
    pitch = slider_pitch.val
    roll  = slider_roll.val
    
    R = get_rotation_matrix(pitch, roll, yaw)
    
    ax.cla()
    scale = 3.5
    ax.set_xlim([-scale, scale])
    ax.set_ylim([-scale, scale])
    ax.set_zlim([-scale, scale])
    
    ax.set_xlabel('(X)')
    ax.set_ylabel('(Y)')
    ax.set_zlabel('(Z)')
    ax.set_title(f"Visualisasi Gaya Spesifik IMU Drone FPV\nYaw: {yaw:.1f}° | Pitch: {pitch:.1f}° | Roll: {roll:.1f}°", fontsize=11, fontweight='bold')

    def to_plot(pts):
        pts_frd = pts.copy()
        pts_frd[:, 2] = -pts_frd[:, 2]
        return (R @ pts_frd.T).T

    bp_rot = to_plot(body_plate)
    ax.add_collection3d(Poly3DCollection([bp_rot], facecolors='gray', edgecolors='black', alpha=0.85))
    
    for arm in arms:
        a_rot = to_plot(arm)
        ax.plot(a_rot[:, 0], a_rot[:, 1], a_rot[:, 2], color='black', linewidth=4)
        
    bv_rot = to_plot(batt_vertices)
    faces = [[bv_rot[0], bv_rot[1], bv_rot[2], bv_rot[3]], [bv_rot[4], bv_rot[5], bv_rot[6], bv_rot[7]], [bv_rot[0], bv_rot[1], bv_rot[5], bv_rot[4]], [bv_rot[2], bv_rot[3], bv_rot[7], bv_rot[6]]]
    ax.add_collection3d(Poly3DCollection(faces, facecolors='gold', edgecolors='goldenrod', alpha=0.9))

    cam_rot = to_plot(cam_box)
    cam_faces = [[cam_rot[0], cam_rot[1], cam_rot[2], cam_rot[3]], [cam_rot[4], cam_rot[5], cam_rot[6], cam_rot[7]], [cam_rot[1], cam_rot[2], cam_rot[6], cam_rot[5]]]
    ax.add_collection3d(Poly3DCollection(cam_faces, facecolors='orangered', edgecolors='darkred', alpha=0.95))
    
    lens_rot = to_plot(lens_center.reshape(1, 3))[0]
    ax.scatter(lens_rot[0], lens_rot[1], lens_rot[2], color='cyan', s=60, edgecolors='black', label='Lensa Kamera FPV')

    for p, m in zip(props, motor_pos):
        p_rot = to_plot(p)
        m_rot = to_plot(m.reshape(1, 3))[0]
        ax.plot(p_rot[:, 0], p_rot[:, 1], p_rot[:, 2], color='deepskyblue', linewidth=1.5, alpha=0.7)
        ax.scatter(m_rot[0], m_rot[1], m_rot[2], color='black', s=35)

    axis_len = 2.0
    xb = R @ np.array([axis_len, 0, 0])
    yb = R @ np.array([0, axis_len, 0])
    zb = R @ np.array([0, 0, -axis_len])
    
    ax.quiver(0, 0, 0, xb[0], xb[1], xb[2], color='red', linewidth=2.5, arrow_length_ratio=0.1)
    ax.text(xb[0]*1.1, xb[1]*1.1, xb[2]*1.1, 'Xb', color='darkred', fontweight='bold')

    ax.quiver(0, 0, 0, yb[0], yb[1], yb[2], color='green', linewidth=2.5, arrow_length_ratio=0.1)
    ax.text(yb[0]*1.1, yb[1]*1.1, yb[2]*1.1, 'Yb', color='darkgreen', fontweight='bold')

    ax.quiver(0, 0, 0, zb[0], zb[1], zb[2], color='blue', linewidth=2.5, arrow_length_ratio=0.1)
    ax.text(zb[0]*1.1, zb[1]*1.1, zb[2]*1.1, 'Zb', color='darkblue', fontweight='bold')

    f_total_vec = np.array([0, 0, 2.5])
    ax.quiver(0, 0, 0, f_total_vec[0], f_total_vec[1], f_total_vec[2], color='darkorange', linewidth=3.5, arrow_length_ratio=0.12, label='f = -g (Reaksi)')

    # Perhitungan fisika presisi menggunakan invers Matriks Rotasi (R Transpose)
    f_world_true = np.array([0.0, 0.0, -g_const]) # Reaksi navigasi di FRD world
    f_body_true  = R.T @ f_world_true
    fx, fy, fz   = f_body_true[0], f_body_true[1], f_body_true[2]

    rad_p = np.radians(pitch)
    len_f = 2.5
    sisi_samping_vec = -np.cos(rad_p) * (zb / axis_len) * len_f
    
    ax.plot([0, sisi_samping_vec[0]], [0, sisi_samping_vec[1]], [0, sisi_samping_vec[2]], color='blue', linestyle='--', linewidth=2.5)
    ax.plot([sisi_samping_vec[0], f_total_vec[0]], [sisi_samping_vec[1], f_total_vec[1]], [sisi_samping_vec[2], f_total_vec[2]], color='red', linestyle='--', linewidth=2.5)
    ax.scatter(sisi_samping_vec[0], sisi_samping_vec[1], sisi_samping_vec[2], color='purple', s=40)

    info_text = (
        f"--- HASIL PEMBACAAN IMU ---\n"
        f"Yaw   (ψ) : {yaw:+.1f}°\n"
        f"Pitch (θ) : {pitch:+.1f}°\n"
        f"Roll  (φ) : {roll:+.1f}°\n"
        f"Gravitasi : {g_const:.2f} m/s²\n\n"
        f"1. SUMBU X (Moncong):\n"
        f"   fx = {fx:+.2f} m/s²\n\n"
        f"2. SUMBU Y (Lengan Kanan):\n"
        f"   fy = {fy:+.2f} m/s²\n\n"
        f"3. SUMBU Z (Perut Bawah):\n"
        f"   fz = {fz:+.2f} m/s²\n"
    )
    
    for txt in fig.texts:
        txt.remove()
        
    fig.text(0.74, 0.25, info_text, fontsize=9.5, family='monospace', bbox=dict(boxstyle='round,pad=0.6', facecolor='whitesmoke', edgecolor='gray'))

    ax.legend(loc='upper left', fontsize=8.5)
    fig.canvas.draw_idle()

# ==========================================
# 7. SLIDER INTERAKTIF (Disesuaikan untuk 3 Sumbu)
# ==========================================
ax_yaw   = plt.axes([0.15, 0.13, 0.50, 0.03], facecolor='lightcyan')
ax_pitch = plt.axes([0.15, 0.08, 0.50, 0.03], facecolor='lightgoldenrodyellow')
ax_roll  = plt.axes([0.15, 0.03, 0.50, 0.03], facecolor='lightpink')

slider_yaw   = Slider(ax_yaw,   'Yaw ψ (°)', -180.0, 180.0, valinit=0.0,  valstep=1.0)
slider_pitch = Slider(ax_pitch, 'Pitch θ (°)', -85.0, 85.0, valinit=30.0, valstep=1.0)
slider_roll  = Slider(ax_roll,  'Roll φ (°)',  -85.0, 85.0, valinit=0.0,  valstep=1.0)

slider_yaw.on_changed(update)
slider_pitch.on_changed(update)
slider_roll.on_changed(update)

resetax = plt.axes([0.68, 0.05, 0.05, 0.05])
button_reset = plt.Button(resetax, 'Reset', color='lightgray', hovercolor='0.975')

def reset(event):
    slider_yaw.reset()
    slider_pitch.reset()
    slider_roll.reset()
button_reset.on_clicked(reset)

ax.view_init(elev=20, azim=-60)
update()

plt.show()