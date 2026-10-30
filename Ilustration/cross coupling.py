import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output
import dash_bootstrap_components as dbc

# Inisialisasi Aplikasi Dash untuk UI Interaktif
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

def create_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Membuat matriks rotasi 3D dari sudut Euler (dalam derajat)"""
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    
    # Rotasi X (Roll)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    
    # Rotasi Y (Pitch)
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    
    # Rotasi Z (Yaw)
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def create_arrow(start, end, color, name, is_ideal=False):
    """Fungsi pembantu untuk membuat garis panah 3D di Plotly"""
    dash_style = 'solid' if not is_ideal else 'dot'
    line_width = 5 if not is_ideal else 3
    
    return go.Scatter3d(
        x=[start[0], end[0]],
        y=[start[1], end[1]],
        z=[start[2], end[2]],
        mode='lines+text',
        text=['', name],
        textposition='top center',
        line=dict(color=color, width=line_width, dash=dash_style),
        name=name
    )

# Tata Letak UI (Dashboard)
app.layout = dbc.Container([
    html.H2("Visualisasi 3D Interaktif: Cross-Coupling Errors (Teori Groves)", className="mt-4 mb-3 text-center"),
    
    dbc.Row([
        # Kolom Kiri: Slider dan Matriks
        dbc.Col([
            html.H5("Atur Kemiringan Sensor (Axis Misalignment)"),
            html.Label("Kemiringan Pitch (Sudut Misalignment X-Z) [Derajat]:"),
            dcc.Slider(id='pitch-slider', min=-15, max=15, step=0.1, value=5.0, marks={i: f'{i}°' for i in range(-15, 16, 5)}),
            
            html.Label("Kemiringan Roll (Sudut Misalignment Y-Z) [Derajat]:", className="mt-3"),
            dcc.Slider(id='roll-slider', min=-15, max=15, step=0.1, value=0.0, marks={i: f'{i}°' for i in range(-15, 16, 5)}),
            
            html.Label("Kemiringan Yaw (Sudut Misalignment X-Y) [Derajat]:", className="mt-3"),
            dcc.Slider(id='yaw-slider', min=-15, max=15, step=0.1, value=0.0, marks={i: f'{i}°' for i in range(-15, 16, 5)}),
            
            html.Hr(),
            html.H5("Matriks Cross-Coupling Giroskop (M_g)"),
            html.P("Menggunakan aproksimasi sudut kecil: m ≈ sin(δθ) ≈ δθ rad"),
            html.Pre(id='matrix-output', style={'backgroundColor': '#f8f9fa', 'padding': '15px', 'border': '1px solid #dee2e6'}),
            
            html.Hr(),
            html.H5("Simulasi Kebocoran Sinyal (Input Rotasi Murni)"),
            html.P("Jika Drone dipaksa berputar murni di sumbu X sebesar 10 rad/s (Input Ideal):"),
            html.Pre(id='leakage-output', style={'backgroundColor': '#e8f4f8', 'padding': '15px', 'border': '1px solid #b8daff', 'fontWeight': 'bold'}),
            
        ], width=4),
        
        # Kolom Kanan: Plot 3D
        dbc.Col([
            dcc.Graph(id='3d-plot', style={'height': '70vh'})
        ], width=8)
    ])
], fluid=True)

# Callback untuk memperbarui Plot dan Matriks secara Real-Time
@app.callback(
    [Output('3d-plot', 'figure'),
     Output('matrix-output', 'children'),
     Output('leakage-output', 'children')],
    [Input('roll-slider', 'value'),
     Input('pitch-slider', 'value'),
     Input('yaw-slider', 'value')]
)
def update_simulation(roll, pitch, yaw):
    # 1. Menghitung Vektor Sumbu
    origin = np.array([0, 0, 0])
    
    # Sumbu Ideal (Body Frame)
    ideal_x = np.array([1, 0, 0])
    ideal_y = np.array([0, 1, 0])
    ideal_z = np.array([0, 0, 1])
    
    # Matriks Rotasi akibat Misalignment
    R = create_rotation_matrix(roll, pitch, yaw)
    
    # Sumbu Sensor Aktual yang Miring
    sensor_x = R @ ideal_x
    sensor_y = R @ ideal_y
    sensor_z = R @ ideal_z
    
    # 2. Menggambar Plot 3D
    fig = go.Figure()
    
    # Gambar Sumbu Ideal (Titik-titik hitam)
    fig.add_trace(create_arrow(origin, ideal_x, 'black', 'X (Ideal)', True))
    fig.add_trace(create_arrow(origin, ideal_y, 'black', 'Y (Ideal)', True))
    fig.add_trace(create_arrow(origin, ideal_z, 'black', 'Z (Ideal)', True))
    
    # Gambar Sumbu Sensor Aktual
    fig.add_trace(create_arrow(origin, sensor_x, 'red', 'X_sensor (Actual)'))
    fig.add_trace(create_arrow(origin, sensor_y, 'green', 'Y_sensor (Actual)'))
    fig.add_trace(create_arrow(origin, sensor_z, 'blue', 'Z_sensor (Actual)'))
    
    # Pengaturan Tampilan 3D
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-1.2, 1.2], title='Sumbu X'),
            yaxis=dict(range=[-1.2, 1.2], title='Sumbu Y'),
            zaxis=dict(range=[-1.2, 1.2], title='Sumbu Z'),
            aspectmode='cube'
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01)
    )
    
    # 3. Menghitung Matriks Cross-Coupling (Groves: m_ij ≈ sin(delta_theta))
    # Matriks M_g secara ideal merepresentasikan matriks rotasi dikurangi identitas
    # (Untuk skala faktor kita asumsikan 0 karena kita fokus ke kopling silang)
    M_g = R - np.eye(3)
    
    matrix_text = f"""M_g = 
[  0.0000   {M_g[0,1]:7.4f}   {M_g[0,2]:7.4f} ]  <- (X_sens reading)
[ {M_g[1,0]:7.4f}    0.0000   {M_g[1,2]:7.4f} ]  <- (Y_sens reading)
[ {M_g[2,0]:7.4f}   {M_g[2,1]:7.4f}    0.0000 ]  <- (Z_sens reading)
"""

    # 4. Simulasi Kebocoran Sinyal
    input_rotation = np.array([10.0, 0.0, 0.0]) # Rotasi Murni 10 rad/s di Sumbu X
    
    # Pembacaan sensor aktual adalah proyeksi dari input rotasi
    # (Menggunakan matriks transformasi (I_3 + M_g) )
    sensor_reading = (np.eye(3) + M_g) @ input_rotation
    
    leakage_text = f"""Sinyal Masuk Aktual : [10.00,  0.00,  0.00] rad/s
Sinyal Terbaca Sensor:
- Sumbu X: {sensor_reading[0]:.4f} rad/s (Valid)
- Sumbu Y: {sensor_reading[1]:.4f} rad/s (BOCOR / Cross-Coupling!)
- Sumbu Z: {sensor_reading[2]:.4f} rad/s (BOCOR / Cross-Coupling!)
"""
    
    return fig, matrix_text, leakage_text

if __name__ == '__main__':
    # Jalankan server lokal
    print("Membuka Simulasi... Buka http://127.0.0.1:8050/ di browser Anda.")
    app.run(debug=True)