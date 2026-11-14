import os
import pandas as pd
from pyulog import ULog

# 1. Tentukan path absolut file ULog Anda
file_ulog = r"E:\Tugas Akhir\New Firmware\Sim\data\dataset.ulg"

print(f"Membuka dan membaca file:\n{file_ulog} ...\n")
ulog = ULog(file_ulog)

# 2. Buat folder untuk menyimpan hasil ekstrak CSV (disimpan di folder data juga)
output_dir = r"E:\Tugas Akhir\New Firmware\Sim\data\output_csv_dataset"
os.makedirs(output_dir, exist_ok=True)

print("=== DAFTAR TOPIK (SINYAL) YANG TERSEDIA DI DALAM ULOG ===")
# 3. Iterasi setiap topik data di dalam ULog
for data in ulog.data_list:
    nama_topik = data.name
    jumlah_baris = len(data.data['timestamp'])
    
    print(f"- {nama_topik} ({jumlah_baris} baris data)")
    
    # 4. Konversi data dictionary pyulog menjadi Pandas DataFrame
    df = pd.DataFrame(data.data)
    
    # 5. Simpan DataFrame ke dalam bentuk CSV
    nama_file_csv = f"{nama_topik}.csv"
    path_simpan = os.path.join(output_dir, nama_file_csv)
    
    df.to_csv(path_simpan, index=False)

print(f"\n[SELESAI] Semua topik berhasil diekstrak menjadi file .csv di dalam folder:\n'{output_dir}'")
print("Silakan periksa folder tersebut, ambil 3 dataset yang Anda butuhkan, dan hapus sisanya.")