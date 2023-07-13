import subprocess
import time
import os
import signal

# Membuat subprocess dan menjalankan main.py
p = subprocess.Popen(["python", "main_module.py"])

# Tunggu selama 10 min
time.sleep(1200)

# Hentikan main.py dengan keyboard interrupt (CTRL+C)
if os.name == "nt":  # Windows
    os.kill(p.pid, signal.CTRL_C_EVENT)
else:  # Unix-based systems
    os.kill(p.pid, signal.SIGINT)