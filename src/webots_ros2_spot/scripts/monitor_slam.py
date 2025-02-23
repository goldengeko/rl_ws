import subprocess
import time

def start_slam():
    slam_process = subprocess.Popen(["ros2", "launch", "webots_spot", "slam_launch.py"])
    return slam_process

def monitor_logs(slam_process):
    while True:
        output = slam_process.stdout.read(1024).decode()
        if "TF_OLD_DATA" in output:
            print("Detected TF error. Restarting SLAM...")
            slam_process.terminate()  # Terminate the current slam process
            slam_process.wait()  # Wait for it to finish
            start_slam()  # Restart the slam process
        time.sleep(1)

if __name__ == "__main__":
    slam_process = start_slam()
    monitor_logs(slam_process)
