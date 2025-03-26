import subprocess

#list of scripts to run
scripts = ["detectOpponent.py", "attackOpponent.py", "defense.py", "detectWalls.py"]

#start each script as a separate process
processes = [subprocess.Popen(["python3", script]) for script in scripts]

#wait for all processes to complete
try:
    for process in processes:
        process.wait()
except KeyboardInterrupt:
    print("Stopping all processes...")
    for process in processes:
        process.terminate()
