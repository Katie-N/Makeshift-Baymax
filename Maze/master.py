# main.py
from multiprocessing import Process, Manager
import detectWalls
import mapGrid

def start_detect_walls(shared_wall_distances):
    detectWalls.main(shared_wall_distances)

def start_map_grid(shared_wall_distances):
    mapGrid.main(shared_wall_distances)

if __name__ == '__main__':
    from multiprocessing import set_start_method
    set_start_method('spawn')  # Required for some platforms like macOS or Windows

    with Manager() as manager:
        shared_wall_distances = manager.list([0.0, 0.0, 0.0, 0.0])

        p1 = Process(target=start_detect_walls, args=(shared_wall_distances,))
        p2 = Process(target=start_map_grid, args=(shared_wall_distances,))

        p1.start()
        p2.start()

        p1.join()
        p2.join()
