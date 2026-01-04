import sys, importlib, argparse
from Racecar import *
from Track import *


def run(track, car, agent, visualizer, display=True):
    if display: print('Starting simulation')
    iteration = 0
    obs, reward, done = car.step('coast')
    totalReward = reward
    maxSpeed = 0
    if visualizer:
        visualizer.update(car, track.getLidar((car._x, car._y), car._d),
                          totalReward, ('straight', 'coast'))
    while not done and iteration < 1000:
        iteration += 1
        action = agent.chooseAction(obs, car.actions())
        obs, reward, done = car.step(action)
        totalReward += reward
        maxSpeed = max(maxSpeed, obs['velocity'])
        if visualizer:
            visualizer.update(car, track.getLidar((car._x, car._y), car._d),
                              totalReward, action)
        lidar = [f'{x:.2}' for x in obs['lidar']]
        lidar = ', '.join(lidar)
        if display: print(
            f'Iteration {iteration}.  State is {car}.  Distance to center: {track.distance((car._x, car._y)):.2f}, Distance from start {track.distanceTravelled(car._x, car._y):.2f}, Lidar {lidar}, Velocity {obs["velocity"]:.2f}, Choosing action {action}, reward {reward:.2f} and totalReward {totalReward:.2f}.')

    if visualizer:
        visualizer.update(car, track.getLidar((car._x, car._y), car._d),
                          totalReward, action)

    if display:
        print(f'Score {totalReward:.3f}')

    return totalReward


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulate the Racecar.')
    parser.add_argument('agent_file', type=str)
    parser.add_argument('track_number', type=int)
    parser.add_argument('-g', type=int, help="size of graphics window")
    parser.add_argument('-t', type=float, help="time delay between steps")
    parser.add_argument('-d', type=str, help="data file to load")
    args = parser.parse_args()

    try:
        moduleName = args.agent_file.split('.')[0]
        agentModule = importlib.import_module(moduleName)
    except ModuleNotFoundError as e:
        print(f"Error: Module '{moduleName}' not found. Details: {e}")
        sys.exit()
    except Exception as e:
        print(f"An unexpected error occurred during import: {e}")
        import traceback

        traceback.print_exc()
        sys.exit()

    track = buildTrack(args.track_number)

    car = Racecar(track)
    agent = agentModule.Agent()

    if args.d:
        agent.load(args.d)

    if args.g:
        from Visualize import *

        visualizer = TrackVisualization(track, args.g)
        if args.t:
            visualizer.setDelay(args.t)
    else:
        visualizer = None

    totalReward = run(track, car, agent, visualizer)

    print('Final score =', totalReward)
