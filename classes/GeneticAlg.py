import random
import numpy as np

# Population Initialization

DIRECTIONS = ["UP", "DOWN", "LEFT", "RIGHT"]

def random_path(length=30):
    return [random.choice(DIRECTIONS) for _ in range(length)]

# Fitness Function
def fitness(path, start, goal, grid):
    x, y = start
    #print(x, y)
    visited = set()
    penalty = 0
    reward = 0
    for move in path:
        if move == "UP":
            y -= 1
        elif move == "DOWN":
            y += 1
        elif move == "LEFT":
            x -= 1
        elif move == "RIGHT":
            x += 1

        # Out of bounds
        if not (0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]):
            penalty += 10000
            break

        # Hit obstacle
        if grid[x, y] > 0.6:
            penalty += 10000
            break
        # Penalize revisiting same location
        if (x, y) in visited:
            penalty += 5
        visited.add((x, y))

        # Reward progress towards goal
        dist_to_goal = abs(goal[0] - x) + abs(goal[1] - y)
        reward -= dist_to_goal*dist_to_goal * 0.005  # scaled down

    # Final distance to goal matters
    final_dist = abs(goal[0] - x) + abs(goal[1] - y)
    if final_dist <= 4:
        reward += 1000  # big reward for reaching goal

    return reward - penalty


# Selection
def select_parents(population, scores):
    sorted_pop = [p for _, p in sorted(zip(scores, population), reverse=True)]
    return sorted_pop[:2]  # best two

# Crossover
def crossover(p1, p2):
    point = random.randint(1, len(p1) - 2)
    return p1[:point] + p2[point:]

# Mutation
def mutate(path, mutation_rate=0.1):
    for i in range(len(path)):
        if random.random() < mutation_rate:
            path[i] = random.choice(DIRECTIONS)

def run_genetic_algorithm(start, goal, grid, generations=100, pop_size=100, path_len=50):
    population = [random_path(path_len) for _ in range(pop_size)]

    for gen in range(generations):
        scores = [fitness(ind, start, goal, grid) for ind in population]
        best = max(scores)
        # print(f"Generation {gen}, best fitness: {best}")
        if best >= -1: break  # found solution

        new_population = []
        for _ in range(pop_size):
            p1, p2 = select_parents(population, scores)
            child = crossover(p1, p2)
            mutate(child)
            new_population.append(child)

        population = new_population

    best_idx = np.argmax(scores)
    return population[best_idx]


def simulate_path(path, start):
    x, y = start
    points = [(x, y)]
    for move in path:
        print(move)
        if move == "UP": y -= 1
        elif move == "DOWN": y += 1
        elif move == "LEFT": x -= 1
        elif move == "RIGHT": x += 1
        points.append((x, y))
    return points

