import heapq
from collections import deque
from time import time

# Define an infinite cost value
INFINITY = float('inf')

# Function to calculate the total cost of a given path based on the graph's adjacency matrix
def calculate_path_cost(path, graph):
    cost = sum(graph[path[i]][path[i + 1]] for i in range(len(path) - 1))
    cost += graph[path[-1]][path[0]]  # Adding the cost to return to the start
    return cost

# Function to print the path using city names for better readability
def display_path(path, city_names):
    if not path:
        print("No path available")
    else:
        print(" -> ".join(city_names[node] for node in path))

# Breadth-First Search (BFS) Implementation
def breadth_first_search(graph, city_names, start):
    q = deque([[start]])  # Queue to manage the paths
    min_cost = INFINITY  # Variable to store the minimum cost found
    best_path = []

    while q:
        current_path = q.popleft()

        # If a complete path is found, close the loop and calculate the cost
        if len(current_path) == len(graph):
            current_path.append(start)  # Close the loop
            cost = calculate_path_cost(current_path, graph)
            if cost < min_cost:
                min_cost = cost
                best_path = current_path
        else:
            # Expand the current path to all unvisited cities
            last_node = current_path[-1]
            for next_node in range(len(graph)):
                if next_node not in current_path:
                    new_path = current_path + [next_node]
                    q.append(new_path)

    print("BFS Best Path: ", end="")
    display_path(best_path, city_names)
    print(f"Total cost: {min_cost}")

# Least Cost Search (Uniform Cost Search) Implementation
def least_cost_search(graph, city_names, start):
    pq = []  # Priority queue to manage paths
    heapq.heappush(pq, (0, [start]))  # Initialize with the start city and cost

    min_cost = INFINITY  # Variable to store the minimum cost found
    best_path = []

    while pq:
        current_cost, current_path = heapq.heappop(pq)

        # If a complete path is found, close the loop and calculate the cost
        if len(current_path) == len(graph):
            current_path.append(start)  # Close the loop
            cost = calculate_path_cost(current_path, graph)
            if cost < min_cost:
                min_cost = cost
                best_path = current_path
        else:
            # Expand the current path to all unvisited cities
            last_node = current_path[-1]
            for next_node in range(len(graph)):
                if next_node not in current_path:
                    new_path = current_path + [next_node]
                    new_cost = current_cost + graph[last_node][next_node]
                    heapq.heappush(pq, (new_cost, new_path))

    print("Least-Cost Search Best Path: ", end="")
    display_path(best_path, city_names)
    print(f"Total cost: {min_cost}")

# A* Search Implementation
def a_star_search(graph, city_names, start):
    # Placeholder heuristic function for A* Search; should be replaced with an actual heuristic
    def heuristic(path):
        return 0

    pq = []  # Priority queue to manage paths
    initial_estimate = heuristic([start])
    heapq.heappush(pq, (initial_estimate, 0, [start]))  # Initialize with the start city, cost, and heuristic estimate

    min_cost = INFINITY  # Variable to store the minimum cost found
    best_path = []

    while pq:
        _, current_cost, current_path = heapq.heappop(pq)

        # If a complete path is found, close the loop and calculate the cost
        if len(current_path) == len(graph):
            current_path.append(start)  # Close the loop
            cost = calculate_path_cost(current_path, graph)
            if cost < min_cost:
                min_cost = cost
                best_path = current_path
        else:
            # Expand the current path to all unvisited cities
            last_node = current_path[-1]
            for next_node in range(len(graph)):
                if next_node not in current_path:
                    new_path = current_path + [next_node]
                    new_cost = current_cost + graph[last_node][next_node]
                    new_estimate = new_cost + heuristic(new_path)
                    heapq.heappush(pq, (new_estimate, new_cost, new_path))

    print("A* Best Path: ", end="")
    display_path(best_path, city_names)
    print(f"Total cost: {min_cost}")

# Main function to test the algorithms
if __name__ == "__main__":
    city_names = ["Bucharest", "Sofia", "London", "Vienna", "Paris"]
    graph = [
        [0, 342, 2103, 858, 2317],  # Bucharest
        [342, 0, 2014, 817, 2186],  # Sofia
        [2103, 2014, 0, 1235, 344],  # London
        [858, 817, 1235, 0, 1034],   # Vienna
        [2317, 2186, 344, 1034, 0]   # Paris
    ]

    start_city = 0  # Bucharest

    start_time = time()
    breadth_first_search(graph, city_names, start_city)  # Call BFS
    least_cost_search(graph, city_names, start_city)  # Call Least Cost Search
    a_star_search(graph, city_names, start_city)  # Call A* Search
    end_time = time()

    print(f"\nElapsed time is: {(end_time - start_time) * 1000:.2f} ms")
