import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from sklearn.cluster import KMeans
import time

# kroA100.tsp 데이터 로드 및 파싱
def load_kroa100(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        cities = []
        for line in lines:
            if line[0].isdigit():
                _, x, y = line.strip().split()
                cities.append((float(x), float(y)))
    return np.array(cities)

# K-means 클러스터링 (출발 지점은 클러스터링에 포함하지 않음)
def kmeans_clustering(cities, k):
    kmeans = KMeans(n_clusters=k, random_state=65)
    kmeans.fit(cities)
    centers = kmeans.cluster_centers_
    labels = kmeans.labels_
    return labels, centers

# 각 클러스터에 대해 TSP 해결
def solve_tsp_in_clusters_with_depot(cities, labels, k, depot):
    tsp_paths = []
    total_distance = 0
    for i in range(k):
        # 클러스터에 출발 지점 추가
        cluster = np.vstack([depot, cities[labels == i]])
        G = nx.complete_graph(len(cluster))
        for i in range(len(cluster)):
            for j in range(i + 1, len(cluster)):
                dist = np.linalg.norm(cluster[i] - cluster[j])
                G.add_edge(i, j, weight=dist)
        tsp_path = nx.approximation.traveling_salesman_problem(G, cycle=True)
        tsp_paths.append(cluster[tsp_path])
        total_distance += calculate_path_distance(cluster[tsp_path])
    return tsp_paths, total_distance

def calculate_path_distance(path):
    distance = 0
    for i in range(len(path) - 1):
        distance += np.linalg.norm(path[i] - path[i + 1])
    distance += np.linalg.norm(path[-1] - path[0])  # 원점 복귀 거리 추가
    print(f"1st task :{path[1]}")
    return distance

def evaluate_performance_with_depot(cities, k, depot):
    # K-means 클러스터링 및 TSP 해결
    start_time = time.time()
    labels, centers = kmeans_clustering(cities, k)
    tsp_paths, total_distance = solve_tsp_in_clusters_with_depot(cities, labels, k, depot)
    computation_time = time.time() - start_time

    # 결과 출력
    print(f"총 경로 거리: {total_distance:.2f}")
    print(f"계산 시간: {computation_time:.2f} 초")

    # 결과 시각화
    plot_clusters_with_paths(cities, labels, centers, tsp_paths, depot)

    return total_distance, computation_time

# 클러스터 경로 시각화
def plot_clusters_with_paths(cities, labels, centers, tsp_paths, depot):
    plt.figure(figsize=(10, 8))
    plt.scatter(cities[:, 0], cities[:, 1], c=labels, cmap='viridis', marker='o')
    plt.scatter(centers[:, 0], centers[:, 1], c='red', marker='x', s=10, label='Cluster Centers')
    plt.scatter(depot[0], depot[1], c='blue', marker='D', s=150, label='Depot')  # 출발 지점 표시
    
    for path in tsp_paths:
        plt.plot(path[:, 0], path[:, 1], marker='o')
    plt.title("K-means Clustering with TSP Paths Including Depot")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.show()

# 데이터 로드
cities = load_kroa100('kroA100.tsp')

# 출발 지점 설정 (예: (0,0))
depot = np.array([2000.0, 0.0])

# K-means 클러스터링 수행
k = 3  # 클러스터 수 설정
total_distance, computation_time = evaluate_performance_with_depot(cities, k, depot)
