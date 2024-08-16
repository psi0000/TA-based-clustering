# import numpy as np
# import matplotlib.pyplot as plt
# import networkx as nx
# from fcmeans import FCM
# import time

# # kroA100.tsp 데이터 로드 및 파싱
# def load_kroa100(file_path):
#     with open(file_path, 'r') as f:
#         lines = f.readlines()
#         cities = []
#         for line in lines:
#             if line[0].isdigit():
#                 _, x, y = line.strip().split()
#                 cities.append((float(x), float(y)))
#     return np.array(cities)

# # Fuzzy C-means 클러스터링
# def fuzzy_c_means_clustering(cities, k):
#     fcm = FCM(n_clusters=k, random_state=65, max_iter=150,m=2,error=1e-5)
#     fcm.fit(cities)
#     centers = fcm.centers
#     u_matrix = fcm.u  # 멤버십 행렬
#     labels = u_matrix.argmax(axis=1)  # 가장 높은 멤버십 값을 갖는 클러스터 라벨
#     return labels, u_matrix, centers

# # 각 클러스터에 대해 TSP 해결
# def solve_tsp_in_clusters(cities, labels, k):
#     clusters = [cities[labels == i] for i in range(k)]
#     tsp_paths = []
#     total_distance = 0
#     for cluster in clusters:
#         G = nx.complete_graph(len(cluster))
#         for i in range(len(cluster)):
#             for j in range(i + 1, len(cluster)):
#                 dist = np.linalg.norm(cluster[i] - cluster[j])
#                 G.add_edge(i, j, weight=dist)
#         tsp_path = nx.approximation.traveling_salesman_problem(G, cycle=True)
#         tsp_paths.append(cluster[tsp_path])
#         total_distance += calculate_path_distance(cluster[tsp_path])
#     return tsp_paths, total_distance

# def calculate_path_distance(path):
#     distance = 0
#     for i in range(len(path) - 1):
#         distance += np.linalg.norm(path[i] - path[i + 1])
#     distance += np.linalg.norm(path[-1] - path[0])  # 원점 복귀 거리 추가
#     return distance

# def evaluate_performance_fuzzy(cities, k):
#     # Fuzzy C-means 클러스터링 및 TSP 해결
#     start_time = time.time()
#     labels, u_matrix, centers = fuzzy_c_means_clustering(cities, k)
#     clustering_time = time.time() - start_time
#     tsp_paths, total_distance = solve_tsp_in_clusters(cities, labels, k)
#     tsp_time = time.time() - clustering_time -start_time
#     computation_time = tsp_time + clustering_time
#     # 결과 출력
#     print(f"총 경로 거리: {total_distance:.2f}")
#     print(f"clustering 계산 시간: {clustering_time:.3f} 초")
#     print(f"tsp 계산 시간: {tsp_time:.3f} 초")
#     print(f"총 계산 시간: {computation_time:.3f} 초")
#     # 멤버십 및 TSP 경로 시각화
#     plot_membership_and_paths(cities, u_matrix, centers, tsp_paths, labels)

#     return total_distance, computation_time

# # 멤버십 및 TSP 경로 시각화
# def plot_membership_and_paths(cities, u_matrix, centers, tsp_paths, labels):
#     plt.figure(figsize=(10, 8))
    
#     # 클러스터 색상 정의
#     colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    
#     # 각 클러스터에 대한 멤버십 값 시각화
#     for j in range(u_matrix.shape[1]):
#         sizes = u_matrix[:, j] * 1000  # 멤버십 값을 기반으로 원 크기 조절
#         plt.scatter(cities[:, 0], cities[:, 1], s=sizes, color=colors[j % len(colors)], alpha=0.5, label=f'Cluster {j+1} Membership')
    
#     # 클러스터 중심 시각화
#     plt.scatter(centers[:, 0], centers[:, 1], c='black', marker='x', s=50, label='Cluster Centers')
    
#     # TSP 경로 시각화 (각 클러스터마다 다른 색상 사용)
#     for i, path in enumerate(tsp_paths):
#         cluster_color = colors[i % len(colors)]  # 클러스터별 색상 선택
#         plt.plot(path[:, 0], path[:, 1], color=cluster_color, marker='o', linestyle='-', label=f'Cluster {i+1} TSP Path')
    
#     plt.title('Fuzzy C-Means Clustering with TSP Paths')
#     plt.xlabel('X Coordinate')
#     plt.ylabel('Y Coordinate')
    
#     # 범례를 그래프 밖으로 이동
#     plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
#     plt.tight_layout()  # 레이아웃을 자동으로 조정
    
#     plt.show()

# # 데이터 로드
# cities = load_kroa100('kroA200.tsp')

# # 기존 Fuzzy C-means 클러스터링 수행
# k = 3  # 클러스터 수 설정
# print("기존 데이터에 대한 성능 평가:")
# total_distance, computation_time = evaluate_performance_fuzzy(cities, k)

# # 새로운 task 데이터 추가
# def add_new_tasks(cities, new_tasks):
#     # 새로운 task 데이터를 기존 데이터에 추가
#     updated_cities = np.vstack((cities, new_tasks))
#     return updated_cities

# # 예시로 새로운 task 데이터 생성
# new_tasks = np.array([
#     [1500.0, 1600.0],  # 새로운 도시 1
#     [1400.0, 1550.0],   # 새로운 도시 2
#     [2000.0, 250.0]   # 새로운 도시 3
# ])

# # 새로운 데이터로 Fuzzy C-means 클러스터링 및 성능 평가
# updated_cities = add_new_tasks(cities, new_tasks)
# print("새로운 데이터 추가 후 성능 평가:")
# total_distance, computation_time = evaluate_performance_fuzzy(updated_cities, k)
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from fcmeans import FCM
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

# Fuzzy C-means 클러스터링
def fuzzy_c_means_clustering(cities, k):
    fcm = FCM(n_clusters=k, random_state=65, max_iter=150, m=2, error=1e-5)
    fcm.fit(cities)
    centers = fcm.centers
    u_matrix = fcm.u  # 멤버십 행렬
    labels = u_matrix.argmax(axis=1)  # 가장 높은 멤버십 값을 갖는 클러스터 라벨
    return labels, u_matrix, centers

# 각 클러스터에 대해 TSP 해결
def solve_tsp_in_clusters(cities, labels, k):
    clusters = [cities[labels == i] for i in range(k)]
    tsp_paths = []
    total_distance = 0
    for cluster in clusters:
        G = nx.complete_graph(len(cluster))
        pos = {i: cluster[i] for i in range(len(cluster))}
        nx.set_node_attributes(G, pos, 'pos')
        
        # 각 엣지의 가중치로 유클리드 거리 설정
        for i in range(len(cluster)):
            for j in range(i + 1, len(cluster)):
                dist = np.linalg.norm(cluster[i] - cluster[j])
                G.add_edge(i, j, weight=dist)
        
        tsp_path = nx.approximation.traveling_salesman_problem(G, cycle=True)
        ordered_path = [cluster[node] for node in tsp_path]
        tsp_paths.append(np.array(ordered_path))
        total_distance += calculate_path_distance(ordered_path)
    return tsp_paths, total_distance

def calculate_path_distance(path):
    distance = 0
    for i in range(len(path) - 1):
        distance += np.linalg.norm(path[i] - path[i + 1])
    distance += np.linalg.norm(path[-1] - path[0])  # 원점 복귀 거리 추가
    return distance

def evaluate_performance_fuzzy(cities, k):
    # Fuzzy C-means 클러스터링 및 TSP 해결
    start_time = time.time()
    labels, u_matrix, centers = fuzzy_c_means_clustering(cities, k)
    clustering_time = time.time() - start_time
    tsp_paths, total_distance = solve_tsp_in_clusters(cities, labels, k)
    tsp_time = time.time() - clustering_time - start_time
    computation_time = tsp_time + clustering_time
    # 결과 출력
    print(f"총 경로 거리: {total_distance:.2f}")
    print(f"clustering 계산 시간: {clustering_time:.3f} 초")
    print(f"tsp 계산 시간: {tsp_time:.3f} 초")
    print(f"총 계산 시간: {computation_time:.3f} 초")
    # 멤버십 및 TSP 경로 시각화
    plot_membership_and_paths(cities, u_matrix, centers, tsp_paths, labels)

    return total_distance, computation_time

# 멤버십 및 TSP 경로 시각화
def plot_membership_and_paths(cities, u_matrix, centers, tsp_paths, labels):
    plt.figure(figsize=(10, 8))
    
    # 클러스터 색상 정의
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    
    # 각 클러스터에 대한 멤버십 값 시각화
    for j in range(u_matrix.shape[1]):
        sizes = u_matrix[:, j] * 1000  # 멤버십 값을 기반으로 원 크기 조절
        plt.scatter(cities[:, 0], cities[:, 1], s=sizes, color=colors[j % len(colors)], alpha=0.5, label=f'Cluster {j+1} Membership')
    
    # 클러스터 중심 시각화
    plt.scatter(centers[:, 0], centers[:, 1], c='black', marker='x', s=50, label='Cluster Centers')
    
    # TSP 경로 시각화 (각 클러스터마다 다른 색상 사용)
    for i, path in enumerate(tsp_paths):
        cluster_color = colors[i % len(colors)]  # 클러스터별 색상 선택
        plt.plot(path[:, 0], path[:, 1], color=cluster_color, marker='o', linestyle='-', label=f'Cluster {i+1} TSP Path')
        plt.scatter(path[0, 0], path[0, 1], c=cluster_color, s=100, edgecolors='k', label=f'Start of Cluster {i+1}')

    plt.title('Fuzzy C-Means Clustering with TSP Paths')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    
    # 범례를 그래프 밖으로 이동
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()  # 레이아웃을 자동으로 조정
    
    plt.show()

# 기존 데이터 로드 및 성능 평가
cities = load_kroa100('kroA100.tsp')

k = 3  # 클러스터 수 설정
print("기존 데이터에 대한 성능 평가:")
total_distance, computation_time = evaluate_performance_fuzzy(cities, k)

# 새로운 task 데이터 추가
def add_new_tasks(cities, new_tasks):
    # 새로운 task 데이터를 기존 데이터에 추가
    updated_cities = np.vstack((cities, new_tasks))
    return updated_cities

# 예시로 새로운 task 데이터 생성
new_tasks = np.array([
    [1500.0, 1600.0],  # 새로운 도시 1
    [1400.0, 1550.0], # 새로운 도시 2
    [2000.0, 250.0]   # 새로운 도시 3
])

# 새로운 데이터로 Fuzzy C-means 클러스터링 및 성능 평가
updated_cities = add_new_tasks(cities, new_tasks)
print("새로운 데이터 추가 후 성능 평가:")
total_distance, computation_time = evaluate_performance_fuzzy(updated_cities, k)
