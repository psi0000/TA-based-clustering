clc
clear


% kroA100.tsp 데이터를 로드합니다.
fileID = fopen('kroA200.tsp', 'r');
data = [];

% 파일에서 좌표 데이터를 추출
while ~feof(fileID)
    line = fgetl(fileID);
    % 'NODE_COORD_SECTION' 이후의 좌표 데이터를 추출
    if contains(line, 'NODE_COORD_SECTION')
        break;
    end
end

% 좌표 데이터 읽기
while ~feof(fileID)
    line = fgetl(fileID);
    if contains(line, 'EOF')
        break;
    end
    nums = sscanf(line, '%d %f %f');
    data = [data; nums(2:3)'];
end

fclose(fileID);
%parfeval 호출 시 초기화 
pool = gcp('nocreate'); % 이미 열려있는 병렬 풀이 있는지 확인
if isempty(pool)
    parpool; % 병렬 풀 열기
end

% FCM 클러스터링을 수행합니다. (클러스터 개수 = 3)
tic;
num_clusters = 3;
[centers, U] = fcm(data, num_clusters);
% 클러스터 할당
[~, cluster_labels] = max(U);

times=toc;
disp(['initial FCM : ', num2str(times), ' 초']);

% 로봇의 초기 위치 설정 (하단 중앙, 검정색 X 위치)
robot_start = [mean(data(:, 1)), min(data(:, 2)) - 10];

% 클러스터링 결과에 따라 각 클러스터의 점을 분리
cluster_points = cell(num_clusters, 1);
for i = 1:num_clusters
    cluster_points{i} = data(cluster_labels == i, :);
end


% 클러스터 내 최적 경로 계산 (ACO 사용)
optimal_paths = cell(num_clusters, 1);
cluster_distances = zeros(num_clusters, 1); % 클러스터 내 최적 경로 거리

for i = 1:num_clusters
    cluster_size = size(cluster_points{i}, 1);
    if cluster_size > 1
        % ACO 알고리즘을 적용하여 최적 경로를 찾음
        [optimal_path, opt_distance] = tspACO(cluster_points{i}, robot_start);
        optimal_paths{i} = optimal_path;
        
        % 클러스터 내 거리 저장
        cluster_distances(i) = opt_distance;
    else
        optimal_paths{i} = cluster_points{i};  % 하나의 점만 있을 경우 최적화할 필요 없음
        cluster_distances(i) = 0;  % 하나의 점만 있는 경우, 거리는 0
    end
end

% 총 거리를 계산
total_distance = sum(cluster_distances);

% 각 클러스터 내 거리 출력
for i = 1:num_clusters
    disp(['클러스터 ', num2str(i), ': ', num2str(cluster_distances(i))]);
end
disp(['전체 경로 최적화 후 총 거리: ', num2str(total_distance)]);

% 최적 경로 시각화 (점선으로 표시) 및 확률 기반 시각적 표현
figure;
subplot(2,2,1); % 왼쪽에 기존 경로 표시
title('initial FCM path planning');
hold on;
colors = {'b', 'r', 'g'}; % 클러스터 별 색상: 파랑, 빨강, 초록

for i = 1:num_clusters
    for j = 1:size(data, 1)
        % 각 포인트의 확률에 따라 크기 및 불투명도 설정
        point_size = 500 * U(i, j);  % 확률에 따라 점 크기 조정
        alpha = max(0.6, U(i, j));  % 확률에 따라 불투명도 설정
        scatter(data(j,1), data(j,2), point_size, 'MarkerFaceColor', colors{i}, ...
                'MarkerEdgeColor', colors{i}, 'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);
    end
    
    path = [optimal_paths{i}; optimal_paths{i}(1, :)]; % 순환 경로로 만듦
    plot(path(:, 1), path(:, 2), '--', 'Color', colors{i}); % 점선으로 경로 표시
end

% 로봇의 초기 위치 표시
plot(robot_start(1), robot_start(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);

% 축 설정
xlim([min(data(:, 1)) - 200, max(data(:, 1)) + 200]);
ylim([min(data(:, 2)) - 200, max(data(:, 2)) + 200]);

% 로봇의 이동 시뮬레이션
robot_pos = repmat(robot_start, num_clusters, 1);  % 각 로봇의 현재 위치
robot_speed = 10;  % 각 로봇의 속도 (고정된 속도)
time_step = 0.5;  % 시간 스텝
robot_paths = cell(num_clusters, 1); 
robot_markers = gobjects(num_clusters, 1); % 로봇 마커 핸들 초기화
current_target_index = ones(num_clusters, 1);  % 각 로봇의 현재 목표점 인덱스 초기화

% 로봇 초기 위치 설정
for i = 1:num_clusters
    robot_paths{i} = robot_start; % 초기 위치 저장
    robot_markers(i) = plot(robot_pos(i, 1), robot_pos(i, 2), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', colors{i});
end

add_new_data_time = 200;  % 새로운 데이터를 추가할 iteration 수
new_data_added = false;    % 새로운 데이터 추가 여부 플래그

% 모든 클러스터의 task를 완료할 때까지 반복
all_tasks_completed = false;
iteration_count = 0;

while ~all_tasks_completed
    iteration_count = iteration_count + 1;
    all_tasks_completed = true;  % 모든 작업이 완료되었는지 체크

    % 특정 iteration에 새로운 데이터를 추가
    if iteration_count == add_new_data_time && ~new_data_added
        new_data = [randi([1500, 2500], 3, 1), randi([200, 600], 3, 1)];
        scatter(new_data(:, 1), new_data(:, 2), 50, 'k', '^');  % 새로운 데이터 세모로 표시
        new_data_added = true;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % 방문 완료된 점을 포함 + 새로운 데이터로 클러스터링 수행
        global_data = [data; new_data];  % 남은 데이터와 새로운 데이터를 결합
        tic;
        % 새로운 클러스터링 작업을 비동기적으로 수행
        %f = parfeval(@performClustering, 2, global_data, num_clusters);
        %wait(f);
        [new_U, new_cluster_labels] = performClustering(global_data, num_clusters);
        %[new_U, new_cluster_labels] = fetchOutputs(f); % 결과 가져오기 
        times=toc;
        disp(['FCM including completed task : ', num2str(times), ' 초']);
        % 기존 클러스터 중심과 새로운 클러스터 중심 간의 매칭
        global_centers = calculateCenters(global_data, new_cluster_labels, num_clusters);
        center_match = matchClusters(centers, global_centers);

        %시각화 (3번째 subplot)
        disp('new clustering including completed task');
        subplot(2, 2, 3);
        hold on;
        title('new clustering including completed task');
        % 전체 데이터를 클러스터링한 결과를 표시합니다.
        for i = 1:num_clusters
            color_idx = center_match(i);  % 매칭된 색상 인덱스
            for j = 1:size(global_data, 1)
                point_size = 500 * new_U(i, j);
                alpha = max(0.6, new_U(i, j));
        
                % 새로운 데이터를 삼각형으로 표시
                if any(all(global_data(j, :) == new_data, 2))
                    scatter(global_data(j, 1), global_data(j, 2), point_size, ...
                        'MarkerFaceColor', colors{color_idx}, 'MarkerEdgeColor', colors{color_idx}, ...
                        'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha, ...
                        'Marker', '^');
                else
                    % 기존 데이터를 원형으로 표시
                    scatter(global_data(j, 1), global_data(j, 2), point_size, ...
                        'MarkerFaceColor', colors{color_idx}, 'MarkerEdgeColor', colors{color_idx}, ...
                        'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);
                end
            end
        end
        
        % 완료된 task를 클러스터링 결과에 포함시켜 검정색 X로 표시합니다.
        for i = 1:num_clusters
            completed_idx = ismember(global_data, optimal_paths{i}(1:current_target_index(i)-1, :), 'rows');
            % 기존 클러스터 색상에 X를 추가하여 표시
            scatter(global_data(completed_idx, 1), global_data(completed_idx, 2), 300, 'kx', 'LineWidth', 2);
        end
        
        % 로봇의 초기 위치 표시
        plot(robot_start(1), robot_start(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
        
        xlim([min(data(:, 1)) - 200, max(data(:, 1)) + 200]);
        ylim([min(data(:, 2)) - 200, max(data(:, 2)) + 200]);
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % 방문 완료된 점과 방문 중인 점을 제외한 나머지 점 + 새로운 데이터로 클러스터링 수행
        remaining_data = [];
        for i = 1:num_clusters
            if current_target_index(i) <= size(optimal_paths{i}, 1)
                remaining_data = [remaining_data; optimal_paths{i}(current_target_index(i):end, :)];
            end
        end
        merge_data = [remaining_data; new_data];  % 남은 데이터와 새로운 데이터를 결합
        tic;
        % 새로운 클러스터링 작업을 비동기적으로 수행
        %f = parfeval(@performClustering, 2, merge_data, num_clusters);
        %wait(f);
        
        %[new_U, new_cluster_labels] = fetchOutputs(f); % 결과 가져오기 
        [new_U, new_cluster_labels] = performClustering(merge_data, num_clusters);        
        times=toc;
        disp(['FCM excluding completed task : ', num2str(times), ' 초']);
        % 기존 클러스터 중심과 새로운 클러스터 중심 간의 매칭
        new_centers = calculateCenters(merge_data, new_cluster_labels, num_clusters);
        center_match = matchClusters(centers, new_centers);

        % 시각화
        disp('new clustering excluding completed task');
        subplot(2, 2, 2);
        hold on;
        
        title('new clustering excluding completed task');
        for i = 1:num_clusters
            color_idx = center_match(i);  % 매칭된 색상 인덱스
            for j = 1:size(remaining_data, 1)
                point_size = 500 * new_U(i, j);
                alpha = max(0.6, new_U(i, j));
                scatter(remaining_data(j,1), remaining_data(j,2), point_size, 'MarkerFaceColor', colors{color_idx}, ...
                        'MarkerEdgeColor', colors{color_idx}, 'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);
                if(j<=3)
                    scatter(new_data(j, 1), new_data(j, 2), point_size, ...
                    'MarkerFaceColor', colors{color_idx}, 'MarkerEdgeColor', colors{color_idx}, ...
                    'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha, ...
                    'Marker', '^');
                end
            end
        end
    
        plot(robot_start(1), robot_start(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
        
        xlim([min(data(:, 1)) - 200, max(data(:, 1)) + 200]);
        ylim([min(data(:, 2)) - 200, max(data(:, 2)) + 200]);
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 가우시안 분포 기반의 가중치를 이용한 클러스터 확률 계산
        sigma = 200;  % 가우시안 분포의 표준편차 (거리 영향 범위)
        radius = 500;  % 주변 Task를 고려할 반경
        num_new_data = size(new_data, 1);
        cluster_probabilities = zeros(num_new_data, num_clusters);
        tic;
        
        % 전체 데이터와 새 데이터 간의 거리 계산 (벡터화)
        for i = 1:num_new_data
            % 반경 내에 있는 주변 Task들만 선택 (벡터화)
            distances_to_new_point = sqrt(sum((data - new_data(i, :)).^2, 2));
            neighbors_idx = find(distances_to_new_point <= radius);
            neighbors = data(neighbors_idx, :);
            
            % 주변 Task들의 클러스터 확률 (벡터화)
            neighbor_probs = U(:, neighbors_idx);
            
            % 각 클러스터에 대해 주변 Task들의 클러스터 확률과 거리 기반 가중치 계산 (벡터화)
            gauss_weights = exp(-distances_to_new_point(neighbors_idx).^2 / (2 * sigma^2));
            gauss_weights = gauss_weights(:)';  % gauss_weights를 행 벡터로 변환하여 브로드캐스팅 가능하게 만듦
            
            % 원소별 곱셈 수행 (neighbor_probs와 gauss_weights의 크기를 맞춤)
            weighted_probs = neighbor_probs .* gauss_weights;
            
            % 최종 클러스터 확률 계산
            cluster_probabilities(i, :) = sum(weighted_probs, 2)' / sum(gauss_weights);
        end
        
        times = toc;
        disp(['My IDEA with Vectorization: ', num2str(times), ' 초']);
        % 시각화
        subplot(2, 2, 4);
        hold on;
        title('calculate with neighbor');
        % 기존의 data
        for i = 1:num_clusters
            for j = 1:size(data, 1)
                % 각 포인트의 확률에 따라 크기 및 불투명도 설정
                point_size = 500 * U(i, j);  % 확률에 따라 점 크기 조정
                alpha = max(0.6, U(i, j));  % 확률에 따라 불투명도 설정
                scatter(data(j,1), data(j,2), point_size, 'MarkerFaceColor', colors{i}, ...
                        'MarkerEdgeColor', colors{i}, 'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);
            end
        end
        % new data
        for i = 1:num_new_data
            for j = 1:num_clusters
                scatter(new_data(i, 1), new_data(i, 2), 500 * cluster_probabilities(i, j), ...
                        'MarkerFaceColor', colors{j}, 'MarkerEdgeColor', colors{j}, ...
                        'MarkerFaceAlpha', cluster_probabilities(i, j), 'MarkerEdgeAlpha', cluster_probabilities(i, j),'Marker', '^');
            end
        end
        plot(robot_start(1), robot_start(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
        
        xlim([min(data(:, 1)) - 200, max(data(:, 1)) + 200]);
        ylim([min(data(:, 2)) - 200, max(data(:, 2)) + 200]);

        
        subplot(2,2,1)
    end
    % 기존 로봇의 이동 시뮬레이션 계속
    for i = 1:num_clusters
        if current_target_index(i) <= size(optimal_paths{i}, 1)
            all_tasks_completed = false;  % 아직 할 일이 남아 있음
            target_point = optimal_paths{i}(current_target_index(i), :);
            
            % 현재 위치와 목표 위치가 같은 경우 (중복된 경우) 다음 목표로 이동
            if isequal(robot_pos(i, :), target_point)
                current_target_index(i) = current_target_index(i) + 1;
                continue;  % 다음 루프로 건너뜀
            end

            % 로봇이 목표점으로 이동
            direction = (target_point - robot_pos(i,:)) / norm(target_point - robot_pos(i,:));
            robot_pos(i,:) = robot_pos(i,:) + robot_speed * direction * time_step;

            % 로봇이 목표점에 도달하면 다음 목표점으로 이동
            if norm(robot_pos(i,:) - target_point) < robot_speed * time_step
                % 해당 점을 채워진 원으로 표시
                plot(target_point(1), target_point(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', colors{i});
                current_target_index(i) = current_target_index(i) + 1;  % 다음 목표점으로 이동
            end

            % 경로 점을 그리기
            plot(robot_paths{i}(:,1), robot_paths{i}(:,2), 'Color', colors{i}, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 2); 
            
            % 로봇의 이전 마커 삭제 및 새로운 위치로 업데이트
            delete(robot_markers(i));
            robot_markers(i) = plot(robot_pos(i, 1), robot_pos(i, 2), 'o', 'MarkerSize',6 , 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', colors{i});

            % 현재 위치를 경로에 추가
            robot_paths{i} = [robot_paths{i}; robot_pos(i,:)]; % 경로 기록
        end
    end
    pause(0.05);  % 시뮬레이션 속도 조절
end

hold off;

% 클러스터 중심 계산 함수
function centers = calculateCenters(data, cluster_labels, num_clusters)
    centers = zeros(num_clusters, 2);
    for i = 1:num_clusters
        cluster_data = data(cluster_labels == i, :);
        centers(i, :) = mean(cluster_data, 1);
    end
end

% 기존 클러스터와 새로운 클러스터 간의 매칭 함수
function match = matchClusters(old_centers, new_centers)
    num_clusters = size(old_centers, 1);
    match = zeros(1, num_clusters);
    for i = 1:num_clusters
        distances = pdist2(new_centers(i, :), old_centers);
        [~, match(i)] = min(distances);
    end
end

% 비동기 클러스터링 함수
function [new_U, new_cluster_labels] = performClustering(data, num_clusters)
    options = [NaN NaN 0.01 1000];
    [new_centers, new_U] = fcm(data, num_clusters, options);
    [~, new_cluster_labels] = max(new_U);

end

% ACO 알고리즘 구현
function [best_path, best_cost] = tspACO(cluster_points, start_point)
    num_ants = 50;  % 개미의 수
    num_iterations = 1000;  % 반복 횟수
    alpha = 1;  % 페로몬 중요도
    beta = 7;  % 거리 중요도
    rho = 0.3;  % 페로몬 증발 계수
    Q = 50;  % 페로몬 증가 계수
    
    n = size(cluster_points, 1);
    dist_matrix = squareform(pdist([start_point; cluster_points]));
    pheromone = ones(n+1, n+1);  % 출발점을 포함한 페로몬 행렬 초기화
    
    best_cost = inf;
    best_path = [];
    
    for iteration = 1:num_iterations
        all_paths = zeros(num_ants, n+1);  % 개미들이 찾은 경로 저장
        all_costs = zeros(num_ants, 1);  % 각 개미의 경로 비용 저장
        
        for k = 1:num_ants
            % 출발점에서 시작
            path = zeros(1, n+1);
            path(1) = 1;  % 출발점 (start_point)
            unvisited = 2:(n+1);  % 클러스터 내 점들
            
            for i = 2:n+1
                current_city = path(i-1);
                probabilities = zeros(1, length(unvisited));
                
                % 각 도시로 갈 확률 계산
                for j = 1:length(unvisited)
                    next_city = unvisited(j);
                    probabilities(j) = (pheromone(current_city, next_city)^alpha) * ...
                                       ((1 / dist_matrix(current_city, next_city))^beta);
                end
                
                % 확률 기반으로 다음 도시 선택
                probabilities = probabilities / sum(probabilities);
                cumulative_probabilities = cumsum(probabilities);
                random_choice = rand;
                selected_index = find(cumulative_probabilities >= random_choice, 1);
                next_city = unvisited(selected_index);
                
                path(i) = next_city;  % 경로에 다음 도시 추가
                unvisited(selected_index) = [];  % 방문한 도시를 목록에서 제거
            end
            
            all_paths(k, :) = path;
            all_costs(k) = calculateCost(path, dist_matrix);
            
            % 최적해 갱신
            if all_costs(k) < best_cost
                best_cost = all_costs(k);
                best_path = cluster_points(path(2:end)-1, :);  % 최적 경로 업데이트
            end
        end
        
        % 페로몬 업데이트
        pheromone = (1 - rho) * pheromone;  % 페로몬 증발
        for k = 1:num_ants
            path = all_paths(k, :);
            path_cost = all_costs(k);
            for i = 1:n
                pheromone(path(i), path(i+1)) = pheromone(path(i), path(i+1)) + Q / path_cost;
                pheromone(path(i+1), path(i)) = pheromone(path(i+1), path(i)) + Q / path_cost;
            end
        end
    end
    
    % 시작점으로 돌아오는 거리 포함
    best_cost = best_cost + norm(best_path(end, :) - start_point);
    best_path = [start_point; best_path; start_point];
end

% 비용 계산 함수 (시작점으로 돌아오는 것까지 포함)
function cost = calculateCost(order, dist_matrix)
    cost = 0;
    for i = 1:length(order)-1
        cost = cost + dist_matrix(order(i), order(i+1));
    end
    cost = cost + dist_matrix(order(end), order(1)); % 시작점으로 돌아오는 거리 포함
end

function weights = calculateGaussianWeightsForNeighbors(new_point, neighbors, cluster_probs, sigma)
    % 새로 추가된 점과 주변 Task들 사이의 거리 계산
    distances = sqrt(sum((neighbors - new_point).^2, 2));
    
    % 거리 기반 가우시안 가중치 계산
    gauss_weights = exp(-distances.^2 / (2 * sigma^2));
    
    % cluster_probs를 열 벡터로 변환
    cluster_probs = cluster_probs(:);
    
    % 최종 가중치는 가우시안 가중치와 클러스터 확률의 곱
    weights = cluster_probs .* gauss_weights;
end
