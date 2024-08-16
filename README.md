# Fuzzy C-Means (FCM) Clustering for Task Allocation with Popup Tasks

This project demonstrates the application of the Fuzzy C-Means (FCM) clustering algorithm for task allocation, particularly focusing on the effect of completed tasks and popup tasks on clustering behavior. The approach contrasts with the traditional k-means method by incorporating task flexibility and task completion history into the clustering process.

## Overview

In task allocation scenarios, popup tasks may arise dynamically during the process, and how previously completed tasks are handled can significantly influence the clustering outcome. This repository contains a simple testbed to visualize the impact of different strategies on clustering when popup tasks appear.

### Clustering Scenarios

- **Initial FCM Path Planning**:  
  The initial clustering and path planning are shown based on the FCM algorithm, with distinct clusters for tasks. The planned paths between the clusters are visualized.

- **New Clustering Excluding Completed Tasks**:  
  In this scenario, completed tasks are excluded from the clustering process when popup tasks appear. This results in a new clustering layout that excludes previous tasks.

- **New Clustering Including Completed Tasks**:  
  Here, completed tasks are included in the clustering when popup tasks are considered. The inclusion of completed tasks leads to a different clustering outcome.

- **Calculate with Neighbor**:  
  This final approach considers both the original clusters and nearby clusters, taking into account the proximity of popup tasks when recalculating the clusters. This is part of an ongoing idea that is being developed.

## Key Insights

- **FCM vs K-Means**:  
  Unlike k-means clustering, which assigns each point strictly to one cluster, FCM allows for a degree of membership to multiple clusters. This provides flexibility in task allocation, particularly useful when dealing with dynamic or popup tasks.

- **Popup Task Handling**:  
  How previously completed tasks are treated—whether they are included or excluded—affects the task distribution and cluster formation. This project explores these variations to better understand the behavior of the clustering algorithm in dynamic environments.
  
![New Clustering](images(1).png)
*Fig 2: New clustering strategy including completed and popup tasks.*