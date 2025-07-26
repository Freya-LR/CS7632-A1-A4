
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Priority_Queue;
using UnityEngine;


namespace GameAICourse
{


    public class AStarPathSearchImpl
    {

        // Please change this string to your name
        public const string StudentAuthorName = "Dongning Li";

        // Null Heuristic for Dijkstra
        public static float HeuristicNull(Vector2 nodeA, Vector2 nodeB)
        {
            return 0f;
        }

        // Null Cost for Greedy Best First
        public static float CostNull(Vector2 nodeA, Vector2 nodeB)
        {
            return 0f;
        }

        // Heuristic distance fuction implemented with manhattan distance
        public static float HeuristicManhattan(Vector2 nodeA, Vector2 nodeB)
        {
            // |x1 - x2| + |y1 - y2|
            return Mathf.Abs(nodeA.x - nodeB.x) + Mathf.Abs(nodeA.y - nodeB.y);
        }

        // Heuristic distance function implemented with Euclidean distance
        public static float HeuristicEuclidean(Vector2 nodeA, Vector2 nodeB)
        {
            //(x^2+y^2)^0.5
            return Vector2.Distance(nodeA,nodeB);
        }

        // Cost is only ever called on adjacent nodes. So we will always use Euclidean distance.
        // We could use Manhattan dist for 4-way connected grids and avoid sqrroot and mults.
        // But we will avoid that for simplicity.
        public static float Cost(Vector2 nodeA, Vector2 nodeB)
        {
            return HeuristicEuclidean(nodeA, nodeB);
        }

        public static PathSearchResultType FindPathIncremental(
            GetNodeCount getNodeCount,
            GetNode getNode,
            GetNodeAdjacencies getAdjacencies,
            CostCallback G,
            CostCallback H,
            int startNodeIndex, int goalNodeIndex,
            int maxNumNodesToExplore, bool doInitialization,
            ref int currentNodeIndex,
            ref Dictionary<int, PathSearchNodeRecord> searchNodeRecords,
            ref SimplePriorityQueue<int, float> openNodes, ref HashSet<int> closedNodes, ref List<int> returnPath)
        {
            PathSearchResultType pathResult = PathSearchResultType.InProgress;
           
            var nodeCount = getNodeCount();

            // If start = goal
            if (startNodeIndex == goalNodeIndex)
            {
                _ = G(getNode(startNodeIndex), getNode(goalNodeIndex));
                searchNodeRecords ??= new Dictionary<int, PathSearchNodeRecord>();
                openNodes ??= new SimplePriorityQueue<int, float>();
                closedNodes ??= new HashSet<int>();
                returnPath ??= new List<int>();
                var startRecord = new PathSearchNodeRecord(
                startNodeIndex,-1,0f,H(getNode(startNodeIndex), getNode(goalNodeIndex))
                );
                searchNodeRecords[startNodeIndex] = startRecord;
                closedNodes.Add(startNodeIndex);
                returnPath.Add(startNodeIndex);
                return PathSearchResultType.Complete;
            }

            if (startNodeIndex >= nodeCount || goalNodeIndex >= nodeCount ||
                startNodeIndex < 0 || goalNodeIndex < 0 ||
                maxNumNodesToExplore <= 0
                || (!doInitialization &&
                 (openNodes == null || closedNodes == null || currentNodeIndex < 0 ||
                  currentNodeIndex >= nodeCount)))

                return PathSearchResultType.InitializationError;

            searchNodeRecords ??= new Dictionary<int, PathSearchNodeRecord>();
            openNodes ??= new SimplePriorityQueue<int, float>();
            closedNodes ??= new HashSet<int>();
            returnPath ??= new List<int>();

            // If start a new search
            if (doInitialization)
            {
                searchNodeRecords.Clear();
                openNodes.Clear();
                closedNodes.Clear();
                returnPath.Clear();
                // Initialize with start node
                var startRecord = new PathSearchNodeRecord(startNodeIndex,-1, 0f,
                    H(getNode(startNodeIndex), getNode(goalNodeIndex)) // Total estimated cost
                    );

                searchNodeRecords[startNodeIndex] = startRecord;
                openNodes.Enqueue(startNodeIndex, startRecord.EstimatedTotalCost);
                currentNodeIndex = startNodeIndex;
            }

            // Process nodes
            int nodesExplored = 0;
            while (nodesExplored < maxNumNodesToExplore && openNodes.Count > 0)
            {
                currentNodeIndex = openNodes.Dequeue();
                var currentRecord = searchNodeRecords[currentNodeIndex];
                closedNodes.Add(currentNodeIndex);

                // Check if vwe've reached the goal
                if (currentNodeIndex == goalNodeIndex)
                {
                    returnPath.Clear();
                    int current = currentNodeIndex;

                    while (current != -1 && searchNodeRecords.ContainsKey(current))
                    {
                        returnPath.Insert(0, current);
                        current = searchNodeRecords[current].FromNodeIndex;
                    }
                    return PathSearchResultType.Complete;
                }

                // Explore neighbors                
                foreach (var neighborIndex in getAdjacencies(currentNodeIndex))
                {
                    if (closedNodes.Contains(neighborIndex)) continue;

                    // Calculate new cost
                    Vector2 currentPos = getNode(currentNodeIndex);
                    Vector2 neighborPos = getNode(neighborIndex);
                    float newCost = currentRecord.CostSoFar + G(currentPos, neighborPos);
                    float newHeuristic = H(neighborPos, getNode(goalNodeIndex));
                    float newEstimatedTotal = newCost + newHeuristic;

                    // Check if we need to update this neighbor
                    if (!searchNodeRecords.TryGetValue(neighborIndex, out var neighborRecord) ||
                    newCost < neighborRecord.CostSoFar)
                    {
                        // Create or update record
                        neighborRecord = new PathSearchNodeRecord(
                            neighborIndex,
                            currentNodeIndex,
                            newCost,
                            newEstimatedTotal);

                        searchNodeRecords[neighborIndex] = neighborRecord;

                        // Update priority queue
                        if (openNodes.Contains(neighborIndex))
                        {
                            //update priority to the newest estimatedTotalCost if already in opennode
                            openNodes.UpdatePriority(neighborIndex, newEstimatedTotal); 
                        }
                        else
                        {
                            openNodes.Enqueue(neighborIndex, newEstimatedTotal);
                        }
                    }
                }
                nodesExplored++;
            }

            // Completion states
            if (openNodes.Count == 0)
            {
                int closedNode = -1;
                float minDistance = float.MaxValue;
                float minCost = float.MaxValue;
                var goalPos = getNode(goalNodeIndex);

                foreach (var nodeIndex in closedNodes)
                {
                    if (!searchNodeRecords.TryGetValue(nodeIndex, out var record))
                        continue;

                    float distance;
                    if (H == HeuristicNull)
                    {
                        distance = Vector2.Distance(getNode(nodeIndex), goalPos);
                    }
                    else
                    {
                        // distance = H(getNode(nodeIndex), goalPos);
                        distance = record.EstimatedTotalCost - record.CostSoFar;
                    }

                    if (distance < minDistance ||
                        (Mathf.Approximately(distance, minDistance) &&
                        record.CostSoFar < minCost))
                    {
                        minDistance = distance;
                        minCost = record.CostSoFar;
                        closedNode = nodeIndex;
                    }
                }

                // Reconstruct path to closest node
                returnPath = new List<int>();
                if (closedNode != -1)
                {
                    int current = closedNode;
                    while (current != -1 && searchNodeRecords.ContainsKey(current))
                    {
                        returnPath.Insert(0,current);
                        current = searchNodeRecords[current].FromNodeIndex;
                    }
                }
                
                return PathSearchResultType.Partial;
            }
            return PathSearchResultType.InProgress;
            }    
                            
    }
}