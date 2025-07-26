

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GameAICourse
{

    public class CreateGrid
    {

        // Please change this string to your name
        public const string StudentAuthorName = "Dongning Li";


        // Helper method provided to help you implement this file. Leave as is.
        // Returns true if point p is inside (or on edge) the polygon defined by pts (CCW winding). False, otherwise
        static bool IsPointInsidePolygon(Vector2Int[] pts, Vector2Int p)
        {
            return CG.InPoly1(pts, p) != CG.PointPolygonIntersectionType.Outside;
        }


        // Helper method provided to help you implement this file. Leave as is.
        // Returns float converted to int according to default scaling factor (1000)
        static int Convert(float v)
        {
            return CG.Convert(v);
        }

        // Helper method provided to help you implement this file. Leave as is.
        // Returns Vector2 converted to Vector2Int according to default scaling factor (1000)
        static Vector2Int Convert(Vector2 v)
        {
            return CG.Convert(v);
        }

        // Helper method provided to help you implement this file. Leave as is.
        // Returns true is segment AB intersects CD properly or improperly
        static bool Intersects(Vector2Int a, Vector2Int b, Vector2Int c, Vector2Int d)
        {
            return CG.Intersect(a, b, c, d);
        }


        // IsPointInsideBoundingBox(): Determines whether a point (Vector2Int:p) is On/Inside a bounding box (such as a grid cell) defined by
        // minCellBounds and maxCellBounds (both Vector2Int's).
        // Returns true if the point is ON/INSIDE the cell and false otherwise
        // This method should return true if the point p is on one of the edges of the cell.
        // This is more efficient than PointInsidePolygon() for an equivalent dimension poly
        // Preconditions: minCellBounds <= maxCellBounds, per dimension
        static bool IsPointInsideAxisAlignedBoundingBox(Vector2Int minCellBounds, Vector2Int maxCellBounds, Vector2Int p)
        {
            //TODO IMPLEMENT

            // placeholder logic to be replaced by the student
            return p.x >= minCellBounds.x && p.x <= maxCellBounds.x &&
                    p.y >= minCellBounds.y && p.y <= maxCellBounds.y;
        }


        // IsRangeOverlapping(): Determines if the range (inclusive) from min1 to max1 overlaps the range (inclusive) from min2 to max2.
        // The ranges are considered to overlap if one or more values is within the range of both.
        // Returns true if overlap, false otherwise.
        // Preconditions: min1 <= max1 AND min2 <= max2
        static bool IsRangeOverlapping(int min1, int max1, int min2, int max2)
        {
            // TODO IMPLEMENT
            return max1 >= min2 && max2 >= min1;
        }

        // IsAxisAlignedBouningBoxOverlapping(): Determines if the AABBs defined by min1,max1 and min2,max2 overlap or touch
        // Returns true if overlap, false otherwise.
        // Preconditions: min1 <= max1, per dimension. min2 <= max2 per dimension
        static bool IsAxisAlignedBoundingBoxOverlapping(Vector2Int min1, Vector2Int max1, Vector2Int min2, Vector2Int max2)
        {

            // TODO IMPLEMENT
            // HINT: Call IsRangeOverlapping()

            return IsRangeOverlapping(min1.x, max1.x, min2.x, max2.x) &&
                    IsRangeOverlapping(min1.y, max1.y, min2.y, max2.y);
        }


        // IsTraversable(): returns true if the grid is traversable from grid[x,y] in the direction dir, false otherwise.
        // The grid boundaries are not traversable. If the grid position x,y is itself not traversable but the grid cell in direction
        // dir is traversable, the function will return false.
        // returns false if the grid is null, grid rank is not 2 dimensional, or any dimension of grid is zero length
        // returns false if x,y is out of range
        // Note: public methods are autograded
        public static bool IsTraversable(bool[,] grid, int x, int y, TraverseDirection dir)
        {

            // Check for null grid or empty dimensions
            if (grid == null || grid.Rank != 2 || grid.GetLength(0) == 0 || grid.GetLength(1) == 0)
                return false;

            // Check if starting position is out of bounds or not traversable
            if (x < 0 || x >= grid.GetLength(0) || y < 0 || y >= grid.GetLength(1) || !grid[x, y])
                return false;

            (int newX, int newY) = dir switch
            {
                TraverseDirection.Up => (x, y + 1),
                TraverseDirection.Down => (x, y - 1),
                TraverseDirection.Left => (x - 1, y),
                TraverseDirection.Right => (x + 1, y),
                TraverseDirection.UpLeft => (x - 1, y + 1),
                TraverseDirection.UpRight => (x + 1, y + 1),
                TraverseDirection.DownLeft => (x - 1, y - 1),
                TraverseDirection.DownRight => (x + 1, y - 1),
                _ => (x, y)

            };

            // Check if target position out of bounds
            if (newX < 0 || newX >= grid.GetLength(0) || newY < 0 || newY >= grid.GetLength(1))
                return false;

            return grid[newX, newY];

        }
        
        // Create(): Creates a grid lattice discretized space for navigation.
        // canvasOrigin: bottom left corner of navigable region in world coordinates
        // canvasWidth: width of navigable region in world dimensions
        // canvasHeight: height of navigable region in world dimensions
        // cellWidth: target cell width (of a grid cell) in world dimensions
        // obstacles: a list of collider obstacles
        // grid: an array of bools. A cell is true if navigable, false otherwise
        //    Example: grid[x_pos, y_pos]

        public static void Create(Vector2 canvasOrigin, float canvasWidth, float canvasHeight, float cellWidth,
            List<Polygon> obstacles,
            out bool[,] grid
            )
        {
            // ignoring the obstacles for this limited demo; 
            // Marks cells of the grid untraversable if geometry intersects interior!
            // Carefully consider all possible geometry interactions

            // also ignoring the world boundary defined by canvasOrigin and canvasWidth and canvasHeight

            // ==== Input Validation ====
            if (canvasWidth <= 0f || canvasHeight <= 0f || cellWidth <= 0f)
            {
                grid = null; // Invalid input → return null
                return;
            }

            // Calculate grid dimensions
            int cols = Mathf.FloorToInt(canvasWidth / cellWidth);
            int rows = Mathf.FloorToInt(canvasHeight / cellWidth);

            grid = new bool[cols, rows];

            // Calculate the actual cell size
            float actualCellWidth = canvasWidth / cols;
            float actualCellHeight = canvasHeight / rows;

             float shrinkDistance = 0.01f * cellWidth;
            // World boundaries (in world coordinates)
            Vector2 worldMin = canvasOrigin;
            Vector2 worldMax = canvasOrigin + new Vector2(canvasWidth, canvasHeight);


            // Pre-convert all obstacle points to Vector2Int for faster comparison
            var obstacleAABBs = new List<(Vector2Int min, Vector2Int max)>();
            foreach (var obstacle in obstacles)
            {
                obstacleAABBs.Add((obstacle.MinIntBounds, obstacle.MaxIntBounds));
            }

            // Check each cell for obstacle coolisions
            for (int x = 0; x < cols; x++)
            {
                for (int y = 0; y < rows; y++)
                {
                    // Default to traversable
                    grid[x, y] = true;

                    Vector2 cellMin = new Vector2(
                        canvasOrigin.x + x * actualCellWidth + shrinkDistance,
                         canvasOrigin.y + y * actualCellHeight + shrinkDistance);
                    Vector2 cellMax = new Vector2(
                        cellMin.x + actualCellWidth - 2*shrinkDistance,
                         cellMin.y + actualCellHeight - 2*shrinkDistance);


                    // Skip invalid cells
                    if (cellMax.x <= cellMin.x || cellMax.y <= cellMin.y ||
                    cellMax.x <= worldMin.x || cellMin.x >= worldMax.x ||
                    cellMax.y <= worldMin.y || cellMin.y >= worldMax.y)
                    {
                        grid[x, y] = false;
                        continue;
                    }

                    // Create cell polygon in Vector2Int space
                    Vector2Int cellMinInt = Convert(cellMin);
                    Vector2Int cellMaxInt = Convert(cellMax);
                    Vector2Int[] cellPoly = new Vector2Int[]
                    {
                        cellMinInt,
                        new Vector2Int(cellMaxInt.x, cellMinInt.y),
                        cellMaxInt,
                        new Vector2Int(cellMinInt.x, cellMaxInt.y)
                    };

                
                    // Check against all obstacles
                    foreach (var obstacle in obstacles)
                    {
                        Vector2Int[] obstaclePoints = obstacle.getIntegerPoints();

                        // Quick AABB check first
                        if (!IsAxisAlignedBoundingBoxOverlapping(
                            cellMinInt, cellMaxInt,
                            obstacle.MinIntBounds, obstacle.MaxIntBounds))
                        {
                            continue;
                        }

                        bool blocksCell = false;

                        // 1. Check if any cell corner is inside the obstacle

                        foreach (var corner in cellPoly)
                        {
                            
                            if (IsPointInsidePolygon(obstaclePoints, corner))
                            {
                                blocksCell = true;
                                break;
                            }
                        }


                        // 2. Check if any obstacle vertex is inside cell (not just on edge)

                        foreach (var point in obstaclePoints)
                        {
                            if (point.x > cellMinInt.x && point.x < cellMaxInt.x &&
                                point.y > cellMinInt.y && point.y < cellMaxInt.y)
                            {
                                blocksCell = true;
                                break;
                            }
                        }

                        // 3. Check for edge intersections between cell and obstacle

                        for (int j = 0; j < cellPoly.Length; j++)
                        {
                            Vector2Int a = cellPoly[j];
                            Vector2Int b = cellPoly[(j + 1) % cellPoly.Length];

                            for (int k = 0; k < obstaclePoints.Length; k++)
                            {
                                Vector2Int c = obstaclePoints[k];
                                Vector2Int d = obstaclePoints[(k + 1) % obstaclePoints.Length];
                                if (Intersects(a, b, c, d))
                                {
                                    // Additional check to exclude simple endpoint touches
                                    if (!(a == c || a == d || b == c || b == d))
                                    {
                                        blocksCell = true;
                                        break;
                                    }
                                }
                            }
                            if (blocksCell) break;
                        }
                        if (blocksCell)
                        {
                            grid[x, y] = false;
                            continue;
                        }

                        // 4. check if cell center is inside obstacle
                        Vector2Int cellCenter = new Vector2Int(
                            (cellMinInt.x + cellMaxInt.x) / 2,
                            (cellMinInt.y + cellMaxInt.y) / 2);
                        if (IsPointInsidePolygon(obstaclePoints, cellCenter))
                        {
                            grid[x, y] = false;
                        }
                            }

                    }
                }
            }
        }
    }
