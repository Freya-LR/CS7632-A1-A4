
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GameAICourse
{

    public class CreatePathNetwork
    {

        public const string StudentAuthorName = "Dongning Li!";


        // Helper method provided to help you implement this file. Leave as is.
        // Returns Vector2 converted to Vector2Int according to default scaling factor (1000)
        public static Vector2Int ConvertToInt(Vector2 v)
        {
            return CG.Convert(v);
        }

        // Helper method provided to help you implement this file. Leave as is.
        // Returns float converted to int according to default scaling factor (1000)
        public static int ConvertToInt(float v)
        {
            return CG.Convert(v);
        }

        // Helper method provided to help you implement this file. Leave as is.
        // Returns Vector2Int converted to Vector2 according to default scaling factor (1000)
        public static Vector2 ConvertToFloat(Vector2Int v)
        {
            float f = 1f / (float)CG.FloatToIntFactor;
            return new Vector2(v.x * f, v.y * f);
        }

        // Helper method provided to help you implement this file. Leave as is.
        // Returns int converted to float according to default scaling factor (1000)
        public static float ConvertToFloat(int v)
        {
            float f = 1f / (float)CG.FloatToIntFactor;
            return v * f;
        }


        // Helper method provided to help you implement this file. Leave as is.
        // Returns true is segment AB intersects CD properly or improperly
        static public bool Intersects(Vector2Int a, Vector2Int b, Vector2Int c, Vector2Int d)
        {
            return CG.Intersect(a, b, c, d);
        }


        //Get the shortest distance from a point to a line
        //Line is defined by the lineStart and lineEnd points
        public static float DistanceToLineSegment(Vector2Int point, Vector2Int lineStart, Vector2Int lineEnd)
        {
            return CG.DistanceToLineSegment(point, lineStart, lineEnd);
        }


        //Get the shortest distance from a point to a line
        //Line is defined by the lineStart and lineEnd points
        public static float DistanceToLineSegment(Vector2 point, Vector2 lineStart, Vector2 lineEnd)
        {
            return CG.DistanceToLineSegment(point, lineStart, lineEnd);
        }


        // Helper method provided to help you implement this file. Leave as is.
        // Determines if a point is inside/on a CCW polygon and if so returns true. False otherwise.
        public static bool IsPointInPolygon(Vector2Int[] polyPts, Vector2Int point)
        {
            return CG.PointPolygonIntersectionType.Outside != CG.InPoly1(polyPts, point);
        }

        // Returns true iff p is strictly to the left of the directed
        // line through a to b.
        // You can use this method to determine if 3 adjacent CCW-ordered
        // vertices of a polygon form a convex or concave angle

        public static bool Left(Vector2Int a, Vector2Int b, Vector2Int p)
        {
            return CG.Left(a, b, p);
        }

        // Vector2 version of above
        public static bool Left(Vector2 a, Vector2 b, Vector2 p)
        {
            return CG.Left(CG.Convert(a), CG.Convert(b), CG.Convert(p));
        }


        //Student code to build the path network from the given pathNodes and Obstacles
        //Obstacles - List of obstacles on the plane
        //agentRadius - the radius of the traversing agent
        //minPoVDist AND maxPoVDist - used for Points of Visibility (see assignment doc)
        //pathNodes - ref parameter that contains the pathNodes to connect (or return if pathNetworkMode is set to PointsOfVisibility)
        //pathEdges - out parameter that will contain the edges you build.
        //  Edges cannot intersect with obstacles or boundaries. Edges must be at least agentRadius distance
        //  from all obstacle/boundary line segments. No self edges, duplicate edges. No null lists (but can be empty)
        //pathNetworkMode - enum that specifies PathNetwork type (see assignment doc)

        public static void Create(Vector2 canvasOrigin, float canvasWidth, float canvasHeight,
            List<Polygon> obstacles, float agentRadius, float minPoVDist, float maxPoVDist, ref List<Vector2> pathNodes, out List<List<int>> pathEdges,
            PathNetworkMode pathNetworkMode = PathNetworkMode.Predefined)
        {

            pathEdges = new List<List<int>>();

            for (int i = 0; i < pathNodes.Count; i++)
            {
                pathEdges.Add(new List<int>());
            }

            for (int i = 0; i < pathNodes.Count; i++)
            {
                for (int j = i + 1; j < pathNodes.Count; j++)
                {

                    if (IsValidConnection(pathNodes[i], pathNodes[j], obstacles, agentRadius, canvasOrigin, canvasWidth, canvasHeight))
                    {
                        pathEdges[i].Add(j);
                        pathEdges[j].Add(i);
                    }
                }
            }

        }
        

        private static bool IsPointInBounds(Vector2 point, Vector2 canvasOrigin, float canvasWidth, float canvasHeight)
        {
            return point.x >= canvasOrigin.x && point.x <= canvasOrigin.x + canvasWidth &&
                   point.y >= canvasOrigin.y && point.y <= canvasOrigin.y + canvasHeight;
        }
        
        public static bool IsPointInObstacle(Vector2 point, Polygon obstacle, float agentRadius)
        {

            Vector2Int pointInt = ConvertToInt(point);
            Vector2Int[] obstaclePoints = obstacle.getIntegerPoints();
            
            if (IsPointInPolygon(obstaclePoints, pointInt))
                return true;

            // Check if point is within agentRadius of any obstacle edge
            for (int i = 0, j = obstaclePoints.Length - 1; i < obstaclePoints.Length; j = i++)
            {
                float distance = DistanceToLineSegment(pointInt, obstaclePoints[j], obstaclePoints[i]);
                if (distance < ConvertToInt(agentRadius))
                    return true;
            }

            return false;

        }

        public static bool IsValidConnection(Vector2 a, Vector2 b, List<Polygon> obstacles, float agentRadius,Vector2 canvasOrigin, float canvasWidth, float canvasHeight)
        {
            if (!IsPointInBounds(a, canvasOrigin, canvasWidth, canvasHeight) ||
            !IsPointInBounds(b, canvasOrigin, canvasWidth, canvasHeight)) return false;

            Vector2Int aInt = ConvertToInt(a);
            Vector2Int bInt = ConvertToInt(b);
            
            Vector2[] edgeSamples = new Vector2[]
            {
                a,
                Vector2.Lerp(a, b, 0.25f),
                Vector2.Lerp(a, b, 0.5f),
                Vector2.Lerp(a, b, 0.75f),
                b
            };
                    
            foreach (var obstacle in obstacles)
            {

                if (obstacle == null || obstacle.getPoints() == null || obstacle.getPoints().Length < 3) continue;

                Vector2[] points = obstacle.getPoints();
                Vector2Int[] intPoints = obstacle.getIntegerPoints();

                // Compute obstacle AABB
                Rect obstacleBounds = ComputeBounds(points);
                obstacleBounds.xMin -= agentRadius;
                obstacleBounds.xMax += agentRadius;
                obstacleBounds.yMin -= agentRadius;
                obstacleBounds.yMax += agentRadius;

                // Compute edge AABB
                Rect edgeBounds = Rect.MinMaxRect(
                    Mathf.Min(a.x, b.x), Mathf.Min(a.y, b.y),
                    Mathf.Max(a.x, b.x), Mathf.Max(a.y, b.y)
                );


                if (!edgeBounds.Overlaps(obstacleBounds))
                {
                    continue;
                }

                // Check if either endpoint is inside the expanded obstacle
                // if (IsPointInObstacle(a, obstacle, agentRadius))
                // {
                //     return false;
                // }

                // if (IsPointInObstacle(b, obstacle, agentRadius))
                // {
                //     return false;
                // }
                foreach (var sample in edgeSamples)
                {
                    if (IsPointInObstacle(sample, obstacle, agentRadius))
                        return false;
                }

                for (int i = 0; i < points.Length; i++)
                {
                    Vector2Int c = intPoints[i];
                    Vector2Int d = intPoints[(i + 1) % points.Length];

                    // Skip if this is actually the same segment
                    if ((aInt == c && bInt == d) || (aInt == d && bInt == c))
                        continue;

                    // Proper intersection check
                    if (Intersects(aInt, bInt, c, d))
                    {
                        return false;
                    }

                    // Check distance from edge to every obstacle VERTEX
                    float vertexDist = DistanceToLineSegment(c, a, b);
                    if (vertexDist < agentRadius)
                        return false;
                            foreach (var sample in edgeSamples)
                            {
                                float edgeDist = DistanceToLineSegment(sample, c, d);
                                if (edgeDist < agentRadius)
                                    return false;
                            }   
                        }
                  

            }
            // Check clearance from canvas boundaries
            Vector2[] boundaryCorners = new Vector2[]
            {
                canvasOrigin,
                canvasOrigin + new Vector2(canvasWidth, 0),
                canvasOrigin + new Vector2(canvasWidth, canvasHeight),
                canvasOrigin + new Vector2(0, canvasHeight)
            };

            for (int i = 0; i < 4; i++)
            {
                Vector2 c = boundaryCorners[i];
                Vector2 d = boundaryCorners[(i + 1) % 4];

                foreach (var sample in edgeSamples)
                {
                    if (DistanceToLineSegment(sample, c, d) < agentRadius)
                        return false;
                }
            }
            return true;
        }
        private static Rect ComputeBounds(Vector2[] points)
        {
            float minX = float.MaxValue, maxX = float.MinValue;
            float minY = float.MaxValue, maxY = float.MinValue;

            foreach (var pt in points)
            {
                if (pt.x < minX) minX = pt.x;
                if (pt.x > maxX) maxX = pt.x;
                if (pt.y < minY) minY = pt.y;
                if (pt.y > maxY) maxY = pt.y;
            }

            return Rect.MinMaxRect(minX, minY, maxX, maxY);
        }

    }

}