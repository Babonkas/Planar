using Microsoft.VisualBasic;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public static class Collisions
	{

		public static void PointSegmentDistance(PlanarVector p, PlanarVector a, PlanarVector b, out float distanceSquared, out PlanarVector cp)
		{
			PlanarVector ab = b - a;
			PlanarVector ap = p - a;

			float proj = PlanarMath.Dot(ap, ab);

			float abLenSq = PlanarMath.LengthSquared(ab);
			float d = proj / abLenSq;

			if(d <= 0f)
			{
				cp = a;
			}
			else if(d >= 1f)
			{
				cp = b;
			}
			else
			{
				cp = a + ab * d;
			}
			distanceSquared = PlanarMath.DistanceSquared(p, cp);
		}

		public static bool IntersectAABBs(PlanarAABB a, PlanarAABB b)
		{
			if(a.Max.X <= b.Min.X || b.Max.X <= a.Min.X ||
				a.Max.Y <= b.Min.Y || b.Max.Y <= a.Min.Y)
			{
				return false;
			}
			return true;
		}
		
		public static void FindContactPoints(PlanarRigidbody bodyA, PlanarRigidbody bodyB, 
			out PlanarVector contact1, out PlanarVector contact2, out int contactCount)
		{
			contact1 = PlanarVector.Zero;
			contact2 = PlanarVector.Zero;
			contactCount = 0;

			//ShapeType shapeTypeA = bodyA.ShapeType;
			//ShapeType shapeTypeB = bodyB.ShapeType;

			if (bodyA.Collider is PlanarPolygonCollider polygonColliderA)
			{
				if (bodyB.Collider is PlanarPolygonCollider polygonColliderB)
				{
					//Console.WriteLine("TESTSTTTT");
					Collisions.FindPolygonsContactPoints(
						polygonColliderA.GetTransformedVertices(bodyA.PlanarTransform), polygonColliderB.GetTransformedVertices(bodyB.PlanarTransform),
						out contact1, out contact2, out contactCount);
				}
				else if (bodyB.Collider is PlanarCircleCollider circleColliderB)
				{
					Collisions.FindCirclePolygonContactPoint(bodyB.Position, circleColliderB.Radius, bodyA.Position, polygonColliderA.GetTransformedVertices(bodyA.PlanarTransform), out contact1);
					contactCount = 1;
				}
			}
			else if (bodyA.Collider is PlanarCircleCollider circleColliderA)
			{
				if (bodyB.Collider is PlanarPolygonCollider polygonColliderB)
				{
					Collisions.FindCirclePolygonContactPoint(bodyA.Position, circleColliderA.Radius, bodyB.Position, polygonColliderB.GetTransformedVertices(bodyB.PlanarTransform), out contact1);
					contactCount = 1;
				}
				else if (bodyB.Collider is PlanarCircleCollider circleColliderB)
				{
					Collisions.FindCirclesContactPoint(bodyA.Position, circleColliderA.Radius, bodyB.Position, out contact1);
					contactCount = 1;
				}
			}
		}
		private static void FindPolygonsContactPoints(
			PlanarVector[] verticesA, PlanarVector[] verticesB,
			out PlanarVector contact1, out PlanarVector contact2, out int contactCount)
		{
			contact1 = PlanarVector.Zero;
			contact2 = PlanarVector.Zero;
			contactCount = 0;

			float minDistSq = float.MaxValue;

			for (int i = 0; i < verticesA.Length; i++)
			{
				PlanarVector p = verticesA[i];

				for (int j = 0; j < verticesB.Length; j++)
				{
					PlanarVector va = verticesB[j];
					PlanarVector vb = verticesB[(j + 1) % verticesB.Length];

					Collisions.PointSegmentDistance(p, va, vb, out float distSq, out PlanarVector cp);

					if (PlanarMath.NearlyEqual(distSq, minDistSq))
					{
						if (!PlanarMath.NearlyEqual(cp, contact1))
						{
							contactCount = 2;
							contact2 = cp;
						}
					}
					else if (distSq < minDistSq)
					{
						minDistSq = distSq;
						contactCount = 1;
						contact1 = cp;
					}
				}
			}

			for (int i = 0; i < verticesB.Length; i++)
			{
				PlanarVector p = verticesB[i];

				for (int j = 0; j < verticesA.Length; j++)
				{
					PlanarVector va = verticesA[j];
					PlanarVector vb = verticesA[(j + 1) % verticesA.Length];

					Collisions.PointSegmentDistance(p, va, vb, out float distSq, out PlanarVector cp);

					if (PlanarMath.NearlyEqual(distSq, minDistSq))
					{
						if (!PlanarMath.NearlyEqual(cp, contact1))
						{
							contactCount = 2;
							contact2 = cp;
						}
					}
					else if (distSq < minDistSq)
					{
						minDistSq = distSq;
						contactCount = 1;
						contact1 = cp;
					}
				}
			}
		}
		private static void FindCirclePolygonContactPoint(
			PlanarVector circleCenter, float circleRadius, 
			PlanarVector polygonCenter, PlanarVector[] polygonVertices,
			out PlanarVector cp)
		{
			cp = PlanarVector.Zero;
			float minDistSq = float.MaxValue;
			for(int i = 0; i < polygonVertices.Length; i++)
			{
				PlanarVector va = polygonVertices[i];
				PlanarVector vb = polygonVertices[(i+1) % polygonVertices.Length];

				Collisions.PointSegmentDistance(circleCenter, va, vb, out float distSq, out PlanarVector contact);

				if(distSq < minDistSq)
				{
					minDistSq = distSq;
					cp = contact;
				}
			}
		}
		private static void FindCirclesContactPoint(PlanarVector centerA, float radiusA, PlanarVector centerB, out PlanarVector cp)
		{
			PlanarVector ab = centerB - centerA;
			PlanarVector dir = PlanarMath.Normalize(ab);
			cp = centerA +dir * radiusA;
		}

		public static bool Collide(PlanarRigidbody bodyA, PlanarRigidbody bodyB, out PlanarVector normal, out float depth)
		{
					//Console.WriteLine("colliding");
			normal = PlanarVector.Zero;
			depth = 0f;

			//PlanarCollider colliderA = bodyA.Collider;
			//PlanarCollider colliderB = bodyB.Collider;


			if (bodyA.Collider is PlanarPolygonCollider polygonColliderA)
			{
				if (bodyB.Collider is PlanarPolygonCollider polygonColliderB)
				{
					//Console.WriteLine(bodyA.ShapeType + " : " + bodyB.ShapeType);
					bool result = Collisions.IntersectPolygons(
						bodyA.Position, polygonColliderA.GetTransformedVertices(bodyA.PlanarTransform),
						bodyB.Position, polygonColliderB.GetTransformedVertices(bodyB.PlanarTransform),
						out normal, out depth);
					//Console.WriteLine(result);
					return result;
				}
				else if (bodyB.Collider is PlanarCircleCollider circleColliderB)
				{
					bool result = Collisions.IntersectCirclePolygon(
						bodyB.Position, circleColliderB.Radius,
						bodyA.Position, polygonColliderA.GetTransformedVertices(bodyA.PlanarTransform),
						out normal, out depth);
					normal = -normal;
					return result;
				}
			}
			else if (bodyA.Collider is PlanarCircleCollider circleColliderA)
			{
				if (bodyB.Collider is PlanarPolygonCollider polygonColliderB)
				{
					return Collisions.IntersectCirclePolygon(
						bodyA.Position, circleColliderA.Radius,
						bodyB.Position, polygonColliderB.GetTransformedVertices(bodyB.PlanarTransform),
						out normal, out depth);
				}
				else if (bodyB.Collider is PlanarCircleCollider circleColliderB)
				{
					return Collisions.IntersectCircles(
						bodyA.Position, circleColliderA.Radius,
						bodyB.Position, circleColliderB.Radius,
						out normal, out depth);
				}
			}
			return false;
		}

		public static bool IntersectCirclePolygon(PlanarVector circleCenter, float circleRadius,
			PlanarVector polygonCenter, PlanarVector[] vertices,
			out PlanarVector normal, out float depth)
		{
			normal = PlanarVector.Zero;
			depth = float.MaxValue;

			PlanarVector axis = PlanarVector.Zero;
			float axisDepth = 0f;
			float minA, maxA, minB, maxB;

			for (int i = 0; i < vertices.Length; i++)
			{
				PlanarVector va = vertices[i];
				PlanarVector vb = vertices[(i + 1) % vertices.Length];

				PlanarVector edge = vb - va;
				axis = new PlanarVector(-edge.Y, edge.X);
				axis = PlanarMath.Normalize(axis);

				Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
				Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if (axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}
			int cpIndex = Collisions.FindClosestPointOnPolygon(circleCenter, vertices);
			PlanarVector cp = vertices[cpIndex];

			axis = cp - circleCenter;
			axis = PlanarMath.Normalize(axis);

			Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
			Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

			if (minA >= maxB || minB >= maxA)
			{
				return false;
			}

			axisDepth = MathF.Min(maxB - minA, maxA - minB);

			if (axisDepth < depth)
			{
				depth = axisDepth;
				normal = axis;
			}

			PlanarVector direction = polygonCenter - circleCenter;

			if (PlanarMath.Dot(direction, normal) < 0f)
			{
				normal = -normal;
			}


			return true;
		}

		private static int FindClosestPointOnPolygon(PlanarVector circleCenter, PlanarVector[] vertices)
		{
			int result = -1;
			float minDistance = float.MaxValue;

			for(int i = 0; i < vertices.Length; i++)
			{
				PlanarVector v = vertices[i];
				float distance = PlanarMath.Distance(v, circleCenter);

				if(distance < minDistance)
				{
					minDistance = distance;
					result = i;
				}
			}
			return result;
		}

		private static void ProjectCircle(PlanarVector center, float radius, PlanarVector axis, out float min, out float max)
		{
			PlanarVector direction = PlanarMath.Normalize(axis);
			PlanarVector directionAndRadius = direction*radius;

			PlanarVector p1 = center + directionAndRadius;
			PlanarVector p2 = center - directionAndRadius;

			min = PlanarMath.Dot(p1, axis);
			max = PlanarMath.Dot(p2, axis);

			if(min > max)
			{
				//swap
				float t = min;
				min = max;
				max = t;
			}
		}

		public static bool IntersectPolygons(PlanarVector centerA, PlanarVector[] verticesA, PlanarVector centerB, PlanarVector[] verticesB, out PlanarVector normal, out float depth)
		{
			normal = PlanarVector.Zero;
			depth = float.MaxValue;

			for(int i = 0; i < verticesA.Length; i++)
			{
				PlanarVector va = verticesA[i];
				PlanarVector vb = verticesA[(i + 1) % verticesA.Length];

				PlanarVector edge = vb - va;
				PlanarVector axis = new PlanarVector(-edge.Y, edge.X);
				axis = PlanarMath.Normalize(axis);


				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

				if(minA >= maxB || minB >= maxA)
				{
					//Console.WriteLine("efasdfasffef");
					return false;
				}

				float axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if(axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}

			for (int i = 0; i < verticesB.Length; i++)
			{
				PlanarVector va = verticesB[i];
				PlanarVector vb = verticesB[(i + 1) % verticesB.Length];

				PlanarVector edge = vb - va;
				PlanarVector axis = new PlanarVector(-edge.Y, edge.X);
				axis = PlanarMath.Normalize(axis);

				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

				if (minA >= maxB || minB >= maxA)
				{
					//Console.WriteLine("efef");
					return false;
				}

				float axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if (axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}

			PlanarVector direction = centerB - centerA;

			if(PlanarMath.Dot(direction, normal) < 0f)
			{
				normal = -normal;
			}

			return true;
		}

		private static void ProjectVertices(PlanarVector[] vertices, PlanarVector axis, out float min, out float max)
		{
			min = float.MaxValue;
			max = float.MinValue;
			for(int i = 0; i < vertices.Length; i++)
			{
				PlanarVector v = vertices[i];
				float proj = PlanarMath.Dot(v, axis);

				if (proj < min) { min = proj; }
				if (proj > max) { max = proj; }
			}
		}


		public static bool IntersectCircles(PlanarVector centerA, float radiusA, PlanarVector centerB, float radiusB,
			out PlanarVector normal, out float depth)
		{
			normal = PlanarVector.Zero;
			depth = 0;

			float distance = PlanarMath.Distance(centerA, centerB);
			float radii = radiusA + radiusB;
			
			if(distance >= radii)
			{
				return false;
			}

			normal = PlanarMath.Normalize(centerB - centerA);
			depth = radii - distance;

			return true;
		}


	}
}
