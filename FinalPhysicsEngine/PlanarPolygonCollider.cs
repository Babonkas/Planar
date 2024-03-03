using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public class PlanarPolygonCollider : PlanarCollider 
	{
		private PlanarVector[] transformedVertices;



		public int VerticesCount
		{
			get;
			private set;
		}
		public int[] Triangles
		{
			get;
			private set;
		}
		public PlanarVector[] Vertices
		{
			get;
			private set;
		}
		public PlanarPolygonCollider(PlanarVector[] vertices, float staticFriction, float dynamicFriction)
			: base(staticFriction, dynamicFriction, PolygonArea(vertices))
		{
			Vertices = vertices;
			VerticesCount = vertices.Length;
			if(Triangles == null)
			{
				Triangles = CreatePolygonTriangles(Vertices);
			}
			transformedVertices = new PlanarVector[Vertices.Length];
		}
		public PlanarPolygonCollider(PlanarVector[] vertices, int[] triangles, float staticFriction, float dynamicFriction, float area)
			: base(staticFriction, dynamicFriction, area)
		{
			Vertices = vertices;
			VerticesCount = vertices.Length;
			Triangles = triangles;
			transformedVertices = new PlanarVector[Vertices.Length];
		}

		public PlanarVector[] GetTransformedVertices(PlanarTransform transform)
		{
			for (int i = 0; i < Vertices.Length; i++)
			{
				PlanarVector v = Vertices[i];
				transformedVertices[i] = PlanarVector.Transform(v, transform);
			}
			return transformedVertices;
		}



		private static float PolygonArea(PlanarVector[] Vertices)
		{
			// Initialize area
			int n = Vertices.Length;
			float area = 0.0f;

			// Calculate value of shoelace formula
			int j = n - 1;

			for (int i = 0; i < n; i++)
			{
				area += (Vertices[j].X + Vertices[i].X) * (Vertices[j].Y - Vertices[i].Y);

				// j is previous vertex to i
				j = i;
			}

			// Return absolute value
			return MathF.Abs(area / 2.0f);
		}

		private static int[] CreatePolygonTriangles(PlanarVector[] Vertices)
		{


			if (Vertices is null)
			{
				throw new Exception("THE VERTEX LIST IS NULL");

			}
			if (Vertices.Length < 3)
			{
				throw new Exception("TOO LITTLE VERTICES");

			}

			//if (!PlanarCollider.IsSimplePolygon(Vertices))
			//{
			//	errorMessage = "not simple";
			//	return false;
			//}
			//if (!PlanarCollider.ContainsColinearEdges(Vertices))
			//{
			//	errorMessage = "vertex list contains colinear edges";
			//	return false;
			//}



			List<int> indexList = new List<int>();
			for (int i = 0; i < Vertices.Length; i++)
			{
				indexList.Add(i);
			}

			int totalTriangleCount = Vertices.Length - 2;
			int totalTriangleIndexCount = totalTriangleCount * 3;

			int[] Triangles = new int[totalTriangleIndexCount];
			int triangleIndexCount = 0;

			while (indexList.Count > 3)
			{
				for (int i = 0; i < indexList.Count; i++)
				{
					int a = indexList[i];
					int b = PlanarMath.GetItem(indexList, i - 1);
					int c = PlanarMath.GetItem(indexList, i + 1);

					PlanarVector va = Vertices[a];
					PlanarVector vb = Vertices[b];
					PlanarVector vc = Vertices[c];

					PlanarVector va_to_vb = vb - va;
					PlanarVector va_to_vc = vc - va;


					//is ear test convex
					if (PlanarMath.Cross(va_to_vb, va_to_vc) < 0f)
					{
						continue;
					}

					bool isEar = true;


					//does test ear have polygon Vertices
					for (int j = 0; j < Vertices.Length; j++)
					{
						if (j == a || j == b || j == c)
						{
							continue;
						}
						PlanarVector p = Vertices[j];

						if (PlanarPolygonCollider.IsPointInTriangle(p, vb, va, vc))
						{
							isEar = false;
							break;
						}

					}
					if (isEar)
					{
						Triangles[triangleIndexCount++] = b;
						Triangles[triangleIndexCount++] = a;
						Triangles[triangleIndexCount++] = c;

						indexList.RemoveAt(i);
						break;
					}

				}
			}

			Triangles[triangleIndexCount++] = indexList[0];
			Triangles[triangleIndexCount++] = indexList[1];
			Triangles[triangleIndexCount++] = indexList[2];

			return Triangles;

		}
		public static bool IsPointInTriangle(PlanarVector p, PlanarVector a, PlanarVector b, PlanarVector c)
		{
			PlanarVector ab = b - a;
			PlanarVector bc = c - b;
			PlanarVector ca = a - c;

			PlanarVector ap = p - a;
			PlanarVector bp = p - b;
			PlanarVector cp = p - c;

			float cross1 = PlanarMath.Cross(ab, ap);
			float cross2 = PlanarMath.Cross(bc, bp);
			float cross3 = PlanarMath.Cross(ca, cp);

			if (cross1 > 0f || cross2 > 0f || cross3 > 0f)
			{
				return false;
			}
			return true;
		}

		public static bool ContainsColinearEdges(PlanarVector[] Vertices)
		{
			throw new NotImplementedException();
		}

		public static bool IsSimplePolygon(PlanarVector[] Vertices)
		{
			throw new NotImplementedException();
		}
	}
}
