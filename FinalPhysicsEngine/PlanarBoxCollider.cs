using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public sealed class PlanarBoxCollider : PlanarPolygonCollider
	{
		public float Width
		{
			get;
		}
		public float Height
		{
			get;
		}
		public PlanarBoxCollider(float width, float height, float staticFriction, float dynamicFriction)
			: base(CreateBoxVertices(width, height), CreateBoxTriangles(), staticFriction, dynamicFriction, width * height)
		{
			Width = width;
			Height = height;
		}
		private static PlanarVector[] CreateBoxVertices(float width, float height)
		{
			float left = -width / 2f;
			float right = left + width;
			float bottom = -height / 2f;
			float top = bottom + height;

			PlanarVector[] vertices = new PlanarVector[4];
			vertices[0] = new PlanarVector(left, top);
			vertices[1] = new PlanarVector(right, top);
			vertices[2] = new PlanarVector(right, bottom);
			vertices[3] = new PlanarVector(left, bottom);

			return vertices;
		}

		private static int[] CreateBoxTriangles()
		{
			int[] triangles = new int[6];
			triangles[0] = 0;
			triangles[1] = 1;
			triangles[2] = 2;
			triangles[3] = 0;
			triangles[4] = 2;
			triangles[5] = 3;
			return triangles;
		}
	}
}
