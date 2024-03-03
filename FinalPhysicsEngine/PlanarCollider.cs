using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public class PlanarCollider
	{
		public readonly float Area;

		public readonly float StaticFriction;
		public readonly float DynamicFriction;

		public PlanarAABB AABB
		{
			get;
			private set;
		}

		public PlanarCollider(float staticFriction, float dynamicFriction, float area)
		{
			//0.6
			StaticFriction = staticFriction;
			//0.4
			DynamicFriction = dynamicFriction;

			Area = area;
		}

		public void ComputeAABB(PlanarTransform planarTransform)
		{
			float minX = float.MaxValue;
			float minY = float.MaxValue;
			float maxX = float.MinValue;
			float maxY = float.MinValue;

			PlanarVector position = planarTransform.Position;
			if (this is PlanarPolygonCollider polygonCollider)
			{
				PlanarVector[] vertices = polygonCollider.GetTransformedVertices(planarTransform);

				for(int i = 0; i < vertices.Length; i++)
				{
					PlanarVector v = vertices[i];

					if (v.X < minX) { minX = v.X; }
					if (v.X > maxX) { maxX = v.X; }
					if (v.Y < minY) { minY = v.Y; }
					if (v.Y > maxY) { maxY = v.Y; }
				}
			}
			else if (this is PlanarCircleCollider circleCollider)
			{
				float Radius = circleCollider.Radius;
				minX = position.X - Radius;
				minY = position.Y - Radius;
				maxX = position.X + Radius;
				maxY = position.Y + Radius;
			}

			AABB = new PlanarAABB(minX, minY, maxX, maxY);
		}
	}
}
