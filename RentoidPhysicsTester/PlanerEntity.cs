using Flat;
using Flat.Graphics;
using Microsoft.Xna.Framework;
using PlanarPhysicsEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsTester
{
	public sealed class PlanarEntity
	{
		public readonly PlanarRigidbody Body;
		public readonly Color Color;

		public PlanarEntity(PlanarRigidbody body)
		{
			this.Body = body;
			this.Color = RandomHelper.RandomColor();
		}
		public PlanarEntity(PlanarRigidbody body, Color color)
		{
			this.Body = body;
			this.Color = color;
		}
		public void Draw(Shapes shapes)
		{
			Vector2 position = PlanarConverter.ToVector2(Body.Position);

			if (Body.Collider is PlanarCircleCollider circleCol)
			{
				Vector2 va = Vector2.Zero;
				Vector2 vb = new Vector2(circleCol.Radius, 0);
				Flat.FlatTransform transform = new Flat.FlatTransform(position, Body.Angle, 1);
				va = Flat.FlatUtil.Transform(va, transform);
				vb = Flat.FlatUtil.Transform(vb, transform);

				shapes.DrawCircleFill(position, circleCol.Radius, 26, this.Color);
				shapes.DrawCircle(position, circleCol.Radius, 26, Color.White);
				shapes.DrawLine(va, vb, Color.White);
			}
			else if(Body.Collider is PlanarBoxCollider boxCol)
			{
				shapes.DrawBoxFill(position, boxCol.Width, boxCol.Height, Body.Angle, this.Color);
				shapes.DrawBox(position, boxCol.Width, boxCol.Height, Body.Angle, Color.White);
			}
			else if(Body.Collider is PlanarPolygonCollider polygonCol)
			{
				PlanarVector[] tranVertices = polygonCol.GetTransformedVertices(Body.PlanarTransform);
				Vector2[] tranVerticesV2 = new Vector2[tranVertices.Length];

				for (int i = 0; i < tranVertices.Length; i++)
				{
					tranVerticesV2[i] = PlanarConverter.ToVector2(tranVertices[i]);
				}
				shapes.DrawPolygonFill(tranVerticesV2, polygonCol.Triangles, this.Color);
				//shapes.DrawPolygon(tranVerticesV2, Color.White);
				shapes.DrawPolygonTriangles(tranVerticesV2, polygonCol.Triangles, Color.White);

			}
		}
	}
}
