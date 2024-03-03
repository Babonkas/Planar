using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public sealed class PlanarCircleCollider : PlanarCollider
	{
		public float Radius
		{
			get;
			private set;
		}
		public PlanarCircleCollider(float radius, float staticFriction, float dynamicFriction)
			: base(staticFriction, dynamicFriction, radius*radius*MathF.PI)
		{
			Radius = radius;
		}
	}
}
