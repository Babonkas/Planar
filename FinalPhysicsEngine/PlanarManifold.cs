using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public readonly struct PlanarManifold
	{
		public readonly PlanarRigidbody BodyA;
		public readonly PlanarRigidbody BodyB;
		public readonly float Depth;
		public readonly PlanarVector Normal;
		public readonly PlanarVector Contact1;
		public readonly PlanarVector Contact2;
		public readonly int ContactCount;

		public PlanarManifold(
			PlanarRigidbody bodyA, PlanarRigidbody bodyB,
			PlanarVector normal, float depth,
			PlanarVector contact1, PlanarVector contact2, int contactCount)
		{
			BodyA = bodyA;
			BodyB = bodyB;
			Normal = normal;
			Depth = depth;
			Contact1 = contact1;
			Contact2 = contact2;
			ContactCount = contactCount;
		}
	}
}
