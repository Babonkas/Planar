using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public readonly struct PlanarAABB
	{
		public readonly PlanarVector Min; 
		public readonly PlanarVector Max;

		public PlanarAABB(PlanarVector min, PlanarVector max)
		{
			Min = min;
			Max = max;
		}
		public PlanarAABB(float minX, float minY, float maxX, float maxY)
		{
			Min = new PlanarVector(minX, minY);
			Max = new PlanarVector(maxX, maxY);
		}
	}
}
