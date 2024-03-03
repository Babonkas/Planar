using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public readonly struct PlanarTransform
	{
		public readonly PlanarVector Position;
		public readonly float Sin;
		public readonly float Cos;

		public readonly static PlanarTransform Zero = new PlanarTransform(0f, 0f, 0f);

		public PlanarTransform(PlanarVector position, float angle)
		{
			Position = position;
			Sin = MathF.Sin(angle);
			Cos = MathF.Cos(angle);
		}
		public PlanarTransform(float x, float y, float angle)
		{
			Position = new PlanarVector(x, y);
			Sin = MathF.Sin(angle);
			Cos = MathF.Cos(angle);
		}
	}
}
