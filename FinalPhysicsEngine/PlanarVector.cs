using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public readonly struct PlanarVector
	{
		public readonly float X;
		public readonly float Y;

		public static readonly PlanarVector Zero = new PlanarVector(0f, 0f);

		public PlanarVector(float x, float y)
		{
			X = x;
			Y = y;
		}

		public static PlanarVector operator +(PlanarVector a, PlanarVector b)
		{
			return new PlanarVector(a.X + b.X, a.Y + b.Y);
		}
		public static PlanarVector operator -(PlanarVector a, PlanarVector b)
		{
			return new PlanarVector(a.X - b.X, a.Y - b.Y);
		}
		public static PlanarVector operator -(PlanarVector v)
		{
			return new PlanarVector(-v.X, -v.Y);
		}


		public static PlanarVector operator *(PlanarVector v, float s)
		{
			return new PlanarVector(v.X * s, v.Y * s);
		}
		public static PlanarVector operator *(float s, PlanarVector v)
		{
			return new PlanarVector(v.X * s, v.Y * s);
		}
		public static PlanarVector operator /(PlanarVector v, float s)
		{
			return new PlanarVector(v.X / s, v.Y / s);
		}

		public bool Equals(PlanarVector other)
		{
			return X == other.X && Y == other.Y;
		}

		internal static PlanarVector Transform(PlanarVector v, PlanarTransform transform)
		{

			return new PlanarVector(
				transform.Cos * v.X - transform.Sin * v.Y + transform.Position.X, 
				transform.Sin * v.X + transform.Cos * v.Y + transform.Position.Y);
		}


		public override bool Equals(object obj)
		{
			if(obj is PlanarVector other)
			{
				return Equals(other);
			}
			return false;
		}
		public override int GetHashCode()
		{
			return new {X, Y}.GetHashCode();
		}
		public override string ToString()
		{
			return $"X: {X}, Y: {Y}";
		}
	}
}
