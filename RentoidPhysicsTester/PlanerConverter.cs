using System;
using System.Runtime.CompilerServices;
using Microsoft.Xna.Framework;
using PlanarPhysicsEngine;

namespace PlanarPhysicsTester
{
	public static class PlanarConverter
	{
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector2 ToVector2(PlanarVector v)
		{
			return new Vector2(v.X, v.Y);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static PlanarVector ToPlanarVector(Vector2 v)
		{
			return new PlanarVector(v.X, v.Y);
		}


		public static void ToVector2Array(PlanarVector[] src, ref Vector2[] dst)
		{
			if(dst is null || src.Length != dst.Length)
			{
				dst = new Vector2[src.Length];
			}

			for(int i = 0; i < src.Length; i++)
			{
				PlanarVector v = src[i];
				dst[i] = new Vector2(v.X, v.Y);
			}
		}

	}
}
