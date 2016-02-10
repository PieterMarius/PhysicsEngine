using System;
using ObjectDefinition;

namespace CollisionEngine
{
	public class SweepAndPrune
	{
		#region Object fields

		struct EndPoint
		{
			public int index;
			public float mValue;
			public bool mIsMin;
		};

		struct Box
		{
			public EndPoint[] mMin;
			public EndPoint[] mMax;
		};

		#endregion

		#region Constructor

		public SweepAndPrune (ObjectGeometry[] objects)
		{
			
		}

		#endregion
	}
}

