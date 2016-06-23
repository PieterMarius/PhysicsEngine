using System;

namespace SimulationObjectDefinition
{
	public class AABB
	{

		#region Object fields

		public double[] Min;
		public double[] Max;
		public bool positionAABBChanged { get; private set; }

		#endregion

		#region Constructor

		public AABB (
			double minX,
			double maxX,
			double minY,
			double maxY,
			double minZ,
			double maxZ,
			bool positionChanged)
		{
			Min = new double[3];
			Max = new double[3];
			Min[0] = minX;
			Max[0] = maxX;
			Min[1] = minY;
			Max[1] = maxY;
			Min[2] = minZ;
			Max[2] = maxZ;
			positionAABBChanged = positionChanged;
		}

		#endregion

		#region Public methods

		public void SetPositionChanged(bool value)
		{
			positionAABBChanged = value; 
		}

		#endregion
	}
}

