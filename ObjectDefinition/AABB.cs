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
			this.Min = new double[3];
			this.Max = new double[3];
			this.Min[0] = minX;
			this.Max[0] = maxX;
			this.Min[1] = minY;
			this.Max[1] = maxY;
			this.Min[2] = minZ;
			this.Max[2] = maxZ;
			this.positionAABBChanged = positionChanged;
		}

		#endregion

		#region Public methods

		public void SetPositionChanged(bool value)
		{
			this.positionAABBChanged = value; 
		}

		#endregion
	}
}

