using System;

namespace SimulationObjectDefinition
{
	public class AABB
	{
		#region Object fields

		public double MinX { get; private set; }
		public double MaxX { get; private set; }
		public double MinY { get; private set; }
		public double MaxY { get; private set; }
		public double MinZ { get; private set; }
		public double MaxZ { get; private set; }
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
			this.MinX = minX;
			this.MaxX = maxX;
			this.MinY = minY;
			this.MaxY = maxY;
			this.MinZ = minZ;
			this.MaxZ = maxZ;
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

