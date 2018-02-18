using System.Collections.Generic;

namespace SharpPhysicsEngine
{
	internal class SpatialPartition
	{
		#region Public Fields

		public List<ContactIndex> ObjectList;

		#endregion

		#region Constructor

		public SpatialPartition (
			List<ContactIndex> objectList)
		{
			ObjectList = objectList;
		}

		#endregion
	}
}

