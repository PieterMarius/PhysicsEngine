using System;
using System.Collections.Generic;
using ObjectDefinition;

namespace MonoPhysicsEngine
{
	public class SpatialPartition
	{
		#region Public Fields

		public List<CollisionPointStructure> ObjectList;

		#endregion

		#region Constructor

		public SpatialPartition (
			List<CollisionPointStructure> objectList)
		{
			this.ObjectList = objectList;
		}

		public SpatialPartition()
		{
		}

		#endregion
	}
}

