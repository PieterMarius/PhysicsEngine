using System;

namespace MonoPhysicsEngine
{
	public class ObjectConstraint
	{
		#region public properties

		public readonly int IndexA;
		public readonly int IndexB;
		public IConstraint[] ConstraintList { get; set; }

		#endregion

		#region Contructor

		public ObjectConstraint (
			int indexA,
			int indexB,
			IConstraint[] constraintList)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.ConstraintList = constraintList;
		}

		#endregion
	}
}

