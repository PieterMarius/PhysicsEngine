
namespace SharpPhysicsEngine.ShapeDefinition
{
	public sealed class StartImpulseProperties
	{
		#region Fields

		public double StartImpulseValue { get; private set; }

		#endregion

		#region Constructor

		public StartImpulseProperties(
			double startImpulseValue)
		{
			StartImpulseValue = startImpulseValue;
		}

		#endregion

		#region Public Properties

		public void SetStartValue(double startValue)
		{
			StartImpulseValue = startValue;
		}

		#endregion

	}
}

