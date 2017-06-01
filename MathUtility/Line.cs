
namespace SharpEngineMathUtility
{
    public struct Line
    {
        #region Public Properties

        public readonly Vector3 a;
        public readonly Vector3 b;
        
        #endregion

        #region Constructors

        public Line(
            Vector3 a,
            Vector3 b)
        {
            this.a = a;
            this.b = b;
        }

        #endregion
    }
}
