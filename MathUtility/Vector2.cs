using System;

namespace PhysicsEngineMathUtility
{
    public struct Vector2
    {
        #region Public Properties

        public readonly double x;
        public readonly double y;

        #endregion

        #region Constructors

        public Vector2(
            double x,
            double y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2(double[] vec)
        {
            if (vec.Length == 2)
            {
                x = vec[0];
                y = vec[1];
            }
            else
            {
                throw new ArgumentException(COMPONENT_EXCEPTION);
            }
        }

        public Vector2(Vector2 v)
        {
            x = v.x;
            y = v.y;
        }


        #endregion

        #region Const

        private const string COMPONENT_EXCEPTION = "Vector must contain three components (x,y,z)";

        //TODO aggiungere eventuali altre eccezioni

        #endregion
    }
}
