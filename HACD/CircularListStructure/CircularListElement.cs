using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HACD.CircularListStructure
{
    public class CircularListElement<T>
    {
        #region Private Fields

        private T m_data;
        private CircularListElement<T> m_next;
        private CircularListElement<T> m_prev;

        #endregion

        #region Constructor

        private CircularListElement(T data)
        {
            m_data = data;
        }

        private CircularListElement(CircularListElement<T> rhs)
        { }

        #endregion

        #region Public Methods

        public T GetData()
        {
            return m_data;
        }

        public CircularListElement<T> GetNext()
        {
            return m_next;
        }

        public CircularListElement<T> GetPrev()
        {
            return m_prev;
        }

        #endregion
    }
}
