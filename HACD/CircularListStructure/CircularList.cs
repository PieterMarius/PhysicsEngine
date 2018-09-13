using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HACD.CircularListStructure
{
    public class CircularList<T>
    {
        #region Private Fields

        CircularListElement<T> m_head;
        int m_size;
        //HeapManager

        #endregion

        #region Constructor

        #endregion


        //        public:
        //        HeapManager* const GetHeapManager() const { return m_heapManager;}
        //    void SetHeapManager(HeapManager* const heapManager) { m_heapManager = heapManager; }
        //    CircularListElement<T>*  &             GetHead() { return m_head; }
        //    const CircularListElement<T>* GetHead() const { return m_head;}
        //bool IsEmpty() const { return (m_size == 0);}
        //		size_t GetSize() const { return m_size; }
        //		const T &                               GetData() const { return m_head->GetData(); }        
        //		T &                                     GetData() { return m_head->GetData(); }
        //bool Delete();
        //bool Delete(CircularListElement<T>* element);
        //CircularListElement<T>* Add(const T* data = 0);
        //CircularListElement<T>* Add(const T & data);
        //bool Next();
        //bool Prev();
        //void Clear() { while (Delete()) ; };
        //const CircularList&						operator=(const CircularList& rhs);
        ////!	Constructor											
        //CircularList(HeapManager* heapManager)
        //{
        //    m_head = 0;
        //    m_size = 0;
        //    m_heapManager = heapManager;
        //}
        //CircularList(const CircularList& rhs);
        ////! Destructor
        //virtual ~CircularList(void) {Clear();};
        //	private:
        //		CircularListElement<T>* m_head;     //!< a pointer to the head of the circular list
        //size_t m_size;      //!< number of element in the circular list
        //HeapManager* m_heapManager;


    }
}
