using System;

namespace Utility
{
	public class ArrayUtility
	{
		public ArrayUtility ()
		{
		}

		public T[] InitializeArray<T>(int length) where T : new()
		{
			T[] array = new T[length];
			for (int i = 0; i < length; ++i)
			{
				array[i] = new T();
			}

			return array;
		}
	}
}

