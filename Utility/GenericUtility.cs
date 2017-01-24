using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Utility
{
    public static class GenericUtility
    {
        public static double InvariantCultureDoubleConverter(string value)
        {
            string text = value.Replace(',', '.');
            double tmp;
            double.TryParse(text, NumberStyles.Any, CultureInfo.InvariantCulture, out tmp);
            return tmp;
        }
    }
}
