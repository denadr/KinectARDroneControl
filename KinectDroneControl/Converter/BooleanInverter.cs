using System;
using System.Diagnostics;
using Windows.UI.Xaml.Data;

namespace KinectDroneControl.Converter
{
    public class BooleanInverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return !((bool)value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to BooleanInverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
