using System;
using System.Diagnostics;
using Windows.UI.Xaml.Data;

namespace KinectDroneControl.Converter
{
    public class FlyingConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return (bool)value ? "LAND" : "TAKE OFF";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to FlyingConverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
