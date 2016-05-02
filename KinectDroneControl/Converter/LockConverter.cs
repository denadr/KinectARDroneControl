using System;
using System.Diagnostics;
using Windows.UI.Xaml.Data;

namespace KinectDroneControl.Converter
{
    public class LockConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return (bool)value ? "\uE1F6" : "\uE1F7";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to LockConverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
