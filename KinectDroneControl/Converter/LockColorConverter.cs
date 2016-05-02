using System;
using System.Diagnostics;
using Windows.UI;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Media;

namespace KinectDroneControl.Converter
{
    public class LockColorConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return new SolidColorBrush((bool)value ? Colors.Red : Colors.Transparent);
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to LockColorConverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
