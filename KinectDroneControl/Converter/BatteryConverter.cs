using System;
using System.Diagnostics;
using Windows.UI.Xaml.Data;

namespace KinectDroneControl.Converter
{
    public class BatteryConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            return value.ToString() + "%";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to BatteryConverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
