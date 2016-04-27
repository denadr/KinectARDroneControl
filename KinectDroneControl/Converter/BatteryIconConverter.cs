using System;
using System.Diagnostics;
using Windows.UI.Xaml.Data;

namespace KinectDroneControl.Converter
{
    public class BatteryIconConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, string language)
        {
            int percentage = (int)value;

            if      (percentage < 10) return "\uE850";
            else if (percentage < 20) return "\uE851";
            else if (percentage < 30) return "\uE852";
            else if (percentage < 40) return "\uE853";
            else if (percentage < 50) return "\uE854";
            else if (percentage < 60) return "\uE855";
            else if (percentage < 70) return "\uE856";
            else if (percentage < 80) return "\uE857";
            else if (percentage < 90) return "\uE858";
            else if (percentage < 97) return "\uE859";
            else                      return "\uE83F";
        }

        public object ConvertBack(object value, Type targetType, object parameter, string language)
        {
            Debug.WriteLine("Unexpected call to BatteryIconConverter.ConvertBack(object value, Type targetType, object parameter, string language).");
            return null;
        }
    }
}
