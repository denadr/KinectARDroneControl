using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using ARDrone2Client.Common;
using KinectDroneControl.Extensions.Kinect;
using KinectDroneControl.KinectDroneInterface;
using Windows.ApplicationModel.Resources;
using Windows.Foundation;
using Windows.UI;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Shapes;
using WindowsPreview.Kinect;

namespace KinectDroneControl.Views
{
    public sealed partial class MainPage : Page, INotifyPropertyChanged
    {
        private ResourceLoader m_ResourceLoader = ResourceLoader.GetForCurrentView("Resources");
        private const double c_HighConfidenceHandSize = 40;
        private const double c_LowConfidenceHandSize = 20;
        private const double c_JointThickness = 8.0;
        private const double c_TrackedBoneThickness = 4.0;
        private const double c_InferredBoneThickness = 1.0;
        private const double c_ClipBoundsThickness = 5;
        private const float c_InferredZPositionClamp = 0.1f;
        private KinectSensor m_KinectSensor = null;
        private CoordinateMapper m_CoordinateMapper = null;
        private BodyFrameReader m_BodyFrameReader = null;
        private Body[] m_Bodies = null;
        private Canvas m_DrawingCanvas;
        private BodyInfo[] m_BodyInfos;
        private List<Color> m_BodyColors;
        private Rectangle m_LeftClipEdge;
        private Rectangle m_RightClipEdge;
        private Rectangle m_TopClipEdge;
        private Rectangle m_BottomClipEdge;
        private int m_BodyCount
        {
            set
            {
                if (value == 0)
                {
                    m_BodyInfos = null;
                    return;
                }
                // creates instances of BodyInfo objects for potential number of bodies
                if (m_BodyInfos == null || m_BodyInfos.Length != value)
                {
                    m_BodyInfos = new BodyInfo[value];
                    for (int bodyIndex = 0; bodyIndex < m_Bodies.Length; bodyIndex++)
                        m_BodyInfos[bodyIndex] = new BodyInfo(m_BodyColors[bodyIndex]);
                }
            }
            get { return m_BodyInfos == null ? 0 : m_BodyInfos.Length; }
        }
        private float m_JointSpaceWidth;
        private float m_JointSpaceHeight;

        private IGestureTranslator m_Translator = new GestureTranslator();

        private void InitializeKinect()
        {
            m_KinectSensor = KinectSensor.GetDefault();
            m_CoordinateMapper = m_KinectSensor.CoordinateMapper;
            var frameDescription = m_KinectSensor.DepthFrameSource.FrameDescription;// get the depth (display) extents
            // get size of joint space
            m_JointSpaceWidth = frameDescription.Width;
            m_JointSpaceHeight = frameDescription.Height;
            m_Bodies = new Body[m_KinectSensor.BodyFrameSource.BodyCount];// get total number of bodies from BodyFrameSource
            m_BodyFrameReader = m_KinectSensor.BodyFrameSource.OpenReader();
            m_BodyFrameReader.FrameArrived += Reader_BodyFrameArrived;
            m_KinectSensor.IsAvailableChanged += Sensor_IsAvailableChanged;
            m_BodyColors = new List<Color>
            {
                Colors.Red,
                Colors.Orange,
                Colors.Green,
                Colors.Blue,
                Colors.Indigo,
                Colors.Violet
            };
            // sets total number of possible tracked bodies
            // create ellipses and lines for drawing bodies
            m_BodyCount = m_KinectSensor.BodyFrameSource.BodyCount;
            m_DrawingCanvas = new Canvas();
            m_KinectSensor.Open();

            StatusText = m_KinectSensor.IsAvailable ? m_ResourceLoader.GetString("RunningStatusText")
                                                            : m_ResourceLoader.GetString("NoSensorStatusText");
        }

        public event PropertyChangedEventHandler PropertyChanged;
        private void RaisePropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        private string m_StatusText;
        public string StatusText
        {
            get { return m_StatusText; }
            set
            {
                if (m_StatusText != value)
                {
                    m_StatusText = value;
                    RaisePropertyChanged(nameof(StatusText));
                }
            }
        }
        private void MainPage_Unloaded(object sender, RoutedEventArgs e)
        {
            if (m_BodyFrameReader != null)
            {
                m_BodyFrameReader.Dispose();
                m_BodyFrameReader = null;
            }
            if (m_Bodies != null)
            {
                foreach (var body in m_Bodies)
                {
                    if (body != null)
                        body.Dispose();
                }
            }
            if (m_KinectSensor != null)
            {
                m_KinectSensor.Close();
                m_KinectSensor = null;
            }
        }

        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            bool hasTrackedBody = false;
            using (var bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    bodyFrame.GetAndRefreshBodyData(m_Bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                BeginBodiesUpdate();
                for (int bodyIndex = 0; bodyIndex < m_Bodies.Length; bodyIndex++)
                {
                    var body = m_Bodies[bodyIndex];
                    if (body.IsTracked)
                    {
                        // check if this body clips an edge
                        UpdateClippedEdges(body, hasTrackedBody);
                        UpdateBody(body, bodyIndex);

                        hasTrackedBody = true;
                    }
                    else
                        m_BodyInfos[bodyIndex].Clear();// collapse this body from canvas as it goes out of view
                        //ClearBody(bodyIndex);
                }
                if (!hasTrackedBody)
                    ClearClippedEdges();
            }
        }

        private void BeginBodiesUpdate()
        {
            if (m_BodyInfos != null)
            {
                foreach (var bodyInfo in m_BodyInfos)
                {
                    bodyInfo.Updated = false;
                }
            }
        }
        
        private void UpdateHand(Ellipse ellipse, HandState handState, TrackingConfidence trackingConfidence, Point point)
        {
            ellipse.Fill = new SolidColorBrush(HandStateUtilities.HandStateToColor(handState));
            // draw handstate ellipse based on tracking confidence
            ellipse.Width = ellipse.Height = (trackingConfidence == TrackingConfidence.Low) ? c_LowConfidenceHandSize : c_HighConfidenceHandSize;
            ellipse.Visibility = Visibility.Visible;
            if (!double.IsInfinity(point.X) && !Double.IsInfinity(point.Y))// don't draw handstate if hand joints are not tracked
            {
                Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
                Canvas.SetTop(ellipse, point.Y - ellipse.Width / 2);
            }
        }

        private void UpdateJoint(Ellipse ellipse, Joint joint, Point point)
        {
            var trackingState = joint.TrackingState;
            if (trackingState != TrackingState.NotTracked)// only draw if joint is tracked or inferred
            {
                if (trackingState == TrackingState.Tracked)
                    ellipse.Fill = new SolidColorBrush(Colors.Green);
                else
                    ellipse.Fill = new SolidColorBrush(Colors.Yellow);// inferred joints are yellow
                Canvas.SetLeft(ellipse, point.X - c_JointThickness / 2);
                Canvas.SetTop(ellipse, point.Y - c_JointThickness / 2);
                ellipse.Visibility = Visibility.Visible;
            }
            else
                ellipse.Visibility = Visibility.Collapsed;
        }

        private void UpdateBone(Line line, Joint startJoint, Joint endJoint, Point startPoint, Point endPoint)
        {
            // don't draw if neither joints are tracked
            if (startJoint.TrackingState == TrackingState.NotTracked || endJoint.TrackingState == TrackingState.NotTracked)
            {
                line.Visibility = Visibility.Collapsed;
                return;
            }
            line.StrokeThickness = c_InferredBoneThickness;// all lines are inferred thickness unless both joints are tracked
            if (startJoint.TrackingState == TrackingState.Tracked &&
                endJoint.TrackingState == TrackingState.Tracked)
                line.StrokeThickness = c_TrackedBoneThickness;
            line.Visibility = Visibility.Visible;
            line.X1 = startPoint.X;
            line.Y1 = startPoint.Y;
            line.X2 = endPoint.X;
            line.Y2 = endPoint.Y;
        }

        private void UpdateClippedEdges(Body body, bool hasTrackedBody)
        {
            var clippedEdges = body.ClippedEdges;
            if (clippedEdges.HasFlag(FrameEdges.Left))
                m_LeftClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)// don't clear this edge if another body is triggering clipped edge
                m_LeftClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Right))
                m_RightClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_RightClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Top))
                m_TopClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_TopClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
                m_BottomClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_BottomClipEdge.Visibility = Visibility.Collapsed;
        }

        private void ClearClippedEdges()
        {
            m_LeftClipEdge.Visibility = Visibility.Collapsed;
            m_RightClipEdge.Visibility = Visibility.Collapsed;
            m_TopClipEdge.Visibility = Visibility.Collapsed;
            m_BottomClipEdge.Visibility = Visibility.Collapsed;
        }
        
        private void PopulateVisualObjects()
        {
            // create clipped edges and set to collapsed initially
            m_LeftClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = c_ClipBoundsThickness,
                Height = DisplayGrid.Height,
                Visibility = Visibility.Collapsed
            };
            m_RightClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = c_ClipBoundsThickness,
                Height = DisplayGrid.Height,
                Visibility = Visibility.Collapsed
            };
            m_TopClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = DisplayGrid.Width,
                Height = c_ClipBoundsThickness,
                Visibility = Visibility.Collapsed
            };
            m_BottomClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = DisplayGrid.Width,
                Height = c_ClipBoundsThickness,
                Visibility = Visibility.Collapsed
            };
            foreach (var bodyInfo in m_BodyInfos)
            {
                // add left and right hand ellipses of all bodies to canvas
                m_DrawingCanvas.Children.Add(bodyInfo.HandLeftEllipse);
                m_DrawingCanvas.Children.Add(bodyInfo.HandRightEllipse);
                foreach (var joint in bodyInfo.JointPoints)// add joint ellipses of all bodies to canvas
                {
                    m_DrawingCanvas.Children.Add(joint.Value);
                }
                foreach (var bone in bodyInfo.Bones)// add bone lines of all bodies to canvas
                {
                    m_DrawingCanvas.Children.Add(bodyInfo.BoneLines[bone]);
                }
            }
            // add clipped edges rectanges to main canvas
            m_DrawingCanvas.Children.Add(m_LeftClipEdge);
            m_DrawingCanvas.Children.Add(m_RightClipEdge);
            m_DrawingCanvas.Children.Add(m_TopClipEdge);
            m_DrawingCanvas.Children.Add(m_BottomClipEdge);
            // position the clipped edges
            Canvas.SetLeft(m_LeftClipEdge, 0);
            Canvas.SetTop(m_LeftClipEdge, 0);
            Canvas.SetLeft(m_RightClipEdge, DisplayGrid.Width - c_ClipBoundsThickness);
            Canvas.SetTop(m_RightClipEdge, 0);
            Canvas.SetLeft(m_TopClipEdge, 0);
            Canvas.SetTop(m_TopClipEdge, 0);
            Canvas.SetLeft(m_BottomClipEdge, 0);
            Canvas.SetTop(m_BottomClipEdge, DisplayGrid.Height - c_ClipBoundsThickness);
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (!m_KinectSensor.IsAvailable)
                StatusText = m_ResourceLoader.GetString("SensorNotAvailableStatusText");
            else
                StatusText = m_ResourceLoader.GetString("RunningStatusText");
        }

        private void UpdateBody(Body body, int bodyIndex)
        {
            var joints = body.Joints;
            var jointPointsInDepthSpace = new Dictionary<JointType, Point>();
            var bodyInfo = m_BodyInfos[bodyIndex];
            var coordinateMapper = m_KinectSensor.CoordinateMapper;
            foreach (var jointType in body.Joints.Keys)
            {
                // sometimes the depth(Z) of an inferred joint may show as negative
                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                var position = body.Joints[jointType].Position;
                if (position.Z < 0)
                    position.Z = c_InferredZPositionClamp;
                // map joint position to depth space
                var depthSpacePoint = coordinateMapper.MapCameraPointToDepthSpace(position);
                jointPointsInDepthSpace[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                // modify the joint's visibility and location
                UpdateJoint(bodyInfo.JointPoints[jointType], joints[jointType], jointPointsInDepthSpace[jointType]);
                // modify hand ellipse colors based on hand states
                // modity hand ellipse sizes based on tracking confidences
                if (jointType == JointType.HandRight)
                    UpdateHand(bodyInfo.HandRightEllipse, body.HandRightState, body.HandRightConfidence, jointPointsInDepthSpace[jointType]);
                if (jointType == JointType.HandLeft)
                    UpdateHand(bodyInfo.HandLeftEllipse, body.HandLeftState, body.HandLeftConfidence, jointPointsInDepthSpace[jointType]);
            }
            foreach (var bone in bodyInfo.Bones)
            {
                UpdateBone(bodyInfo.BoneLines[bone], joints[bone.Item1], joints[bone.Item2],
                                jointPointsInDepthSpace[bone.Item1],
                                jointPointsInDepthSpace[bone.Item2]);
            }
            
            if (m_Connected && !m_Master) // convert the gestures of the body and send them as a command to the drone
                SendGesturesToDrone(m_Translator.Translate(body.HandLeftState, body.HandRightState, jointPointsInDepthSpace, m_Flying));
        }
      
        public MainPage()
        {
            InitializeDrone();
            InitializeKinect();
            DataContext = this;
            InitializeComponent();
            m_DrawingCanvas.Clip = new RectangleGeometry();// set the clip rectangle to prevent rendering outside the canvas
            m_DrawingCanvas.Clip.Rect = new Rect(0.0, 0.0, DisplayGrid.Width, DisplayGrid.Height);
            PopulateVisualObjects();
            DisplayGrid.Children.Add(m_DrawingCanvas);
            ResetFactors();
            UpdateUIElements();
            //DataContext = drone;
            //VideoGrid.Children.Add(DroneVideoElem);
            //DroneVideoElem.Visibility = Visibility.Collapsed;
        }

        private DroneClient m_Drone;
        private bool m_Connected;
        private bool m_Flying;
        private bool m_Master;
        private float m_GlobalSpeed;
        private float m_CurveFactor;
        private const float c_DefaultSpeed = 0.1f;
        private const float m_DefaultCurveFactor = 0.3f;
        private DispatcherTimer m_BatteryTimer;
        private DispatcherTimer m_BlinkerTimer;
        //private readonly string videoUrl = "ardrone://192.168.1.1";
        //MediaElement DroneVideoElem;
        private void InitializeDrone()
        {
            m_Drone = DroneClient.Instance;
            m_Connected = false;
            m_Flying = false;
            m_Master = false;
            m_GlobalSpeed = c_DefaultSpeed;
            m_CurveFactor = m_DefaultCurveFactor;
            m_BatteryTimer = new DispatcherTimer();
            m_BatteryTimer.Tick += BatteryTimerTick;
            m_BatteryTimer.Interval = new TimeSpan(0, 0, 5); // 10 seconds
            m_BlinkerTimer = new DispatcherTimer();
            m_BlinkerTimer.Tick += BlinkerTimer_Tick;
            m_BlinkerTimer.Interval = new TimeSpan(0, 0, 0, 0, 500); // 0.5 seconds
            //DroneVideoElem = new MediaElement();
            //DroneVideoElem.MediaFailed += DroneVideoElem_MediaFailed;
        }

        private void globalSpeedSliderValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            m_GlobalSpeed = (float)globalSpeedSlider.Value;
        }

        private void curveFactorSliderValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            m_CurveFactor = (float)curveFactorSlider.Value;
        }
        
        private void BatteryTimerTick(object sender, object e)
        {
            if (m_Connected)
            {
                ShowBatteryState(m_Drone.NavigationDataViewModel.BatteryPercentage);
                if (m_Drone.NavigationDataViewModel.BatteryIsLow.Equals("True"))
                    m_BlinkerTimer.Start();
            }
            else BatteryTextBlock.Text = "";
        }

        void BlinkerTimer_Tick(object sender, object e)
        {
            if (BatteryTextBlock.Visibility == Visibility.Collapsed) BatteryTextBlock.Visibility = Visibility.Visible;
            else if (BatteryTextBlock.Visibility == Visibility.Visible) BatteryTextBlock.Visibility = Visibility.Collapsed;
        }

        private readonly Brush r_BatteryGreen = new SolidColorBrush(Colors.Green);
        private readonly Brush r_BatteryOrange = new SolidColorBrush(Colors.Orange);
        private readonly Brush r_BatteryRed = new SolidColorBrush(Colors.Red);
        private void ShowBatteryState(string percentage)
        {
            try
            {
                int _percentage = Convert.ToInt32(percentage);
                if (_percentage >= 65) BatteryTextBlock.Foreground = r_BatteryGreen;
                else if (_percentage >= 36) BatteryTextBlock.Foreground = r_BatteryOrange;
                else if (_percentage < 36) BatteryTextBlock.Foreground = r_BatteryRed;
                BatteryTextBlock.Text = percentage + " %";
            }
            catch { BatteryTextBlock.Foreground = new SolidColorBrush(Colors.White); }
        }

        private async Task SendGesturesToDrone(KinectInputState gestureCommand)
        {
            if (gestureCommand.Flag == "land")
            {
                m_Drone.InputState.Update(0, 0, 0, 0);
                m_Drone.Land();
                m_Master = true;
            }
            else if (gestureCommand.Flag == "takeoff")
            {
                if (m_Drone.NavigationDataViewModel.BatteryIsLow.Equals("False"))
                {
                    m_Drone.TakeOff();
                    m_Drone.InputState.Update(0, 0, 0, 0);
                }
                else await (new MessageDialog("Battery too low.")).ShowAsync();
            }
            else
            {
                float speed = 0;
                if (gestureCommand.SpeedUp > 0) speed = -m_GlobalSpeed;
                else if (gestureCommand.SpeedUp < 0) speed = m_GlobalSpeed;
                m_Drone.InputState.Update(gestureCommand.Roll*m_CurveFactor, speed, gestureCommand.Rotate, gestureCommand.Ascend);
            }
            m_Flying = gestureCommand.Flying;
            UpdateUIElements();
        }

        private async void ConnectionButton_Click(object sender, RoutedEventArgs e)
        {
            if (!m_Connected)
            {
                if (await m_Drone.ConnectAsync()) 
                {
                    ShowBatteryState(m_Drone.NavigationDataViewModel.BatteryPercentage);
                    m_BatteryTimer.Start();
                    m_Connected = true;
                    m_Master = true;
                    //ShowDroneCamera();
                }
                else await (new MessageDialog("Connection failed...got WLAN-Connection to Drone?")).ShowAsync();
            }
            else
            {
                m_Drone.Dispose();
                m_Drone.Close();
                m_Connected = false;
            }
            ResetFactors();
            UpdateUIElements();
        }

        private async void TakeoffLandButton_Click(object sender, RoutedEventArgs e)
        {
            if (!m_Flying)
            {
                if (m_Drone.NavigationDataViewModel.BatteryIsLow.Equals("False"))
                {
                    m_Drone.TakeOff();
                    m_Drone.InputState.Update(0, 0, 0, 0);
                    m_Flying = true;
                }
                else await (new MessageDialog("Battery too low.")).ShowAsync();
            }
            else
            {
                m_Drone.InputState.Update(0, 0, 0, 0);
                m_Master = true;
                m_Drone.Land();
                m_Flying = false;
            }
            ResetFactors();
            UpdateUIElements();
        }

        private void MasterButton_Click(object sender, RoutedEventArgs e)
        {
            m_Master = !m_Master;
            m_Drone.InputState.Update(0, 0, 0, 0);
            ResetFactors();
            UpdateUIElements();
        }

        //private void ChangeViewButton_Click(object sender, RoutedEventArgs e)
        //{
        //    if (DroneVideoElem.Visibility == Visibility.Collapsed)
        //    {
        //        DroneVideoElem.Visibility = Visibility.Visible;
        //        ShowDroneCamera();
        //    }
        //    else
        //    {
        //        DroneVideoElem.Source = null;
        //        DroneVideoElem.Visibility = Visibility.Collapsed;
        //    }

        //    if (KinectViewBox.Visibility == Visibility.Collapsed)
        //        KinectViewBox.Visibility = Visibility.Visible;
        //    else KinectViewBox.Visibility = Visibility.Collapsed;
        //}

        //private void ShowDroneCamera()
        //{
        //    //IRandomAccessStream stream;
        //    //DroneVideoElem.SetSource(stream, "video/mp4");
        //    DroneVideoElem.Source = new Uri(@videoUrl);
        //    DroneVideoElem.Play(); 
        //}

        //private async void DroneVideoElem_MediaFailed(object sender, ExceptionRoutedEventArgs e)
        //{
        //    await (new MessageDialog(e.ErrorMessage)).ShowAsync();
        //}

        private void ResetFactors()
        {
            m_GlobalSpeed = c_DefaultSpeed;
            globalSpeedSlider.Value = m_GlobalSpeed;
            m_CurveFactor = m_DefaultCurveFactor;
            curveFactorSlider.Value = m_CurveFactor;
            //resetControlSliders();
        }

        private readonly Brush r_MasterYellow = new SolidColorBrush(Colors.Yellow);
        private readonly Brush r_TransparentBrush = new SolidColorBrush(Colors.Transparent);
        private void UpdateUIElements()
        {
            if (m_Connected)
            {
                if (m_Flying && m_Master)
                {
                    ConnectionButton.IsEnabled = false;
                    TakeoffLandButton.Content = "LAND";
                    MasterBorder.BorderBrush = r_MasterYellow;
                }
                else if (m_Flying)
                {
                    ConnectionButton.IsEnabled = false;
                    TakeoffLandButton.Content = "LAND";
                    MasterBorder.BorderBrush = r_TransparentBrush;
                }
                else if (m_Master)
                {
                    ConnectionButton.IsEnabled = true;
                    TakeoffLandButton.Content = "TAKE OFF";
                    MasterBorder.BorderBrush = r_MasterYellow;
                }
                else
                {
                    ConnectionButton.IsEnabled = true;
                    TakeoffLandButton.Content = "TAKE OFF";
                    MasterBorder.BorderBrush = r_TransparentBrush;
                }
                TakeoffLandButton.IsEnabled = true;
                MasterButton.IsEnabled = true;
                globalSpeedSlider.IsEnabled = true;
                curveFactorSlider.IsEnabled = true;
                //ChangeViewButton.IsEnabled = true;
            }
            else
            {
                ConnectionButton.IsEnabled = true;
                TakeoffLandButton.IsEnabled = false;
                TakeoffLandButton.Content = "TAKE OFF";
                MasterBorder.BorderBrush = r_TransparentBrush;
                MasterButton.IsEnabled = false;
                globalSpeedSlider.IsEnabled = false;
                curveFactorSlider.IsEnabled = false;
                //KinectViewBox.Visibility = Visibility.Visible;
                //DroneVideoElem.Visibility = Visibility.Collapsed;
                //ChangeViewButton.IsEnabled = false;
                BatteryTextBlock.Text = "";
                m_Flying = false;
                m_Master = false;
            }
        }
    }
}
