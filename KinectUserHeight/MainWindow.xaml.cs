using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace KinectUserHeight
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
     
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.KinectSensors.Where(x => x.Status == KinectStatus.Connected).FirstOrDefault();

            if (_sensor != null)
            {
                _sensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(Sensor_SkeletonFrameReady);
                _sensor.SkeletonStream.Enable();

                _sensor.Start();
            }
        }

        void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (var frame = e.OpenSkeletonFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    Skeleton[] skeletons = new Skeleton[frame.SkeletonArrayLength];

                    frame.CopySkeletonDataTo(skeletons);

                    var skeleton = skeletons.Where(s => s.TrackingState == SkeletonTrackingState.Tracked).FirstOrDefault();

                    if (skeleton != null)
                    {
                        // Calculate height.
                        double height = Math.Round(skeleton.Height(), 2);
                        double left = Math.Round(skeleton.LeftHand(), 2);
                        double right = Math.Round(skeleton.RightHand(), 2);
                        double rightArmRelativeAngle = Math.Round(skeleton.RightArmRelativeAngle(), 2);
                        double neckRelativeAngle = Math.Round(skeleton.NeckRelativeAngle(), 2);

                        // Draw skeleton joints.
                        foreach (JointType joint in Enum.GetValues(typeof(JointType)))
                        {
                            DrawJoint(skeleton.Joints[joint].ScaleTo(640, 480));
                        }

                        // Display height.
                        tblHeight.Text = "Height: " + height.ToString() + "m";

                        //Display Arms
                        tblLeft.Text = "Left: " + left.ToString() + "m";
                        tblRight.Text = "Right: " + right.ToString() + "m";
                        //tblAngleLeft.Text = "Relative Angle Left: " + leftArmRelativeAngle.ToString() + "º";
                        tblAngleRight.Text = "Relative Angle Right: " + rightArmRelativeAngle.ToString() + "º";
                        tblAngleNeck.Text = "Relative Angle Neck: " + neckRelativeAngle.ToString() + "º";

                        //Display SkeletonPositions
                        //Display LeftArmPositions
                        tblPositionHeaderX.Text = "Position Head X: " +skeleton.Joints[JointType.Head].Position.X;
                        tblPositionHeaderY.Text = "Position Head Y: " + skeleton.Joints[JointType.Head].Position.Y;
                        tblPositionHeaderZ.Text = "Position Head Z: " + skeleton.Joints[JointType.Head].Position.Z;
                        tblPositionShoulderRightX.Text = "Position Right Shoulder Y: " + skeleton.Joints[JointType.ShoulderRight].Position.Y;
                        tblPositionShoulderRightY.Text = "Position Right Shoulder X: " + skeleton.Joints[JointType.ShoulderRight].Position.X;
                        tblPositionShoulderRightZ.Text = "Position Right Shoulder Y: " + skeleton.Joints[JointType.ShoulderRight].Position.Y;
                        tblPositionElbowRightX.Text = "Position Rigth Elbow Y: " + skeleton.Joints[JointType.ElbowRight].Position.Y;
                        tblPositionElbowRightY.Text = "Position Rigth Elbow X: " + skeleton.Joints[JointType.ElbowRight].Position.X;
                        tblPositionElbowRightZ.Text = "Position Rigth Elbow Y: " + skeleton.Joints[JointType.ElbowRight].Position.Y;

                        
                    }
                }
            }
        }

        private void DrawJoint(Joint joint)
        {
            Ellipse ellipse = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = new SolidColorBrush(Colors.LightCoral)
            };

            Canvas.SetLeft(ellipse, joint.Position.X);
            Canvas.SetTop(ellipse, joint.Position.Y);

            canvas.Children.Add(ellipse);
        }

        private void btnRecord_Click(object sender, RoutedEventArgs e)
        {
            btnStop.IsEnabled = true;
        }

        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            btnRecord.Focus();
            btnStop.IsEnabled = false;
        }
    }
}
